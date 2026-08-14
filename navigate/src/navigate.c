/*
 * navigate - drive to a point on the map slam2d built, with no floating point.
 *
 * This is the piece that makes "tap a destination and it goes there" mean
 * something, and it is also the piece that can move a vehicle on its own, so it
 * is worth being exact about the shape of it.
 *
 *   ouster-edge --ring--> navigate --TELE--> agx-cmd --BCAN--> can-bridge --> CAN
 *
 * It emits the same TELE frames a human at a tablet would, which is deliberate:
 * everything downstream stays unchanged and keeps its own protections. agx-cmd
 * still refuses to emit until armed, still ramps down and sends explicit stops
 * when frames go quiet, and can-bridge still needs allow_inject before anything
 * reaches the bus. Autonomy is a new *source* of intent, not a new path to the
 * motors.
 *
 * ## The deadman does not go away, it changes what it watches
 *
 * Under teleoperation the deadman watches the operator's link, and that is
 * right: if the tablet is gone, the person steering is gone. Under autonomy that
 * test is wrong in both directions - the tablet being absent is now normal, and
 * the tablet being present says nothing about whether the robot still knows
 * where it is.
 *
 * So the link deadman is replaced by more watchers, not fewer:
 *
 *   - the match score collapses            the map no longer explains the scan
 *   - the match sits at its search edge    the matcher is not keeping up
 *   - the ring stops arriving              the sensor or ouster-edge is gone
 *   - a zone in the ring is occupied       something is close, ouster-edge said so
 *   - the plan makes no progress           stuck, or the goal is unreachable
 *   - the goal is reached                  the ordinary way to finish
 *
 * Any of them disarms and stops. A robot that has lost its position and keeps
 * driving is worse than one that stops when the tablet disconnects.
 *
 * ## Planning
 *
 * A wavefront from the goal outwards over the coarse pyramid level, which is
 * 20 cm cells, then the controller walks downhill. Unknown ground counts as
 * blocked: the vehicle may drive where it has looked, and nowhere else. That is
 * why a destination has to be somewhere already mapped.
 */
#include <arpa/inet.h>
#include <errno.h>
#include <fcntl.h>
#include <getopt.h>
#include <poll.h>
#include <signal.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <syslog.h>
#include <sys/socket.h>
#include <time.h>
#include <unistd.h>

#include "slam2d.h"

#define RING_HDR	20
#define MAX_SECTORS	4096
#define TELE_LEN	32
#define PLAN_LEVEL	2		/* 20 cm cells when the map is 5 cm */
#define BLOCKED		0xFFFFu
#define STEP_ORTHO	10
#define STEP_DIAG	14

enum state {
	ST_IDLE,			/* no goal */
	ST_DRIVING,
	ST_ARRIVED,
	ST_BLOCKED_FAULT,		/* watcher tripped; needs a new goal */
};

static const char *state_name(enum state s)
{
	switch (s) {
	case ST_IDLE:    return "idle";
	case ST_DRIVING: return "driving";
	case ST_ARRIVED: return "arrived";
	default:         return "stopped";
	}
}

static struct {
	int ring_port, cmd_port;
	int ring_fd, cmd_fd, tele_fd;
	struct sockaddr_in tele_to;
	bool have_tele;

	int32_t map_cm, res_cm, max_range_cm;
	int32_t win_xy_cm, win_a;
	int32_t robot_radius_cm;
	int32_t arrive_cm;
	int32_t min_score_pct;
	int32_t max_linear_pct;		/* of whatever agx-cmd calls full speed */
	int32_t max_yaw_pct;
	int32_t stall_ms;
	int32_t ring_timeout_ms;
	int min_returns;

	char *status_path;
	char *plan_dump;
	char *map_export;
	int map_level;
	int status_ms;
	bool foreground;
	bool dry_run;

	struct s2_map map;
	struct s2_pose pose, prev;
	bool have_prev, mapped;

	uint16_t *cost;			/* wavefront, PLAN_LEVEL sized */
	int32_t *queue;
	int32_t goal_x_cm, goal_y_cm;
	bool have_goal;

	enum state state;
	const char *fault;

	uint32_t seq;
	uint64_t rings, matched, tele_sent, replans;
	int32_t last_score_pct;
	int32_t last_remaining_cm, best_remaining_cm;
	uint64_t last_ring_ms, last_progress_ms;
	bool zone_alarm;
} g;

static volatile sig_atomic_t stop_requested;

static void on_signal(int sig)
{
	(void)sig;
	stop_requested = 1;
}

static void logmsg(int prio, const char *fmt, ...)
{
	va_list ap;

	va_start(ap, fmt);
	if (g.foreground) {
		vfprintf(stderr, fmt, ap);
		fputc('\n', stderr);
	} else {
		vsyslog(prio, fmt, ap);
	}
	va_end(ap);
}

static uint64_t now_ms(void)
{
	struct timespec ts;

	clock_gettime(CLOCK_MONOTONIC, &ts);
	return (uint64_t)ts.tv_sec * 1000u + ts.tv_nsec / 1000000u;
}

/* --------------------------------------------------------------- planning */

static inline int32_t plan_w(void) { return g.map.w[PLAN_LEVEL]; }
static inline int32_t plan_h(void) { return g.map.h[PLAN_LEVEL]; }
static inline int32_t plan_res(void) { return g.map.res_cm << PLAN_LEVEL; }

static inline int32_t plan_cx(int32_t x_cm)
{
	int32_t d = x_cm - g.map.origin_x_cm;

	return d >= 0 ? d / plan_res() : -(((-d) + plan_res() - 1) / plan_res());
}

static inline int32_t plan_cy(int32_t y_cm)
{
	int32_t d = y_cm - g.map.origin_y_cm;

	return d >= 0 ? d / plan_res() : -(((-d) + plan_res() - 1) / plan_res());
}

/*
 * Cost class per planning cell: 0 blocked, 1 known free, UNKNOWN_MULT otherwise.
 *
 * Blocked means *confidently occupied*, plus everything within the robot's
 * radius of it. The pyramid has already taken the maximum over each cell's
 * children, so a 20 cm cell counts as occupied if any 5 cm part of it does -
 * obstacles grow rather than disappear as resolution drops, which is the
 * direction that keeps a planner honest.
 *
 * Unknown is *not* blocked, and getting that wrong cost a working test. The
 * first version refused to plan through anything not confidently free, on the
 * reasoning that the vehicle should drive only where it has looked. But rays
 * diverge: at 3 m, 360 of them are 5.2 cm apart, so cells between rays are
 * never marked and stay unknown. Max-pooled to 20 cm, one unmarked child made
 * the whole cell unknown, and after a full sweep of a room almost every cell
 * was "unexplored". Nothing was reachable and the goal came back unreachable.
 *
 * So unknown is traversable at a multiple of the cost. Paths prefer ground that
 * has actually been seen and will detour a long way to stay on it, but a gap
 * between two rays no longer amounts to a wall.
 */
#define UNKNOWN_MULT	4
/* The planner's own view, for when it says no and the map looks fine.
 * black = blocked, grey = unknown-but-passable, white = known free. */
static void dump_plan(const uint8_t *blocked, const char *path)
{
	int32_t w = plan_w(), h = plan_h(), x, y;
	FILE *f = fopen(path, "wb");

	if (!f)
		return;
	fprintf(f, "P5\n%d %d\n255\n", (int)w, (int)h);
	for (y = h - 1; y >= 0; y--)
		for (x = 0; x < w; x++)
			fputc(blocked[y * w + x] == 0 ? 0 :
			      blocked[y * w + x] == 1 ? 255 : 128, f);
	fclose(f);
}

static bool build_blocked(uint8_t *blocked)
{
	int32_t w = plan_w(), h = plan_h();
	int32_t inflate = (g.robot_radius_cm + plan_res() - 1) / plan_res();
	const uint8_t *cell = g.map.cell[PLAN_LEVEL];
	int32_t x, y, i;
	bool any_free = false;

	for (i = 0; i < w * h; i++) {
		uint8_t v = cell[i];

		if (v > S2_UNKNOWN + 8)
			blocked[i] = 0;			/* occupied */
		else if (v < S2_UNKNOWN - 8)
			blocked[i] = 1;			/* seen, free */
		else
			blocked[i] = UNKNOWN_MULT;	/* never looked at */
		if (blocked[i] == 1)
			any_free = true;
	}
	if (!any_free)
		return false;

	/*
	 * Wherever the vehicle actually is, it can be.
	 *
	 * Inflation grows obstacles by the robot's radius, and a vehicle that
	 * has ended up inside that margin - having driven there before the
	 * obstacle was mapped, which is exactly what happens on a first plan -
	 * would otherwise be standing on a cell the planner calls impossible.
	 * The wavefront could not reach it, and the only report was "no route",
	 * with a map that looked perfectly fine. Letting it out is both more
	 * useful and safer than freezing it in place; anything genuinely
	 * occupied stays blocked.
	 */
	for (i = 0; i < inflate; i++) {
		static uint8_t *tmp;
		static int32_t tmp_n;

		if (tmp_n < w * h) {
			free(tmp);
			tmp = malloc((size_t)w * h);
			tmp_n = tmp ? w * h : 0;
		}
		if (!tmp)
			break;
		memcpy(tmp, blocked, (size_t)w * h);
		for (y = 0; y < h; y++) {
			for (x = 0; x < w; x++) {
				int dy, dx;

				if (!tmp[y * w + x])
					continue;	/* already blocked */
				for (dy = -1; dy <= 1; dy++) {
					for (dx = -1; dx <= 1; dx++) {
						int32_t nx = x + dx, ny = y + dy;

						if (nx < 0 || ny < 0 ||
						    nx >= w || ny >= h)
							continue;
						if (!tmp[ny * w + nx])
							blocked[y * w + x] = 0;
					}
				}
			}
		}
	}

	{
		int32_t rx = plan_cx(g.pose.x_cm), ry = plan_cy(g.pose.y_cm);
		int dy, dx;

		for (dy = -1; dy <= 1; dy++) {
			for (dx = -1; dx <= 1; dx++) {
				int32_t nx = rx + dx, ny = ry + dy;

				if (nx < 0 || ny < 0 || nx >= w || ny >= h)
					continue;
				if (cell[ny * w + nx] > S2_UNKNOWN + 8)
					continue;	/* really is an obstacle */
				if (!blocked[ny * w + nx])
					blocked[ny * w + nx] = 1;
			}
		}
	}
	return true;
}

/*
 * Cost to reach the goal from every reachable cell.
 *
 * SPFA rather than a heap: with only two edge weights the frontier stays small,
 * and an "already queued" flag keeps each cell in the ring buffer at most once,
 * which is what lets the queue be exactly one entry per cell. Without the flag a
 * cell can be re-relaxed many times and the queue overflows - on 200x200 cells
 * that is the difference between a plan and a truncated one.
 */
static bool plan(void)
{
	int32_t w = plan_w(), h = plan_h(), n = w * h;
	static uint8_t *blocked, *inq;
	static int32_t alloc_n;
	int32_t gx = plan_cx(g.goal_x_cm), gy = plan_cy(g.goal_y_cm);
	int32_t head = 0, count = 0;
	int32_t i;

	if (gx < 0 || gy < 0 || gx >= w || gy >= h)
		return false;

	/*
	 * The pyramid the planner reads is only rebuilt by s2_match, and the
	 * map is marked dirty again by every update that follows one. Planning
	 * without this read a pyramid one revolution stale - and on the very
	 * first goal, before any match had happened, it read a pyramid that was
	 * still entirely unknown. Everything looked passable, the route went
	 * straight through a wall, and the vehicle drove into it before the
	 * real map arrived.
	 */
	if (g.map.dirty)
		s2_map_build_pyramid(&g.map);

	if (alloc_n < n) {
		free(blocked);
		free(inq);
		blocked = malloc((size_t)n);
		inq = malloc((size_t)n);
		alloc_n = (blocked && inq) ? n : 0;
	}
	if (!blocked || !inq || !build_blocked(blocked))
		return false;
	if (g.plan_dump)
		dump_plan(blocked, g.plan_dump);
	if (!blocked[gy * w + gx])
		return false;			/* goal is inside an obstacle */

	for (i = 0; i < n; i++) {
		g.cost[i] = BLOCKED;
		inq[i] = 0;
	}
	g.cost[gy * w + gx] = 0;
	g.queue[0] = gy * w + gx;
	inq[gy * w + gx] = 1;
	count = 1;

	while (count) {
		int32_t idx = g.queue[head];
		int32_t cx = idx % w, cy = idx / w;
		uint16_t c = g.cost[idx];
		int dy, dx;

		head = (head + 1) % n;
		count--;
		inq[idx] = 0;

		for (dy = -1; dy <= 1; dy++) {
			for (dx = -1; dx <= 1; dx++) {
				int32_t nx = cx + dx, ny = cy + dy, ni;
				uint16_t nc;

				if ((!dx && !dy) || nx < 0 || ny < 0 ||
				    nx >= w || ny >= h)
					continue;
				ni = ny * w + nx;
				if (!blocked[ni])
					continue;	/* obstacle */
				nc = (uint16_t)(c + blocked[ni] *
						((dx && dy) ? STEP_DIAG
							    : STEP_ORTHO));
				if (nc < g.cost[ni]) {
					g.cost[ni] = nc;
					if (!inq[ni] && count < n) {
						g.queue[(head + count) % n] = ni;
						inq[ni] = 1;
						count++;
					}
				}
			}
		}
	}
	g.replans++;
	return true;
}

/*
 * Walk downhill from the vehicle for a few cells and report where that leads.
 *
 * Steering at the immediately adjacent cell makes the vehicle chase the grid
 * and wobble; looking several cells ahead smooths the heading without needing
 * any path smoothing machinery.
 */
static bool lookahead(int32_t *tx_cm, int32_t *ty_cm, int32_t *remaining_cm)
{
	int32_t w = plan_w(), h = plan_h();
	int32_t cx = plan_cx(g.pose.x_cm), cy = plan_cy(g.pose.y_cm);
	int steps;

	if (cx < 0 || cy < 0 || cx >= w || cy >= h)
		return false;
	if (g.cost[cy * w + cx] == BLOCKED)
		return false;

	*remaining_cm = (int32_t)g.cost[cy * w + cx] * plan_res() / STEP_ORTHO;

	for (steps = 0; steps < 5; steps++) {
		int32_t bx = cx, by = cy;
		uint16_t best = g.cost[cy * w + cx];
		int dy, dx;

		for (dy = -1; dy <= 1; dy++) {
			for (dx = -1; dx <= 1; dx++) {
				int32_t nx = cx + dx, ny = cy + dy;

				if (nx < 0 || ny < 0 || nx >= w || ny >= h)
					continue;
				if (g.cost[ny * w + nx] < best) {
					best = g.cost[ny * w + nx];
					bx = nx;
					by = ny;
				}
			}
		}
		if (bx == cx && by == cy)
			break;
		cx = bx;
		cy = by;
	}
	*tx_cm = g.map.origin_x_cm + cx * plan_res() + plan_res() / 2;
	*ty_cm = g.map.origin_y_cm + cy * plan_res() + plan_res() / 2;
	return true;
}

/* ------------------------------------------------------------------ TELE */

static void tele_send(bool armed, int32_t strafe, int32_t fwd, int32_t yaw)
{
	uint8_t f[TELE_LEN];
	uint64_t t = now_ms();
	int i;

	if (!g.have_tele || g.dry_run)
		return;
	memset(f, 0, sizeof(f));
	memcpy(f, "TELE", 4);
	f[4] = 1;
	f[5] = armed ? 1 : 0;
	g.seq++;
	for (i = 0; i < 4; i++)
		f[8 + i] = (uint8_t)(g.seq >> (8 * i));
	for (i = 0; i < 8; i++)
		f[12 + i] = (uint8_t)(t >> (8 * i));
	{
		int16_t ax[4] = { (int16_t)strafe, (int16_t)fwd,
				  (int16_t)yaw, 0 };

		for (i = 0; i < 4; i++) {
			f[20 + 2 * i] = (uint8_t)(ax[i] & 0xff);
			f[21 + 2 * i] = (uint8_t)((ax[i] >> 8) & 0xff);
		}
	}
	if (sendto(g.tele_fd, f, sizeof(f), 0,
		   (struct sockaddr *)&g.tele_to, sizeof(g.tele_to)) > 0)
		g.tele_sent++;
}

/* Disarm and stop, and say why the first time. agx-cmd will ramp down and emit
 * its own explicit stops; sending a disarmed frame is what starts that, rather
 * than simply going quiet and waiting for its deadman. */
static void halt(enum state st, const char *why)
{
	if (g.state != st || g.fault != why) {
		logmsg(st == ST_ARRIVED ? LOG_NOTICE : LOG_WARNING,
		       "%s: %s", state_name(st), why);
		g.state = st;
		g.fault = why;
	}
	tele_send(false, 0, 0, 0);
}

/* ------------------------------------------------------------- the loop */

static int32_t clamp(int32_t v, int32_t lo, int32_t hi)
{
	return v < lo ? lo : (v > hi ? hi : v);
}

/*
 * One control step, run when a revolution has been matched.
 *
 * Order matters: every reason to stop is checked before anything that could
 * produce motion, so there is no path through this function that commands a
 * vehicle whose position is in doubt.
 */
static void control_step(void)
{
	int32_t tx, ty, remaining, want, err, fwd, yaw;
	uint64_t t = now_ms();

	if (!g.have_goal || g.state == ST_ARRIVED ||
	    g.state == ST_BLOCKED_FAULT) {
		tele_send(false, 0, 0, 0);
		return;
	}

	/* --- the watchers, before anything else --- */
	if (g.last_score_pct < g.min_score_pct) {
		halt(ST_BLOCKED_FAULT, "match score collapsed; position is not "
		     "trustworthy");
		return;
	}
	if (g.zone_alarm) {
		halt(ST_BLOCKED_FAULT, "ouster-edge zone is occupied");
		return;
	}
	if (t - g.last_ring_ms > (uint64_t)g.ring_timeout_ms) {
		halt(ST_BLOCKED_FAULT, "no ring from ouster-edge");
		return;
	}

	if (!lookahead(&tx, &ty, &remaining)) {
		/* The vehicle is somewhere the wavefront never reached. Try
		 * once more with the map as it now is before giving up: the
		 * usual cause is that the plan predates the ground the vehicle
		 * is standing on being mapped. */
		if (!plan() || !lookahead(&tx, &ty, &remaining)) {
			/* Say where, not just that. Which cell the vehicle
			 * believes it is in, and what the planner made of it,
			 * is the difference between a diagnosable stop and a
			 * mysterious one. */
			int32_t cx = plan_cx(g.pose.x_cm);
			int32_t cy = plan_cy(g.pose.y_cm);
			int32_t w = plan_w();
			int in = cx >= 0 && cy >= 0 && cx < w && cy < plan_h();

			logmsg(LOG_WARNING,
			       "no route: pose %d,%d cm is plan cell %d,%d "
			       "(%s), occupancy %d, cost %u",
			       (int)g.pose.x_cm, (int)g.pose.y_cm,
			       (int)cx, (int)cy,
			       in ? "in the map" : "OUTSIDE the map",
			       in ? g.map.cell[PLAN_LEVEL][cy * w + cx] : -1,
			       in ? g.cost[cy * w + cx] : 0);
			halt(ST_BLOCKED_FAULT, "no route to the goal from here");
			return;
		}
	}

	g.last_remaining_cm = remaining;
	if (remaining < g.best_remaining_cm - 10) {
		g.best_remaining_cm = remaining;
		g.last_progress_ms = t;
	}
	if (t - g.last_progress_ms > (uint64_t)g.stall_ms) {
		halt(ST_BLOCKED_FAULT, "no progress towards the goal");
		return;
	}

	{
		int32_t dx = g.goal_x_cm - g.pose.x_cm;
		int32_t dy = g.goal_y_cm - g.pose.y_cm;

		if (dx * dx + dy * dy <= g.arrive_cm * g.arrive_cm) {
			halt(ST_ARRIVED, "goal reached");
			return;
		}
	}

	/* --- only now, motion --- */
	want = s2_atan2(ty - g.pose.y_cm, tx - g.pose.x_cm);
	err = s2_angle_diff(want, g.pose.a);

	/* Yaw proportional to the heading error, saturating at a quarter turn
	 * of error. All integer: err is in 1/4096 turn, and a quarter turn is
	 * S2_TURN/4. */
	yaw = err * g.max_yaw_pct * 100 / (S2_TURN / 4);
	yaw = clamp(yaw, -g.max_yaw_pct * 100, g.max_yaw_pct * 100);

	/* Forward speed falls off with heading error, so the vehicle turns in
	 * place when badly aligned instead of driving a wide arc into whatever
	 * it was avoiding, and eases off as it arrives. */
	fwd = g.max_linear_pct * 100;
	{
		int32_t e = err < 0 ? -err : err;

		if (e > S2_TURN / 8)
			fwd = 0;
		else
			fwd = fwd * (S2_TURN / 8 - e) / (S2_TURN / 8);
	}
	if (remaining < 100)
		fwd = fwd * remaining / 100;

	g.state = ST_DRIVING;
	g.fault = NULL;
	tele_send(true, 0, clamp(fwd, 0, 10000), clamp(yaw, -10000, 10000));
}

static void handle_ring(const uint8_t *p, size_t len)
{
	static uint16_t ranges[MAX_SECTORS];
	struct s2_match_result res;
	struct s2_pose seed;
	uint16_t sectors;
	int i, returns = 0;

	g.rings++;
	if (len < RING_HDR || memcmp(p, "OSED", 4))
		return;
	sectors = (uint16_t)(p[6] | (p[7] << 8));
	if (!sectors || sectors > MAX_SECTORS ||
	    len < (size_t)RING_HDR + 3u * sectors)
		return;

	/* The ring carries ouster-edge's own zone verdict at offset 10, so the
	 * reflex layer needs no extra plumbing to be heard here. */
	g.zone_alarm = p[10] != 0;
	g.last_ring_ms = now_ms();

	for (i = 0; i < sectors; i++) {
		ranges[i] = (uint16_t)(p[RING_HDR + 2 * i] |
				       (p[RING_HDR + 2 * i + 1] << 8));
		if (ranges[i] && ranges[i] != 0xFFFF &&
		    (!g.max_range_cm || ranges[i] <= g.max_range_cm))
			returns++;
	}
	if (returns < g.min_returns)
		return;

	if (!g.mapped) {
		s2_map_update(&g.map, &g.pose, ranges, sectors, g.max_range_cm);
		g.mapped = true;
		g.matched++;
		return;
	}

	seed = g.pose;
	if (g.have_prev) {
		seed.x_cm = g.pose.x_cm + (g.pose.x_cm - g.prev.x_cm);
		seed.y_cm = g.pose.y_cm + (g.pose.y_cm - g.prev.y_cm);
		seed.a = (g.pose.a + (g.pose.a - g.prev.a)) & S2_ANG_MASK;
	}
	if (!s2_match(&g.map, &seed, ranges, sectors, g.max_range_cm,
		      g.win_xy_cm, g.win_a, &res))
		return;

	g.last_score_pct = res.max_score
			 ? (int32_t)((int64_t)res.score * 100 / res.max_score)
			 : 0;
	if (res.at_edge && g.state == ST_DRIVING)
		halt(ST_BLOCKED_FAULT, "matcher pinned to its search window");

	g.prev = g.pose;
	g.have_prev = true;
	g.pose = res.pose;
	g.matched++;
	s2_map_update(&g.map, &g.pose, ranges, sectors, g.max_range_cm);

	/* Replan on the map as it now is. Cheap enough at this size to do every
	 * revolution, which removes a whole class of stale-plan bug. */
	if (g.have_goal && g.state == ST_DRIVING)
		plan();

	control_step();
}

static void handle_cmd(char *line)
{
	int32_t x, y;

	while (*line == ' ')
		line++;
	if (!strncmp(line, "GOAL", 4) &&
	    sscanf(line + 4, "%d %d", &x, &y) == 2) {
		g.goal_x_cm = x;
		g.goal_y_cm = y;
		g.have_goal = true;
		g.best_remaining_cm = 0x7FFFFFFF;
		g.last_progress_ms = now_ms();
		g.fault = NULL;
		if (!plan()) {
			halt(ST_BLOCKED_FAULT,
			     "goal is unreachable or not on mapped ground");
			return;
		}
		g.state = ST_DRIVING;
		logmsg(LOG_NOTICE, "goal set to %d,%d cm", (int)x, (int)y);
	} else if (!strncmp(line, "STOP", 4) || !strncmp(line, "CLEAR", 5)) {
		g.have_goal = false;
		g.state = ST_IDLE;
		g.fault = NULL;
		halt(ST_IDLE, "stopped by request");
	} else {
		logmsg(LOG_WARNING, "unknown command: %.32s", line);
	}
}

static void status_write(void)
{
	char tmp[256];
	FILE *fp;

	if (!g.status_path)
		return;
	snprintf(tmp, sizeof(tmp), "%s.tmp", g.status_path);
	fp = fopen(tmp, "w");
	if (!fp)
		return;
	fprintf(fp,
		"{\n"
		"\t\"state\": \"%s\",\n"
		"\t\"fault\": %s%s%s,\n"
		"\t\"pose_cm\": {\"x\": %d, \"y\": %d, \"a\": %d},\n"
		"\t\"goal_cm\": {\"x\": %d, \"y\": %d, \"set\": %s},\n"
		"\t\"remaining_cm\": %d,\n"
		"\t\"match_score_pct\": %d,\n"
		"\t\"zone_alarm\": %s,\n"
		"\t\"rings\": %llu,\n"
		"\t\"matched\": %llu,\n"
		"\t\"replans\": %llu,\n"
		"\t\"tele_sent\": %llu,\n"
		"\t\"commanding\": %s\n"
		"}\n",
		state_name(g.state),
		g.fault ? "\"" : "", g.fault ? g.fault : "null",
		g.fault ? "\"" : "",
		(int)g.pose.x_cm, (int)g.pose.y_cm, (int)g.pose.a,
		(int)g.goal_x_cm, (int)g.goal_y_cm,
		g.have_goal ? "true" : "false",
		(int)g.last_remaining_cm, (int)g.last_score_pct,
		g.zone_alarm ? "true" : "false",
		(unsigned long long)g.rings, (unsigned long long)g.matched,
		(unsigned long long)g.replans,
		(unsigned long long)g.tele_sent,
		(g.have_tele && !g.dry_run) ? "true" : "false");
	fclose(fp);
	rename(tmp, g.status_path);
}

/* ------------------------------------------------------------------ main */

static bool parse_hostport(const char *s, struct sockaddr_in *out, int defport)
{
	char buf[128], *colon;

	snprintf(buf, sizeof(buf), "%s", s);
	memset(out, 0, sizeof(*out));
	out->sin_family = AF_INET;
	out->sin_port = htons((uint16_t)defport);
	colon = strrchr(buf, ':');
	if (colon) {
		*colon = '\0';
		out->sin_port = htons((uint16_t)atoi(colon + 1));
	}
	return inet_pton(AF_INET, buf, &out->sin_addr) == 1;
}

static int udp_bind(int port)
{
	struct sockaddr_in a;
	int fd = socket(AF_INET, SOCK_DGRAM | SOCK_CLOEXEC, 0);
	int one = 1;

	if (fd < 0)
		return -1;
	setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, &one, sizeof(one));
	memset(&a, 0, sizeof(a));
	a.sin_family = AF_INET;
	a.sin_addr.s_addr = htonl(INADDR_ANY);
	a.sin_port = htons((uint16_t)port);
	if (bind(fd, (struct sockaddr *)&a, sizeof(a)) < 0) {
		close(fd);
		return -1;
	}
	fcntl(fd, F_SETFL, fcntl(fd, F_GETFL, 0) | O_NONBLOCK);
	return fd;
}

static const char usage[] =
"Usage: navigate [options]\n"
"  -p, --ring-port PORT     ouster-edge rings arrive here (default 7602)\n"
"  -c, --cmd-port PORT      'GOAL x_cm y_cm' / 'STOP' arrive here (default 7604)\n"
"  -T, --tele HOST[:PORT]   send TELE frames to agx-cmd (default port 7721)\n"
"  -m, --map-size CM        square map edge (default 4000)\n"
"  -r, --resolution CM      cell size (default 5)\n"
"  -R, --max-range CM       ignore returns beyond this (default 3000)\n"
"  -w, --window CM          match translation half-window (default 40)\n"
"  -a, --window-angle N     match rotation half-window, 1/4096 turn (default 120)\n"
"  -b, --robot-radius CM    inflate obstacles by this (default 40)\n"
"  -g, --arrive CM          how close counts as arrived (default 25)\n"
"  -s, --min-score PCT      stop below this match quality (default 25)\n"
"  -l, --max-linear PCT     forward command ceiling (default 35)\n"
"  -y, --max-yaw PCT        yaw command ceiling (default 50)\n"
"  -k, --stall MS           stop after this long without progress (default 8000)\n"
"  -t, --ring-timeout MS    stop after this long without a ring (default 1000)\n"
"  -n, --min-returns N      refuse a scan with fewer returns (default 40)\n"
"  -S, --status PATH        JSON status (default /var/run/navigate.json)\n"
"  -I, --status-interval MS (default 500)\n"
"  -D, --dry-run            plan and report, never send TELE\n"
"  -P, --plan-dump PATH     write the planner's blocked/free view as a PGM\n"
"  -X, --map-export PATH    map, geometry and pose in one file for a client\n"
"  -L, --map-level N        which pyramid level to export (default 2, 20 cm)\n"
"  -f, --foreground         log to stderr\n"
"  -h, --help               this text\n"
"\n"
"Without --tele nothing is ever sent, which is the default. The vehicle also\n"
"needs agx-cmd enabled and can-bridge's allow_inject, both off by default.\n";

int main(int argc, char **argv)
{
	static const struct option opts[] = {
		{ "ring-port",       required_argument, NULL, 'p' },
		{ "cmd-port",        required_argument, NULL, 'c' },
		{ "tele",            required_argument, NULL, 'T' },
		{ "map-size",        required_argument, NULL, 'm' },
		{ "resolution",      required_argument, NULL, 'r' },
		{ "max-range",       required_argument, NULL, 'R' },
		{ "window",          required_argument, NULL, 'w' },
		{ "window-angle",    required_argument, NULL, 'a' },
		{ "robot-radius",    required_argument, NULL, 'b' },
		{ "arrive",          required_argument, NULL, 'g' },
		{ "min-score",       required_argument, NULL, 's' },
		{ "max-linear",      required_argument, NULL, 'l' },
		{ "max-yaw",         required_argument, NULL, 'y' },
		{ "stall",           required_argument, NULL, 'k' },
		{ "ring-timeout",    required_argument, NULL, 't' },
		{ "min-returns",     required_argument, NULL, 'n' },
		{ "status",          required_argument, NULL, 'S' },
		{ "status-interval", required_argument, NULL, 'I' },
		{ "dry-run",         no_argument,       NULL, 'D' },
		{ "plan-dump",       required_argument, NULL, 'P' },
		{ "map-export",      required_argument, NULL, 'X' },
		{ "map-level",       required_argument, NULL, 'L' },
		{ "foreground",      no_argument,       NULL, 'f' },
		{ "help",            no_argument,       NULL, 'h' },
		{ NULL, 0, NULL, 0 }
	};
	struct sigaction sa;
	uint64_t last_status = 0;
	int c;

	g.ring_port = 7602;
	g.cmd_port = 7604;
	g.map_cm = 4000;
	g.res_cm = 5;
	g.max_range_cm = 3000;
	g.win_xy_cm = 40;
	g.win_a = 120;
	g.robot_radius_cm = 40;
	g.arrive_cm = 25;
	g.min_score_pct = 25;
	g.max_linear_pct = 35;
	g.max_yaw_pct = 50;
	g.stall_ms = 8000;
	g.ring_timeout_ms = 1000;
	g.min_returns = 40;
	g.status_path = (char *)"/var/run/navigate.json";
	g.status_ms = 500;
	g.map_level = 2;

	while ((c = getopt_long(argc, argv,
				"p:c:T:m:r:R:w:a:b:g:s:l:y:k:t:n:S:I:P:X:L:Dfh",
				opts, NULL)) != -1) {
		switch (c) {
		case 'p': g.ring_port = atoi(optarg); break;
		case 'c': g.cmd_port = atoi(optarg); break;
		case 'T':
			if (!parse_hostport(optarg, &g.tele_to, 7721)) {
				fprintf(stderr, "bad --tele %s\n", optarg);
				return 2;
			}
			g.have_tele = true;
			break;
		case 'm': g.map_cm = atoi(optarg); break;
		case 'r': g.res_cm = atoi(optarg); break;
		case 'R': g.max_range_cm = atoi(optarg); break;
		case 'w': g.win_xy_cm = atoi(optarg); break;
		case 'a': g.win_a = atoi(optarg); break;
		case 'b': g.robot_radius_cm = atoi(optarg); break;
		case 'g': g.arrive_cm = atoi(optarg); break;
		case 's': g.min_score_pct = atoi(optarg); break;
		case 'l': g.max_linear_pct = atoi(optarg); break;
		case 'y': g.max_yaw_pct = atoi(optarg); break;
		case 'k': g.stall_ms = atoi(optarg); break;
		case 't': g.ring_timeout_ms = atoi(optarg); break;
		case 'n': g.min_returns = atoi(optarg); break;
		case 'S': g.status_path = optarg; break;
		case 'I': g.status_ms = atoi(optarg); break;
		case 'D': g.dry_run = true; break;
		case 'P': g.plan_dump = optarg; break;
		case 'X': g.map_export = optarg; break;
		case 'L': g.map_level = atoi(optarg); break;
		case 'f': g.foreground = true; break;
		case 'h': fputs(usage, stdout); return 0;
		default:  fputs(usage, stderr); return 2;
		}
	}

	if (!g.foreground)
		openlog("navigate", LOG_PID, LOG_DAEMON);

	if (!s2_map_init(&g.map, g.map_cm, g.map_cm, g.res_cm)) {
		logmsg(LOG_ERR, "map %d cm at %d cm: out of memory",
		       (int)g.map_cm, (int)g.res_cm);
		return 1;
	}
	{
		int32_t n = g.map.w[PLAN_LEVEL] * g.map.h[PLAN_LEVEL];

		g.cost = malloc(sizeof(uint16_t) * (size_t)n);
		g.queue = malloc(sizeof(int32_t) * (size_t)n);
		if (!g.cost || !g.queue) {
			logmsg(LOG_ERR, "planner: out of memory");
			return 1;
		}
		logmsg(LOG_NOTICE,
		       "map %dx%d at %d cm, planning on %dx%d at %d cm, "
		       "robot radius %d cm",
		       (int)g.map.w[0], (int)g.map.h[0], (int)g.res_cm,
		       (int)g.map.w[PLAN_LEVEL], (int)g.map.h[PLAN_LEVEL],
		       (int)(g.res_cm << PLAN_LEVEL), (int)g.robot_radius_cm);
	}

	g.ring_fd = udp_bind(g.ring_port);
	g.cmd_fd = udp_bind(g.cmd_port);
	g.tele_fd = socket(AF_INET, SOCK_DGRAM | SOCK_CLOEXEC, 0);
	if (g.ring_fd < 0 || g.cmd_fd < 0 || g.tele_fd < 0) {
		logmsg(LOG_ERR, "bind: %s", strerror(errno));
		return 1;
	}
	if (g.have_tele && !g.dry_run)
		logmsg(LOG_WARNING,
		       "ENABLED: TELE frames will be sent, so this can move a "
		       "vehicle if agx-cmd and can-bridge allow_inject are on");
	else
		logmsg(LOG_NOTICE, "planning only; no TELE destination");

	memset(&sa, 0, sizeof(sa));
	sa.sa_handler = on_signal;
	sigaction(SIGINT, &sa, NULL);
	sigaction(SIGTERM, &sa, NULL);

	while (!stop_requested) {
		struct pollfd pfd[2] = {
			{ g.ring_fd, POLLIN, 0 },
			{ g.cmd_fd,  POLLIN, 0 },
		};
		uint8_t buf[65536];
		ssize_t n;
		uint64_t t;

		if (poll(pfd, 2, 100) < 0 && errno != EINTR)
			break;

		while ((n = recv(g.ring_fd, buf, sizeof(buf), 0)) > 0)
			handle_ring(buf, (size_t)n);
		while ((n = recv(g.cmd_fd, buf, sizeof(buf) - 1, 0)) > 0) {
			buf[n] = 0;
			handle_cmd((char *)buf);
		}

		/* A ring that stops arriving must still be noticed, and nothing
		 * above runs when nothing arrives. */
		t = now_ms();
		if (g.state == ST_DRIVING &&
		    t - g.last_ring_ms > (uint64_t)g.ring_timeout_ms)
			halt(ST_BLOCKED_FAULT, "no ring from ouster-edge");
		if (t - last_status >= (uint64_t)g.status_ms) {
			last_status = t;
			status_write();
			if (g.map_export) {
				/* The exported level is a pyramid level, and
				 * s2_match only rebuilds those when it runs. A
				 * client polling faster than the vehicle moves
				 * would otherwise be shown a map one revolution
				 * stale for no reason. */
				if (g.map.dirty)
					s2_map_build_pyramid(&g.map);
				s2_map_write_export(&g.map, &g.pose,
						    g.map_level, g.map_export);
			}
		}
	}

	logmsg(LOG_NOTICE, "stopping after %llu rings, %llu matched",
	       (unsigned long long)g.rings, (unsigned long long)g.matched);
	/* Leave the vehicle disarmed rather than relying on agx-cmd's deadman
	 * to notice we are gone. */
	tele_send(false, 0, 0, 0);
	tele_send(false, 0, 0, 0);
	status_write();
	s2_map_free(&g.map);
	return 0;
}
