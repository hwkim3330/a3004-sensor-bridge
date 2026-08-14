/*
 * slam2d-daemon - build a map from ouster-edge's range ring, without floats.
 *
 * Listens for the OSED datagrams described in doc/RING-FORMAT.md, matches each
 * revolution against the map it has built so far, folds it in, and publishes
 * the pose. It writes nothing to the network and commands nothing; a navigator
 * is a separate concern and a separate process.
 *
 * Two things about the input matter more than anything in here.
 *
 * The ring is a *minimum over a band of elevations*, not a horizontal slice. Run
 * with ouster-edge's default channel_band of every beam and, mounted on a
 * vehicle, every sector reports the floor - a uniform circle with no map in it.
 * See doc/COMPUTE.md for the numbers. This daemon cannot check that for you: it
 * sees ranges, not which channels produced them, and a floor at a constant 0.8 m
 * is a perfectly well formed scan. Setting ouster-edge's channel_band to a
 * near-horizontal slice is a precondition, and the symptom of getting it wrong
 * is a map that stays a small circle no matter how far the vehicle drives.
 *
 * There is no odometry yet, because nothing has ever read the vehicle's CAN
 * bus. The seed for each match is therefore constant velocity from the last two
 * poses, which is enough at 10 Hz and a metre per second, and is the first thing
 * to replace when an adapter exists.
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
#include <sys/time.h>
#include <time.h>
#include <unistd.h>

#include "slam2d.h"

#define RING_MAGIC	0x4445534fu	/* "OSED" little-endian */
#define RING_HDR	20
#define MAX_SECTORS	4096

static struct {
	int port;
	int32_t map_cm;
	int32_t res_cm;
	int32_t max_range_cm;
	int32_t win_xy_cm;
	int32_t win_a;
	int32_t min_returns;
	char *status_path;
	char *map_path;
	char *map_export;
	int map_level;
	int status_ms;
	int map_ms;
	bool foreground;

	struct s2_map map;
	struct s2_pose pose, prev;
	bool have_prev;

	uint64_t rings, matched, skipped_short, skipped_bad, duplicates;
	uint16_t last_frame;
	bool have_last_frame;
	int32_t last_score, last_max_score;
	uint32_t last_candidates;
	uint32_t last_match_us;
	bool last_at_edge;
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

static uint64_t now_us(void)
{
	struct timespec ts;

	clock_gettime(CLOCK_MONOTONIC, &ts);
	return (uint64_t)ts.tv_sec * 1000000u + ts.tv_nsec / 1000u;
}

static uint64_t now_ms(void)
{
	return now_us() / 1000u;
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

	/* Centimetres and 1/4096 turn, the units the core works in, plus the
	 * same values in metres and degrees so a human reading the file does
	 * not have to convert. Deriving them here rather than in every consumer
	 * is the only floating point in the program, and it is on the way out
	 * to a text file rather than anywhere a result depends on it. */
	fprintf(fp,
		"{\n"
		"\t\"pose_cm\": {\"x\": %d, \"y\": %d, \"a\": %d},\n"
		"\t\"pose\": {\"x_m\": %.2f, \"y_m\": %.2f, \"heading_deg\": %.1f},\n"
		"\t\"rings\": %llu,\n"
		"\t\"matched\": %llu,\n"
		"\t\"skipped_too_few_returns\": %llu,\n"
		"\t\"skipped_malformed\": %llu,\n"
		"\t\"duplicate_frames\": %llu,\n"
		"\t\"score\": %d,\n"
		"\t\"score_max\": %d,\n"
		"\t\"score_frac_pct\": %d,\n"
		"\t\"candidates\": %u,\n"
		"\t\"match_us\": %u,\n"
		"\t\"at_search_edge\": %s,\n"
		"\t\"map\": {\"cells\": %d, \"res_cm\": %d, \"bytes\": %ld}\n"
		"}\n",
		(int)g.pose.x_cm, (int)g.pose.y_cm, (int)g.pose.a,
		g.pose.x_cm / 100.0, g.pose.y_cm / 100.0,
		g.pose.a * 360.0 / S2_TURN,
		(unsigned long long)g.rings,
		(unsigned long long)g.matched,
		(unsigned long long)g.skipped_short,
		(unsigned long long)g.skipped_bad,
		(unsigned long long)g.duplicates,
		(int)g.last_score, (int)g.last_max_score,
		g.last_max_score ? (int)((int64_t)g.last_score * 100 /
					 g.last_max_score) : 0,
		g.last_candidates, g.last_match_us,
		g.last_at_edge ? "true" : "false",
		(int)(g.map.w[0] * g.map.h[0]), (int)g.map.res_cm,
		(long)(g.map.w[0] * g.map.h[0]));
	fclose(fp);
	rename(tmp, g.status_path);
}

static int count_returns(const uint16_t *r, int n)
{
	int i, c = 0;

	for (i = 0; i < n; i++)
		if (r[i] && r[i] != 0xFFFF &&
		    (!g.max_range_cm || r[i] <= g.max_range_cm))
			c++;
	return c;
}

static void handle_ring(const uint8_t *p, size_t len)
{
	uint16_t sectors;
	static uint16_t ranges[MAX_SECTORS];
	struct s2_match_result res;
	struct s2_pose seed;
	uint64_t t0;
	int i;

	g.rings++;
	if (len < RING_HDR) {
		g.skipped_bad++;
		return;
	}
	if (memcmp(p, "OSED", 4) != 0) {
		g.skipped_bad++;
		return;
	}
	sectors = (uint16_t)(p[6] | (p[7] << 8));
	if (!sectors || sectors > MAX_SECTORS ||
	    len < (size_t)RING_HDR + 3u * sectors) {
		g.skipped_bad++;
		return;
	}
	for (i = 0; i < sectors; i++)
		ranges[i] = (uint16_t)(p[RING_HDR + 2 * i] |
				       (p[RING_HDR + 2 * i + 1] << 8));


	/*
	 * The same revolution twice is a configuration mistake, not a sensor fault,
	 * and it has to be survivable rather than merely avoided.
	 *
	 * A ring list containing both a broadcast address and 127.0.0.1 delivers
	 * every frame twice, because a router receives its own broadcast. That
	 * doubled the work and quietly damaged the match: two copies of one scan
	 * have no motion between them, so the constant-velocity seed was flattened
	 * every other frame and the search started from a worse guess than it had.
	 *
	 * frame_id is in the header for exactly this. Comparing against the previous
	 * one only - not a set - keeps it to a single integer and still catches every
	 * duplicate that arrives back to back, which is the shape this failure has.
	 */
	{
		uint16_t fid = (uint16_t)(p[8] | (p[9] << 8));

		if (g.have_last_frame && fid == g.last_frame) {
			g.duplicates++;
			return;
		}
		g.last_frame = fid;
		g.have_last_frame = true;
	}

	/* A scan with almost nothing in it will match anywhere. Refusing it
	 * keeps a bad pose out of the map, which is far more expensive to
	 * recover from than a dropped revolution. */
	if (count_returns(ranges, sectors) < g.min_returns) {
		g.skipped_short++;
		return;
	}

	if (!g.matched) {
		/* Nothing to match against yet. */
		s2_map_update(&g.map, &g.pose, ranges, sectors,
			      g.max_range_cm);
		g.matched++;
		return;
	}

	seed = g.pose;
	if (g.have_prev) {
		seed.x_cm = g.pose.x_cm + (g.pose.x_cm - g.prev.x_cm);
		seed.y_cm = g.pose.y_cm + (g.pose.y_cm - g.prev.y_cm);
		seed.a = (g.pose.a + (g.pose.a - g.prev.a)) & S2_ANG_MASK;
	}

	t0 = now_us();
	if (!s2_match(&g.map, &seed, ranges, sectors, g.max_range_cm,
		      g.win_xy_cm, g.win_a, &res)) {
		g.skipped_short++;
		return;
	}
	g.last_match_us = (uint32_t)(now_us() - t0);

	g.prev = g.pose;
	g.have_prev = true;
	g.pose = res.pose;
	g.last_score = res.score;
	g.last_max_score = res.max_score;
	g.last_candidates = res.candidates;
	g.last_at_edge = res.at_edge;
	g.matched++;

	if (res.at_edge)
		logmsg(LOG_WARNING,
		       "match hit the edge of its search window; the vehicle is "
		       "moving faster than win_xy_cm=%d allows between scans",
		       (int)g.win_xy_cm);

	s2_map_update(&g.map, &g.pose, ranges, sectors, g.max_range_cm);
}

static const char usage[] =
"Usage: slam2d-daemon [options]\n"
"  -p, --port PORT        listen for ouster-edge rings here (default 7602)\n"
"  -m, --map-size CM      square map edge in centimetres (default 4000)\n"
"  -r, --resolution CM    cell size (default 5)\n"
"  -R, --max-range CM     ignore returns beyond this (default 3000)\n"
"  -w, --window CM        translation search half-width (default 40)\n"
"  -a, --window-angle N   rotation search half-width, 1/4096 turn (default 120)\n"
"  -n, --min-returns N    refuse a scan with fewer returns (default 40)\n"
"  -S, --status PATH      JSON status file (default /var/run/slam2d.json)\n"
"  -I, --status-interval MS  how often to write it (default 500)\n"
"  -M, --map PATH         periodically write the map as a PGM\n"
"  -T, --map-interval MS  how often to write it (default 5000)\n"
"  -X, --map-export PATH  map, geometry and pose in one file for a client\n"
"  -L, --map-level N      which pyramid level to export (default 2, 20 cm)\n"
"  -f, --foreground       log to stderr\n"
"  -h, --help             this text\n"
"\n"
"The ring must come from an ouster-edge configured with a channel_band that is\n"
"a near-horizontal slice. With the default band of every beam, a sensor mounted\n"
"on a vehicle reports the floor in every direction and the map is meaningless.\n";

int main(int argc, char **argv)
{
	static const struct option opts[] = {
		{ "port",            required_argument, NULL, 'p' },
		{ "map-size",        required_argument, NULL, 'm' },
		{ "resolution",      required_argument, NULL, 'r' },
		{ "max-range",       required_argument, NULL, 'R' },
		{ "window",          required_argument, NULL, 'w' },
		{ "window-angle",    required_argument, NULL, 'a' },
		{ "min-returns",     required_argument, NULL, 'n' },
		{ "status",          required_argument, NULL, 'S' },
		{ "status-interval", required_argument, NULL, 'I' },
		{ "map",             required_argument, NULL, 'M' },
		{ "map-interval",    required_argument, NULL, 'T' },
		{ "map-export",      required_argument, NULL, 'X' },
		{ "map-level",       required_argument, NULL, 'L' },
		{ "foreground",      no_argument,       NULL, 'f' },
		{ "help",            no_argument,       NULL, 'h' },
		{ NULL, 0, NULL, 0 }
	};
	struct sockaddr_in addr;
	struct sigaction sa;
	uint64_t last_status = 0, last_map = 0;
	int sock, c;

	g.port = 7602;
	g.map_cm = 4000;
	g.res_cm = 5;
	g.max_range_cm = 3000;
	g.win_xy_cm = 40;
	g.win_a = 120;
	g.min_returns = 40;
	g.status_path = (char *)"/var/run/slam2d.json";
	g.status_ms = 500;
	g.map_ms = 5000;
	g.map_level = 2;

	while ((c = getopt_long(argc, argv, "p:m:r:R:w:a:n:S:I:M:T:X:L:fh", opts,
				NULL)) != -1) {
		switch (c) {
		case 'p': g.port = atoi(optarg); break;
		case 'm': g.map_cm = atoi(optarg); break;
		case 'r': g.res_cm = atoi(optarg); break;
		case 'R': g.max_range_cm = atoi(optarg); break;
		case 'w': g.win_xy_cm = atoi(optarg); break;
		case 'a': g.win_a = atoi(optarg); break;
		case 'n': g.min_returns = atoi(optarg); break;
		case 'S': g.status_path = optarg; break;
		case 'I': g.status_ms = atoi(optarg); break;
		case 'M': g.map_path = optarg; break;
		case 'T': g.map_ms = atoi(optarg); break;
		case 'X': g.map_export = optarg; break;
		case 'L': g.map_level = atoi(optarg); break;
		case 'f': g.foreground = true; break;
		case 'h': fputs(usage, stdout); return 0;
		default:  fputs(usage, stderr); return 2;
		}
	}

	if (!g.foreground)
		openlog("slam2d", LOG_PID, LOG_DAEMON);

	if (!s2_map_init(&g.map, g.map_cm, g.map_cm, g.res_cm)) {
		logmsg(LOG_ERR, "map %d cm at %d cm per cell: out of memory",
		       (int)g.map_cm, (int)g.res_cm);
		return 1;
	}
	logmsg(LOG_NOTICE,
	       "map %dx%d cells at %d cm (%ld kB), search +/-%d cm and +/-%d deg",
	       (int)g.map.w[0], (int)g.map.h[0], (int)g.res_cm,
	       (long)(g.map.w[0] * g.map.h[0] / 1024),
	       (int)g.win_xy_cm, (int)(g.win_a * 360 / S2_TURN));

	sock = socket(AF_INET, SOCK_DGRAM | SOCK_CLOEXEC, 0);
	if (sock < 0) {
		logmsg(LOG_ERR, "socket: %s", strerror(errno));
		return 1;
	}
	{
		int one = 1;

		setsockopt(sock, SOL_SOCKET, SO_REUSEADDR, &one, sizeof(one));
	}
	memset(&addr, 0, sizeof(addr));
	addr.sin_family = AF_INET;
	addr.sin_addr.s_addr = htonl(INADDR_ANY);
	addr.sin_port = htons((uint16_t)g.port);
	if (bind(sock, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
		logmsg(LOG_ERR, "bind %d: %s", g.port, strerror(errno));
		return 1;
	}
	fcntl(sock, F_SETFL, fcntl(sock, F_GETFL, 0) | O_NONBLOCK);

	/* No SA_RESTART: a blocking recv must be interruptible or a stop
	 * request waits for the next revolution. */
	memset(&sa, 0, sizeof(sa));
	sa.sa_handler = on_signal;
	sigaction(SIGINT, &sa, NULL);
	sigaction(SIGTERM, &sa, NULL);

	while (!stop_requested) {
		struct pollfd pfd = { sock, POLLIN, 0 };
		uint8_t buf[65536];
		ssize_t n;
		uint64_t t;

		if (poll(&pfd, 1, g.status_ms) < 0 && errno != EINTR)
			break;

		while ((n = recv(sock, buf, sizeof(buf), 0)) > 0)
			handle_ring(buf, (size_t)n);

		t = now_ms();
		if (t - last_status >= (uint64_t)g.status_ms) {
			last_status = t;
			status_write();
		}
		if (g.map_path && t - last_map >= (uint64_t)g.map_ms) {
			last_map = t;
			s2_map_write_pgm(&g.map, g.map_path);
		}
		if (g.map_export && t - last_status < 2) {
			/* Alongside the status, so a client polling both sees
			 * one instant rather than two. The exported level is a
			 * pyramid level and only s2_match rebuilds those. */
			if (g.map.dirty)
				s2_map_build_pyramid(&g.map);
			s2_map_write_export(&g.map, &g.pose, g.map_level,
					    g.map_export);
		}
	}

	logmsg(LOG_NOTICE, "stopping after %llu rings, %llu matched",
	       (unsigned long long)g.rings, (unsigned long long)g.matched);
	status_write();
	if (g.map_path)
		s2_map_write_pgm(&g.map, g.map_path);
	s2_map_free(&g.map);
	close(sock);
	return 0;
}
