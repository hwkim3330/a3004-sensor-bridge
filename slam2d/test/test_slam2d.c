/*
 * Drive slam2d around a simulated room and check the trajectory it recovers
 * against the one it was actually given.
 *
 * The point of the test is that last part. A SLAM that produces a map which
 * *looks* plausible is easy; one whose pose estimate stays on top of ground
 * truth is the thing worth having, and it is the only property a downstream
 * navigator can rely on.
 *
 * The simulator uses doubles. That is deliberate and does not weaken anything:
 * it stands in for the physical world, which is not obliged to be integer, and
 * everything it hands to slam2d is the same uint16 centimetre array the ring
 * format carries. The code under test never sees a float.
 */
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#include "slam2d.h"

#define SECTORS		360
#define MAX_RANGE_CM	3000

static int fails;

static void check(const char *name, bool ok, const char *detail)
{
	printf("  %s  %s%s%s\n", ok ? "PASS" : "FAIL", name,
	       detail && *detail ? ": " : "", detail ? detail : "");
	if (!ok)
		fails++;
}

/* ------------------------------------------------------- the fake world */

struct seg { double x0, y0, x1, y1; };

/* A 12 x 8 m room with an internal wall, so the place is not symmetric and a
 * wrong heading cannot score as well as the right one. */
static const struct seg world[] = {
	{ -6, -4,  6, -4 },
	{  6, -4,  6,  4 },
	{  6,  4, -6,  4 },
	{ -6,  4, -6, -4 },
	{  1, -4,  1,  0 },		/* stub wall from the south */
	{ -3,  4, -3,  1 },		/* stub wall from the north */
};

static double ray_hit(double px, double py, double ang)
{
	double dx = cos(ang), dy = sin(ang), best = 1e9;
	size_t i;

	for (i = 0; i < sizeof(world) / sizeof(world[0]); i++) {
		const struct seg *s = &world[i];
		double ex = s->x1 - s->x0, ey = s->y1 - s->y0;
		double den = dx * ey - dy * ex;
		double t, u;

		if (fabs(den) < 1e-12)
			continue;
		t = ((s->x0 - px) * ey - (s->y0 - py) * ex) / den;
		u = ((s->x0 - px) * dy - (s->y0 - py) * dx) / den;
		if (t > 0.05 && u >= 0 && u <= 1 && t < best)
			best = t;
	}
	return best;
}

/* Produce the array ouster-edge would publish for a sensor at this pose. */
static void simulate(double x, double y, double th, uint16_t *out)
{
	int i;

	for (i = 0; i < SECTORS; i++) {
		double a = th + 2 * M_PI * i / SECTORS;
		double r = ray_hit(x, y, a);

		out[i] = (r > 30.0) ? 0xFFFF : (uint16_t)(r * 100.0 + 0.5);
	}
}

/* --------------------------------------------------------------- checks */

static void test_trig(void)
{
	int32_t a;
	double worst = 0;

	for (a = 0; a < S2_TURN; a++) {
		double want = sin(2 * M_PI * a / S2_TURN);
		double got = s2_sin(a) / 32768.0;
		double err = fabs(want - got);

		if (err > worst)
			worst = err;
	}
	{
		char buf[64];

		snprintf(buf, sizeof(buf), "worst error %.5f", worst);
		check("cordic sine matches libm", worst < 0.002, buf);
	}
}

int main(void)
{
	struct s2_map map;
	uint16_t scan[SECTORS];
	struct s2_pose truth = { 0, 0, 0 }, est = { 0, 0, 0 }, prev_est;
	double err_sum = 0, err_max = 0, ang_err_max = 0;
	int step, n = 0;
	struct timespec t0, t1;
	double match_ms_total = 0;
	uint32_t cand_total = 0;

	printf("slam2d verification\n\n");
	test_trig();

	if (!s2_map_init(&map, 4000, 4000, 5)) {
		printf("  FAIL  map init\n");
		return 1;
	}
	printf("  map %dx%d cells at %d cm, %d pyramid levels\n",
	       (int)map.w[0], (int)map.h[0], (int)map.res_cm, S2_LEVELS);

	/* Seed the map with the first scan at the origin. Matching against an
	 * empty map is meaningless - every pose scores the same - so the first
	 * one is inserted rather than matched. */
	simulate(0, 0, 0, scan);
	s2_map_update(&map, &est, scan, SECTORS, MAX_RANGE_CM);

	/* A loop around the room, turning as it goes. */
	for (step = 1; step <= 120; step++) {
		double t = step * 0.05;
		double tx = 3.0 * sin(t), ty = 2.0 * (1 - cos(t));
		double tth = 0.35 * sin(t * 1.3);
		struct s2_match_result r;
		double dx, dy, e, ae;

		truth.x_cm = (int32_t)(tx * 100);
		truth.y_cm = (int32_t)(ty * 100);
		truth.a = ((int32_t)(tth * S2_TURN / (2 * M_PI))) & S2_ANG_MASK;

		simulate(tx, ty, tth, scan);

		/* Constant-velocity seed, which is all that is available with no
		 * odometry on the bus yet. */
		prev_est = est;
		{
			static struct s2_pose prev2;
			struct s2_pose seed = est;

			if (step > 1) {
				seed.x_cm = est.x_cm + (est.x_cm - prev2.x_cm);
				seed.y_cm = est.y_cm + (est.y_cm - prev2.y_cm);
				seed.a = (est.a + (est.a - prev2.a)) & S2_ANG_MASK;
			}
			prev2 = prev_est;

			clock_gettime(CLOCK_MONOTONIC, &t0);
			if (!s2_match(&map, &seed, scan, SECTORS, MAX_RANGE_CM,
				      40, 120, &r)) {
				printf("  FAIL  match returned nothing at step %d\n",
				       step);
				fails++;
				break;
			}
			clock_gettime(CLOCK_MONOTONIC, &t1);
			match_ms_total += (t1.tv_sec - t0.tv_sec) * 1000.0 +
					  (t1.tv_nsec - t0.tv_nsec) / 1e6;
			cand_total += r.candidates;
			est = r.pose;
		}

		s2_map_update(&map, &est, scan, SECTORS, MAX_RANGE_CM);

		dx = (est.x_cm - truth.x_cm) / 100.0;
		dy = (est.y_cm - truth.y_cm) / 100.0;
		e = sqrt(dx * dx + dy * dy);
		{
			int32_t d = (est.a - truth.a) & S2_ANG_MASK;

			if (d > S2_TURN / 2)
				d -= S2_TURN;
			ae = fabs(d * 360.0 / S2_TURN);
		}
		err_sum += e;
		if (e > err_max)
			err_max = e;
		if (ae > ang_err_max)
			ang_err_max = ae;
		n++;
	}

	printf("\n  %d steps matched\n", n);
	printf("  position error   mean %.3f m   worst %.3f m\n",
	       n ? err_sum / n : 0.0, err_max);
	printf("  heading error    worst %.2f deg\n", ang_err_max);
	printf("  match cost       %.2f ms per scan, %u candidates per scan\n",
	       n ? match_ms_total / n : 0.0, n ? cand_total / n : 0);

	{
		char buf[80];

		snprintf(buf, sizeof(buf), "worst %.3f m", err_max);
		check("trajectory stays within 25 cm of truth", err_max < 0.25,
		      buf);
		snprintf(buf, sizeof(buf), "worst %.2f deg", ang_err_max);
		check("heading stays within 5 degrees", ang_err_max < 5.0, buf);
		snprintf(buf, sizeof(buf), "mean %.3f m", n ? err_sum / n : 9.9);
		check("mean position error under 10 cm",
		      n && err_sum / n < 0.10, buf);
	}

	s2_map_write_pgm(&map, "/tmp/slam2d_map.pgm");
	printf("\n  wrote /tmp/slam2d_map.pgm\n");
	s2_map_free(&map);

	printf("\n%s\n", fails ? "FAILED" : "all checks passed");
	return fails ? 1 : 0;
}
