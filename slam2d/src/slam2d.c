/*
 * slam2d core. See slam2d.h for why none of this uses floating point.
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "slam2d.h"

/* ------------------------------------------------------------------ trig */

/*
 * A quarter turn of sine in Q15, built once by CORDIC.
 *
 * CORDIC rather than a table in the source because the table would be a
 * thousand numbers nobody can check, and rather than libm because libm means
 * doubles - which is the whole thing being avoided. Sixteen iterations gives
 * better than one part in 32768, which is far finer than anything downstream
 * cares about: a Q15 least bit is 3e-5, and the ranges it multiplies are
 * centimetres.
 */
#define QUARTER		(S2_TURN / 4)
#define SUB		256		/* CORDIC works in 1/SUB angle units */

static int16_t sin_tab[QUARTER + 1];
static bool trig_ready;

static void cordic(int32_t angle_sub, int32_t *cos_out, int32_t *sin_out)
{
	/* atan(2^-i) in units of 1/(S2_TURN*SUB) of a revolution */
	static const int32_t atan_tab[16] = {
		131072,   77376,   40884,   20753,
		 10417,    5213,    2607,    1304,
		   652,     326,     163,      81,
		    41,      20,      10,       5,
	};
	/* Seeded with K * 32767 so the gain of the rotations lands at unity. */
	int32_t x = 19898, y = 0, z = angle_sub;
	int i;

	for (i = 0; i < 16; i++) {
		int32_t xn, yn;

		if (z >= 0) {
			xn = x - (y >> i);
			yn = y + (x >> i);
			z -= atan_tab[i];
		} else {
			xn = x + (y >> i);
			yn = y - (x >> i);
			z += atan_tab[i];
		}
		x = xn;
		y = yn;
	}
	*cos_out = x;
	*sin_out = y;
}

static void trig_init(void)
{
	int32_t i, c, s;

	for (i = 0; i <= QUARTER; i++) {
		cordic(i * SUB, &c, &s);
		sin_tab[i] = (int16_t)(s > 32767 ? 32767 : s);
	}
	trig_ready = true;
}

int16_t s2_sin(int32_t a)
{
	int32_t q;

	if (!trig_ready)
		trig_init();
	a &= S2_ANG_MASK;
	q = a / QUARTER;
	a -= q * QUARTER;
	switch (q) {
	case 0:  return sin_tab[a];
	case 1:  return sin_tab[QUARTER - a];
	case 2:  return (int16_t)-sin_tab[a];
	default: return (int16_t)-sin_tab[QUARTER - a];
	}
}

int16_t s2_cos(int32_t a)
{
	return s2_sin(a + QUARTER);
}

int32_t s2_atan2(int32_t y, int32_t x)
{
	static const int32_t atan_tab[16] = {
		131072,   77376,   40884,   20753,
		 10417,    5213,    2607,    1304,
		   652,     326,     163,      81,
		    41,      20,      10,       5,
	};
	int32_t z = 0;
	int i, quad = 0;

	if (!x && !y)
		return 0;

	/* Fold into the right half plane, where the loop converges, and add the
	 * half turn back afterwards. */
	if (x < 0) {
		x = -x;
		y = -y;
		quad = S2_TURN / 2;
	}

	/*
	 * Normalise the magnitude, in both directions.
	 *
	 * Down, because the rotations grow it by about 1.647 and a scan point in
	 * centimetres is already tens of thousands. Up, because a short vector
	 * carries few significant bits and the shifts in the loop throw them
	 * away: without this, the worst error was 0.75 degrees, eight times the
	 * 0.088 degree resolution of the angle unit itself.
	 */
	while (x > (1 << 20) || y > (1 << 20) || y < -(1 << 20)) {
		x >>= 1;
		y >>= 1;
	}
	while (x < (1 << 18) && y < (1 << 18) && y > -(1 << 18)) {
		x <<= 1;
		y <<= 1;
	}

	for (i = 0; i < 16; i++) {
		int32_t xn, yn;

		if (y > 0) {
			xn = x + (y >> i);
			yn = y - (x >> i);
			z += atan_tab[i];
		} else {
			xn = x - (y >> i);
			yn = y + (x >> i);
			z -= atan_tab[i];
		}
		x = xn;
		y = yn;
	}
	/* Round rather than truncate on the way back to whole angle units. */
	z = z >= 0 ? (z + SUB / 2) / SUB : -((-z + SUB / 2) / SUB);
	return (z + quad) & S2_ANG_MASK;
}

int32_t s2_angle_diff(int32_t a, int32_t b)
{
	int32_t d = (a - b) & S2_ANG_MASK;

	return d > S2_TURN / 2 ? d - S2_TURN : d;
}

/* ------------------------------------------------------------------- map */

bool s2_map_init(struct s2_map *m, int32_t width_cm, int32_t height_cm,
		 int32_t res_cm)
{
	int lvl;

	if (res_cm <= 0 || width_cm <= 0 || height_cm <= 0)
		return false;

	memset(m, 0, sizeof(*m));
	m->res_cm = res_cm;
	m->w[0] = width_cm / res_cm;
	m->h[0] = height_cm / res_cm;
	/* Centre the map on the origin, which is where the robot starts. */
	m->origin_x_cm = -width_cm / 2;
	m->origin_y_cm = -height_cm / 2;

	for (lvl = 0; lvl < S2_LEVELS; lvl++) {
		if (lvl) {
			m->w[lvl] = (m->w[lvl - 1] + 1) / 2;
			m->h[lvl] = (m->h[lvl - 1] + 1) / 2;
		}
		m->cell[lvl] = malloc((size_t)m->w[lvl] * m->h[lvl]);
		if (!m->cell[lvl]) {
			s2_map_free(m);
			return false;
		}
		memset(m->cell[lvl], S2_UNKNOWN,
		       (size_t)m->w[lvl] * m->h[lvl]);
	}
	m->dirty = true;
	if (!trig_ready)
		trig_init();
	return true;
}

void s2_map_free(struct s2_map *m)
{
	int lvl;

	for (lvl = 0; lvl < S2_LEVELS; lvl++) {
		free(m->cell[lvl]);
		m->cell[lvl] = NULL;
	}
}

static inline int32_t cx_of(const struct s2_map *m, int32_t x_cm)
{
	return (x_cm - m->origin_x_cm) / m->res_cm;
}

static inline int32_t cy_of(const struct s2_map *m, int32_t y_cm)
{
	return (y_cm - m->origin_y_cm) / m->res_cm;
}

static inline bool inside(const struct s2_map *m, int lvl, int32_t cx,
			  int32_t cy)
{
	return cx >= 0 && cy >= 0 && cx < m->w[lvl] && cy < m->h[lvl];
}

static void bump(struct s2_map *m, int32_t cx, int32_t cy, int delta)
{
	int32_t v;

	if (!inside(m, 0, cx, cy))
		return;
	v = m->cell[0][cy * m->w[0] + cx] + delta;
	if (v < S2_OCC_MIN)
		v = S2_OCC_MIN;
	if (v > S2_OCC_MAX)
		v = S2_OCC_MAX;
	m->cell[0][cy * m->w[0] + cx] = (uint8_t)v;
}

/* Bresenham from the sensor to the return, marking what the beam passed
 * through as free. Integer by construction, which is the point. */
static void ray_free(struct s2_map *m, int32_t x0, int32_t y0, int32_t x1,
		     int32_t y1)
{
	int32_t dx = x1 > x0 ? x1 - x0 : x0 - x1;
	int32_t dy = y1 > y0 ? y1 - y0 : y0 - y1;
	int32_t sx = x0 < x1 ? 1 : -1;
	int32_t sy = y0 < y1 ? 1 : -1;
	int32_t err = dx - dy;

	dy = -dy;
	for (;;) {
		if (x0 == x1 && y0 == y1)
			break;
		bump(m, x0, y0, -S2_MISS);
		{
			int32_t e2 = 2 * err;

			if (e2 >= dy) {
				err += dy;
				x0 += sx;
			}
			if (e2 <= dx) {
				err += dx;
				y0 += sy;
			}
		}
	}
}

/* Where sector i of a scan taken at pose p lands, in centimetres. */
static inline void sector_point(const struct s2_pose *p, int sector,
				int sectors, int32_t range_cm,
				int32_t *x_cm, int32_t *y_cm)
{
	int32_t a = p->a + (int32_t)((int64_t)sector * S2_TURN / sectors);

	*x_cm = p->x_cm + (range_cm * s2_cos(a) >> 15);
	*y_cm = p->y_cm + (range_cm * s2_sin(a) >> 15);
}

void s2_map_update(struct s2_map *m, const struct s2_pose *p,
		   const uint16_t *ranges_cm, int sectors, int32_t max_range_cm)
{
	int32_t sx = cx_of(m, p->x_cm), sy = cy_of(m, p->y_cm);
	int i;

	for (i = 0; i < sectors; i++) {
		uint16_t r = ranges_cm[i];
		int32_t px, py, ex, ey;

		if (r == 0 || r == 0xFFFF)
			continue;
		if (max_range_cm && r > max_range_cm)
			continue;
		sector_point(p, i, sectors, r, &px, &py);
		ex = cx_of(m, px);
		ey = cy_of(m, py);
		ray_free(m, sx, sy, ex, ey);
		bump(m, ex, ey, S2_HIT);
	}
	m->dirty = true;
}

void s2_map_build_pyramid(struct s2_map *m)
{
	int lvl;

	for (lvl = 1; lvl < S2_LEVELS; lvl++) {
		int32_t x, y;

		for (y = 0; y < m->h[lvl]; y++) {
			for (x = 0; x < m->w[lvl]; x++) {
				int32_t sx = x * 2, sy = y * 2, best = 0;
				int dy, dx;

				/* Max, not mean. A coarse cell has to mean
				 * "something is somewhere in here", otherwise
				 * a coarse score is not an upper bound on the
				 * fine one and the coarse-to-fine search can
				 * discard the right answer. */
				for (dy = 0; dy < 2; dy++) {
					for (dx = 0; dx < 2; dx++) {
						int32_t px = sx + dx;
						int32_t py = sy + dy;
						int32_t v;

						if (!inside(m, lvl - 1, px, py))
							continue;
						v = m->cell[lvl - 1]
							[py * m->w[lvl - 1] + px];
						if (v > best)
							best = v;
					}
				}
				m->cell[lvl][y * m->w[lvl] + x] = (uint8_t)best;
			}
		}
	}
	m->dirty = false;
}

/* ---------------------------------------------------------------- match */

/*
 * Precomputed scan points for one candidate heading, in centimetres relative
 * to the sensor. Rotating once per heading and then only adding a translation
 * per candidate is what keeps this affordable: the inner loop over translations
 * costs an add and a table lookup per point, with no multiplies at all.
 */
struct rotated {
	int32_t *dx;		/* centimetres from the sensor */
	int32_t *dy;
	int32_t *qx;		/* the same points as cell indices */
	int32_t *qy;
	int n;
};

/* Floor division. C truncates towards zero, which puts the half-cell either
 * side of the map origin into the same cell and makes the identity below false
 * for negative coordinates. */
static inline int32_t fdiv(int32_t a, int32_t b)
{
	return a >= 0 ? a / b : -(((-a) + b - 1) / b);
}

/*
 * Score a candidate whose cell coordinates are already known.
 *
 * The division that used to be here - two per point per candidate - is gone,
 * and that is the whole optimisation. The search steps translation by exactly
 * one cell of the current level, and floor((base + k*res) / res) is
 * floor(base/res) + k exactly, so the quotient can be computed once per
 * rotation and the inner loop reduced to an add, a bounds test and a lookup.
 * Integer division is microcoded on a 1004Kc; multiplying it by 360 points and
 * a few hundred candidates was most of the matcher.
 */
static int32_t score_cells(const struct s2_map *m, int lvl,
			   const int32_t *qx, const int32_t *qy, int n,
			   int32_t kx, int32_t ky)
{
	int32_t w = m->w[lvl], h = m->h[lvl];
	const uint8_t *cell = m->cell[lvl];
	int32_t sum = 0;
	int i;

	for (i = 0; i < n; i++) {
		int32_t cx = qx[i] + kx;
		int32_t cy = qy[i] + ky;
		int32_t v;

		if (cx < 0 || cy < 0 || cx >= w || cy >= h)
			continue;
		/*
		 * Only the occupied part counts. Scoring the raw cell gave
		 * unknown ground - 128 of a possible 255 - half the credit of a
		 * real wall, so poses that pushed the scan into unexplored
		 * space scored nearly as well as correct ones and the peak was
		 * blunt. Free and unknown both scoring zero makes the peak as
		 * sharp as the map is confident.
		 */
		v = cell[cy * w + cx];
		if (v > S2_UNKNOWN)
			sum += v - S2_UNKNOWN;
	}
	return sum;
}

static void rotate_scan(struct rotated *r, const uint16_t *ranges_cm,
			int sectors, int32_t max_range_cm, int32_t a)
{
	int i;

	r->n = 0;
	for (i = 0; i < sectors; i++) {
		uint16_t v = ranges_cm[i];
		int32_t ang;

		if (v == 0 || v == 0xFFFF)
			continue;
		if (max_range_cm && v > max_range_cm)
			continue;
		ang = a + (int32_t)((int64_t)i * S2_TURN / sectors);
		r->dx[r->n] = (int32_t)v * s2_cos(ang) >> 15;
		r->dy[r->n] = (int32_t)v * s2_sin(ang) >> 15;
		r->n++;
	}
}

bool s2_match(struct s2_map *m, const struct s2_pose *seed,
	      const uint16_t *ranges_cm, int sectors, int32_t max_range_cm,
	      int32_t win_xy_cm, int32_t win_a, struct s2_match_result *out)
{
	struct rotated rot;
	struct s2_pose best = *seed;
	int32_t best_score = -1;
	uint32_t tried = 0;
	int lvl;
	bool ok = false;

	if (m->dirty)
		s2_map_build_pyramid(m);

	rot.dx = malloc(sizeof(int32_t) * (size_t)sectors);
	rot.dy = malloc(sizeof(int32_t) * (size_t)sectors);
	rot.qx = malloc(sizeof(int32_t) * (size_t)sectors);
	rot.qy = malloc(sizeof(int32_t) * (size_t)sectors);
	if (!rot.dx || !rot.dy || !rot.qx || !rot.qy) {
		free(rot.dx); free(rot.dy); free(rot.qx); free(rot.qy);
		return false;
	}

	/*
	 * Coarse to fine. The first pass covers the whole window on the
	 * coarsest level, where a step is res << (LEVELS-1); each finer level
	 * only has to look one coarse step either side of what the level above
	 * chose. That is what turns an O(window^3) search into something a
	 * router could run.
	 */
	{
	int32_t prev_a_step = win_a / 8 > 0 ? win_a / 8 : 1;

	for (lvl = S2_LEVELS - 1; lvl >= 0; lvl--) {
		int32_t step = m->res_cm << lvl;
		int32_t span, a_span, a_step;
		struct s2_pose centre = best;
		int32_t da;

		if (lvl == S2_LEVELS - 1) {
			/* Coarsest pass covers the whole window. */
			span = win_xy_cm;
			a_step = prev_a_step;
			a_span = win_a;
		} else {
			/* Each finer pass only revisits one coarser cell
			 * either side of what the pass above chose, and halves
			 * the angular step. */
			span = m->res_cm << (lvl + 1);
			a_span = prev_a_step;
			a_step = prev_a_step / 2 > 0 ? prev_a_step / 2 : 1;
		}
		prev_a_step = a_step;

		best_score = -1;
		for (da = -a_span; da <= a_span; da += a_step) {
			int32_t kx, ky, cells = span / step;
			int i;

			rotate_scan(&rot, ranges_cm, sectors, max_range_cm,
				    centre.a + da);
			if (!rot.n)
				continue;
			ok = true;

			/* Cell coordinates of every scan point at the search
			 * centre, computed once. The translation search then
			 * moves in whole cells and needs no division at all. */
			for (i = 0; i < rot.n; i++) {
				rot.qx[i] = fdiv(centre.x_cm + rot.dx[i] -
						 m->origin_x_cm, step);
				rot.qy[i] = fdiv(centre.y_cm + rot.dy[i] -
						 m->origin_y_cm, step);
			}

			for (ky = -cells; ky <= cells; ky++) {
				for (kx = -cells; kx <= cells; kx++) {
					int32_t s = score_cells(m, lvl, rot.qx,
								rot.qy, rot.n,
								kx, ky);
					int32_t dx = kx * step, dy = ky * step;

					tried++;
					if (s > best_score) {
						best_score = s;
						best.x_cm = centre.x_cm + dx;
						best.y_cm = centre.y_cm + dy;
						best.a = (centre.a + da)
							 & S2_ANG_MASK;
					}
				}
			}
		}
		if (!ok)
			break;
	}
	}

	if (ok && out) {
		out->pose = best;
		out->score = best_score;
		out->max_score = rot.n * (S2_OCC_MAX - S2_UNKNOWN);
		out->candidates = tried;
		out->at_edge =
			(best.x_cm - seed->x_cm >= win_xy_cm) ||
			(seed->x_cm - best.x_cm >= win_xy_cm) ||
			(best.y_cm - seed->y_cm >= win_xy_cm) ||
			(seed->y_cm - best.y_cm >= win_xy_cm);
	}

	free(rot.dx);
	free(rot.dy);
	free(rot.qx);
	free(rot.qy);
	return ok;
}

/* ---------------------------------------------------------------- output */

bool s2_map_write_pgm(const struct s2_map *m, const char *path)
{
	FILE *f = fopen(path, "wb");
	int32_t y;

	if (!f)
		return false;
	fprintf(f, "P5\n%d %d\n255\n", (int)m->w[0], (int)m->h[0]);
	/* Flip vertically so +y is up, which is what a person expects. */
	for (y = m->h[0] - 1; y >= 0; y--) {
		int32_t x;

		for (x = 0; x < m->w[0]; x++) {
			/* occupied dark, free light, unknown mid grey */
			uint8_t v = m->cell[0][y * m->w[0] + x];

			fputc(255 - v, f);
		}
	}
	fclose(f);
	return true;
}
