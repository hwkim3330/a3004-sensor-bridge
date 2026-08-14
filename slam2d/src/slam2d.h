/*
 * slam2d - 2D occupancy mapping and scan matching with no floating point.
 *
 * The input is what ouster-edge already publishes: one minimum range per
 * azimuth sector, in centimetres, as uint16. That is already integer data, and
 * keeping it that way the whole way through buys three things:
 *
 *  - it runs on the router. MT7621 has no FPU and OpenWrt builds mipsel with
 *    CONFIG_SOFT_FLOAT, so every float operation is a libgcc call. Scan matching
 *    is otherwise almost entirely floating point, which is what ruled the router
 *    out in doc/COMPUTE.md.
 *  - it runs identically everywhere else. The same core compiles for the tablet
 *    through the NDK and for a PC, and gives bit-identical results, so where it
 *    runs becomes a measurement rather than an argument.
 *  - it is reproducible. No rounding differences between hosts, so a trajectory
 *    recorded on one machine replays exactly on another.
 *
 * The algorithm choice follows from the constraint rather than fighting it.
 * ICP needs an SVD per iteration, which is unpleasant in fixed point.
 * Correlative scan matching - score candidate poses by summing what the scan
 * hits in the occupancy grid, and keep the best - is a sum of table lookups,
 * which is exactly what an integer machine is good at. It is also what
 * Cartographer's real-time correlative matcher does, so this is a small version
 * of a known-good design rather than an invention.
 *
 * Units, fixed everywhere:
 *   distance   centimetres, int32
 *   angle      1/4096 of a revolution, int32, wrapping
 *   trig       Q15, int16, so sin(a) is s2_sin(a) and x*s2_sin(a) >> 15
 */
#ifndef SLAM2D_H
#define SLAM2D_H

#include <stdint.h>
#include <stdbool.h>

/* A full revolution. A power of two so wrapping is a mask, not a modulo:
 * division is expensive on this class of part and this happens per point per
 * candidate pose. 4096 gives 0.088 degrees, finer than any pose this will
 * resolve. */
#define S2_TURN		4096
#define S2_ANG_MASK	(S2_TURN - 1)

/* Log-odds occupancy, held in a uint8 so the map is one byte per cell.
 * 128 is unknown, 0 is certainly free, 255 is certainly occupied. */
#define S2_UNKNOWN	128
#define S2_HIT		12	/* added on a return */
#define S2_MISS		4	/* subtracted along the ray to it */
#define S2_OCC_MIN	0
#define S2_OCC_MAX	255

/* Pyramid levels for the coarse-to-fine search. Level 0 is the map itself;
 * each further level halves the resolution and takes the max of its four
 * children, so a coarse cell says "something is near here" rather than
 * "something is exactly here" - which is what makes a coarse match a valid
 * bound on the fine one. */
#define S2_LEVELS	3

struct s2_map {
	uint8_t *cell[S2_LEVELS];	/* cell[0] is the real map */
	int32_t w[S2_LEVELS];		/* width in cells, per level */
	int32_t h[S2_LEVELS];
	int32_t res_cm;			/* level 0 cell size, centimetres */
	int32_t origin_x_cm;		/* world coordinate of cell (0,0) */
	int32_t origin_y_cm;
	bool dirty;			/* pyramid needs rebuilding */
};

struct s2_pose {
	int32_t x_cm;
	int32_t y_cm;
	int32_t a;			/* 1/4096 turn */
};

/* What the matcher was asked to search and what it found. Kept so a caller can
 * tell "converged in the middle of the window" from "pinned to its edge", which
 * is the difference between a good match and one that wanted to go further. */
struct s2_match_result {
	struct s2_pose pose;
	int32_t score;			/* summed occupancy, higher is better */
	int32_t max_score;		/* what a perfect match would have scored */
	bool at_edge;			/* solution sits on the search boundary */
	uint32_t candidates;		/* poses actually scored */
};

/* Q15 trig by table. No libm, no floats, no per-call work. */
int16_t s2_sin(int32_t a);
int16_t s2_cos(int32_t a);

/* Angle of the vector (x, y), in the same 1/4096 units. CORDIC in vectoring
 * mode, which is the same rotation loop run to drive y to zero instead of the
 * angle - so it costs what a sine costs and needs no atan2 from libm. */
int32_t s2_atan2(int32_t y, int32_t x);

/* Shortest signed difference a - b, in (-S2_TURN/2, S2_TURN/2]. */
int32_t s2_angle_diff(int32_t a, int32_t b);

bool s2_map_init(struct s2_map *m, int32_t width_cm, int32_t height_cm,
		 int32_t res_cm);
void s2_map_free(struct s2_map *m);

/*
 * Fold a scan into the map at a known pose.
 *
 * `ranges_cm` holds `sectors` entries; 0 and 0xFFFF both mean "no return", the
 * latter being what the ring format uses. Sector i is centred on azimuth
 * i/sectors of a revolution, matching doc/RING-FORMAT.md.
 */
void s2_map_update(struct s2_map *m, const struct s2_pose *p,
		   const uint16_t *ranges_cm, int sectors, int32_t max_range_cm);

/*
 * Find the pose that best explains the scan, searching around `seed`.
 *
 * The window is in centimetres and angle units; the search is coarse-to-fine
 * over the pyramid, so the cost is closer to the coarse window than the fine
 * one. Returns false only if the scan had too few returns to say anything.
 */
bool s2_match(struct s2_map *m, const struct s2_pose *seed,
	      const uint16_t *ranges_cm, int sectors, int32_t max_range_cm,
	      int32_t win_xy_cm, int32_t win_a, struct s2_match_result *out);

/* Rebuild the coarse levels. Called automatically by s2_match when needed. */
void s2_map_build_pyramid(struct s2_map *m);

/* Write the map as a binary PGM, for looking at it. Debug only. */
bool s2_map_write_pgm(const struct s2_map *m, const char *path);

#endif /* SLAM2D_H */
