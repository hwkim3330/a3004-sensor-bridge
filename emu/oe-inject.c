/*
 * oe-inject - send a fixed number of synthetic Ouster datagrams at a port.
 *
 * Built for the target and run inside the emulator, on purpose. run-emu.py's
 * feed_lidar() injects from the host through QEMU's user-mode NAT, which is
 * fine for "does the ring path work" but useless for measuring cost: a 12.5 kB
 * datagram fragments, slirp copies it several times, and the host becomes the
 * bottleneck long before the guest does. Sending from inside the guest over
 * loopback removes all of that.
 *
 * The point of a fixed count rather than a duration is subtraction. Run the
 * daemon against N packets and against M packets, and (cpu(N) - cpu(M)) /
 * (N - M) is the per-packet cost with process startup, config parsing and
 * status writes removed. That is the only number worth comparing between two
 * builds.
 *
 *     oe-inject PORT COUNT [USEC_GAP]
 *     oe-inject PORT COUNT USEC_GAP --room
 *
 * With --room the ranges come from a simulated 12 x 8 m room seen by a robot
 * driving through it, rather than from a fixed wall. That turns the whole chain
 * into something testable on the target: oe-inject makes lidar packets,
 * ouster-edge reduces them to a ring, slam2d-daemon maps the ring, and the pose
 * slam2d reports can be compared against the pose oe-inject was simulating.
 * Every stage is the real binary on the real architecture.
 *
 * The room maths uses doubles. That is fine and does not weaken the claim about
 * slam2d: this program stands in for the physical world, and what crosses into
 * the code under test is the same integer packet a sensor would send.
 *
 * The layout matches what run-emu.py generates and what ouster-edge is told to
 * expect with -c 64 -C 16: a 32 byte packet header, 16 columns of a 12 byte
 * column header plus 64 pixels of 12 bytes, and a 32 byte footer.
 */
#include <arpa/inet.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <unistd.h>

#define CH		64
#define COLS		16
#define WIDTH		1024
#define PX		12
#define COL_HDR		12
#define PKT_HDR		32
#define FOOTER		32
#define PKT_SZ		(PKT_HDR + COLS * (COL_HDR + CH * PX) + FOOTER)

static void put32(unsigned char *p, unsigned int v)
{
	p[0] = v & 0xff; p[1] = (v >> 8) & 0xff;
	p[2] = (v >> 16) & 0xff; p[3] = (v >> 24) & 0xff;
}

static void put16(unsigned char *p, unsigned int v)
{
	p[0] = v & 0xff; p[1] = (v >> 8) & 0xff;
}

/* ------------------------------------------------------- simulated room */

/* The same 12 x 8 m room the slam2d tests use, with two stub walls so the
 * place is not symmetric and a wrong heading cannot score as well as the right
 * one. */
static const double room[][4] = {
	{ -6, -4,  6, -4 }, {  6, -4,  6,  4 },
	{  6,  4, -6,  4 }, { -6,  4, -6, -4 },
	{  1, -4,  1,  0 }, { -3,  4, -3,  1 },
};

static double room_ray(double px, double py, double ang)
{
	double dx = cos(ang), dy = sin(ang), best = 1e9;
	unsigned i;

	for (i = 0; i < sizeof(room) / sizeof(room[0]); i++) {
		double x0 = room[i][0], y0 = room[i][1];
		double ex = room[i][2] - x0, ey = room[i][3] - y0;
		double den = dx * ey - dy * ex;
		double t, u;

		if (fabs(den) < 1e-12)
			continue;
		t = ((x0 - px) * ey - (y0 - py) * ex) / den;
		u = ((x0 - px) * dy - (y0 - py) * dx) / den;
		if (t > 0.05 && u >= 0 && u <= 1 && t < best)
			best = t;
	}
	return best;
}

/* Where the robot is at revolution `frame`, and where it is looking. The same
 * path the slam2d tests use, so the numbers are comparable. */
static void room_pose(unsigned frame, double *x, double *y, double *th)
{
	double t = frame * 0.05;

	*x = 3.0 * sin(t);
	*y = 2.0 * (1 - cos(t));
	*th = 0.35 * sin(t * 1.3);
}

int main(int argc, char **argv)
{
	static unsigned char pkt[PKT_SZ];
	struct sockaddr_in to;
	unsigned long count, sent = 0;
	unsigned int gap = 0, mid = 0, frame = 100;
	int s, port, i, use_room = 0;

	if (argc < 3) {
		fprintf(stderr, "usage: %s PORT COUNT [USEC_GAP] [--room]\n",
			argv[0]);
		return 2;
	}
	port = atoi(argv[1]);
	count = strtoul(argv[2], NULL, 10);
	if (argc > 3 && argv[3][0] != '-')
		gap = (unsigned int)strtoul(argv[3], NULL, 10);
	for (i = 3; i < argc; i++)
		if (!strcmp(argv[i], "--room"))
			use_room = 1;

	s = socket(AF_INET, SOCK_DGRAM, 0);
	if (s < 0) {
		perror("socket");
		return 1;
	}
	memset(&to, 0, sizeof(to));
	to.sin_family = AF_INET;
	to.sin_port = htons((unsigned short)port);
	to.sin_addr.s_addr = htonl(INADDR_LOOPBACK);

	/* Connected UDP, so the destination is resolved once rather than per
	 * send. On a slow target that difference is measurable in the injector,
	 * and the injector must not be what limits the rate. */
	if (connect(s, (struct sockaddr *)&to, sizeof(to)) < 0) {
		perror("connect");
		return 1;
	}

	while (sent < count) {
		int c;

		memset(pkt, 0, sizeof(pkt));
		put16(pkt, 1);
		put16(pkt + 2, frame & 0xffff);

		for (c = 0; c < COLS; c++) {
			unsigned char *col = pkt + PKT_HDR
					     + c * (COL_HDR + CH * PX);
			unsigned int m = (mid + c) % WIDTH;
			int ch;

			put32(col, m * 1000);		/* timestamp, low word */
			put16(col + 8, m);		/* measurement id */
			put16(col + 10, 1);		/* status: valid */

			if (use_room) {
				double px, py, th, r;
				unsigned int mm;

				/* frame starts at 100, so the trajectory is
				 * zero-based rather than starting mid-path. */
				room_pose(frame - 100, &px, &py, &th);
				r = room_ray(px, py,
					     th + 2 * M_PI * m / WIDTH);
				mm = (r > 30.0) ? 0 : (unsigned int)(r * 1000);
				/* Every channel gets the same range, so the
				 * minimum ouster-edge takes over the band is
				 * the planar range. A real sensor on a vehicle
				 * would see the floor on the lower beams; that
				 * is a mounting and channel_band question, not
				 * one this harness should silently answer. */
				for (ch = 0; ch < CH; ch++)
					put32(col + COL_HDR + ch * PX, mm);
			} else {
				/* A wall at 4 m, and something inside 2 m in
				 * one arc, so the zone path is exercised
				 * rather than skipped. */
				for (ch = 0; ch < CH; ch++)
					put32(col + COL_HDR + ch * PX,
					      (m >= 100 && m < 130) ? 1800
								    : 4000);
			}
		}

		if (send(s, pkt, sizeof(pkt), 0) < 0) {
			perror("send");
			return 1;
		}

		sent++;
		mid = (mid + COLS) % WIDTH;
		if (mid == 0)
			frame++;
		if (gap)
			usleep(gap);
	}

	printf("sent %lu datagrams of %d bytes\n", sent, PKT_SZ);
	return 0;
}
