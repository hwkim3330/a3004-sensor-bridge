#define _GNU_SOURCE
// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * teleop - accept control intent from a browser and forward it with a deadman
 *
 * A tablet on the router's own AP can drive something, but only if the failure
 * modes are handled, and they are all the same failure: commands stop arriving
 * while the last one is still being acted on. WiFi has no latency bound, a
 * browser tab can be backgrounded, and a person can walk out of range.
 *
 * So this is built around three rules:
 *
 *   1. Nothing is emitted until explicitly armed, and one stale interval
 *      disarms. Arming is a deliberate act, not a side effect of connecting.
 *   2. When commands go stale the output is neutral and the armed flag drops -
 *      it does not simply stop sending, because a receiver cannot distinguish
 *      "no packet" from "packet lost".
 *   3. Every datagram carries a sequence number and the sender's monotonic
 *      clock, so the receiver can run its *own* deadman rather than trusting
 *      this one. Two independent deadmen, because the interesting failure is
 *      the link between them.
 *
 * This daemon holds no control loop. It converts touch input into intent and
 * hands it to whatever does - see doc/pc-side/teleop_receiver.py and CAN.md.
 */

#include <arpa/inet.h>
#include <errno.h>
#include <fcntl.h>
#include <getopt.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <poll.h>
#include <signal.h>
#include <stdarg.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <syslog.h>
#include <time.h>
#include <unistd.h>

#define MAX_CLIENTS	16
#define IDLE_MS		5000
#define AXES		4
#define PKT_LEN		32
#define AXIS_SCALE	10000		/* fixed point: -1.0 .. 1.0 */
#define CMD_MAGIC	"TCMD"
#define CMD_LEN		24

static struct {
	int port;			/* HTTP port for the browser */
	struct sockaddr_in peer;	/* where intent is forwarded */
	bool have_peer;
	int cmd_port;			/* UDP command input, 0 = off */
	int rate_hz;			/* forward cadence */
	int timeout_ms;			/* deadman */
	char *status_path;
	bool foreground;

	/* live state */
	int16_t axis[AXES];
	uint16_t buttons;
	bool armed;
	uint32_t seq_in, seq_out;
	uint64_t last_cmd_ms;
	uint64_t commands, rejected_seq, malformed, deadman_trips, sent;
	uint64_t refused, reaped;
	uint64_t udp_commands, udp_malformed;
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
	return (uint64_t)ts.tv_sec * 1000u + (uint64_t)(ts.tv_nsec / 1000000);
}

struct client {
	int fd;
	char buf[1024];
	size_t len;
	uint64_t last_ms;
};

static struct client cl[MAX_CLIENTS];
static int ncl;

static void neutral(void)
{
	int i;

	for (i = 0; i < AXES; i++)
		g.axis[i] = 0;
	g.buttons = 0;
}

static void disarm(const char *why)
{
	if (g.armed) {
		g.armed = false;
		logmsg(LOG_WARNING, "disarmed: %s", why);
	}
	neutral();
}

/* Forwarded unconditionally at a fixed cadence, armed or not, so the receiver
 * always knows the difference between "neutral" and "gone". */
static void forward(int sock)
{
	uint8_t p[PKT_LEN];
	uint64_t t = now_ms();
	int i;

	if (!g.have_peer || sock < 0)
		return;

	memset(p, 0, sizeof(p));
	memcpy(p, "TELE", 4);
	p[4] = 1;
	p[5] = g.armed ? 1 : 0;
	g.seq_out++;
	for (i = 0; i < 4; i++)
		p[8 + i] = (uint8_t)((g.seq_out >> (8 * i)) & 0xff);
	for (i = 0; i < 8; i++)
		p[12 + i] = (uint8_t)((t >> (8 * i)) & 0xff);
	for (i = 0; i < AXES; i++) {
		uint16_t v = (uint16_t)g.axis[i];

		p[20 + i * 2] = (uint8_t)(v & 0xff);
		p[21 + i * 2] = (uint8_t)(v >> 8);
	}
	p[28] = (uint8_t)(g.buttons & 0xff);
	p[29] = (uint8_t)(g.buttons >> 8);

	if (sendto(sock, p, sizeof(p), 0, (struct sockaddr *)&g.peer,
		   sizeof(g.peer)) == (ssize_t)sizeof(p))
		g.sent++;
}

static void status_write(void)
{
	char tmp[256];
	FILE *f;
	int i;

	if (!g.status_path)
		return;

	snprintf(tmp, sizeof(tmp), "%s.tmp", g.status_path);
	f = fopen(tmp, "w");
	if (!f)
		return;

	fprintf(f, "{\n\t\"armed\": %s,\n", g.armed ? "true" : "false");
	fprintf(f, "\t\"age_ms\": %llu,\n",
		(unsigned long long)(g.last_cmd_ms ? now_ms() - g.last_cmd_ms : 0));
	fprintf(f, "\t\"timeout_ms\": %d,\n", g.timeout_ms);
	fprintf(f, "\t\"rate_hz\": %d,\n", g.rate_hz);
	fprintf(f, "\t\"forwarding\": %s,\n", g.have_peer ? "true" : "false");
	fprintf(f, "\t\"commands\": %llu,\n", (unsigned long long)g.commands);
	fprintf(f, "\t\"rejected_seq\": %llu,\n",
		(unsigned long long)g.rejected_seq);
	fprintf(f, "\t\"malformed\": %llu,\n", (unsigned long long)g.malformed);
	fprintf(f, "\t\"deadman_trips\": %llu,\n",
		(unsigned long long)g.deadman_trips);
	fprintf(f, "\t\"sent\": %llu,\n", (unsigned long long)g.sent);
	fprintf(f, "\t\"clients\": %d,\n", ncl);
	fprintf(f, "\t\"refused\": %llu,\n", (unsigned long long)g.refused);
	fprintf(f, "\t\"reaped\": %llu,\n", (unsigned long long)g.reaped);
	fprintf(f, "\t\"udp_commands\": %llu,\n",
		(unsigned long long)g.udp_commands);
	fprintf(f, "\t\"udp_malformed\": %llu,\n",
		(unsigned long long)g.udp_malformed);
	fprintf(f, "\t\"axes\": [");
	for (i = 0; i < AXES; i++)
		fprintf(f, "%s%.4f", i ? ", " : "",
			(double)g.axis[i] / AXIS_SCALE);
	fprintf(f, "],\n\t\"buttons\": %u\n}\n", g.buttons);
	fclose(f);

	if (rename(tmp, g.status_path) < 0)
		unlink(tmp);
}

static int qs_int(const char *qs, const char *key, int fallback, bool *found)
{
	char pat[32];
	const char *p;

	snprintf(pat, sizeof(pat), "%s=", key);
	p = strstr(qs, pat);
	if (found)
		*found = false;
	if (!p)
		return fallback;
	/* only accept a match at the start or right after a separator, so "a="
	 * does not match inside "ba=" */
	if (p != qs && p[-1] != '?' && p[-1] != '&')
		return fallback;
	p += strlen(pat);
	if (found)
		*found = true;
	return atoi(p);
}

static int clamp_axis(int v)
{
	if (v > AXIS_SCALE)
		return AXIS_SCALE;
	if (v < -AXIS_SCALE)
		return -AXIS_SCALE;
	return v;
}

/*
 * GET /cmd?s=<seq>&arm=<0|1>&a0=..&a1=..&a2=..&a3=..&b=<buttons>
 * Axes are integers in units of 1/10000, which keeps the wire integer-only and
 * avoids locale-dependent float parsing on both sides.
 */
static bool seq_accept(uint32_t seq)
{
	/* Reject replays and reordering. A restarted client resets its sequence,
	 * which would otherwise lock it out for good, so a large backwards jump
	 * is treated as a restart rather than a replay. */
	if (g.commands && seq <= g.seq_in) {
		if (g.seq_in - seq > 1000) {
			logmsg(LOG_NOTICE, "client restarted (seq %u -> %u)",
			       g.seq_in, seq);
		} else {
			g.rejected_seq++;
			return false;
		}
	}
	g.seq_in = seq;
	g.commands++;
	g.last_cmd_ms = now_ms();
	return true;
}

/* One place where a decoded command becomes state, so the HTTP and UDP paths
 * cannot drift apart on arming or clamping. */
static void apply_cmd(bool arm, const int *axes, uint16_t buttons)
{
	int i;

	if (!arm) {
		disarm("client requested");
		return;
	}
	if (!g.armed) {
		g.armed = true;
		logmsg(LOG_NOTICE, "armed");
	}
	for (i = 0; i < AXES; i++)
		g.axis[i] = (int16_t)clamp_axis(axes[i]);
	g.buttons = buttons;
}

static void handle_cmd(const char *qs)
{
	int seq, arm, i, axes[AXES];
	bool have_seq;

	seq = qs_int(qs, "s", -1, &have_seq);
	/*
	 * A negative sequence has to be refused, not just cast. "s=-5" becomes
	 * 4294967291 as a uint32, and every ordinary sequence after it then
	 * looks like a backwards jump of more than 1000 - i.e. a client
	 * restart - so seq_accept() would wave through everything, including
	 * replays, for the rest of the process's life. The UDP path cannot hit
	 * this because it reads four raw bytes.
	 */
	if (!have_seq || seq < 0) {
		g.malformed++;
		return;
	}
	if (!seq_accept((uint32_t)seq))
		return;

	arm = qs_int(qs, "arm", 0, NULL);
	for (i = 0; i < AXES; i++) {
		char key[4];

		snprintf(key, sizeof(key), "a%d", i);
		axes[i] = qs_int(qs, key, 0, NULL);
	}
	apply_cmd(arm != 0, axes, (uint16_t)qs_int(qs, "b", 0, NULL));
}

/*
 * The native app path. A browser has to use HTTP; an app does not, and UDP
 * removes the whole class of problems that came with a request per command -
 * connection limits, keep-alive parsing, and Chrome's cap on in-flight fetches.
 *
 *   0  4  magic "TCMD"
 *   4  1  version 1
 *   5  1  flags: bit0 = arm
 *   6  2  reserved
 *   8  4  uint32 sequence
 *  12  8  4 x int16 axes, units of 1/10000
 *  20  2  uint16 buttons
 *  22  2  reserved
 */
static void handle_udp_cmd(const uint8_t *p, size_t len)
{
	int axes[AXES], i;
	uint32_t seq;

	if (len < CMD_LEN || memcmp(p, CMD_MAGIC, 4) || p[4] != 1) {
		g.udp_malformed++;
		return;
	}

	seq = (uint32_t)p[8] | ((uint32_t)p[9] << 8) |
	      ((uint32_t)p[10] << 16) | ((uint32_t)p[11] << 24);
	if (!seq_accept(seq))
		return;

	for (i = 0; i < AXES; i++)
		axes[i] = (int16_t)((uint16_t)p[12 + i * 2] |
				    ((uint16_t)p[13 + i * 2] << 8));

	g.udp_commands++;
	apply_cmd((p[5] & 1) != 0, axes,
		  (uint16_t)((uint16_t)p[20] | ((uint16_t)p[21] << 8)));
}


static void client_drop(int i)
{
	close(cl[i].fd);
	cl[i] = cl[--ncl];
}

static void client_accept(int lfd)
{
	int fd = accept(lfd, NULL, NULL);
	int on = 1;

	if (fd < 0)
		return;
	if (ncl >= MAX_CLIENTS) {
		/* Counted, not silent: a full table means commands are being
		 * dropped, and that has to be visible in the status file. */
		if (!(g.refused++ % 50))
			logmsg(LOG_WARNING,
			       "connection table full (%d), refusing", ncl);
		close(fd);
		return;
	}
	fcntl(fd, F_SETFL, fcntl(fd, F_GETFL, 0) | O_NONBLOCK);
	setsockopt(fd, IPPROTO_TCP, TCP_NODELAY, &on, sizeof(on));
	cl[ncl].fd = fd;
	cl[ncl].len = 0;
	cl[ncl].last_ms = now_ms();
	ncl++;
}

/* Keep-alive matters here: at 20 Hz a new connection per command would be 20
 * handshakes a second and 20 chances to add latency. */
static const char RESP_OK[] =
	"HTTP/1.1 204 No Content\r\n"
	"Access-Control-Allow-Origin: *\r\n"
	"Cache-Control: no-store\r\n"
	"Connection: keep-alive\r\n"
	"\r\n";
static const char RESP_BAD[] =
	"HTTP/1.1 400 Bad Request\r\n"
	"Access-Control-Allow-Origin: *\r\n"
	"Content-Length: 0\r\n"
	"Connection: keep-alive\r\n"
	"\r\n";

static void client_read(int i)
{
	ssize_t n;
	char *eol;

	n = read(cl[i].fd, cl[i].buf + cl[i].len, sizeof(cl[i].buf) - cl[i].len - 1);
	if (n <= 0) {
		if (n == 0 || (errno != EAGAIN && errno != EWOULDBLOCK &&
			       errno != EINTR))
			client_drop(i);
		return;
	}
	cl[i].len += (size_t)n;
	cl[i].buf[cl[i].len] = '\0';
	cl[i].last_ms = now_ms();

	/* One request line per command; headers are ignored entirely. */
	for (;;) {
		char *e4 = strstr(cl[i].buf, "\r\n\r\n");
		char *e2 = strstr(cl[i].buf, "\n\n");
		size_t consumed, termlen;
		char line[512];
		char *sp;
		const char *resp = RESP_BAD;
		size_t rlen = sizeof(RESP_BAD) - 1;

		/* Consume exactly the terminator that is actually there. Getting
		 * this wrong leaves bytes in the buffer, and every subsequent
		 * request on a keep-alive connection is then misparsed. */
		if (e4 && (!e2 || e4 <= e2)) {
			eol = e4;
			termlen = 4;
		} else if (e2) {
			eol = e2;
			termlen = 2;
		} else {
			break;
		}
		consumed = (size_t)(eol - cl[i].buf) + termlen;

		snprintf(line, sizeof(line), "%s", cl[i].buf);
		sp = strchr(line, '\n');
		if (sp)
			*sp = '\0';

		if (!strncmp(line, "GET /cmd?", 9)) {
			char *end = strchr(line + 9, ' ');

			if (end)
				*end = '\0';
			handle_cmd(line + 9);
			resp = RESP_OK;
			rlen = sizeof(RESP_OK) - 1;
		} else {
			g.malformed++;
		}

		if (write(cl[i].fd, resp, rlen) < 0 && errno != EAGAIN) {
			client_drop(i);
			return;
		}

		memmove(cl[i].buf, cl[i].buf + consumed, cl[i].len - consumed);
		cl[i].len -= consumed;
		cl[i].buf[cl[i].len] = '\0';
	}

	if (cl[i].len > sizeof(cl[i].buf) - 8) {
		/* a request line this long is not one of ours */
		g.malformed++;
		client_drop(i);
	}
}

static void usage(const char *a0)
{
	fprintf(stderr,
"Usage: %s [options]\n"
"  -p, --port PORT        HTTP port the browser posts to (default 8083)\n"
"  -r, --remote HOST:PORT forward intent here as TELE (default 7722,\n"
"                         which is agx-cmd)\n"
"  -H, --rate HZ          forward cadence (default 20)\n"
"  -c, --cmd-port PORT    also accept commands as UDP, for a native app\n"
"  -t, --timeout MS       deadman: neutral and disarm after this long without\n"
"                         a command (default 300)\n"
"  -S, --status PATH      JSON status (default /var/run/teleop.json)\n"
"  -f, --foreground       log to stderr instead of syslog\n", a0);
}

static bool parse_hostport(const char *s, struct sockaddr_in *out, int defport)
{
	char buf[128];
	char *colon;

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

int main(int argc, char **argv)
{
	static const struct option opts[] = {
		{ "port",       required_argument, NULL, 'p' },
		{ "remote",     required_argument, NULL, 'r' },
		{ "rate",       required_argument, NULL, 'H' },
		{ "cmd-port",   required_argument, NULL, 'c' },
		{ "timeout",    required_argument, NULL, 't' },
		{ "status",     required_argument, NULL, 'S' },
		{ "foreground", no_argument,       NULL, 'f' },
		{ "help",       no_argument,       NULL, 'h' },
		{ NULL, 0, NULL, 0 }
	};
	struct pollfd pfd[2 + MAX_CLIENTS];
	struct sockaddr_in a;
	struct sigaction sa;
	int lfd, usock = -1, cfd = -1, on = 1, opt, i;
	uint64_t next_tx = 0, next_status = 0;

	g.port = 8083;
	g.rate_hz = 20;
	g.cmd_port = 7721;
	g.timeout_ms = 300;
	g.status_path = (char *)"/var/run/teleop.json";

	while ((opt = getopt_long(argc, argv, "p:r:H:c:t:S:fh", opts, NULL)) != -1) {
		switch (opt) {
		case 'p': g.port = atoi(optarg); break;
		case 'r':
			if (!parse_hostport(optarg, &g.peer, 7722)) {
				fprintf(stderr, "bad --remote '%s'\n", optarg);
				return 1;
			}
			g.have_peer = true;
			break;
		case 'H': g.rate_hz = atoi(optarg); break;
		case 'c': g.cmd_port = atoi(optarg); break;
		case 't': g.timeout_ms = atoi(optarg); break;
		case 'S': g.status_path = optarg; break;
		case 'f': g.foreground = true; break;
		case 'h': usage(argv[0]); return 0;
		default: usage(argv[0]); return 1;
		}
	}

	if (g.rate_hz < 1 || g.rate_hz > 200) {
		fprintf(stderr, "--rate must be 1..200\n");
		return 1;
	}
	/* A deadman shorter than two forward intervals would trip on ordinary
	 * jitter, which trains people to ignore it. */
	if (g.timeout_ms < 2 * 1000 / g.rate_hz)
		g.timeout_ms = 2 * 1000 / g.rate_hz;

	if (!g.foreground)
		openlog("teleop", LOG_PID, LOG_DAEMON);

	sa.sa_handler = on_signal;
	sigemptyset(&sa.sa_mask);
	sa.sa_flags = 0;
	sigaction(SIGINT, &sa, NULL);
	sigaction(SIGTERM, &sa, NULL);
	signal(SIGPIPE, SIG_IGN);

	lfd = socket(AF_INET, SOCK_STREAM, 0);
	if (lfd < 0) {
		logmsg(LOG_ERR, "socket: %s", strerror(errno));
		return 1;
	}
	setsockopt(lfd, SOL_SOCKET, SO_REUSEADDR, &on, sizeof(on));
	fcntl(lfd, F_SETFL, fcntl(lfd, F_GETFL, 0) | O_NONBLOCK);

	memset(&a, 0, sizeof(a));
	a.sin_family = AF_INET;
	a.sin_addr.s_addr = htonl(INADDR_ANY);
	a.sin_port = htons((uint16_t)g.port);
	if (bind(lfd, (struct sockaddr *)&a, sizeof(a)) < 0 ||
	    listen(lfd, 4) < 0) {
		logmsg(LOG_ERR, "bind :%d: %s", g.port, strerror(errno));
		return 1;
	}

	if (g.have_peer) {
		usock = socket(AF_INET, SOCK_DGRAM, 0);
		if (usock >= 0)
			fcntl(usock, F_SETFL,
			      fcntl(usock, F_GETFL, 0) | O_NONBLOCK);
	}

	if (g.cmd_port > 0) {
		struct sockaddr_in c;

		cfd = socket(AF_INET, SOCK_DGRAM, 0);
		if (cfd >= 0) {
			memset(&c, 0, sizeof(c));
			c.sin_family = AF_INET;
			c.sin_addr.s_addr = htonl(INADDR_ANY);
			c.sin_port = htons((uint16_t)g.cmd_port);
			setsockopt(cfd, SOL_SOCKET, SO_REUSEADDR, &on, sizeof(on));
			if (bind(cfd, (struct sockaddr *)&c, sizeof(c)) < 0) {
				logmsg(LOG_WARNING, "cannot bind command port %d: %s",
				       g.cmd_port, strerror(errno));
				close(cfd);
				cfd = -1;
			} else {
				fcntl(cfd, F_SETFL,
				      fcntl(cfd, F_GETFL, 0) | O_NONBLOCK);
				logmsg(LOG_NOTICE, "UDP commands on :%d",
				       g.cmd_port);
			}
		}
	}

	logmsg(LOG_NOTICE,
	       "teleop on :%d, %d Hz, deadman %d ms, forwarding %s",
	       g.port, g.rate_hz, g.timeout_ms,
	       g.have_peer ? "on" : "off");

	while (!stop_requested) {
		int np = 0, wait_ms;
		uint64_t t;

		pfd[np].fd = lfd;
		pfd[np].events = POLLIN;
		pfd[np++].revents = 0;
		if (cfd >= 0) {
			pfd[np].fd = cfd;
			pfd[np].events = POLLIN;
			pfd[np++].revents = 0;
		}
		for (i = 0; i < ncl; i++) {
			pfd[np].fd = cl[i].fd;
			pfd[np].events = POLLIN;
			pfd[np++].revents = 0;
		}

		t = now_ms();
		wait_ms = next_tx > t ? (int)(next_tx - t) : 0;
		if (wait_ms > 20)
			wait_ms = 20;

		if (poll(pfd, (nfds_t)np, wait_ms) < 0 && errno != EINTR)
			logmsg(LOG_WARNING, "poll: %s", strerror(errno));

		if (pfd[0].revents & POLLIN)
			client_accept(lfd);

		if (cfd >= 0 && (pfd[1].revents & POLLIN)) {
			uint8_t buf[64];
			ssize_t n;

			while ((n = recv(cfd, buf, sizeof(buf), 0)) > 0)
				handle_udp_cmd(buf, (size_t)n);
		}

		for (i = ncl - 1; i >= 0; i--) {
			int slot = (cfd >= 0 ? 2 : 1) + i;

			if (slot >= np)
				continue;
			if (pfd[slot].revents & (POLLERR | POLLHUP | POLLNVAL))
				client_drop(i);
			else if (pfd[slot].revents & POLLIN)
				client_read(i);
		}

		t = now_ms();
		for (i = ncl - 1; i >= 0; i--) {
			if (t - cl[i].last_ms > IDLE_MS) {
				g.reaped++;
				client_drop(i);
			}
		}

		if (g.armed && g.last_cmd_ms &&
		    t - g.last_cmd_ms > (uint64_t)g.timeout_ms) {
			g.deadman_trips++;
			disarm("command timeout");
		}

		if (t >= next_tx) {
			next_tx = t + (uint64_t)(1000 / g.rate_hz);
			forward(usock);
		}

		if (t >= next_status) {
			next_status = t + 100;
			status_write();
		}
	}

	disarm("shutting down");
	forward(usock);		/* one last neutral, disarmed frame */
	logmsg(LOG_NOTICE,
	       "stopping: %llu commands, %llu deadman trips, %llu sent",
	       (unsigned long long)g.commands,
	       (unsigned long long)g.deadman_trips,
	       (unsigned long long)g.sent);
	status_write();
	while (ncl)
		client_drop(0);
	if (usock >= 0)
		close(usock);
	if (cfd >= 0)
		close(cfd);
	close(lfd);
	return 0;
}
