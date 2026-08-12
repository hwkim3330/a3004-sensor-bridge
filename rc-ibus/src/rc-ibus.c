#define _GNU_SOURCE
// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * rc-ibus - read a FlySky receiver's i-BUS output
 *
 * The router cannot receive AFHDS 2A over the air. That protocol is GFSK
 * frequency-hopping driven by an A7105; an 802.11 radio shares the band but
 * not the modulation, and monitor mode yields 802.11 frames rather than raw IQ.
 * So the RF is left to a real receiver (FS-iA6B and friends) and this reads its
 * i-BUS servo output, which is plain asynchronous serial.
 *
 * Frame format, 32 bytes at 115200 8N1, repeated about every 7.5 ms:
 *
 *   byte  0     0x20        length
 *   byte  1     0x40        command: servo data
 *   bytes 2..29 14 x uint16 little-endian channel values, microseconds
 *   bytes 30,31 uint16      checksum = 0xFFFF - sum(bytes 0..29)
 *
 * The checksum is verified on every frame; unverified RC input is worse than
 * none. A gap in valid frames is reported as a link loss, because that is the
 * condition anything acting on these channels has to react to.
 */

#include <errno.h>
#include <fcntl.h>
#include <getopt.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <poll.h>
#include <signal.h>
#include <stdarg.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <syslog.h>
#include <sys/socket.h>
#include <termios.h>
#include <time.h>
#include <unistd.h>

#define IBUS_LEN	32
#define IBUS_CHANNELS	14
#define IBUS_HDR0	0x20
#define IBUS_HDR1	0x40
#define RING		256

static struct {
	char *device;
	int baud;
	int port;			/* UDP forward, 0 = off */
	struct sockaddr_in peer;
	bool have_peer;
	char *status_path;
	int status_ms;
	int timeout_ms;
	bool foreground;

	uint16_t ch[IBUS_CHANNELS];
	bool link;
	uint64_t last_frame_ms;
	uint64_t frames, bad_crc, resyncs, bytes, link_losses;
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

static speed_t baud_const(int baud)
{
	switch (baud) {
	case 9600:	return B9600;
	case 19200:	return B19200;
	case 38400:	return B38400;
	case 57600:	return B57600;
	case 115200:	return B115200;
	default:	return 0;
	}
}

static int serial_open(const char *dev, int baud)
{
	struct termios t;
	speed_t sp = baud_const(baud);
	int fd;

	if (!sp) {
		logmsg(LOG_ERR, "unsupported baud %d", baud);
		return -1;
	}

	fd = open(dev, O_RDONLY | O_NOCTTY | O_NONBLOCK);
	if (fd < 0)
		return -1;

	if (tcgetattr(fd, &t) < 0) {
		close(fd);
		return -1;
	}

	cfmakeraw(&t);
	cfsetispeed(&t, sp);
	cfsetospeed(&t, sp);
	t.c_cflag |= CLOCAL | CREAD;
	t.c_cflag &= ~CRTSCTS;
	t.c_cflag &= ~CSTOPB;			/* 8N1 */
	t.c_cflag &= ~PARENB;
	t.c_cc[VMIN] = 0;
	t.c_cc[VTIME] = 0;

	if (tcsetattr(fd, TCSANOW, &t) < 0) {
		close(fd);
		return -1;
	}
	tcflush(fd, TCIFLUSH);
	return fd;
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

	fprintf(f, "{\n\t\"device\": \"%s\",\n", g.device);
	fprintf(f, "\t\"link\": %s,\n", g.link ? "true" : "false");
	fprintf(f, "\t\"age_ms\": %llu,\n",
		(unsigned long long)(g.last_frame_ms ?
				     now_ms() - g.last_frame_ms : 0));
	fprintf(f, "\t\"frames\": %llu,\n", (unsigned long long)g.frames);
	fprintf(f, "\t\"bad_crc\": %llu,\n", (unsigned long long)g.bad_crc);
	fprintf(f, "\t\"resyncs\": %llu,\n", (unsigned long long)g.resyncs);
	fprintf(f, "\t\"link_losses\": %llu,\n",
		(unsigned long long)g.link_losses);
	fprintf(f, "\t\"channels\": [");
	for (i = 0; i < IBUS_CHANNELS; i++)
		fprintf(f, "%s%u", i ? ", " : "", g.ch[i]);
	fprintf(f, "]\n}\n");
	fclose(f);

	if (rename(tmp, g.status_path) < 0)
		unlink(tmp);
}

/* Wire format for the UDP forward: magic, version, then the 14 channels. */
static void forward(int sock)
{
	uint8_t pkt[8 + IBUS_CHANNELS * 2];
	int i;

	if (!g.have_peer || sock < 0)
		return;

	memcpy(pkt, "IBUS", 4);
	pkt[4] = 1;
	pkt[5] = IBUS_CHANNELS;
	pkt[6] = (uint8_t)g.link;
	pkt[7] = 0;
	for (i = 0; i < IBUS_CHANNELS; i++) {
		pkt[8 + i * 2] = (uint8_t)(g.ch[i] & 0xff);
		pkt[9 + i * 2] = (uint8_t)(g.ch[i] >> 8);
	}
	if (sendto(sock, pkt, sizeof(pkt), 0, (struct sockaddr *)&g.peer,
		   sizeof(g.peer)) < 0 && errno != EAGAIN)
		logmsg(LOG_WARNING, "forward: %s", strerror(errno));
}

static bool frame_valid(const uint8_t *f)
{
	uint32_t sum = 0;
	uint16_t want;
	int i;

	if (f[0] != IBUS_HDR0 || f[1] != IBUS_HDR1)
		return false;

	for (i = 0; i < IBUS_LEN - 2; i++)
		sum += f[i];
	want = (uint16_t)f[IBUS_LEN - 2] | ((uint16_t)f[IBUS_LEN - 1] << 8);
	return (uint16_t)(0xffffu - sum) == want;
}

static void frame_accept(const uint8_t *f, int sock)
{
	int i;

	for (i = 0; i < IBUS_CHANNELS; i++)
		g.ch[i] = (uint16_t)f[2 + i * 2] |
			  ((uint16_t)f[3 + i * 2] << 8);

	g.frames++;
	g.last_frame_ms = now_ms();
	if (!g.link) {
		g.link = true;
		logmsg(LOG_NOTICE, "link up (%u frames)",
		       (unsigned)g.frames);
	}
	forward(sock);
}

static void usage(const char *a0)
{
	fprintf(stderr,
"Usage: %s [options]\n"
"  -D, --device DEV      serial device (default /dev/ttyUSB0)\n"
"  -b, --baud RATE       default 115200, which is what i-BUS uses\n"
"  -r, --remote HOST:PORT forward channels here as UDP\n"
"  -t, --timeout MS      declare link loss after this long without a valid\n"
"                        frame (default 200; frames arrive every ~7.5 ms)\n"
"  -S, --status PATH     JSON status file (default /var/run/rc-ibus.json)\n"
"  -I, --status-interval MS  status rewrite interval (default 100)\n"
"  -f, --foreground      log to stderr instead of syslog\n", a0);
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
		{ "device",          required_argument, NULL, 'D' },
		{ "baud",            required_argument, NULL, 'b' },
		{ "remote",          required_argument, NULL, 'r' },
		{ "timeout",         required_argument, NULL, 't' },
		{ "status",          required_argument, NULL, 'S' },
		{ "status-interval", required_argument, NULL, 'I' },
		{ "foreground",      no_argument,       NULL, 'f' },
		{ "help",            no_argument,       NULL, 'h' },
		{ NULL, 0, NULL, 0 }
	};
	struct sigaction sa;
	struct pollfd pfd[1];
	uint8_t ring[RING];
	size_t have = 0;
	int fd, sock = -1, opt;
	uint64_t last_status = 0;

	g.device = (char *)"/dev/ttyUSB0";
	g.baud = 115200;
	g.status_path = (char *)"/var/run/rc-ibus.json";
	g.status_ms = 100;
	g.timeout_ms = 200;

	while ((opt = getopt_long(argc, argv, "D:b:r:t:S:I:fh", opts, NULL)) != -1) {
		switch (opt) {
		case 'D': g.device = optarg; break;
		case 'b': g.baud = atoi(optarg); break;
		case 'r':
			if (!parse_hostport(optarg, &g.peer, 7710)) {
				fprintf(stderr, "bad --remote '%s'\n", optarg);
				return 1;
			}
			g.have_peer = true;
			break;
		case 't': g.timeout_ms = atoi(optarg); break;
		case 'S': g.status_path = optarg; break;
		case 'I': g.status_ms = atoi(optarg); break;
		case 'f': g.foreground = true; break;
		case 'h': usage(argv[0]); return 0;
		default: usage(argv[0]); return 1;
		}
	}

	if (g.status_ms < 20)
		g.status_ms = 20;
	if (g.timeout_ms < 50)
		g.timeout_ms = 50;

	if (!g.foreground)
		openlog("rc-ibus", LOG_PID, LOG_DAEMON);

	sa.sa_handler = on_signal;
	sigemptyset(&sa.sa_mask);
	sa.sa_flags = 0;		/* no SA_RESTART: poll must be interruptible */
	sigaction(SIGINT, &sa, NULL);
	sigaction(SIGTERM, &sa, NULL);
	signal(SIGPIPE, SIG_IGN);

	fd = serial_open(g.device, g.baud);
	if (fd < 0) {
		logmsg(LOG_ERR, "cannot open %s: %s", g.device, strerror(errno));
		return 1;
	}

	if (g.have_peer) {
		sock = socket(AF_INET, SOCK_DGRAM, 0);
		if (sock >= 0)
			fcntl(sock, F_SETFL,
			      fcntl(sock, F_GETFL, 0) | O_NONBLOCK);
	}

	logmsg(LOG_NOTICE, "%s at %d, i-BUS %d channels, link timeout %d ms",
	       g.device, g.baud, IBUS_CHANNELS, g.timeout_ms);

	while (!stop_requested) {
		ssize_t n;

		pfd[0].fd = fd;
		pfd[0].events = POLLIN;
		pfd[0].revents = 0;

		if (poll(pfd, 1, g.status_ms) < 0 && errno != EINTR)
			logmsg(LOG_WARNING, "poll: %s", strerror(errno));

		if (pfd[0].revents & POLLIN) {
			n = read(fd, ring + have, sizeof(ring) - have);
			if (n > 0) {
				have += (size_t)n;
				g.bytes += (uint64_t)n;
			} else if (n == 0) {
				/* A pty peer closing looks like EOF. Keep the
				 * loop alive rather than exiting, so a receiver
				 * being unplugged and replugged recovers. */
				logmsg(LOG_WARNING, "%s returned EOF", g.device);
				usleep(100000);
			}
		}

		/* Scan for frames. The stream has no escaping, so a resync means
		 * dropping one byte at a time until a header and a checksum
		 * agree - which is why the checksum matters for framing and not
		 * only for integrity. */
		while (have >= IBUS_LEN) {
			if (frame_valid(ring)) {
				frame_accept(ring, sock);
				memmove(ring, ring + IBUS_LEN, have - IBUS_LEN);
				have -= IBUS_LEN;
				continue;
			}
			/* A 0x20 0x40 pair that fails the checksum is counted as a
			 * bad frame, which also catches coincidental header
			 * matches while resynchronising - the two are not
			 * distinguishable, so bad_crc reads slightly high on a
			 * noisy line rather than hiding real corruption. */
			if (ring[0] == IBUS_HDR0 && ring[1] == IBUS_HDR1)
				g.bad_crc++;
			else
				g.resyncs++;
			memmove(ring, ring + 1, have - 1);
			have--;
		}
		if (have == sizeof(ring)) {
			/* Cannot happen while the loop above drains, but never
			 * let a garbage stream wedge the buffer. */
			have = 0;
			g.resyncs++;
		}

		if (g.link && g.last_frame_ms &&
		    now_ms() - g.last_frame_ms > (uint64_t)g.timeout_ms) {
			g.link = false;
			g.link_losses++;
			logmsg(LOG_WARNING, "link lost after %llu ms",
			       (unsigned long long)(now_ms() - g.last_frame_ms));
			forward(sock);	/* tell downstream immediately */
		}

		if (now_ms() - last_status >= (uint64_t)g.status_ms) {
			last_status = now_ms();
			status_write();
		}
	}

	logmsg(LOG_NOTICE, "stopping: %llu frames, %llu bad crc, %llu resyncs",
	       (unsigned long long)g.frames, (unsigned long long)g.bad_crc,
	       (unsigned long long)g.resyncs);
	status_write();
	if (sock >= 0)
		close(sock);
	close(fd);
	return 0;
}
