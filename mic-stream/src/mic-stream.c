#define _GNU_SOURCE
// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * mic-stream - serve a USB microphone as low-latency PCM over HTTP
 *
 * Deliberately uncompressed. 16 kHz mono S16 is 256 kbit/s, which is nothing
 * next to the camera and the lidar, and skipping the codec removes both the
 * encoder's algorithmic delay and any CPU cost on a soft-float MIPS part. The
 * only buffering is one ALSA period.
 *
 *   /pcm   raw S16_LE, for Web Audio or anything that can read a stream
 *   /wav   the same with a WAV header, for players that need one
 *   /info  JSON status
 *
 * Capture is done by running arecord rather than linking alsa-lib, which keeps
 * this a few kilobytes and reuses the device handling that alsa-utils already
 * gets right.
 */

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
#include <sys/wait.h>
#include <syslog.h>
#include <time.h>
#include <unistd.h>

#define MAX_CLIENTS	8
#define CHUNK		4096
#define RETRY_S		5

enum kind { K_NONE = 0, K_PCM, K_WAV };

struct client {
	int fd;
	enum kind kind;
	bool header_sent;
	uint64_t dropped;
};

static struct {
	char *device;
	int rate, channels, period;
	int port;
	bool foreground;

	struct client cl[MAX_CLIENTS];
	int ncl;
	pid_t child;
	uint64_t bytes, chunks, retries;
	bool warned_missing;
	bool ever_read;
} g;

static volatile sig_atomic_t stop_requested;
static volatile sig_atomic_t child_died;

static void on_signal(int sig)
{
	if (sig == SIGCHLD)
		child_died = 1;
	else
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

/*
 * Start arecord writing raw PCM to a pipe. A small period is the whole point:
 * it is the floor on capture latency, and 10 ms is short enough not to matter
 * next to the network path.
 */
static int spawn_arecord(void)
{
	char rate[16], chans[8], period[16], buffer[16];
	int fds[2];
	pid_t pid;

	if (pipe(fds) < 0)
		return -1;

	snprintf(rate, sizeof(rate), "%d", g.rate);
	snprintf(chans, sizeof(chans), "%d", g.channels);
	snprintf(period, sizeof(period), "%d", g.period);
	snprintf(buffer, sizeof(buffer), "%d", g.period * 4);

	pid = fork();
	if (pid < 0) {
		close(fds[0]);
		close(fds[1]);
		return -1;
	}

	if (pid == 0) {
		dup2(fds[1], STDOUT_FILENO);
		close(fds[0]);
		close(fds[1]);

		/*
		 * The "once per outage" guard on our own message does nothing
		 * about arecord's, which procd forwards to syslog under this
		 * service's name. With no capture device that is one
		 * daemon.err line every retry, for ever - about 17k lines a
		 * day on a router with nothing plugged in, burying whatever
		 * someone is actually trying to read.
		 *
		 * So the first attempt of an outage keeps stderr, which puts
		 * the real reason in the log exactly once, and the retries
		 * after it are silenced. warned_missing is cleared when
		 * samples arrive again, so the next outage speaks up too.
		 */
		if (g.warned_missing) {
			int null = open("/dev/null", O_WRONLY);

			if (null >= 0) {
				dup2(null, STDERR_FILENO);
				close(null);
			}
		}
		execlp("arecord", "arecord",
		       "-D", g.device,
		       "-f", "S16_LE",
		       "-c", chans,
		       "-r", rate,
		       "-t", "raw",
		       "--period-size", period,
		       "--buffer-size", buffer,
		       "-q", "-", (char *)NULL);
		_exit(127);
	}

	close(fds[1]);
	g.child = pid;
	fcntl(fds[0], F_SETFL, fcntl(fds[0], F_GETFL, 0) | O_NONBLOCK);
	return fds[0];
}

static void wav_header(uint8_t *h, int rate, int channels)
{
	/* Length fields are left at their maximum: the stream has no end, and
	 * every player that accepts a pipe treats it as "read until EOF". */
	const uint32_t data_len = 0xffffffffu - 36;
	uint32_t byte_rate = (uint32_t)rate * (uint32_t)channels * 2;

	memcpy(h, "RIFF", 4);
	h[4] = 0xff; h[5] = 0xff; h[6] = 0xff; h[7] = 0xff;
	memcpy(h + 8, "WAVEfmt ", 8);
	h[16] = 16; h[17] = h[18] = h[19] = 0;		/* fmt chunk size */
	h[20] = 1; h[21] = 0;				/* PCM */
	h[22] = (uint8_t)channels; h[23] = 0;
	h[24] = (uint8_t)(rate & 0xff);
	h[25] = (uint8_t)((rate >> 8) & 0xff);
	h[26] = (uint8_t)((rate >> 16) & 0xff);
	h[27] = (uint8_t)((rate >> 24) & 0xff);
	h[28] = (uint8_t)(byte_rate & 0xff);
	h[29] = (uint8_t)((byte_rate >> 8) & 0xff);
	h[30] = (uint8_t)((byte_rate >> 16) & 0xff);
	h[31] = (uint8_t)((byte_rate >> 24) & 0xff);
	h[32] = (uint8_t)(channels * 2); h[33] = 0;	/* block align */
	h[34] = 16; h[35] = 0;				/* bits per sample */
	memcpy(h + 36, "data", 4);
	h[40] = (uint8_t)(data_len & 0xff);
	h[41] = (uint8_t)((data_len >> 8) & 0xff);
	h[42] = (uint8_t)((data_len >> 16) & 0xff);
	h[43] = (uint8_t)((data_len >> 24) & 0xff);
}

static void client_drop(int idx)
{
	close(g.cl[idx].fd);
	g.cl[idx] = g.cl[--g.ncl];
}

static void client_accept(int lfd)
{
	char req[512], hdr[512];
	struct client *c;
	enum kind kind;
	int fd, on = 1, n;

	/* CLOEXEC because this process forks arecord: without it, every client
	 * socket open at the moment arecord is (re)started is duplicated into
	 * it, and a browser that disconnects leaves a socket that is not fully
	 * torn down until arecord itself exits - which, since arecord is
	 * restarted only on failure, can be hours. */
	fd = accept4(lfd, NULL, NULL, SOCK_CLOEXEC);
	if (fd < 0)
		return;
	if (g.ncl >= MAX_CLIENTS) {
		close(fd);
		return;
	}

	/* One blocking read is enough: a request line arrives in one segment in
	 * every case that matters here, and anything else gets a 404. */
	n = (int)read(fd, req, sizeof(req) - 1);
	if (n <= 0) {
		close(fd);
		return;
	}
	req[n] = '\0';

	if (strstr(req, "GET /wav"))
		kind = K_WAV;
	else if (strstr(req, "GET /pcm") || strstr(req, "GET / "))
		kind = K_PCM;
	else if (strstr(req, "GET /info")) {
		n = snprintf(hdr, sizeof(hdr),
			"HTTP/1.0 200 OK\r\n"
			"Content-Type: application/json\r\n"
			"Access-Control-Allow-Origin: *\r\n\r\n"
			"{\"rate\":%d,\"channels\":%d,\"period\":%d,"
			"\"clients\":%d,\"bytes\":%llu,\"chunks\":%llu}\n",
			g.rate, g.channels, g.period, g.ncl,
			(unsigned long long)g.bytes,
			(unsigned long long)g.chunks);
		(void)!write(fd, hdr, (size_t)n);
		close(fd);
		return;
	} else {
		(void)!write(fd, "HTTP/1.0 404 Not Found\r\n\r\n", 26);
		close(fd);
		return;
	}

	n = snprintf(hdr, sizeof(hdr),
		"HTTP/1.0 200 OK\r\n"
		"Content-Type: %s\r\n"
		"Cache-Control: no-store\r\n"
		"Connection: close\r\n"
		"Access-Control-Allow-Origin: *\r\n"
		"\r\n",
		kind == K_WAV ? "audio/wav" :
		(g.channels == 1 ? "audio/L16;rate=16000;channels=1" : "audio/L16"));
	if (write(fd, hdr, (size_t)n) != n) {
		close(fd);
		return;
	}

	if (kind == K_WAV) {
		uint8_t h[44];

		wav_header(h, g.rate, g.channels);
		if (write(fd, h, sizeof(h)) != (ssize_t)sizeof(h)) {
			close(fd);
			return;
		}
	}

	/* Nagle would coalesce 10 ms periods into larger, later packets, which
	 * is the opposite of what an uncompressed low-latency stream wants. */
	setsockopt(fd, IPPROTO_TCP, TCP_NODELAY, &on, sizeof(on));
	fcntl(fd, F_SETFL, fcntl(fd, F_GETFL, 0) | O_NONBLOCK);

	c = &g.cl[g.ncl++];
	c->fd = fd;
	c->kind = kind;
	c->header_sent = true;
	c->dropped = 0;
	logmsg(LOG_INFO, "client %d connected (%s), %d total",
	       fd, kind == K_WAV ? "wav" : "pcm", g.ncl);
}

static void fanout(const uint8_t *buf, size_t len)
{
	int i;

	for (i = 0; i < g.ncl; ) {
		ssize_t n = write(g.cl[i].fd, buf, len);

		if (n < 0) {
			if (errno == EAGAIN || errno == EWOULDBLOCK) {
				/* A reader that cannot keep up loses audio rather
				 * than being allowed to stall capture. Dropping
				 * is correct for live audio: there is no value in
				 * delivering a period late. */
				if (!(g.cl[i].dropped++ % 100))
					logmsg(LOG_WARNING,
					       "client %d falling behind (%llu drops)",
					       g.cl[i].fd,
					       (unsigned long long)g.cl[i].dropped);
				i++;
				continue;
			}
			if (errno == EINTR)
				continue;
			logmsg(LOG_INFO, "client %d gone", g.cl[i].fd);
			client_drop(i);
			continue;
		}
		i++;
	}
}

static void usage(const char *a0)
{
	fprintf(stderr,
"Usage: %s [options]\n"
"  -D, --device DEV     ALSA capture device (default hw:0,0)\n"
"  -r, --rate HZ        sample rate (default 16000)\n"
"  -c, --channels N     channels (default 1)\n"
"  -P, --period FRAMES  ALSA period; the floor on capture latency\n"
"                       (default 160 frames = 10 ms at 16 kHz)\n"
"  -p, --port PORT      HTTP port (default 8082)\n"
"  -f, --foreground     log to stderr instead of syslog\n", a0);
}

int main(int argc, char **argv)
{
	static const struct option opts[] = {
		{ "device",     required_argument, NULL, 'D' },
		{ "rate",       required_argument, NULL, 'r' },
		{ "channels",   required_argument, NULL, 'c' },
		{ "period",     required_argument, NULL, 'P' },
		{ "port",       required_argument, NULL, 'p' },
		{ "foreground", no_argument,       NULL, 'f' },
		{ "help",       no_argument,       NULL, 'h' },
		{ NULL, 0, NULL, 0 }
	};
	struct pollfd pfd[2 + MAX_CLIENTS];
	struct sockaddr_in a;
	struct sigaction sa;
	uint8_t buf[CHUNK];
	int lfd, pipefd, on = 1, opt, i;
	uint64_t next_try = 0;

	g.device = (char *)"hw:0,0";
	g.rate = 16000;
	g.channels = 1;
	g.period = 160;
	g.port = 8082;

	while ((opt = getopt_long(argc, argv, "D:r:c:P:p:fh", opts, NULL)) != -1) {
		switch (opt) {
		case 'D': g.device = optarg; break;
		case 'r': g.rate = atoi(optarg); break;
		case 'c': g.channels = atoi(optarg); break;
		case 'P': g.period = atoi(optarg); break;
		case 'p': g.port = atoi(optarg); break;
		case 'f': g.foreground = true; break;
		case 'h': usage(argv[0]); return 0;
		default: usage(argv[0]); return 1;
		}
	}

	if (g.channels < 1 || g.channels > 2 || g.rate < 8000 ||
	    g.rate > 48000 || g.period < 16) {
		fprintf(stderr, "unsupported rate/channels/period\n");
		return 1;
	}

	if (!g.foreground)
		openlog("mic-stream", LOG_PID, LOG_DAEMON);

	sa.sa_handler = on_signal;
	sigemptyset(&sa.sa_mask);
	sa.sa_flags = 0;		/* no SA_RESTART: poll must be interruptible */
	sigaction(SIGINT, &sa, NULL);
	sigaction(SIGTERM, &sa, NULL);
	sigaction(SIGCHLD, &sa, NULL);
	signal(SIGPIPE, SIG_IGN);

	lfd = socket(AF_INET, SOCK_STREAM | SOCK_CLOEXEC, 0);
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

	pipefd = spawn_arecord();
	if (pipefd < 0) {
		logmsg(LOG_WARNING,
		       "cannot start arecord yet (%s); retrying every %d s",
		       strerror(errno), RETRY_S);
		next_try = now_ms() + RETRY_S * 1000;
	}

	logmsg(LOG_NOTICE, "%s %d Hz %d ch, period %d (%d ms), serving :%d",
	       g.device, g.rate, g.channels, g.period,
	       g.period * 1000 / g.rate, g.port);

	while (!stop_requested) {
		int np = 0, n;

		if (pipefd >= 0) {
			pfd[np].fd = pipefd;
			pfd[np].events = POLLIN;
			pfd[np++].revents = 0;
		}
		pfd[np].fd = lfd;
		pfd[np].events = POLLIN;
		pfd[np++].revents = 0;
		for (i = 0; i < g.ncl; i++) {
			pfd[np].fd = g.cl[i].fd;
			pfd[np].events = 0;
			pfd[np++].revents = 0;
		}

		if (poll(pfd, (nfds_t)np, 500) < 0) {
			if (errno == EINTR)
				continue;
			logmsg(LOG_ERR, "poll: %s", strerror(errno));
			break;
		}

		if (child_died) {
			int status = 0;

			if (waitpid(g.child, &status, WNOHANG) == g.child) {
				/* Exiting here put procd into a crash loop whenever
				 * the device was absent - which on the router is
				 * simply "the camera is not plugged in yet". Retry
				 * with a backoff instead, and say so once rather
				 * than filling the log. */
				g.child = -1;
				g.retries++;
				/* Once per outage, not once per attempt: at a 5 s
				 * retry this would otherwise be a slow log flood. */
				if (!g.warned_missing) {
					g.warned_missing = true;
					logmsg(LOG_ERR,
					       "arecord exited (status %d) on %s; retrying every %d s",
					       WIFEXITED(status) ? WEXITSTATUS(status) : -1,
					       g.device, RETRY_S);
				}
				close(pipefd);
				pipefd = -1;
				next_try = now_ms() + RETRY_S * 1000;
			}
			child_died = 0;
		}

		if (pipefd < 0 && now_ms() >= next_try) {
			pipefd = spawn_arecord();
			if (pipefd < 0)
				next_try = now_ms() + RETRY_S * 1000;
			/* Deliberately no "device is back" here: fork succeeding
			 * proves nothing about the device. That is only known once
			 * samples actually arrive, below. */
		}

		if (pfd[pipefd >= 0 ? 1 : 0].revents & POLLIN)
			client_accept(lfd);

		{
			int base = (pipefd >= 0 ? 2 : 1);

			for (i = g.ncl - 1; i >= 0; i--)
				if (base + i < np &&
				    (pfd[base + i].revents &
				     (POLLERR | POLLHUP | POLLNVAL)))
					client_drop(i);
		}

		if (pipefd < 0 || !(pfd[0].revents & (POLLIN | POLLHUP)))
			continue;

		n = (int)read(pipefd, buf, sizeof(buf));
		if (n > 0) {
			if (g.warned_missing) {
				g.warned_missing = false;
				logmsg(LOG_NOTICE,
				       "capture device delivering again after %llu retries",
				       (unsigned long long)g.retries);
				g.retries = 0;
			}
			g.ever_read = true;
			g.bytes += (uint64_t)n;
			g.chunks++;
			if (g.ncl)
				fanout(buf, (size_t)n);
		} else if (n == 0) {
			/* Same reasoning as the child exiting: back off and try
			 * again rather than handing procd a reason to loop. */
			close(pipefd);
			pipefd = -1;
			next_try = now_ms() + RETRY_S * 1000;
		} else if (errno != EAGAIN && errno != EWOULDBLOCK &&
			   errno != EINTR) {
			logmsg(LOG_ERR, "read: %s", strerror(errno));
			break;
		}
	}

	logmsg(LOG_NOTICE, "stopping after %llu chunks / %llu bytes",
	       (unsigned long long)g.chunks, (unsigned long long)g.bytes);

	while (g.ncl)
		client_drop(0);
	if (g.child > 0) {
		kill(g.child, SIGTERM);
		waitpid(g.child, NULL, 0);
	}
	if (pipefd >= 0)
		close(pipefd);
	close(lfd);
	return 0;
}
