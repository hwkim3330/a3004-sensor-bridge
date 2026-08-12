#define _GNU_SOURCE
// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * ouster-edge - on-device processing for Ouster OS-series lidars
 *
 * Runs on the router the sensor is plugged into (MT7621 class hardware) and
 * does the work that is actually cheap enough to do there:
 *
 *   - receives the raw lidar UDP stream (default port 7502)
 *   - optionally relays every packet verbatim to a downstream host, so a
 *     full driver on a real machine still sees an untouched stream
 *   - reduces each revolution to a per-azimuth minimum-range "ring", which
 *     is an integer min-reduction over the range field and costs almost
 *     nothing compared to building point clouds
 *   - evaluates polar zones against that ring and reports intrusions
 *   - publishes the ring as a small UDP datagram and a JSON status file
 *
 * Deliberately not attempted here: cartesian point clouds, ROS transport,
 * anything needing floating point per point. See doc/ARCHITECTURE.md.
 */

#include <arpa/inet.h>
#include <errno.h>
#include <fcntl.h>
#include <getopt.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <signal.h>
#include <stdarg.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <poll.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <syslog.h>
#include <time.h>
#include <unistd.h>

#define BATCH		16
#define MAX_PKT		40000
#define MAX_SECTORS	4096
#define MAX_ZONES	16
#define MAX_SSE		8
#define RING_MAGIC	0x4445534fu	/* "OSED" little-endian */
#define RING_VERSION	1

enum profile {
	PROF_UNKNOWN = 0,
	PROF_LEGACY,	/* 12 B/px, 20-bit range, per-column 16 B hdr + 4 B ftr */
	PROF_SINGLE,	/* RNG19_RFL8_SIG16_NIR16      12 B/px */
	PROF_LOWRATE,	/* RNG15_RFL8_NIR8              4 B/px */
	PROF_DUAL,	/* RNG19_RFL8_SIG16_NIR16_DUAL 16 B/px */
};

static const char *profile_name(enum profile p)
{
	switch (p) {
	case PROF_LEGACY:	return "LEGACY";
	case PROF_SINGLE:	return "RNG19_RFL8_SIG16_NIR16";
	case PROF_LOWRATE:	return "RNG15_RFL8_NIR8";
	case PROF_DUAL:		return "RNG19_RFL8_SIG16_NIR16_DUAL";
	default:		return "unknown";
	}
}

struct layout {
	enum profile profile;
	int channels;		/* pixels_per_column */
	int columns;		/* columns_per_packet */
	int px;			/* bytes per channel data block */
	int pkt_hdr, pkt_ftr;
	int col_hdr, col_ftr;
	int col_stride;
	size_t pkt_size;
};

struct zone {
	int az_start_mdeg;	/* millidegrees, [0, 360000) */
	int az_end_mdeg;	/* may wrap past 360000 */
	uint32_t max_range_mm;
	bool active;
};

struct stats {
	uint64_t packets, bytes, frames, relayed;
	uint64_t bad_size, invalid_cols, gap_cols;
};

static struct {
	/* configuration */
	int listen_port;
	int channels, columns, scan_width, sectors;
	int ch_lo, ch_hi;
	uint32_t min_range_mm, max_range_mm;
	struct sockaddr_in relay_to, ring_to;
	bool have_relay, have_ring;
	char *status_path;
	char *action;
	struct zone zones[MAX_ZONES];
	int nzones;
	bool foreground;

	int status_ms;
	int sse_port;

	/* zone_mask[sector] has bit z set when sector falls inside zone z, so a
	 * column can be tested against every zone with one array lookup instead
	 * of sweeping the whole ring once per revolution. */
	uint16_t zone_mask[MAX_SECTORS];

	/* runtime */
	struct layout layout;
	/* ring_min accumulates the revolution in progress; ring_pub is the last
	 * completed revolution, which is what gets published and evaluated so
	 * readers never see a half-built ring. Units are centimetres, 0xffff
	 * means no return in that sector. */
	uint16_t ring_min[MAX_SECTORS];
	uint16_t ring_pub[MAX_SECTORS];
	uint8_t  ring_refl[MAX_SECTORS];
	uint8_t  ring_refl_pub[MAX_SECTORS];
	int cur_frame;
	int last_mid;
	uint64_t frame_ts;
	uint64_t last_pkt_ms;
	struct stats st;
	bool zone_alarm;
	uint16_t zone_hit;		/* zones touched during this revolution */
	int sse[MAX_SSE];
	int nsse;
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

/* Ouster puts everything on the wire little-endian. Read it explicitly so the
 * parser behaves the same on any host byte order and never depends on
 * alignment. */
static inline uint16_t rd16(const uint8_t *p)
{
	return (uint16_t)p[0] | ((uint16_t)p[1] << 8);
}

static inline uint32_t rd32(const uint8_t *p)
{
	return (uint32_t)p[0] | ((uint32_t)p[1] << 8) |
	       ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

static inline uint64_t rd64(const uint8_t *p)
{
	return (uint64_t)rd32(p) | ((uint64_t)rd32(p + 4) << 32);
}

/*
 * Derive the packet layout from the observed datagram size. The channel count
 * and columns-per-packet come from the sensor metadata (passed in by the init
 * script); everything else follows from the size, which uniquely identifies
 * the profile for a given (channels, columns) pair.
 */
static bool layout_detect(struct layout *l, size_t sz, int channels, int columns)
{
	static const struct {
		enum profile profile;
		int px;
	} candidates[] = {
		{ PROF_SINGLE,  12 },
		{ PROF_LOWRATE,  4 },
		{ PROF_DUAL,    16 },
	};
	size_t i;

	memset(l, 0, sizeof(*l));
	l->channels = channels;
	l->columns = columns;

	/* LEGACY: no packet header/footer, 16 B column header, 4 B status */
	l->col_stride = 16 + channels * 12 + 4;
	if ((size_t)(columns * l->col_stride) == sz) {
		l->profile = PROF_LEGACY;
		l->px = 12;
		l->col_hdr = 16;
		l->col_ftr = 4;
		l->pkt_hdr = l->pkt_ftr = 0;
		l->pkt_size = sz;
		return true;
	}

	/* Configurable profiles: 32 B packet header + footer, 12 B column header */
	for (i = 0; i < sizeof(candidates) / sizeof(candidates[0]); i++) {
		int stride = 12 + channels * candidates[i].px;

		if (sz != (size_t)(32 + columns * stride + 32))
			continue;

		l->profile = candidates[i].profile;
		l->px = candidates[i].px;
		l->pkt_hdr = 32;
		l->pkt_ftr = 32;
		l->col_hdr = 12;
		l->col_ftr = 0;
		l->col_stride = stride;
		l->pkt_size = sz;
		return true;
	}

	memset(l, 0, sizeof(*l));
	return false;
}

static inline uint32_t px_range_mm(const struct layout *l, const uint8_t *p)
{
	switch (l->profile) {
	case PROF_LEGACY:
		return rd32(p) & 0x000fffffu;		/* 20-bit, mm */
	case PROF_SINGLE:
	case PROF_DUAL:
		return rd32(p) & 0x0007ffffu;		/* 19-bit, mm */
	case PROF_LOWRATE:
		return (rd16(p) & 0x7fffu) * 8u;	/* 15-bit, 8 mm units */
	default:
		return 0;
	}
}

static inline uint8_t px_refl(const struct layout *l, const uint8_t *p)
{
	uint16_t v;

	switch (l->profile) {
	case PROF_LEGACY:
		v = rd16(p + 4);			/* 16-bit raw */
		return v > 255 ? 255 : (uint8_t)v;
	case PROF_SINGLE:
	case PROF_DUAL:
		return p[4];
	case PROF_LOWRATE:
		return p[2];
	default:
		return 0;
	}
}

static uint64_t now_ms(void)
{
	struct timespec ts;

	clock_gettime(CLOCK_MONOTONIC, &ts);
	return (uint64_t)ts.tv_sec * 1000u + (uint64_t)(ts.tv_nsec / 1000000);
}

static void zones_index(void)
{
	int i, z;

	memset(g.zone_mask, 0, sizeof(g.zone_mask));
	for (i = 0; i < g.sectors; i++) {
		int az = (int)(((long)i * 360000L) / g.sectors);

		for (z = 0; z < g.nzones; z++) {
			int azw = az < g.zones[z].az_start_mdeg ? az + 360000 : az;

			if (azw >= g.zones[z].az_start_mdeg &&
			    azw <= g.zones[z].az_end_mdeg)
				g.zone_mask[i] |= (uint16_t)(1u << z);
		}
	}
}

static void ring_reset(void)
{
	int i;

	for (i = 0; i < g.sectors; i++) {
		g.ring_min[i] = 0xffff;
		g.ring_refl[i] = 0;
		if (!g.st.frames) {
			g.ring_pub[i] = 0xffff;
			g.ring_refl_pub[i] = 0;
		}
	}
}

static void run_action(const char *event)
{
	char cmd[512];

	if (!g.action)
		return;

	snprintf(cmd, sizeof(cmd), "%s %s &", g.action, event);
	if (system(cmd) < 0)
		logmsg(LOG_WARNING, "action '%s' failed: %s", g.action,
		       strerror(errno));
}

/*
 * Called for every column as it arrives, not once per revolution. A zone
 * crossing is what the local reflex reacts to, so waiting for the rotation to
 * finish would add up to a full 100 ms at 10 Hz for no reason.
 */
static void zones_column(int sector, uint32_t range_mm)
{
	uint16_t mask = g.zone_mask[sector];
	int z;

	if (!mask)
		return;

	for (z = 0; z < g.nzones; z++) {
		if (!(mask & (1u << z)))
			continue;
		if (range_mm > g.zones[z].max_range_mm)
			continue;

		g.zone_hit |= (uint16_t)(1u << z);

		if (!g.zones[z].active) {
			g.zones[z].active = true;
			logmsg(LOG_NOTICE, "zone %d ENTERED", z);
		}
		/* Rising edge fires as soon as one intruding column lands. */
		if (!g.zone_alarm) {
			g.zone_alarm = true;
			run_action("alarm");
		}
	}
}

/*
 * Clearing is the asymmetric half: it takes a full clean revolution to know
 * nothing is there, so it can only be decided at the frame boundary.
 */
static void zones_revolution_end(void)
{
	int z;

	for (z = 0; z < g.nzones; z++) {
		if (g.zones[z].active && !(g.zone_hit & (1u << z))) {
			g.zones[z].active = false;
			logmsg(LOG_NOTICE, "zone %d clear", z);
		}
	}

	if (g.zone_alarm && !g.zone_hit) {
		g.zone_alarm = false;
		run_action("clear");
	}
	g.zone_hit = 0;
}

static void ring_publish(int sock)
{
	uint8_t buf[24 + MAX_SECTORS * 3];
	size_t off = 0;
	int i;

	if (!g.have_ring)
		return;

	buf[0] = RING_MAGIC & 0xff;
	buf[1] = (RING_MAGIC >> 8) & 0xff;
	buf[2] = (RING_MAGIC >> 16) & 0xff;
	buf[3] = (RING_MAGIC >> 24) & 0xff;
	buf[4] = RING_VERSION;
	buf[5] = (uint8_t)g.layout.profile;
	buf[6] = (uint8_t)(g.sectors & 0xff);
	buf[7] = (uint8_t)(g.sectors >> 8);
	buf[8] = (uint8_t)(g.cur_frame & 0xff);
	buf[9] = (uint8_t)((g.cur_frame >> 8) & 0xff);
	buf[10] = (uint8_t)g.zone_alarm;
	buf[11] = 0;
	for (i = 0; i < 8; i++)
		buf[12 + i] = (uint8_t)((g.frame_ts >> (8 * i)) & 0xff);
	off = 20;

	/* ring_min is already in centimetres (see accumulate) */
	for (i = 0; i < g.sectors; i++) {
		buf[off++] = (uint8_t)(g.ring_pub[i] & 0xff);
		buf[off++] = (uint8_t)(g.ring_pub[i] >> 8);
	}
	for (i = 0; i < g.sectors; i++)
		buf[off++] = g.ring_refl_pub[i];

	if (sendto(sock, buf, off, 0, (struct sockaddr *)&g.ring_to,
		   sizeof(g.ring_to)) < 0 && errno != EAGAIN)
		logmsg(LOG_WARNING, "ring sendto: %s", strerror(errno));
}

/*
 * A tiny Server-Sent Events endpoint. Polling a status file costs up to a full
 * poll interval of latency on top of the write interval; pushing costs none.
 * SSE rather than WebSocket because it needs no handshake, no SHA-1, and no
 * frame masking - just a header and "data: ...\n\n" per event.
 */
static int sse_listen(int port)
{
	struct sockaddr_in a;
	int fd, on = 1;

	fd = socket(AF_INET, SOCK_STREAM, 0);
	if (fd < 0)
		return -1;
	setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, &on, sizeof(on));
	fcntl(fd, F_SETFL, fcntl(fd, F_GETFL, 0) | O_NONBLOCK);

	memset(&a, 0, sizeof(a));
	a.sin_family = AF_INET;
	a.sin_addr.s_addr = htonl(INADDR_ANY);
	a.sin_port = htons((uint16_t)port);
	if (bind(fd, (struct sockaddr *)&a, sizeof(a)) < 0 ||
	    listen(fd, 4) < 0) {
		close(fd);
		return -1;
	}
	return fd;
}

static void sse_drop(int idx)
{
	close(g.sse[idx]);
	g.sse[idx] = g.sse[--g.nsse];
}

static void sse_accept(int lfd)
{
	static const char hdr[] =
		"HTTP/1.0 200 OK\r\n"
		"Content-Type: text/event-stream\r\n"
		"Cache-Control: no-store\r\n"
		"Connection: close\r\n"
		"Access-Control-Allow-Origin: *\r\n"
		"\r\n";
	int fd = accept(lfd, NULL, NULL);
	int on = 1;

	if (fd < 0)
		return;
	if (g.nsse >= MAX_SSE) {
		close(fd);
		return;
	}
	fcntl(fd, F_SETFL, fcntl(fd, F_GETFL, 0) | O_NONBLOCK);
	/* Without TCP_NODELAY a 1 kB event can sit in Nagle's queue, which is
	 * exactly the delay this endpoint exists to avoid. */
	setsockopt(fd, IPPROTO_TCP, TCP_NODELAY, &on, sizeof(on));
	if (write(fd, hdr, sizeof(hdr) - 1) < 0) {
		close(fd);
		return;
	}
	g.sse[g.nsse++] = fd;
}

static void sse_broadcast(const char *buf, size_t len)
{
	int i;

	for (i = 0; i < g.nsse; ) {
		ssize_t n = write(g.sse[i], buf, len);

		/* A reader that cannot keep up gets dropped rather than allowed
		 * to back-pressure the lidar path. */
		if (n < 0 && errno != EINTR &&
		    errno != EAGAIN && errno != EWOULDBLOCK) {
			sse_drop(i);
			continue;
		}
		i++;
	}
}

static size_t ring_json(char *buf, size_t cap)
{
	size_t off = 0;
	int i;

	off += (size_t)snprintf(buf + off, cap - off,
		"data: {\"frame_id\":%d,\"sectors\":%d,\"zone_alarm\":%s,"
		"\"packets\":%llu,\"bytes\":%llu,\"frames\":%llu,"
		"\"relayed\":%llu,\"missed_columns\":%llu,\"age_ms\":0,"
		"\"channels\":%d,\"profile\":\"%s\",\"ring_cm\":[",
		g.cur_frame, g.sectors, g.zone_alarm ? "true" : "false",
		(unsigned long long)g.st.packets, (unsigned long long)g.st.bytes,
		(unsigned long long)g.st.frames,
		(unsigned long long)g.st.relayed,
		(unsigned long long)g.st.gap_cols,
		g.layout.channels, profile_name(g.layout.profile));

	for (i = 0; i < g.sectors && off + 8 < cap; i++) {
		if (g.ring_pub[i] == 0xffff)
			off += (size_t)snprintf(buf + off, cap - off,
						i ? ",-1" : "-1");
		else
			off += (size_t)snprintf(buf + off, cap - off,
						i ? ",%u" : "%u",
						g.ring_pub[i]);
	}
	off += (size_t)snprintf(buf + off, cap - off, "]}\n\n");
	return off;
}

static void status_write(void)
{
	char tmp[256];
	FILE *f;
	int z;

	if (!g.status_path)
		return;

	snprintf(tmp, sizeof(tmp), "%s.tmp", g.status_path);
	f = fopen(tmp, "w");
	if (!f)
		return;

	fprintf(f, "{\n");
	fprintf(f, "\t\"profile\": \"%s\",\n", profile_name(g.layout.profile));
	fprintf(f, "\t\"channels\": %d,\n", g.layout.channels);
	fprintf(f, "\t\"columns_per_packet\": %d,\n", g.layout.columns);
	fprintf(f, "\t\"packet_size\": %zu,\n", g.layout.pkt_size);
	fprintf(f, "\t\"scan_width\": %d,\n", g.scan_width);
	fprintf(f, "\t\"sectors\": %d,\n", g.sectors);
	fprintf(f, "\t\"frame_id\": %d,\n", g.cur_frame);
	fprintf(f, "\t\"age_ms\": %llu,\n",
		(unsigned long long)(g.last_pkt_ms ? now_ms() - g.last_pkt_ms : 0));
	fprintf(f, "\t\"packets\": %llu,\n", (unsigned long long)g.st.packets);
	fprintf(f, "\t\"bytes\": %llu,\n", (unsigned long long)g.st.bytes);
	fprintf(f, "\t\"frames\": %llu,\n", (unsigned long long)g.st.frames);
	fprintf(f, "\t\"relayed\": %llu,\n", (unsigned long long)g.st.relayed);
	fprintf(f, "\t\"bad_size\": %llu,\n", (unsigned long long)g.st.bad_size);
	fprintf(f, "\t\"invalid_columns\": %llu,\n",
		(unsigned long long)g.st.invalid_cols);
	fprintf(f, "\t\"missed_columns\": %llu,\n",
		(unsigned long long)g.st.gap_cols);
	fprintf(f, "\t\"zone_alarm\": %s,\n", g.zone_alarm ? "true" : "false");
	fprintf(f, "\t\"zones\": [");
	for (z = 0; z < g.nzones; z++)
		fprintf(f, "%s%s", z ? ", " : "",
			g.zones[z].active ? "true" : "false");
	fprintf(f, "],\n");

	/* The ring goes into the status file as well, so the on-router
	 * dashboard can render it straight from HTTP - a browser cannot read
	 * the binary UDP feed. -1 means "no return in this sector". */
	fprintf(f, "\t\"ring_cm\": [");
	for (z = 0; z < g.sectors; z++) {
		if (z)
			fputc(',', f);
		if (g.ring_pub[z] == 0xffff)
			fputs("-1", f);
		else
			fprintf(f, "%u", g.ring_pub[z]);
	}
	fprintf(f, "]\n}\n");
	fclose(f);

	if (rename(tmp, g.status_path) < 0)
		unlink(tmp);
}

static void frame_complete(int txsock)
{
	g.st.frames++;
	memcpy(g.ring_pub, g.ring_min, sizeof(g.ring_pub[0]) * g.sectors);
	memcpy(g.ring_refl_pub, g.ring_refl, sizeof(g.ring_refl_pub[0]) * g.sectors);
	zones_revolution_end();
	ring_publish(txsock);

	if (g.nsse) {
		static char json[64 + MAX_SECTORS * 8];
		size_t n = ring_json(json, sizeof(json));

		sse_broadcast(json, n);
	}

	ring_reset();
}

static void packet_process(const uint8_t *pkt, size_t len, int txsock)
{
	const struct layout *l = &g.layout;
	int c;

	for (c = 0; c < l->columns; c++) {
		const uint8_t *col = pkt + l->pkt_hdr + (size_t)c * l->col_stride;
		const uint8_t *px = col + l->col_hdr;
		uint32_t best = 0;
		uint8_t best_refl = 0;
		int mid, frame, sector, ch, expected;
		bool valid;

		if ((size_t)(col + l->col_stride - pkt) > len)
			break;

		if (l->profile == PROF_LEGACY) {
			mid = rd16(col + 8);
			frame = rd16(col + 10);
			valid = rd32(col + l->col_hdr + l->channels * l->px) ==
				0xffffffffu;
		} else {
			mid = rd16(col + 8);
			frame = rd16(pkt + 2);
			valid = (rd16(col + 10) & 0x1) != 0;
		}

		if (!valid) {
			g.st.invalid_cols++;
			continue;
		}

		/* Learn the scan width from the sensor rather than trusting a
		 * default that may not match the configured lidar mode. */
		if (mid >= g.scan_width) {
			int w = 512;

			while (w <= mid && w < 4096)
				w <<= 1;
			logmsg(LOG_INFO, "scan width %d -> %d", g.scan_width, w);
			g.scan_width = w;
		}

		expected = (g.last_mid + 1) % g.scan_width;
		if (g.last_mid >= 0 && mid != expected)
			g.st.gap_cols += (mid - expected + g.scan_width) %
					 g.scan_width;
		g.last_mid = mid;

		if (frame != g.cur_frame) {
			if (g.cur_frame >= 0)
				frame_complete(txsock);
			g.cur_frame = frame;
			g.frame_ts = rd64(col);
		}

		for (ch = g.ch_lo; ch <= g.ch_hi && ch < l->channels; ch++) {
			const uint8_t *p = px + (size_t)ch * l->px;
			uint32_t r = px_range_mm(l, p);

			if (r < g.min_range_mm || r > g.max_range_mm)
				continue;
			if (!best || r < best) {
				best = r;
				best_refl = px_refl(l, p);
			}
		}

		if (!best)
			continue;

		sector = (int)(((long)mid * g.sectors) / g.scan_width);
		if (sector < 0 || sector >= g.sectors)
			continue;

		zones_column(sector, best);

		/* Store centimetres so 16 bits covers the full 655 m range. */
		if (best / 10u < g.ring_min[sector]) {
			g.ring_min[sector] = (uint16_t)(best / 10u);
			g.ring_refl[sector] = best_refl;
		}
	}
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

static bool parse_zone(const char *s)
{
	double a0, a1;
	double range_m;
	struct zone *z;

	if (g.nzones >= MAX_ZONES)
		return false;
	if (sscanf(s, "%lf:%lf:%lf", &a0, &a1, &range_m) != 3)
		return false;

	z = &g.zones[g.nzones++];
	z->az_start_mdeg = (int)(a0 * 1000.0);
	z->az_end_mdeg = (int)(a1 * 1000.0);
	if (z->az_end_mdeg < z->az_start_mdeg)
		z->az_end_mdeg += 360000;	/* wraps through 0 deg */
	z->max_range_mm = (uint32_t)(range_m * 1000.0);
	z->active = false;
	return true;
}

static void usage(const char *argv0)
{
	fprintf(stderr,
"Usage: %s [options]\n"
"  -p, --port PORT          lidar UDP port to listen on (default 7502)\n"
"  -c, --channels N         pixels per column, e.g. 64 for OS-64 (default 64)\n"
"  -C, --columns N          columns per packet (default 16)\n"
"  -w, --scan-width N       columns per revolution: 512|1024|2048 (default 1024)\n"
"  -s, --sectors N          azimuth sectors in the published ring (default 360)\n"
"  -b, --channel-band LO:HI channel range to reduce over (default all)\n"
"  -m, --min-range M        ignore returns closer than this (default 0.3)\n"
"  -M, --max-range M        ignore returns further than this (default 200)\n"
"  -r, --relay HOST[:PORT]  forward every raw packet here (default port 7502)\n"
"  -o, --ring HOST[:PORT]   send the derived ring here (default port 7602)\n"
"  -z, --zone A0:A1:R       polar zone, degrees:degrees:metres (repeatable)\n"
"  -a, --action CMD         run 'CMD alarm' / 'CMD clear' on zone changes\n"
"  -S, --status PATH        JSON status file (default /var/run/ouster-edge.json)\n"
"  -I, --status-interval MS  how often to rewrite the status file (default 200)\n"
"  -E, --sse-port PORT      push each revolution as Server-Sent Events (0 = off)\n"
"  -f, --foreground         log to stderr instead of syslog\n"
"  -h, --help               this text\n", argv0);
}

int main(int argc, char **argv)
{
	static const struct option opts[] = {
		{ "port",         required_argument, NULL, 'p' },
		{ "channels",     required_argument, NULL, 'c' },
		{ "columns",      required_argument, NULL, 'C' },
		{ "scan-width",   required_argument, NULL, 'w' },
		{ "sectors",      required_argument, NULL, 's' },
		{ "channel-band", required_argument, NULL, 'b' },
		{ "min-range",    required_argument, NULL, 'm' },
		{ "max-range",    required_argument, NULL, 'M' },
		{ "relay",        required_argument, NULL, 'r' },
		{ "ring",         required_argument, NULL, 'o' },
		{ "zone",         required_argument, NULL, 'z' },
		{ "action",       required_argument, NULL, 'a' },
		{ "status",       required_argument, NULL, 'S' },
		{ "status-interval", required_argument, NULL, 'I' },
		{ "sse-port",     required_argument, NULL, 'E' },
		{ "foreground",   no_argument,       NULL, 'f' },
		{ "help",         no_argument,       NULL, 'h' },
		{ NULL, 0, NULL, 0 }
	};
	struct mmsghdr msgs[BATCH];
	struct iovec iov[BATCH];
	uint8_t (*bufs)[MAX_PKT];
	struct sockaddr_in from[BATCH];
	struct sockaddr_in addr;
	struct sigaction sa;
	struct pollfd pfd[2 + MAX_SSE];
	int sock, txsock, lfd = -1, rcvbuf = 4 * 1024 * 1024, opt, i;
	uint64_t last_status = 0;

	g.listen_port = 7502;
	g.channels = 64;
	g.columns = 16;
	g.scan_width = 1024;
	g.sectors = 360;
	g.ch_lo = 0;
	g.ch_hi = 127;
	g.min_range_mm = 300;
	g.max_range_mm = 200000;
	g.status_path = (char *)"/var/run/ouster-edge.json";
	g.status_ms = 200;
	g.sse_port = 7603;
	g.cur_frame = -1;
	g.last_mid = -1;

	while ((opt = getopt_long(argc, argv, "p:c:C:w:s:b:m:M:r:o:z:a:S:I:E:fh",
				  opts, NULL)) != -1) {
		switch (opt) {
		case 'p': g.listen_port = atoi(optarg); break;
		case 'c': g.channels = atoi(optarg); break;
		case 'C': g.columns = atoi(optarg); break;
		case 'w': g.scan_width = atoi(optarg); break;
		case 's': g.sectors = atoi(optarg); break;
		case 'b':
			if (sscanf(optarg, "%d:%d", &g.ch_lo, &g.ch_hi) != 2) {
				fprintf(stderr, "bad --channel-band '%s'\n", optarg);
				return 1;
			}
			break;
		case 'm': g.min_range_mm = (uint32_t)(atof(optarg) * 1000.0); break;
		case 'M': g.max_range_mm = (uint32_t)(atof(optarg) * 1000.0); break;
		case 'r':
			if (!parse_hostport(optarg, &g.relay_to, 7502)) {
				fprintf(stderr, "bad --relay '%s'\n", optarg);
				return 1;
			}
			g.have_relay = true;
			break;
		case 'o':
			if (!parse_hostport(optarg, &g.ring_to, 7602)) {
				fprintf(stderr, "bad --ring '%s'\n", optarg);
				return 1;
			}
			g.have_ring = true;
			break;
		case 'z':
			if (!parse_zone(optarg)) {
				fprintf(stderr, "bad --zone '%s'\n", optarg);
				return 1;
			}
			break;
		case 'a': g.action = optarg; break;
		case 'S': g.status_path = optarg; break;
		case 'I': g.status_ms = atoi(optarg); break;
		case 'E': g.sse_port = atoi(optarg); break;
		case 'f': g.foreground = true; break;
		case 'h': usage(argv[0]); return 0;
		default: usage(argv[0]); return 1;
		}
	}

	if (g.sectors < 1 || g.sectors > MAX_SECTORS) {
		fprintf(stderr, "--sectors must be 1..%d\n", MAX_SECTORS);
		return 1;
	}
	if (g.channels < 1 || g.channels > 128 || g.columns < 1 ||
	    g.columns > 64) {
		fprintf(stderr, "unsupported --channels/--columns\n");
		return 1;
	}
	if (g.status_ms < 20)
		g.status_ms = 20;
	if (g.ch_lo < 0 || g.ch_hi < g.ch_lo) {
		fprintf(stderr, "bad channel band\n");
		return 1;
	}

	if (!g.foreground)
		openlog("ouster-edge", LOG_PID, LOG_DAEMON);

	/* Deliberately not signal(): glibc and musl both install handlers with
	 * SA_RESTART, which silently restarts the blocking recvmmsg() below, so
	 * SIGTERM would never break the loop and 'service stop' would hang
	 * whenever the sensor is quiet. */
	sa.sa_handler = on_signal;
	sigemptyset(&sa.sa_mask);
	sa.sa_flags = 0;
	sigaction(SIGINT, &sa, NULL);
	sigaction(SIGTERM, &sa, NULL);
	signal(SIGPIPE, SIG_IGN);

	sock = socket(AF_INET, SOCK_DGRAM, 0);
	if (sock < 0) {
		logmsg(LOG_ERR, "socket: %s", strerror(errno));
		return 1;
	}

	/* A revolution is ~64 Mbit/s on an OS-64 at 1024x10; without a large
	 * receive buffer a scheduling hiccup shows up as missing columns. */
	if (setsockopt(sock, SOL_SOCKET, SO_RCVBUF, &rcvbuf, sizeof(rcvbuf)) < 0)
		logmsg(LOG_WARNING, "SO_RCVBUF: %s", strerror(errno));

	memset(&addr, 0, sizeof(addr));
	addr.sin_family = AF_INET;
	addr.sin_addr.s_addr = htonl(INADDR_ANY);
	addr.sin_port = htons((uint16_t)g.listen_port);
	if (bind(sock, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
		logmsg(LOG_ERR, "bind :%d: %s", g.listen_port, strerror(errno));
		return 1;
	}

	txsock = socket(AF_INET, SOCK_DGRAM, 0);
	if (txsock < 0) {
		logmsg(LOG_ERR, "tx socket: %s", strerror(errno));
		return 1;
	}
	fcntl(txsock, F_SETFL, fcntl(txsock, F_GETFL, 0) | O_NONBLOCK);

	bufs = malloc(sizeof(*bufs) * BATCH);
	if (!bufs) {
		logmsg(LOG_ERR, "out of memory");
		return 1;
	}

	/* Non-blocking: poll() below owns the waiting, so a quiet sensor still
	 * lets the status file refresh and SSE clients connect. */
	fcntl(sock, F_SETFL, fcntl(sock, F_GETFL, 0) | O_NONBLOCK);

	if (g.sse_port > 0) {
		lfd = sse_listen(g.sse_port);
		if (lfd < 0)
			logmsg(LOG_WARNING, "cannot listen on :%d for SSE: %s",
			       g.sse_port, strerror(errno));
		else
			logmsg(LOG_NOTICE, "SSE on :%d", g.sse_port);
	}

	zones_index();
	ring_reset();
	logmsg(LOG_NOTICE, "listening on :%d, %d ch x %d col, %d sectors, %d zones",
	       g.listen_port, g.channels, g.columns, g.sectors, g.nzones);

	while (!stop_requested) {
		int n, np = 0;

		pfd[np].fd = sock;
		pfd[np].events = POLLIN;
		pfd[np++].revents = 0;
		if (lfd >= 0) {
			pfd[np].fd = lfd;
			pfd[np].events = POLLIN;
			pfd[np++].revents = 0;
		}
		for (i = 0; i < g.nsse; i++) {
			pfd[np].fd = g.sse[i];
			pfd[np].events = 0;	/* only interested in hangups */
			pfd[np++].revents = 0;
		}

		if (poll(pfd, (nfds_t)np, g.status_ms) < 0 && errno != EINTR)
			logmsg(LOG_WARNING, "poll: %s", strerror(errno));

		if (lfd >= 0 && (pfd[1].revents & POLLIN))
			sse_accept(lfd);

		/* Reap readers that went away, so a closed tab does not leave a
		 * socket occupying a slot forever. */
		for (i = g.nsse - 1; i >= 0; i--) {
			int slot = (lfd >= 0 ? 2 : 1) + i;

			if (slot < np && (pfd[slot].revents & (POLLERR | POLLHUP | POLLNVAL)))
				sse_drop(i);
		}

		memset(msgs, 0, sizeof(msgs));
		for (i = 0; i < BATCH; i++) {
			iov[i].iov_base = bufs[i];
			iov[i].iov_len = MAX_PKT;
			msgs[i].msg_hdr.msg_iov = &iov[i];
			msgs[i].msg_hdr.msg_iovlen = 1;
			msgs[i].msg_hdr.msg_name = &from[i];
			msgs[i].msg_hdr.msg_namelen = sizeof(from[i]);
		}

		n = recvmmsg(sock, msgs, BATCH, 0, NULL);
		if (n < 0) {
			if (errno == EINTR)
				continue;
			if (errno == EAGAIN || errno == EWOULDBLOCK) {
				n = 0;		/* no lidar traffic this tick */
			} else {
				logmsg(LOG_ERR, "recvmmsg: %s", strerror(errno));
				break;
			}
		}

		for (i = 0; i < n; i++) {
			size_t len = msgs[i].msg_len;

			g.st.packets++;
			g.st.bytes += len;
			g.last_pkt_ms = now_ms();

			if (g.have_relay) {
				if (sendto(txsock, bufs[i], len, 0,
					   (struct sockaddr *)&g.relay_to,
					   sizeof(g.relay_to)) == (ssize_t)len)
					g.st.relayed++;
			}

			if (g.layout.profile == PROF_UNKNOWN) {
				if (!layout_detect(&g.layout, len, g.channels,
						   g.columns)) {
					if (!(g.st.bad_size++ % 600))
						logmsg(LOG_WARNING,
						       "unrecognised packet size %zu for %d ch x %d col - check --channels/--columns against the sensor metadata",
						       len, g.channels, g.columns);
					continue;
				}
				logmsg(LOG_NOTICE, "profile %s, %zu B/packet",
				       profile_name(g.layout.profile),
				       g.layout.pkt_size);
			}

			if (len != g.layout.pkt_size) {
				g.st.bad_size++;
				continue;
			}

			packet_process(bufs[i], len, txsock);
		}

		if (now_ms() - last_status >= (uint64_t)g.status_ms) {
			last_status = now_ms();
			status_write();
		}
	}

	logmsg(LOG_NOTICE, "stopping after %llu packets / %llu frames",
	       (unsigned long long)g.st.packets,
	       (unsigned long long)g.st.frames);
	status_write();
	while (g.nsse)
		sse_drop(0);
	if (lfd >= 0)
		close(lfd);
	free(bufs);
	close(sock);
	close(txsock);
	return 0;
}
