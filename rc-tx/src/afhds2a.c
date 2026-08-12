// SPDX-License-Identifier: GPL-2.0-or-later
/* See afhds2a.h and doc/AFHDS2A.md. */

#include "afhds2a.h"

#include <string.h>

/* Both reference implementations advance their generator with this constant.
 * It is the Numerical Recipes LCG, not anything FlySky-specific - more evidence
 * that the hop derivation is the implementer's choice and not part of the
 * protocol. */
#define LCG_MUL		0x0019660du
#define LCG_ADD		0x3c6ef35fu

#define BANDS		4
#define BAND_SPAN	42		/* 4 x 42 = 168 */
#define MAX_PER_BAND	5

static void put16(uint8_t *p, uint16_t v)
{
	p[0] = (uint8_t)(v & 0xff);
	p[1] = (uint8_t)(v >> 8);
}

static uint16_t get16(const uint8_t *p)
{
	return (uint16_t)p[0] | ((uint16_t)p[1] << 8);
}

static uint16_t clamp_us(uint16_t v)
{
	if (v < AFHDS2A_US_MIN)
		return AFHDS2A_US_MIN;
	if (v > AFHDS2A_US_MAX)
		return AFHDS2A_US_MAX;
	return v;
}

static int band_of(uint8_t ch)
{
	int b = (ch - 1) / BAND_SPAN;

	return b > BANDS - 1 ? BANDS - 1 : b;
}

int afhds2a_hop_valid(const uint8_t hop[AFHDS2A_HOPS])
{
	int per_band[BANDS] = { 0 };
	int i, j;

	for (i = 0; i < AFHDS2A_HOPS; i++) {
		if (hop[i] < 1 || hop[i] > 168)
			return 0;
		for (j = 0; j < i; j++)
			if (hop[j] == hop[i])
				return 0;
		per_band[band_of(hop[i])]++;
	}
	for (i = 0; i < BANDS; i++)
		if (per_band[i] > MAX_PER_BAND)
			return 0;
	return 1;
}

int afhds2a_hop_calc(uint32_t seed, uint8_t hop[AFHDS2A_HOPS])
{
	int per_band[BANDS] = { 0 };
	uint32_t rnd = seed;
	int idx = 0;
	long guard = 0;

	memset(hop, 0, AFHDS2A_HOPS);

	while (idx < AFHDS2A_HOPS) {
		uint8_t cand;
		int band, dup, i;

		/* A bounded loop, because the constraints below reject
		 * candidates and an unlucky seed must not spin forever. The
		 * bound is far above what a working seed needs. */
		if (++guard > 100000)
			return -1;

		rnd = rnd * LCG_MUL + LCG_ADD;
		cand = (uint8_t)((rnd >> (idx % 32)) % 168u) + 1;

		/* Keep everything on one parity, which spaces the set by two
		 * channels and is what the reference implementations do. */
		if (((cand ^ (uint8_t)seed) & 0x01) == 0)
			continue;

		dup = 0;
		for (i = 0; i < idx; i++)
			if (hop[i] == cand)
				dup = 1;
		if (dup)
			continue;

		/* Spread across the band. Without this a seed can put most of
		 * the set inside one 20 MHz slice, which is the width of the
		 * WiFi channel this router is also transmitting on. */
		band = band_of(cand);
		if (per_band[band] >= MAX_PER_BAND)
			continue;

		per_band[band]++;
		hop[idx++] = cand;
	}
	return 0;
}

static void header(uint8_t *buf, uint8_t type, const uint8_t txid[4],
		   const uint8_t rxid[4])
{
	memset(buf, 0, AFHDS2A_TX_LEN);
	buf[0] = type;
	memcpy(buf + 1, txid, 4);
	memcpy(buf + 5, rxid, 4);
}

void afhds2a_build_sticks(uint8_t *buf, const uint8_t txid[4],
			  const uint8_t rxid[4],
			  const uint16_t *channels_us, size_t nchannels)
{
	size_t i;

	header(buf, AFHDS2A_TYPE_STICKS, txid, rxid);

	for (i = 0; i < AFHDS2A_CHANNELS; i++) {
		uint16_t v = i < nchannels ? clamp_us(channels_us[i])
					   : AFHDS2A_US_MID;

		/* A channel the caller did not supply is centred rather than
		 * left at zero: zero is not a neutral servo position. */
		put16(buf + 9 + i * 2, v);
	}
	buf[37] = 0x00;
}

void afhds2a_build_failsafe(uint8_t *buf, const uint8_t txid[4],
			    const uint8_t rxid[4],
			    const uint16_t *failsafe_us, size_t nchannels)
{
	size_t i;

	header(buf, AFHDS2A_TYPE_FAILSAFE, txid, rxid);

	for (i = 0; i < AFHDS2A_CHANNELS; i++) {
		uint16_t v = i < nchannels ? failsafe_us[i] : 0;

		/* 0xffff means "no commanded position": the receiver falls back
		 * to whatever it has been configured to do. That is the right
		 * default, since a wrong commanded failsafe is worse than
		 * letting the receiver decide. */
		put16(buf + 9 + i * 2, v ? clamp_us(v) : 0xffff);
	}
	buf[37] = 0x00;
}

void afhds2a_build_bind(uint8_t *buf, int phase, const uint8_t txid[4],
			const uint8_t rxid[4],
			const uint8_t hop[AFHDS2A_HOPS])
{
	static const uint8_t broadcast[4] = { 0xff, 0xff, 0xff, 0xff };
	int i;

	if (phase < 1)
		phase = 1;
	if (phase > 4)
		phase = 4;

	/* Phases 1..3 are broadcast; only phase 4 is addressed to the receiver
	 * that answered. */
	header(buf, phase == 1 ? AFHDS2A_TYPE_BIND1 : AFHDS2A_TYPE_BIND_N,
	       txid, phase == 4 ? rxid : broadcast);

	buf[9] = phase == 1 ? 0x01 : (uint8_t)(phase - 1);
	buf[10] = 0x00;

	/* This is the part that makes the hop derivation a local decision: the
	 * set is handed to the receiver here. */
	for (i = 0; i < AFHDS2A_HOPS; i++)
		buf[11 + i] = hop[i];

	for (i = 27; i < AFHDS2A_TX_LEN; i++)
		buf[i] = 0xff;
	if (phase != 1) {
		buf[27] = 0x01;
		buf[28] = 0x80;
	}
}

size_t afhds2a_parse_sticks(const uint8_t *buf, uint16_t *channels_us,
			    size_t max)
{
	size_t i, n = max < AFHDS2A_CHANNELS ? max : AFHDS2A_CHANNELS;

	if (buf[0] != AFHDS2A_TYPE_STICKS)
		return 0;
	for (i = 0; i < n; i++)
		channels_us[i] = get16(buf + 9 + i * 2);
	return n;
}
