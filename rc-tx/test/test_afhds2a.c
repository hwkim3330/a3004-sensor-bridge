// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Tests for the deterministic half of AFHDS 2A frame building.
 *
 * There is no A7105 on the bench, so the radio cannot be exercised. What can be
 * checked is everything that is pure data transformation - and that is worth
 * checking, because the two reference implementations disagree about the frame
 * length and about how hop channels are derived, so an implementer working from
 * one of them can be confidently wrong.
 *
 *   cc -O2 -Wall -Wextra -o test_afhds2a test_afhds2a.c ../src/afhds2a.c && ./test_afhds2a
 */

#include "../src/afhds2a.h"

#include <stdio.h>
#include <string.h>

static int fails;

static void ok(const char *name, int cond)
{
	printf("  %s  %s\n", cond ? "PASS" : "FAIL", name);
	if (!cond)
		fails++;
}

static void eq_int(const char *name, long got, long want)
{
	int c = got == want;

	printf("  %s  %s: got %ld want %ld\n", c ? "PASS" : "FAIL", name,
	       got, want);
	if (!c)
		fails++;
}

int main(void)
{
	const uint8_t txid[4] = { 0x12, 0x34, 0x56, 0x78 };
	const uint8_t rxid[4] = { 0xde, 0xad, 0xbe, 0xef };
	uint8_t buf[AFHDS2A_TX_LEN];
	uint8_t hop[AFHDS2A_HOPS], hop2[AFHDS2A_HOPS];
	uint16_t ch[AFHDS2A_CHANNELS], back[AFHDS2A_CHANNELS];
	int i;

	printf("AFHDS 2A frame builder\n");

	/* ---- frame geometry, the thing the two references disagree on ---- */
	printf("\n--- geometry ---\n");
	eq_int("tx frame length", AFHDS2A_TX_LEN, 38);
	eq_int("rx frame length", AFHDS2A_RX_LEN, 37);
	/* 14 channels x 2 bytes must exactly fill bytes 9..36 */
	eq_int("channel bytes fill 9..36", 9 + AFHDS2A_CHANNELS * 2, 37);

	/* ---- hop derivation ---- */
	printf("\n--- hop set ---\n");
	eq_int("hop_calc returns 0", afhds2a_hop_calc(0x12345678u, hop), 0);
	ok("hop set is legal", afhds2a_hop_valid(hop));

	/* deterministic for a seed */
	afhds2a_hop_calc(0x12345678u, hop2);
	ok("deterministic for one seed", memcmp(hop, hop2, sizeof(hop)) == 0);

	/* different seeds must not collide, or two transmitters in a room
	 * would share a hop set */
	afhds2a_hop_calc(0x87654321u, hop2);
	ok("different seed gives a different set",
	   memcmp(hop, hop2, sizeof(hop)) != 0);

	/* sweep a lot of seeds: every one must produce a legal set, and none
	 * may hit the guard */
	{
		int bad = 0, err = 0;
		uint32_t s;

		for (s = 1; s < 4000; s++) {
			uint8_t h[AFHDS2A_HOPS];

			if (afhds2a_hop_calc(s * 2654435761u, h) != 0) {
				err++;
				continue;
			}
			if (!afhds2a_hop_valid(h))
				bad++;
		}
		eq_int("3999 seeds: derivation failures", err, 0);
		eq_int("3999 seeds: illegal sets", bad, 0);
	}

	/* a hand-built invalid set must be rejected */
	{
		uint8_t bad_dup[AFHDS2A_HOPS];
		uint8_t bad_range[AFHDS2A_HOPS];
		uint8_t bad_band[AFHDS2A_HOPS];

		for (i = 0; i < AFHDS2A_HOPS; i++)
			bad_dup[i] = (uint8_t)(i * 2 + 1);
		bad_dup[5] = bad_dup[4];
		ok("duplicate rejected", !afhds2a_hop_valid(bad_dup));

		memcpy(bad_range, hop, sizeof(hop));
		bad_range[3] = 200;
		ok("out of range rejected", !afhds2a_hop_valid(bad_range));

		/* all sixteen inside the first band */
		for (i = 0; i < AFHDS2A_HOPS; i++)
			bad_band[i] = (uint8_t)(i + 1);
		ok("bunched into one band rejected",
		   !afhds2a_hop_valid(bad_band));
	}

	/* ---- sticks frame ---- */
	printf("\n--- sticks frame ---\n");
	for (i = 0; i < AFHDS2A_CHANNELS; i++)
		ch[i] = (uint16_t)(1000 + i * 50);
	afhds2a_build_sticks(buf, txid, rxid, ch, AFHDS2A_CHANNELS);

	eq_int("type byte", buf[0], AFHDS2A_TYPE_STICKS);
	ok("txid placed", memcmp(buf + 1, txid, 4) == 0);
	ok("rxid placed", memcmp(buf + 5, rxid, 4) == 0);
	eq_int("trailing byte", buf[37], 0x00);

	/* little-endian, so 1500 is dc 05 */
	{
		uint16_t mid[AFHDS2A_CHANNELS];

		for (i = 0; i < AFHDS2A_CHANNELS; i++)
			mid[i] = AFHDS2A_US_MID;
		afhds2a_build_sticks(buf, txid, rxid, mid, AFHDS2A_CHANNELS);
		eq_int("1500 low byte", buf[9], 0xdc);
		eq_int("1500 high byte", buf[10], 0x05);
	}

	/* round trip */
	afhds2a_build_sticks(buf, txid, rxid, ch, AFHDS2A_CHANNELS);
	eq_int("parse returns channel count",
	       (long)afhds2a_parse_sticks(buf, back, AFHDS2A_CHANNELS),
	       AFHDS2A_CHANNELS);
	ok("round trip", memcmp(ch, back, sizeof(ch)) == 0);

	/* clamping, not rejecting */
	{
		uint16_t wild[AFHDS2A_CHANNELS];

		for (i = 0; i < AFHDS2A_CHANNELS; i++)
			wild[i] = (i & 1) ? 60000 : 10;
		afhds2a_build_sticks(buf, txid, rxid, wild, AFHDS2A_CHANNELS);
		afhds2a_parse_sticks(buf, back, AFHDS2A_CHANNELS);
		eq_int("clamped low", back[0], AFHDS2A_US_MIN);
		eq_int("clamped high", back[1], AFHDS2A_US_MAX);
	}

	/* a short caller array centres the rest rather than sending zero,
	 * because zero is not a neutral servo position */
	{
		uint16_t two[2] = { 1200, 1800 };

		afhds2a_build_sticks(buf, txid, rxid, two, 2);
		afhds2a_parse_sticks(buf, back, AFHDS2A_CHANNELS);
		eq_int("supplied ch0", back[0], 1200);
		eq_int("supplied ch1", back[1], 1800);
		eq_int("unsupplied ch2 centred", back[2], AFHDS2A_US_MID);
		eq_int("unsupplied ch13 centred", back[13], AFHDS2A_US_MID);
	}

	/* ---- failsafe frame ---- */
	printf("\n--- failsafe frame ---\n");
	{
		uint16_t fs[AFHDS2A_CHANNELS];

		memset(fs, 0, sizeof(fs));
		fs[2] = 1000;
		afhds2a_build_failsafe(buf, txid, rxid, fs, AFHDS2A_CHANNELS);
		eq_int("type byte", buf[0], AFHDS2A_TYPE_FAILSAFE);
		eq_int("unset channel is 0xffff low", buf[9], 0xff);
		eq_int("unset channel is 0xffff high", buf[10], 0xff);
		eq_int("set channel low", buf[9 + 4], 0xe8);
		eq_int("set channel high", buf[10 + 4], 0x03);
	}

	/* ---- bind frames ---- */
	printf("\n--- bind frames ---\n");
	afhds2a_hop_calc(0x12345678u, hop);

	afhds2a_build_bind(buf, 1, txid, rxid, hop);
	eq_int("phase 1 type", buf[0], AFHDS2A_TYPE_BIND1);
	ok("phase 1 is broadcast",
	   buf[5] == 0xff && buf[6] == 0xff && buf[7] == 0xff && buf[8] == 0xff);
	eq_int("phase 1 marker", buf[9], 0x01);
	ok("hop table carried at 11..26",
	   memcmp(buf + 11, hop, AFHDS2A_HOPS) == 0);

	afhds2a_build_bind(buf, 2, txid, rxid, hop);
	eq_int("phase 2 type", buf[0], AFHDS2A_TYPE_BIND_N);
	eq_int("phase 2 marker", buf[9], 1);
	eq_int("phase 2 tail 27", buf[27], 0x01);
	eq_int("phase 2 tail 28", buf[28], 0x80);

	afhds2a_build_bind(buf, 4, txid, rxid, hop);
	ok("phase 4 is addressed to the receiver",
	   memcmp(buf + 5, rxid, 4) == 0);
	eq_int("phase 4 marker", buf[9], 3);

	/* out of range phases must clamp rather than write past the frame */
	afhds2a_build_bind(buf, 99, txid, rxid, hop);
	eq_int("phase clamped high", buf[9], 3);
	afhds2a_build_bind(buf, -5, txid, rxid, hop);
	eq_int("phase clamped low", buf[9], 0x01);

	printf("\n");
	if (fails) {
		printf("FAILED (%d)\n", fails);
		return 1;
	}
	printf("all checks passed\n");
	return 0;
}
