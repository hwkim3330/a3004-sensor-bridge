/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * afhds2a - build the packets a FlySky AFHDS 2A transmitter sends
 *
 * This is the half of "make the router act as the remote" that does not need an
 * A7105 in front of you: choosing a hop set, laying out the 38-byte frames, and
 * encoding channels. It is plain C with no platform dependencies, so it drops
 * into either a router talking to an A7105 over spidev or an MCU doing the same
 * over its own SPI.
 *
 * What it does NOT do is any radio work. See doc/AFHDS2A.md for what remains and
 * why none of it can be verified without the chip.
 *
 * Protocol facts here are cross-checked between two independent open
 * implementations, DeviationTX and DIY-Multiprotocol-TX-Module, because they
 * disagree in places and the disagreements matter.
 */

#ifndef AFHDS2A_H
#define AFHDS2A_H

#include <stdint.h>
#include <stddef.h>

/* Transmitted frames are 38 bytes. Telemetry coming back is 37 - the "37 byte
 * packet" seen in forum write-ups is the receive direction, and using it for
 * transmit is a whole evening wasted. */
#define AFHDS2A_TX_LEN		38
#define AFHDS2A_RX_LEN		37

/* AFHDS 2A carries 14 channels: bytes 9..36 inclusive is 28 bytes, two per
 * channel. This is the same channel count rc-ibus decodes on the way in. */
#define AFHDS2A_CHANNELS	14

/* 16 hop channels, sent to the receiver during bind. */
#define AFHDS2A_HOPS		16

/* Servo range in microseconds, as the receiver expects it. */
#define AFHDS2A_US_MIN		875
#define AFHDS2A_US_MID		1500
#define AFHDS2A_US_MAX		2125

/* Bind alternates between these two fixed channels. Neither is channel 0. */
#define AFHDS2A_BIND_CH_A	0x0d
#define AFHDS2A_BIND_CH_B	0x8c

/* Frame type byte at offset 0. */
enum afhds2a_type {
	AFHDS2A_TYPE_STICKS	= 0x58,
	AFHDS2A_TYPE_SETTINGS	= 0xaa,
	AFHDS2A_TYPE_FAILSAFE	= 0x56,
	AFHDS2A_TYPE_BIND1	= 0xbb,
	AFHDS2A_TYPE_BIND_N	= 0xbc,
};

/*
 * Derive the 16 hop channels from a seed.
 *
 * The two reference implementations use *different* derivations, which is the
 * most useful thing to know about this protocol: the transmitter picks the hop
 * set and hands it to the receiver in the bind frame, so it never has to match
 * what FlySky's own radios would have chosen. Any legal, well-spread set works.
 *
 * "Legal" here means: 1..168, no duplicates, and no more than five in any of the
 * four bands, so the set cannot bunch up in one part of the band and lose to a
 * WiFi carrier sitting on it.
 *
 * Deterministic for a given seed. Returns 0 on success.
 */
int afhds2a_hop_calc(uint32_t seed, uint8_t hop[AFHDS2A_HOPS]);

/* True if a hop set satisfies the constraints above. Exposed so a caller can
 * check a set that came from somewhere else. */
int afhds2a_hop_valid(const uint8_t hop[AFHDS2A_HOPS]);

/*
 * Build a sticks frame. channels_us are microseconds and are clamped, not
 * rejected: a control frame that is a bit wrong beats no control frame.
 * buf must be at least AFHDS2A_TX_LEN.
 */
void afhds2a_build_sticks(uint8_t *buf, const uint8_t txid[4],
			  const uint8_t rxid[4],
			  const uint16_t *channels_us, size_t nchannels);

/*
 * Build a failsafe frame. A channel with failsafe_us[i] == 0 is sent as 0xffff,
 * which tells the receiver to hold its own configured failsafe for that channel
 * rather than being commanded to a position.
 */
void afhds2a_build_failsafe(uint8_t *buf, const uint8_t txid[4],
			    const uint8_t rxid[4],
			    const uint16_t *failsafe_us, size_t nchannels);

/*
 * Build a bind frame. phase is 1..4. Phases 1..3 broadcast to 0xffffffff;
 * phase 4 addresses the receiver that answered, so rxid must be set by then.
 */
void afhds2a_build_bind(uint8_t *buf, int phase, const uint8_t txid[4],
			const uint8_t rxid[4],
			const uint8_t hop[AFHDS2A_HOPS]);

/* Read the channels back out of a sticks frame. For tests and for decoding a
 * capture; returns the number of channels written. */
size_t afhds2a_parse_sticks(const uint8_t *buf, uint16_t *channels_us,
			    size_t max);

#endif /* AFHDS2A_H */
