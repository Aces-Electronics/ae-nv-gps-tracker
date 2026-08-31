#pragma once

#include <Arduino.h>

// Bit rates worth trying on a powersports bus, in the order canDetectBitrate()
// tries them.
enum class CanBitrate {
    Unknown,
    Rate250k,
    Rate500k,
    Rate125k,
};

// Most of what a first look at an unknown bus needs: which IDs exist, how often
// each one talks, and what the last payload looked like.
struct CanFrameSummary {
    uint32_t id       = 0;
    bool     extended = false;
    uint8_t  dlc      = 0;
    uint8_t  lastData[8] = {0};
    uint32_t count    = 0;

    // Per-byte range across every frame seen for this ID. A byte that never
    // moves is a constant, a header or padding; a byte that does is carrying
    // something. On a bus of 16 IDs x 8 bytes that distinction is the whole
    // difference between a wall of hex and a short list worth decoding.
    uint8_t  minData[8] = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};
    uint8_t  maxData[8] = {0};
};

// How many distinct IDs the table can hold before it stops taking new ones.
static const size_t CAN_MAX_IDS = 64;

// Brings the transceiver out of standby and starts the TWAI controller in
// LISTEN-ONLY mode.
//
// Listen-only never drives the bus -- not even the ACK bit -- so a wrong guess
// about the bit rate cannot corrupt traffic on a running vehicle. Everything in
// this file stays listen-only for that reason; transmitting is a decision to
// take deliberately later, not something to fall into during bring-up.
bool canBeginListenOnly(CanBitrate rate);

// Stops the controller and puts the transceiver back into standby.
void canEnd();

// Listens at each candidate rate in turn and keeps the first that hears valid
// frames. Returns Unknown if nothing was heard anywhere, which on a live bus
// means the wiring, not the rate. Leaves the controller stopped.
CanBitrate canDetectBitrate(uint32_t msPerRate = 750);

// Accumulates unique IDs for `durationMs` into the table. Requires
// canBeginListenOnly() first. Returns the number of distinct IDs known so far.
size_t canSniff(uint32_t durationMs);

// Drains and records for `ms` without printing anything. canSniff() logs a
// summary, which is noise inside a loop that is emitting CSV.
void canDrain(uint32_t ms);

void canResetSeen();
size_t canSeenCount();
const CanFrameSummary* canSeen(size_t index);
void canLogSeen();

// Bench self-test: does the CAN hardware work at all, with no other node?
//
// TWAI_MODE_NO_ACK transmits without requiring an acknowledgement, and the
// transceiver loops the bus back into its own receiver, so a frame that returns
// has proved controller -> GPIO17 -> driver -> bus -> receiver -> GPIO18 ->
// controller. That is the whole path bar another node.
//
// Caveat worth knowing before reading a failure as broken hardware: JP1 has
// been cut, so the bus carries no termination. A lone node usually still hears
// itself, but a marginal result here is not proof of a fault.
bool canSelfTest(CanBitrate rate = CanBitrate::Rate250k);

// Engine RPM from a 0x102 frame, or -1 if it is not one.
//
// Bosch ME17.8.5, b0/b1 big-endian at a quarter rpm per bit. CONFIRMED against
// the tacho on a running ski, not inferred: idle decoded to a 1455 mean against
// a gauge reading 1400-1500, and a rev decoded to 6554 against a reported ~6500.
// Both frames captured with the engine stopped read 0000.
int32_t canDecodeRpm(const CanFrameSummary& f);

// What one short listen on the ski's bus yielded.
struct EngineData {
    bool    busAlive = false; // any frame at all -- i.e. is the ski awake
    bool    rpmValid = false;
    int32_t rpm      = 0;
    bool    tempValid = false;
    uint8_t tempRaw  = 0;     // 0x342 b4; scaling still unknown, sent raw
};

// Listens for `listenMs` and pulls out what the tracker reports. Cheap enough
// to do on every wake: the bus runs at ~980 frames/s, so a second is a
// thousand frames and everything of interest repeats at 50-100Hz.
//
// The bit rate is not swept here. It is known to be 500 kbit/s on this ski
// (docs/can/seadoo-can.md) and a sweep would cost three times as long on the
// overwhelmingly common case of a sleeping bus. busAlive false just means the
// ignition is off, which is the normal state for a parked ski.
EngineData canReadEngine(uint32_t listenMs = 1000);

// Captures every frame for `seconds` into RAM and then dumps it as CSV.
//
// Streaming at full rate is not an option: ~980 frames/s is roughly 39 kB/s of
// text against an 11.5 kB/s serial link, so a live dump would silently drop
// most of the bus and look like a quiet one. Buffer first, print after.
void canRawCapture(uint32_t seconds);

// Time-series watch on the IDs carrying engine data, printed several times a
// second. Ranges say which bytes are alive; only a series says what they mean,
// because that is what can be held against a gauge reading.
void canWatchLoop();

// Boots straight into detect-then-sniff and never returns. This is what the
// can-sniffer build runs instead of the tracker's normal cycle.
void canSnifferLoop();

const char* canBitrateName(CanBitrate r);
