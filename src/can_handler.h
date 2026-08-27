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

void canResetSeen();
size_t canSeenCount();
const CanFrameSummary* canSeen(size_t index);
void canLogSeen();

// Boots straight into detect-then-sniff and never returns. This is what the
// can-sniffer build runs instead of the tracker's normal cycle.
void canSnifferLoop();

const char* canBitrateName(CanBitrate r);
