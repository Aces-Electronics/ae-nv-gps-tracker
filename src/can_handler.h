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

// True if a frame carries a valid checksum: b7 is the XOR of bytes 0-6.
//
// CONFIRMED against the capture, not assumed. It holds on 18/18 complete frames
// across all nine IDs that carry the b6 rolling counter, and it independently
// explains the "fixed high nibble per ID" that b7 was seen to have: with a
// static payload XOR(b0..b5) is a constant K and b6 sweeps 0-F, so b7 covers
// exactly K&0xF0..K|0x0F -- which is what all nine of those IDs did, nibble for
// nibble. A brute force over every 8-bit polynomial finds only 0x01 (x^8+1),
// whose CRC *is* a byte-wise XOR, so no real CRC8 fits either.
//
// Only for the counter-carrying IDs: 0x010, 0x012, 0x102, 0x103, 0x110, 0x122,
// 0x230, 0x408, 0x410. The static config frames do not run the scheme at all --
// their b6 is a fixed byte, not a counter -- and this returns false for them.
bool canChecksumValid(const uint8_t* data, uint8_t dlc);

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

    // Coolant temperature, 0x102 b3.
    //
    // Identified by behaviour, which is why it is trusted without a gauge: over
    // a 65 s run it climbed 62->6E one count at a time and never once stepped
    // back, then kept climbing after shutdown. A monotonic rise while running
    // followed by a post-shutdown rise is heat soak, and nothing else on the
    // bus does it. Sent raw because the *scaling* is still unconfirmed -- the
    // standard raw-40 gives a believable 58->70 C, but 0.5 C/count fits the
    // warm-up rate about as well, and one gauge reading would settle it.
    bool    coolantValid = false;
    uint8_t coolantRaw   = 0;

    // 0x342 b4. Previously read as *the* engine temperature; it is not. It
    // jitters with rpm and falls after shutdown while the coolant byte rises,
    // so it is something with far less thermal mass -- exhaust is the guess.
    // Kept because it is still a real signal, just not the one it was labelled.
    bool    tempValid = false;
    uint8_t tempRaw  = 0;

    // iBR up/down position, 0x012 b2. A 1..9 step count, not a percentage.
    //
    // CONFIRMED: over two full up-and-down cycles the 32 step changes here
    // matched the 32 command pulses on 0x408 b0 one for one, with the position
    // trailing the command by a mean 24 ms. It reports with the engine stopped.
    // b2 is not multiplexed -- the slots on 0x012 never disagreed about it in
    // 200 windows -- so it can be read from any frame of this ID.
    bool    trimValid = false;
    uint8_t trimPos   = 0;    // 1..9, 0 means not seen

    // Engine hour meter, 0x342 b6:b7 big-endian, in MINUTES.
    //
    // CONFIRMED: 0x16E0 = 5856 min = 97h36m, which is exactly what the dash
    // read at the time of the capture. This is why 0x342 carries no checksum --
    // on the seven IDs without a rolling counter, b6/b7 are payload.
    bool     hoursValid    = false;
    uint16_t engineMinutes = 0;
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

// Captures frames for `seconds` into RAM and then dumps them as CSV.
//
// Pass `ids`/`nIds` to record only those IDs. The dump, not the capture, is the
// bottleneck: 30 s of the whole bus is ~29,000 frames and takes about two and a
// half minutes to print at 115200, which is long enough that it is tempting to
// skip the capture altogether -- and skipping it loses the one record that can
// answer a question the change watcher can only point at. Filtering to the two
// or three IDs under investigation keeps full fidelity and makes the dump
// short enough that there is no reason to go without it.
//
// Streaming at full rate is not an option: ~980 frames/s is roughly 39 kB/s of
// text against an 11.5 kB/s serial link, so a live dump would silently drop
// most of the bus and look like a quiet one. Buffer first, print after.
void canRawCapture(uint32_t seconds, const uint32_t* ids = nullptr, size_t nIds = 0);

// Time-series watch on the IDs carrying engine data, printed several times a
// second. Ranges say which bytes are alive; only a series says what they mean,
// because that is what can be held against a gauge reading.
void canWatchLoop();

// Learn, then record and narrate one interaction with the vehicle.
//
// This is canWatchChanges() and canRawCapture() fused, because keeping them
// apart was a trap: whichever ran first was the one recording, and the operator
// naturally acted on the *second* prompt, so three sessions in a row narrated a
// control being worked while the recorder had already stopped. One learn, one
// GO, one window that both records and narrates.
//
// The fusion also covers the watcher's blind spot. It hides any byte that moved
// during learning, which permanently hides both halves of a multiplexed signal
// -- the selector alternates every frame, so the selector and the multiplexed
// payload byte always look like noise. On this bus that hides 0x012 b1 and b4,
// which is exactly where a signal the watcher cannot see would live. The raw
// record has no such blind spot, so the narration is a convenience and the CSV
// is the evidence.
//
// Prints the excluded bytes up front. A blind spot you can see is a lead.
void canProbeControl(uint32_t seconds, const uint32_t* ids = nullptr,
                     size_t nIds = 0, uint32_t learnMs = 8000);

// Watches for bytes that were stable and then move, and prints only those.
//
// The instrument for "I am going to operate a control, tell me what changed."
// Hunting a rider input by eye does not work: the bus is 16 IDs x 8 bytes at
// ~980 frames/s, and a control that was never touched during a capture is
// perfectly static, which is indistinguishable from a configuration byte.
//
// So it learns first. For `learnMs` it watches every byte with the vehicle
// left alone and marks any that moves as noise -- engine signals, counters,
// checksums, all of it, with no hardcoded list to go stale. After that it
// reports only the bytes that held still through learning and then changed,
// as `0xID bN: XX -> YY`. Operate one control at a time and the output is a
// short list naming that control's bytes.
//
// Learning is the whole trick, so leave the machine genuinely undisturbed for
// those seconds. Anything moving then is invisible for the rest of the run.
void canWatchChanges(uint32_t learnMs = 8000);

// Boots straight into detect-then-sniff and never returns. This is what the
// can-sniffer build runs instead of the tracker's normal cycle.
void canSnifferLoop();

const char* canBitrateName(CanBitrate r);
