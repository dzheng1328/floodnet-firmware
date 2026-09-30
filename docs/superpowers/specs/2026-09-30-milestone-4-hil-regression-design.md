# FloodNet Milestone 4: Link Regression and Signal-Integrity Tool

Date: 2026-09-30
Status: implemented
Parent spec: `docs/superpowers/specs/2026-09-20-floodnet-firmware-design.md`
Previous milestone: `docs/superpowers/specs/2026-09-29-milestone-3-power-management-design.md`

## Purpose

The original lab work included a hardware-in-the-loop regression and signal-integrity tool: a rig that sent known packets over the LoRa link and counted bit and packet errors, and a CI-style runner that drove firmware through scripted scenarios and passed or failed it on telemetry.
Nothing in this repository reconstructs it yet.

Milestone 4 rebuilds that tool against FloodNet.
No board is available, so the tool runs against the host simulation now and defines a target contract that a real Teensy node and gateway can satisfy later.

This milestone reports what it measures.
Every constant, sweep point, sample size, seed and pass criterion below is frozen here, before any of the tool's code exists.
None of them is chosen to produce a particular number, and none may be changed after seeing output without a dated entry in "Revisions" that says so.

## What the tool measures

### Channel model: raw bit-error probability

Noise enters as a per-bit flip probability *p*, applied independently to every bit of a transmitted frame.
The tool makes no claim about LoRa radio physics: it does not model SNR, spreading-factor processing gain, interleaving or LoRa's own forward error correction.
The question it answers is "given this bit-error rate on the air, what does the firmware and gateway do?"

On a real board the corresponding input would be the measured link, and the gateway's RSSI column would be the noise observation.
The host tool does not measure noise; it applies it.

### Frozen sweep

| Constant | Value |
|---|---|
| BER points *p* | 0, 1e-6, 1e-5, 1e-4, 3e-4, 1e-3, 3e-3, 1e-2 |
| Frame length | `PACKET_SIZE` = 45 bytes = 360 bits |
| Link frames per BER point | 100 000 |
| Undetected-corruption frames at *p* = 1e-2 | 1 000 000 |
| End-to-end run length | 24 simulated hours per build per BER point |
| Builds | `interrupt` and `duty_cycled`, the two builds `PowerRig` runs |
| PRNG | splitmix64, seed `0x466C6F6F644E6574` (ASCII "FloodNet") |

`node_polling` is out of scope because `PowerRig` cannot run it.

### Metric 1: link delivery ratio

For each BER point, `N` frames are encoded, passed through the channel, and given to the real `decode_packet()`.

- `accepted` is the number `decode_packet()` returns true for.
- Link delivery ratio is `accepted / N`.
- Expected ratio is `q = (1 - p)^360`, the probability that no bit flips.

Under random independent errors this ratio is a property of the channel and the frame length, not of the firmware build.
It is reported once, not per build, and serves as the tool's self-check: if the measured ratio disagrees with `q`, the channel model or the harness is wrong.

### Metric 2: undetected corruption

A frame is an undetected corruption when at least one bit flipped, `decode_packet()` accepted it, and its bytes differ from the transmitted frame.
This is counted at every BER point, and over 1 000 000 frames at *p* = 1e-2.

This is the signal-integrity result: how often corruption reaches the application disguised as a valid packet.
It depends on the CRC-16/CCITT-FALSE check and the magic, version and length checks together.

### Metric 3: end-to-end record delivery, per build (the headline)

For each build and each BER point, a `PowerRig` runs 24 simulated hours with the channel between the node's radio and the gateway's decode and dedup, and no faults.

- Numerator: packets the gateway accepts and does not discard as duplicates, whatever their `gps_valid`.
- Denominator: sequence numbers the node issued, summed over every boot, i.e. every record the node queued for transmission.
- End-to-end delivery is numerator divided by denominator.

Queue drops, transmit timeouts and channel loss all count against it, so this is where the builds can legitimately differ.
It is a measurement, not a pass/fail check.

### Metric 4: regression pass rate

A fixed scenario table, each row with a pass criterion stated below.
The report prints one line per scenario and a pass count per build and per group.

## Frozen scenario table and pass criteria

### Link group (build-independent, 8 rows)

One row per BER point.
PASS when `|accepted - N q| <= 3 sqrt(N q (1 - q))`, the measured count within three standard deviations of the binomial expectation.
At *p* = 0 this reduces to `accepted == N`.

### Integrity group (build-independent, 1 row)

The 1 000 000-frame run at *p* = 1e-2.
Let `C` be the number of frames with at least one flipped bit, and `lambda = C / 65536`, the count a 16-bit check with a uniformly distributed residue would pass.
PASS when `undetected <= lambda + 3 sqrt(lambda) + 1`.

This bound is an approximation stated in advance, not a derived property of this CRC.
If the count exceeds it, the row fails and the README reports the failure and the count.

### Transparency group (per build, 2 rows)

A 24-hour run with the channel installed at *p* = 0 must produce a gateway record list identical to the same run with no channel.
PASS when the two lists are equal field by field.
This proves the channel hook changes nothing when it adds no noise.

### Fault group (per build, 10 rows)

The five milestone 3 experiment 2 fault cases, run exactly as `run_experiment_2_fault()` runs them, with no channel installed.
PASS when the run ends with the node not down (`DowntimeReport::down_at_end` false), the same test as milestone 3's `recovered` column.

These results are already published by milestone 3: `duty_cycled` passes all five and `interrupt` passes three.
This group re-checks them as regressions; it is not new evidence.

### End-to-end group

The 16 end-to-end runs (2 builds x 8 BER points) are reported, not graded.

### Totals

Graded rows: 8 link + 1 integrity + 2 transparency + 10 fault = 21.
The report prints passes out of 21 overall and passes per build for the per-build groups.

## Components

Everything new is host-side.
No file under `lib/` or `src/` changes.

### `test/support/noisy_channel.hpp`

`NoisyChannel(double p, uint64_t seed)` with one method, `size_t corrupt(uint8_t *frame, size_t len)`, which flips bits in place and returns how many it flipped.
It samples the gap to the next flipped bit from a geometric distribution rather than drawing once per bit, so 1 000 000 frames stay fast.
At *p* = 0 it never touches the frame and never draws from the generator.
At *p* = 1 every bit flips.

### `PowerRig` channel hook

`PowerRig::set_channel(NoisyChannel *)`, default none.
When a channel is set, `collect_delivery()` copies the transmitted payload, corrupts the copy, and decodes the copy.
When none is set, `collect_delivery()` is byte-for-byte the current code path.

`PowerRig` also gains `records_queued()`, the sum of `next_seq()` over every boot including the current one, which is Metric 3's denominator.

### `test/test_hil/test_main.cpp`

The runner.
It executes the scenario table in order, prints the lines below, and asserts only harness validity (see Testing), never a measured value.

### Output format

```text
LINK,ber,frames,accepted,expected_accepted,ratio,expected_ratio,undetected,pass
INTEGRITY,ber,frames,corrupted,undetected,bound,pass
TRANSPARENT,build,records_with_channel,records_without,identical,pass
FAULT,build,fault,recovered,pass
E2E,build,ber,records_queued,accepted_unique,delivery
REG,group,build,passed,total
```

`build` is `-` for build-independent rows.
The first complete run is saved unchanged as `docs/results/milestone-4-hil-regression.txt`, and the README quotes its lines verbatim.

## Target contract, for a real board later

A target is anything that can run a scenario and report the counters the output lines need.
The host target is `PowerRig` plus `NoisyChannel`.

A board target would be one Teensy running `node_duty_cycled` or `node_interrupt` and one running the gateway, with a host script reading the gateway's `REC` lines over USB serial.
It must report, per scenario: frames transmitted (from the node's sequence numbers), frames accepted (from `REC` lines), decode errors (the gateway's `ERR,decode` count), and RSSI per accepted frame.
Its channel is the real air, optionally worsened by attenuators; *p* is then measured, not set.

The board target is not built in this milestone.
The README says so.

## Testing

- `NoisyChannel`: *p* = 0 leaves a frame unchanged and draws nothing; *p* = 1 flips all 360 bits; at the frozen seed and *p* = 1e-3 over 100 000 frames, the measured flip rate lies within three standard deviations of 1e-3.
- Undetected-corruption detector: a hand-built frame with a corrupted payload and a CRC recomputed to match is counted as undetected; a frame corrupted without recomputing the CRC is counted as rejected.
- Every harness assertion must be shown to fail on a deliberately broken local edit that is then reverted, per the project `CLAUDE.md`.
- The milestone 1 and 2 benchmark rows and the milestone 3 LIFE and DOWNTIME lines must stay byte-identical, because `PowerRig` changes.

## Out of scope

- The board target and any attenuator rig.
- SNR-based noise models.
- The polling build.
- Forward error correction, retransmission or acknowledgements.
- Burst-error channel models; errors here are independent per bit.

## Known limitations, stated in advance

- Independent bit errors are the easiest case for a CRC; real LoRa errors after demodulation come in bursts.
- LoRa's own coding rate and interleaving are not modeled, so the BER here is the post-demodulation BER the firmware sees, not the raw channel BER.
- The fault group re-runs milestone 3 results; its pass counts add no new evidence.
- Nothing here is validated on hardware.

## Revisions

Both were made during implementation, before the one kept run of `test_hil`, and neither changes a sweep point, sample size, seed or pass criterion.

- **2026-09-30: one extra channel test at *p* = 0.5.** An off-by-one in the geometric gap, used as a deliberate-break check, passed every planned channel test, because at *p* = 1e-3 it moves the flip rate by about 0.1%, inside three standard deviations.
  `test_every_bit_position_flips_at_rate_p` checks the flip rate of each bit position at *p* = 0.5, where that off-by-one or a wrong bit mask moves the rate far outside the band.
- **2026-09-30: plain comparisons instead of Unity's double assertions.** This build of Unity has double precision disabled, so `TEST_ASSERT_EQUAL_DOUBLE` fails on every call; the same checks are written as `TEST_ASSERT_TRUE` comparisons.
