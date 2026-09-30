# FloodNet Milestone 6: Burst Bit Errors

Date: 2026-09-30
Status: design approved, not implemented
Parent spec: `docs/superpowers/specs/2026-09-20-floodnet-firmware-design.md`
Previous milestone: `docs/superpowers/specs/2026-09-30-milestone-5-board-target-design.md`
Extends: `docs/superpowers/specs/2026-09-30-milestone-4-hil-regression-design.md`

## Purpose

Milestone 4 measured the link and the CRC under independent bit errors, and its README states the obvious caveat: independent errors are the easy case for a CRC, and real LoRa errors after demodulation arrive in bursts.
Milestone 6 tests the burst case, within a single frame.

Every constant, sweep point, sample size, seed and pass criterion below is frozen here, before any of the milestone's code exists.
None of them is chosen to produce a particular number, and none may be changed after seeing output without a dated entry in "Revisions" that says so.

Milestone 4's 21 regression rows, its runner and its committed output are not touched.

## A property of the wire format, found while designing this

`crc16_ccitt()` in `lib/floodnet_core/src/packet.cpp` processes each byte most significant bit first, which is the bit order in which a 16-bit polynomial code is guaranteed to detect every error burst of 16 bits or fewer.
`encode_packet()` and `encode_packet_v2()` then store the CRC with `put_u16()`, which is little-endian: byte 43 is the CRC's low byte and byte 44 its high byte.
The channel, like the air, flips bits in transmission order: byte 0 to byte 44, most significant bit first within each byte.

So in transmission order, the two CRC bytes are swapped relative to the order the burst guarantee assumes.
The guarantee therefore holds for every burst lying wholly within bytes 0 to 42.
For a burst that touches bytes 43 or 44 it may not hold.
Whether any such burst is actually accepted has not been computed; the exhaustive check below finds out, and what a failure means is fixed here in advance.

## What the milestone measures

### Channel model: two-state Gilbert channel, reset per frame

A Gilbert channel has a good state G, in which no bit flips, and a bad state B, in which each bit flips independently with probability `h`.
It moves between states once per bit: G to B with probability `s`, B to G with probability `r`.

It is parameterised by the average bit-error rate *p* and the mean bad-state run length *L*, in bits:

- `h` = 0.5, the conventional choice: a bit in the bad state carries no information.
- `r` = 1 / *L*.
- The long-run share of bits in B is `pi_B` = *p* / `h` = 2*p*, so *p* is at most 0.5.
- `s` = `r` * `pi_B` / (1 - `pi_B`).

Each frame starts in B with probability `pi_B` and in G otherwise, drawn from the channel's own generator, and no state carries over from one frame to the next.
Starting from the long-run distribution, rather than always in G, is what makes every bit's flip probability exactly *p*, so the sweep's average BER equals milestone 4's.

*L* is the mean length of a bad-state run, not the span of the resulting error pattern: with `h` = 0.5, about half the bits in a run flip, and a run's first and last bits need not.

As in milestone 4, *p* is the post-demodulation bit-error rate the firmware sees, set as an input.
The channel does not model LoRa's coding rate, interleaving or symbol structure.

### Frozen constants

| Constant | Value |
|---|---|
| BER points *p* | 1e-6, 1e-5, 1e-4, 3e-4, 1e-3, 3e-3, 1e-2 (milestone 4's, without 0) |
| Mean burst lengths *L* | 2, 4, 8, 16, 32 bits |
| Bad-state flip probability `h` | 0.5 |
| Frame | `build_link_frame(i)` from `test/support/hil.hpp`, a v0x02 packet, 360 bits |
| Burst link frames per (*p*, *L*) | 100 000 |
| Burst integrity run | 1 000 000 frames at *p* = 1e-2, *L* = 32 |
| PRNG | splitmix64, seed `0x466C6F6F644E6574` (ASCII "FloodNet"), milestone 4's |

### Group 1: burst link sweep (35 rows, reported, not graded)

For each (*p*, *L*), 100 000 frames go through the burst channel into the real `decode_packet()`.
Each row reports frames accepted, the acceptance ratio, the independent-error ratio `(1 - p)^360` at the same *p* for comparison, frames corrupted, and frames with undetected corruption.

The rows are not graded because the Gilbert channel's frame-survival probability is not the quantity this milestone checks, and no pass band for it is claimed.
Clustering errors at a fixed average BER is expected to corrupt fewer frames, each more heavily; the rows show by how much.

### Group 2: exhaustive short-burst check (2 rows, graded)

Every error pattern whose span, from its first flipped bit to its last, is 16 bits or fewer is applied to frame 0, at every position in the 360-bit frame, and given to `decode_packet()`.
For a span of 1 there is one pattern per position; for a span `w` of 2 to 16 the first and last bits are set and the `w - 2` bits between them take every value.
This is deterministic and involves no random draws.

| Row | Patterns | Pass |
|---|---|---|
| `data`: every flipped bit in bytes 0 to 42 (bits 0 to 343) | 10 813 439 | 0 undetected |
| `crc_field`: at least one flipped bit in bytes 43 or 44 (bits 344 to 359) | 524 288 | 0 undetected |

The `data` row checks a theorem, so a failure there is a bug in the harness or the decoder.

The `crc_field` row is held to the same bar on purpose.
If it fails, that is the published finding: storing the CRC little-endian weakens burst detection at the end of the frame.
The fix, sending the CRC most significant byte first, changes the wire format, so it is recorded as a proposal for a future packet version and not made in this milestone.
Neither the row nor its criterion changes to turn it green.

### Group 3: burst integrity run (1 row, graded)

1 000 000 frames at *p* = 1e-2, *L* = 32, counting frames corrupted and frames with undetected corruption, with milestone 4's frozen bound: pass when undetected is at most `lambda + 3 sqrt(lambda) + 1`, `lambda` = corrupted / 65536 (`integrity_bound()` in `test/support/hil.hpp`).

### Regression total

3 graded rows: `data`, `crc_field`, integrity.
They are reported as `BURST_REG,all,-,passed,3` and are not added to milestone 4's 19 of 21.

## Components

### `test/support/burst_channel.hpp`

`BurstChannel(double p, double L, uint64_t seed)`, with the same interface as `NoisyChannel`: `size_t corrupt(uint8_t *frame, size_t len)` flips bits in place and returns how many; `frames()` counts calls.
It uses milestone 4's splitmix64 and draws run lengths geometrically rather than stepping bit by bit, so a million-frame run stays fast.
In a bad run, each bit flips with probability `h` from its own draw.

### `test/support/hil.hpp`

Gains `run_link_with(Channel &, uint32_t frames)`, the body of `run_link()` made generic over the channel, with `run_link()` reduced to a call to it so milestone 4's output cannot change.
Gains the exhaustive enumerator, returning patterns tried and undetected per region.

### `test/test_hil_burst/test_main.cpp`

The runner, a separate test so milestone 4's `test_hil` is unchanged.
It prints the lines below and asserts only harness validity, never a measured value.

### Output format

```text
BURST_LINK,ber,L,frames,accepted,ratio,independent_ratio,corrupted,undetected
BURST_EXHAUSTIVE,region,patterns,undetected,pass
BURST_INTEGRITY,ber,L,frames,corrupted,undetected,bound,pass
BURST_REG,all,-,passed,total
```

The first complete run is saved unchanged as `docs/results/milestone-6-burst-errors.txt`, and the README quotes its lines verbatim.

## Testing

- `BurstChannel` at the frozen seed:
  - *p* = 0 leaves a frame unchanged and draws nothing.
  - Over 100 000 frames at *p* = 1e-3, *L* = 8, the measured flip rate lies within 5% of *p*.
    The band is wider than milestone 4's three standard deviations because correlated flips inflate the variance, and 5% is fixed here, not fitted.
  - Over 100 000 frames at *p* = 0.25, *L* = 4, every bit position's flip rate lies within 5% of 0.25, so an off-by-one in a run length or a wrong bit mask fails, the check milestone 4 had to add in its Revisions.
  - Clustering is real: at *p* = 1e-2, *L* = 32, the share of corrupted frames is below the independent channel's `1 - (1 - p)^360` at the same *p*, and the mean flips per corrupted frame is above it.
  - The same seed gives the same flips twice.
- The exhaustive enumerator's pattern counts equal the table above, 10 813 439 and 524 288, computed independently in the test from the closed form.
- The enumerator's per-pattern check, given a hand-built pattern whose flips are the CRC polynomial itself (`0x11021`, a 17-bit span in the data bytes), counts it as undetected, so the check can report a failure.
- `run_link_with` on a `NoisyChannel` reproduces milestone 4's committed `LINK` lines.
- Every harness assertion is shown to fail on a deliberately broken local edit that is then reverted, per the project `CLAUDE.md`.

## Out of scope

- Bursts that span frames (fades), and anything they would do to downtime.
- A LoRa symbol-level error model.
- Changing the CRC byte order, whatever the `crc_field` row shows.
- The board target: `floodnet-hil` has no way to set a burst length, because on the air the burst structure is whatever the link does.

## Known limitations, stated in advance

- A Gilbert channel with `h` = 0.5 is one burst model among many; the results are for this model, not for LoRa.
- Resetting the state per frame means no burst crosses a frame boundary, by construction.
- The exhaustive check uses one frame; the CRC is linear, so whether a pattern is detected does not depend on the frame's contents, but the check does not show that by running other frames.
- Nothing here is validated on hardware.

## Revisions

None yet.
