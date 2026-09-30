# FloodNet Milestone 5: Board Target for the Link Regression Tool

Date: 2026-09-30
Status: implemented
Parent spec: `docs/superpowers/specs/2026-09-20-floodnet-firmware-design.md`
Previous milestone: `docs/superpowers/specs/2026-09-30-milestone-4-hil-regression-design.md`

## Purpose

Milestone 4 built a link regression and signal-integrity tool that runs only against the host simulation, and wrote down a "Target contract" for a real Teensy node and gateway.
Milestone 5 builds that board target: the firmware output both boards need, and a host tool that captures both boards' USB serial and turns it into a per-scenario report.
It also delivers the receiver-side sequence-gap counting the README deferred to this milestone.

No hardware is available.
The board target is therefore built and tested against simulated serial transcripts, produced by the same `PowerRig` that milestones 3 and 4 use.
This milestone makes no new measurements.
Its claim is narrower: the tooling works end to end on simulated transcripts and agrees exactly with the simulation's own counts, so the first real numbers need only two boards and one `capture` run.

## Scope

In:

- A transmit line printed by `node_interrupt` and `node_duty_cycled`.
- The gateway's `REC` and `ERR,decode` lines, formatted by shared library code.
- A native test that writes simulated node and gateway transcripts.
- A Python host tool, `tools/hil/`, with `capture` and `analyze` commands.
- Unit tests for the tool, and a cross-check test against the simulation.
- CI steps for the tool.

Out:

- `node_polling`: `PowerRig` cannot run it, so it gets no transmit line.
- Any control of the radio link, attenuators or scenarios from software.
- An RSSI model.
- Recorded-trace replay of sensor input.

## Firmware

### Node: the transmit line

When a transmit ends, the node prints one line on its USB serial:

```text
TX,<node_id>,<boot_count>,<seq>,<millis>,<outcome>
```

- `outcome` is `ok` when the radio reported completion, and `timeout` when `InterruptSampler` aborted the transmit after `TX_TIMEOUT_MS` (5000 ms).
- A `timeout` frame may or may not have reached the air, so the host tool counts it separately and never as sent.
- `boot_count` is 0 for `node_interrupt`, which sends v0x01 and has no boot counter; `REC` prints 0 for it too, so the keys match.
- `millis` is the node's `millis()` at the moment the sampler observed the end of the transmit.

Mechanism:

- `lib/floodnet_hal` gains `class ITxListener { virtual void on_tx_end(uint16_t node_id, uint16_t boot_count, uint32_t seq, uint32_t now_ms, bool completed) = 0; };`.
- `InterruptSampler` gains `set_tx_listener(ITxListener *)`, default none, and remembers the `seq` and `boot_count` of the frame in flight so it can report them when the transmit ends.
- `DutyCycledNode` gains `set_tx_listener(ITxListener *)`, which passes the listener to its sampler.
- On the board, `src/main.cpp` installs a listener that prints the line with `Serial`.
- With no listener installed, the sampler's behaviour is unchanged; the milestone 1 and 2 benchmark rows, the milestone 3 `LIFE`/`DOWNTIME` lines and the milestone 4 `test_hil` lines must stay byte-identical.

### Gateway: shared line formatting

- `lib/floodnet_core` gains `size_t format_rec_line(const Packet &p, int16_t rssi, char *out, size_t out_len)` and `size_t format_decode_error_line(uint16_t count, char *out, size_t out_len)`.
- Each writes the line without its line ending and returns the length written, or 0 if `out` is too small.
- `format_rec_line` produces exactly today's field order: `REC,node_id,seq,time_ms,lat_1e7,lon_1e7,alt_mm,satellites,yaw_cd,pitch_cd,roll_cd,drops,crc_errors,rssi,gps_valid,imu_valid,boot_count,tx_timeouts,battery_mv`.
- `format_decode_error_line` produces `ERR,decode,<count>`.
- `src/gateway_main.cpp` prints these with `Serial.println`, so its output is unchanged; a unit test pins the exact string for a fully specified packet.
- `ERR,short` stays in `gateway_main.cpp`; the simulation cannot produce a short frame.

## Simulated boards

A native test, `test_hil_transcripts`, runs frozen scenarios on `PowerRig` and writes, per scenario, three files under `build/hil-transcripts/<scenario>/`:

- `node.log`: the `TX` lines, from an `ITxListener` installed through the same hook the firmware uses.
- `gateway.log`: a `REC` line for each accepted, non-duplicate frame, and an `ERR,decode` line for each frame that fails to decode, both from the shared formatters.
- `expected.json`: the rig's own counts for that run.

Every log line is prefixed with a timestamp and a space: `<t_ms> <line>`.
In the simulation `t_ms` is the rig's clock; in a capture it is milliseconds since the capture started, on the host.
RSSI is printed as 0 in the simulation, because nothing models it.

`PowerRig` gains a count of frames that failed to decode, and a way to install an `ITxListener` on whichever build it runs.

### Frozen scenarios

| Scenario | Build | Channel p | Fault | Run length |
|---|---|---|---|---|
| `interrupt_p0` | `interrupt` | 0 | none | 1 h |
| `interrupt_p1e-3` | `interrupt` | 1e-3 | none | 1 h |
| `interrupt_p1e-2` | `interrupt` | 1e-2 | none | 1 h |
| `duty_cycled_p0` | `duty_cycled` | 0 | none | 24 h |
| `duty_cycled_p1e-3` | `duty_cycled` | 1e-3 | none | 24 h |
| `duty_cycled_p1e-2` | `duty_cycled` | 1e-2 | none | 24 h |
| `duty_cycled_lost_completion` | `duty_cycled` | 0 | lost completion at 2 h | 24 h |
| `duty_cycled_hang` | `duty_cycled` | 0 | hang at 2 h | 24 h |

The channel uses milestone 4's `kChannelSeed`.
The fault scenarios exercise the `timeout` outcome and matching across a watchdog reboot.
The interrupt runs are 1 hour so the transcripts stay small; one hour is about 1200 transmits.

### `expected.json`

```json
{"scenario": "...", "build": "...", "p": 0.001,
 "tx_ok": 0, "tx_timeout": 0, "rec_lines": 0, "unmatched_rec": 0, "decode_errors": 0}
```

- `tx_ok` is the radio's completed-frame count (`FakeRadio::sent_count()`).
- `tx_timeout` is the sampler's `tx_timeouts()` summed over every boot.
- `rec_lines` is `deliveries().size()`, the `REC` lines the gateway printed.
- `unmatched_rec` is the number of those whose `(node_id, boot_count, seq)` differs from the frame that was transmitted: undetected corruption that altered the key. The rig computes it by decoding the transmitted payload as well as the received one.
- `decode_errors` is the rig's new decode-failure count.

## Host tool: `tools/hil/`

A Python package, `floodnet_hil`, with a `pyproject.toml`, `requires-python >= 3.10`, and `pyserial` as its only runtime dependency.

### `floodnet-hil capture --node PORT --gateway PORT --label NAME --seconds N [--baud 115200] [--out DIR]`

- Opens both serial ports and reads both at once until N seconds have passed.
- Writes `DIR/NAME/node.log` and `DIR/NAME/gateway.log`, each line prefixed with host milliseconds since the capture started.
- Writes nothing else and computes nothing: the raw logs are the artifact.
- A port that cannot be opened is a fatal error with the port name in the message.

### `floodnet-hil analyze DIR/NAME`

Parses both logs and prints one summary line:

```text
HIL,label,tx_ok,tx_timeout,accepted,delivery,decode_errors,short_frames,unmatched_rec,seq_gaps,rssi_min,rssi_median,rssi_max,malformed
```

- **Matching:** a `REC` is accepted when its `(node_id, boot_count, seq)` matches a `TX ... ok` line; repeated `REC` lines for one key count once. A `TX ... timeout` line never matches.
- `accepted` counts matched keys; `delivery` is `accepted / tx_ok`, printed as `-` when `tx_ok` is 0.
- `unmatched_rec` counts `REC` keys with no `TX ... ok` line. On hardware that means undetected corruption that altered the key, a gap in the node's log, or a frame the node timed out on that still reached the gateway.
- `seq_gaps` counts, per `(node_id, boot_count)`, sequence numbers missing between the lowest and highest `seq` the gateway accepted. It is receiver-side only and does not need the node's log.
- `rssi_*` are over accepted frames, printed as `-` when there are none; the median of an even count is the lower middle value, so it is always a value that was received.
- `malformed` counts lines in either log that fail to parse, including a truncated final line. They are never silently skipped.
- Lines that are neither `TX`, `REC` nor `ERR` (for example a boot banner) are ignored and not counted as malformed.

## Testing

- **Firmware (native, Unity):**
  - `format_rec_line` produces an exact pinned string for a fully specified packet, and returns 0 when the buffer is too small.
  - `InterruptSampler` with a listener reports `ok` with the right `seq` and `boot_count`, and `timeout` after a lost completion.
  - A sampler with no listener behaves identically (the byte-identity guards).
- **Host tool (`pytest`):** parsing, including malformed and truncated lines and ignored banners; matching across a reboot; duplicate `REC` lines; `timeout` accounting; gap counting; RSSI with zero, one and two accepted frames; `delivery` with zero `tx_ok`.
- **Cross-check:** for each frozen scenario, `analyze` on the simulated transcripts must equal `expected.json` exactly on `tx_ok`, `tx_timeout`, `unmatched_rec` and `decode_errors`, report `accepted` = `rec_lines - unmatched_rec`, and report `malformed` = 0.
  If any of these differs, the tool or the hook is wrong; nothing is adjusted to make them agree.
- Every new harness assertion must be shown to fail on a deliberately broken local edit, then the edit reverted (project `CLAUDE.md`).
- All four firmware images build with no new warnings.

### CI

The test job runs, in order: the HAL seam grep, `pio test -e native` (which writes the transcripts), `pip install -e tools/hil[test]`, `ruff check tools/hil`, and `pytest tools/hil`.
The cross-check fails, rather than skips, when the transcripts are missing, with a message naming the `pio test` command that writes them.
`build/` is added to `.gitignore`.

## Known limitations, stated in advance

- No board has run any of this; the `capture` command is exercised only by its unit tests against fake serial ports.
- The simulation prints RSSI 0; RSSI is meaningful only on real boards.
- Host timestamps in a capture are arrival times on the host, not radio times.
- A `timeout` frame is never counted as sent, although on hardware the gateway may have received it; the gateway's `REC` for it would then appear in `unmatched_rec`.
- `node_polling` has no transmit line.

## Revisions

Made while planning, before any code.

- **2026-09-30: runs end quiet.** A transcript run could stop between the radio completing a frame (which the gateway sees) and the sampler observing it (which prints the `TX` line), leaving a `REC` with no `TX`.
  Each transcript run therefore continues past its end time, one pass at a time, until no transmit is in flight, bounded at 10 s past the end.
- **2026-09-30: frames orphaned by a reboot.** A frame in flight when the watchdog resets the node can still complete in the radio, but the sampler that started it is gone, so no `TX` line is ever printed for it.
  The rig counts these as orphans, recognised by a `boot_count` in the transmitted frame that differs from the current node's.
  `expected.json` gains `tx_orphaned`; `tx_ok` is the radio's completed-frame count minus orphans, and `unmatched_rec` includes accepted orphans as well as key-altering corruption.
  The same can happen on hardware, and the README will say so.

Made during implementation.

- **2026-09-30: tests import from `src`.** The pytest configuration sets `pythonpath = ["src"]`, because on macOS the editable install's `.pth` file was marked hidden and Python 3.14 skips hidden `.pth` files.
  For the same reason the README tells bench users to install `floodnet-hil` with a regular `pip install ./tools/hil`.
- **2026-09-30: generated files ignored.** `.gitignore` gains `*.egg-info/`, `.pytest_cache/` and `.ruff_cache/`, which the editable install and the test tools create under `tools/hil/`.

Made after the final review.

- **2026-09-30: no orphan counting.** The planning revision "frames orphaned by a reboot" is withdrawn.
  In `node_duty_cycled` the power plan never has the IMU and the radio on together, and the only fault that triggers a watchdog reset is an IMU hang, so no frame can be in flight at a reset; the orphan counter could never count anything.
  The rig counts resets that strike with a transmit in flight instead, `test_hil_transcripts` asserts that count is 0, and `expected.json` has no `tx_orphaned` field; `tx_ok` is the radio's completed-frame count.
- **2026-09-30: `seq_gaps` over matched keys.** With a node log, `seq_gaps` uses only the keys that match a `TX ... ok` line, because a key that undetected corruption altered could add billions of phantom gaps; with no node log it uses every `REC` key.
- **2026-09-30: `capture` reopens a port that drops.** A reset makes the Teensy leave and rejoin USB; the capture discards the cut-off partial line, reopens the port every 0.5 s until it answers or the capture ends, and reports reconnects per port.
- **2026-09-30: `Serial.flush()` before deep sleep.** Whether plain `Serial` survives `Snooze.deepSleep` on a Teensy 4.1 is unverified; the README lists it for the first bench capture.
