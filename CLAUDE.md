# FloodNet firmware: agent guards

Mistakes already made on this project, recorded so they are not repeated.

## Guards

- Never let the boot counter share an SNVS slot with anything Snooze writes.
  SnoozeAlarm writes `SNVS_LPGPR` (offset 0x68, which the RT1060 reference manual maps as the legacy alias of `LPGPR0`), so the boot counter lives in slot 3, `SNVS_LPGPR3`.
- The benchmark identity check compares the first 12 `BENCH` lines.
  `pio test -v` output echoes 4 of the existing rows a second time, so an unfiltered diff reports 4 lines even on an untouched tree.
- Plan code is not authoritative over the real code.
  `NmeaLineAssembler` completes a sentence on `'\r'`, which the plan's `FakeGps` fix-detection condition got wrong.
- Every number in the README must come from a committed artifact or a committed test.
  No probe-only claims.
- A harness assertion that cannot fail is not a check.
  Demonstrate that each new one fails on a deliberately broken local edit, then revert the edit.
- Nothing measured is tuned after it is seen.
