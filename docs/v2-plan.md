# SHIFTR for Saris v2: architecture and plan

Status: **draft for review**, branch `v2`, based on `v1.6.1-ftms` (`saris-ftms` stays the
frozen v1 line and only gets bug fixes).

## Goals

1. **Power2Max NG as a second sensor.** Connect over Bluetooth alongside the trainer
   and read its power, cadence, balance and torque.
2. **Complete logging to the Zwift PC.** Every second, everything the bridge knows, plus
   events (shifts, Zwift commands, trainer commands, connects/drops), recorded by a
   browser page on the PC into a CSV file.
3. **Better power calibration to Zwift.** First learn from logs which meter is right,
   then choose in Settings what Zwift receives: trainer, Power2Max, or the trainer
   corrected by a learned factor.

Decisions so far: the Power2Max isn't connected to anything indoors; logging goes
through a browser page; the reference meter is decided from the logs; the power Zwift
receives is selectable in Settings.

## What we know about the hardware

| | Saris H1 (trainer) | Power2Max NG |
|---|---|---|
| Radio | BLE (FTMS, CPS, FE-C) | ANT+ and BLE (Cycling Power Service) |
| Measures | Power at the hub, after the chain | Torque at the chainring spider, before the chain |
| Update rate | ~4 Hz notifications | 1 Hz |
| Cadence | Estimated from flywheel speed, or CPS crank data | Measured, no magnet |
| Zero / calibration | Spindown (Saris app; FTMS op 0x13 if supported) | Auto zero after 2 s without force; manual zero optional |
| Stated accuracy | ±2 % | ±1 % |
| Connections | One BLE controller | BLE normally allows one connection; **ANT+ broadcasts to any number of receivers at the same time** |

Consequences for the design:

- With perfect meters the trainer should read **2–4 % lower** than the Power2Max
  (chain and bearing losses between spider and hub). A 10 % gap is well outside that,
  so one of the two is off by about 6–8 %, and logging should tell us which kind of
  error it is (see Calibration).
- The Power2Max sleeps after ~2 min without pedalling and only advertises for pairing
  for about a minute after it wakes. The bridge must keep scanning for its stored
  address while it is missing and connect as soon as it advertises.
- Once the bridge holds the Power2Max's BLE connection, a phone app can't connect to it
  over BLE, but a **Garmin/Wahoo head unit can still record it over ANT+**. That gives
  a free independent recording to check the bridge's logs against.
- NimBLE allows 3 simultaneous connections by default: trainer + Power2Max fits.

## Architecture

```
            BLE                     ESP32 (WT32-ETH01)                    Ethernet
 Saris H1 ───────► TrainerLink ──┐                                ┌─► DirCon ──► Zwift
                                 ├─► RideData ─► PowerOutput ─────┤
 Power2Max ──────► SensorLink ───┘     │  (all values,      │     └─► ERG / VS control
                   (CPS)               │   freshness)       │          (TrainerControl)
                                       ▼                    │
                                   Telemetry ◄──────────────┘
                              (1 Hz samples + events,
                               ring buffer, /log)
                                       │ HTTP poll
                                       ▼
                         /logger page in Chrome/Edge on the Zwift PC
                         ──► CSV file on the PC (File System Access API)
```

### New and changed modules

| Module | New/changed | Job |
|---|---|---|
| `BLELink` (from `BTDeviceManager`) | changed | One class per connected peripheral: scan, connect, subscribe (CCCD with response), reconnect, RSSI. Two instances: trainer and power meter. Shared scanner. The trainer path keeps v1 behaviour. |
| `PowerMeter` | new | Subscribes to Cycling Power Measurement (0x2A63), reads CP Feature (0x2A65). Parses power, crank revolutions (reusing `TrainerProtocol::CrankCadence`), balance, accumulated torque. Offset compensation through the CP Control Point (0x2A66, op 0x0C) if the feature bit says it's supported. |
| `RideData` | new | Single source of truth: latest value + timestamp per source (trainer power/cadence/speed, Power2Max power/cadence/balance/torque). Stale after 3 s. Running statistics for calibration. |
| `PowerOutput` | new | Decides what Zwift sees: power source + scale factor, cadence source, fallback to the trainer when the Power2Max is stale. Every path to Zwift goes through it: the Zwift async riding data, the emulated Indoor Bike Data, **and** the trainer notifications forwarded as-is today (CPS/IBD payloads get the power and cadence fields rewritten). |
| `Telemetry` | new | Samples every value at 1 Hz (setting: 1/2/4 Hz) into a RAM ring buffer with sequence numbers; records events. Serves `/log?since=<seq>`. |
| `DirConManager` | changed | Uses `PowerOutput` instead of `TrainerControl::getPower()`. Emits events to `Telemetry`. Virtual-shifting cadence comes from `PowerOutput` (Power2Max cadence when fresh). |
| `TrainerControl` | changed | ERG target can be corrected by `PowerOutput` (power match). Reads FTMS feature bits and logs them (spin-down support etc.). |
| Web UI | changed | Status: Power2Max row and live trainer/Power2Max ratio. Settings: Power2Max pairing (scan list, stored address), power source, scale factors, cadence source. New `/logger` page. |

### Logging to the PC (`/logger` page)

**On the bridge:**
- 1 Hz sample row, about 100 bytes packed. A 10-minute buffer is about 60 KB of RAM;
  the size is set from the measured free heap in phase 0.
- Event rows go into a separate small buffer.
- `/log?since=N` returns every row after sequence N as CSV text, so the page catches up
  after any gap. The response reports when rows were overwritten before the page could
  fetch them.

**On the PC:**
- Open `http://<bridge>/logger` in Chrome or Edge and click *Start*. Pick the file once;
  the page then appends to it continuously through the File System Access API, so
  nothing builds up in browser memory. A *Download* fallback covers browsers without
  that API.
- Fetching runs in a Web Worker. Chrome slows timers on hidden tabs to once a minute
  after 5 minutes, which matters while Zwift is full screen. The 10-minute buffer
  covers that either way.
- Each row gets the PC's clock time, so a log lines up with the Zwift activity FIT
  file (Documents\Zwift\Activities) and a head unit's FIT file.
- The page shows rows written, gaps and the file name. A *Marker* button adds a note
  row ("start ERG test", "spindown done").

**Sample row columns (first cut):**

| Group | Columns |
|---|---|
| Time | `pc_time`, `uptime_ms`, `seq` |
| Trainer | `tr_power`, `tr_cadence`, `tr_cadence_src`, `tr_speed`, `tr_rssi`, `tr_age_ms` |
| Power2Max | `pm_power`, `pm_cadence`, `pm_balance`, `pm_torque`, `pm_rssi`, `pm_age_ms` |
| Sent to Zwift | `out_power`, `out_cadence`, `out_source`, `out_scale` |
| Zwift | `mode`, `grade`, `smoothed_grade`, `gear`, `gear_ratio`, `erg_target` |
| Model | `model_power`, `drag_power`, `trainer_cw`, `residual_drag` |
| To the trainer | `cmd_type`, `cmd_grade`, `cmd_cw`, `cmd_target`, `blocked_writes` |
| Health | `free_heap`, `packets_ftms`, `packets_cps`, `packets_pm` |

Event rows: `pc_time, uptime_ms, seq, type, detail`, for example `shift,36x15 (11/24)`,
`zwift_grade,4.20`, `trainer_cmd,Grade 1.80 % Cw 0.14`, `pm_connected,-61 dBm`,
`pm_zero_offset,…`, `marker,start ERG test`.

An offline analysis page (`tools/analyse.html`, opened from disk on the PC) loads one
or more CSVs and plots power from both meters, their ratio against time/power/cadence,
and the steady segments used for calibration.

## Calibration approach

**1. Measure (phase 3).** Use steady segments only: at least 10 s with cadence within
±3 rpm and power within ±5 %, because the two meters average differently (1 Hz vs about
4 Hz). For each segment, R = trainer ÷ Power2Max. Look at R against:

| R varies with… | Points to |
|---|---|
| Nothing, constant | A slope error in one meter. Can't tell which without a third reference (head-unit ANT+ recording from another meter, or a borrowed pedal meter). |
| Power, more at low power | A zero-offset error. Check the Power2Max zero (coast 2 s, or trigger offset compensation) and the trainer spindown. |
| Flywheel speed / virtual gear | The trainer's internal resistance model, often temperature-related. |
| Time in the first 10–20 min | Trainer warm-up drift. Spindown after warming up and compare. |

**2. Choose what Zwift sees (phase 4).** The setting `Power to Zwift` has three options:
- *Trainer* (v1 behaviour).
- *Power2Max*, falling back to the trainer if the Power2Max goes quiet for 3 s.
- *Trainer × factor*. The factor is learned from steady segments and shown on the
  status page; it is only stored when you click *Use this factor*, so it can't drift on
  its own.

There's also a manual scale factor per meter, and `Cadence to Zwift` (Power2Max when
fresh, otherwise the trainer).

**3. Close the loop (phase 5).**
- **ERG power match:** with Power2Max selected, the target sent to the trainer is
  Zwift's target × the learned trainer/Power2Max ratio, plus a slow correction (about a
  10 s time constant, ±15 % limit). The Power2Max then shows what Zwift asked for.
- **Zero/spindown helpers:** a *Zero Power2Max* button (if CP offset compensation is
  supported) and *Trainer spindown* (if FTMS spin-down control is supported; otherwise
  instructions for the Saris app). Both are logged as events.
- The virtual-shifting Calibration panel compares the simulated power with the selected
  power source.

## Phases

| Phase | Scope | Done when |
|---|---|---|
| 0. Groundwork | Branch, free-heap and NimBLE connection budget, list what Zwift subscribes to in VS mode, capture the Power2Max's services and CP features. One diagnostic build for you to run. | We know RAM per buffer row, which notification paths carry power to Zwift, and what the Power2Max supports. |
| 1. Logging | `Telemetry`, `/log`, `/logger` page, events, trainer-only columns. Useful before the Power2Max is connected. | A 1-hour ride logs with no gaps while Zwift is full screen. |
| 2. Power2Max link | `BLELink` refactor, `PowerMeter`, pairing in Settings, reconnect after sleep, status row, columns in the log. | Both meters logged for a full ride; the Power2Max reconnects after a 5-minute stop without touching anything. |
| 3. Analysis | `tools/analyse.html`, steady-segment detection, R plots. Test rides: warm-up, ERG steps 150/200/250/300 W at 70/90 rpm, a free ride. | We know which kind of error the ~10 % gap is. |
| 4. Selectable output | `PowerOutput`, rewriting the forwarded notifications, fallback, cadence source, scale factors, learned factor with *Use this factor*. | Zwift shows the chosen source; unplugging the Power2Max falls back within 3 s. |
| 5. Power match and helpers | ERG power match, zero and spindown helpers, VS calibration panel on the selected source. | In ERG at 200 W, the Power2Max averages 200 ± 3 W after 30 s. |

Each phase is a pre-release `v2.0.0-alpha.N-ftms` that you can install over the network
(and roll back to `v1.6.1-ftms` the same way). Settings get new fields, so the first v2
install resets settings once; v2 will show the old values to re-enter.

## Risks

| Risk | Mitigation |
|---|---|
| A second BLE connection makes the trainer link less stable (one radio, plus scanning) | Scan only while the Power2Max is missing, with a low duty cycle; longer connection interval for the 1 Hz Power2Max; RSSI and drops logged; trainer link has priority. |
| RAM | Size the buffer from measured free heap; packed binary rows; turn the logger off in Settings if needed. |
| Rewriting forwarded trainer notifications breaks something in Zwift | Off unless *Power to Zwift* isn't *Trainer*; unit tests for every rewritten payload; phase 0 confirms which characteristics Zwift reads. |
| Power2Max BLE behaviour differs from the standard (e.g. no offset compensation over BLE) | Feature bits are read, not assumed; buttons hidden when unsupported. |
| Browser throttling or a closed tab loses data | Sequence numbers and a 10-minute buffer; the page reports gaps; logger state on the status page. |

## Open questions

- Browser on the Zwift PC (Chrome/Edge needed for writing directly to a file).
- Sample rate: 1 Hz default, or 2/4 Hz for faster trainer dynamics (more RAM, larger
  files).
- Is there a head unit that can record the Power2Max over ANT+ during test rides
  (independent cross-check)?
- Power2Max firmware version (shown in the power2max app), in case BLE behaviour depends
  on it.
