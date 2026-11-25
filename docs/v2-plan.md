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
| Update rate | 1 Hz (FTMS Indoor Bike Data and CPS, measured) | 1 Hz (measured) |
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
- 1 Hz sample row, packed to about 48 bytes. Phase 0 measured only 67–75 KB free while
  riding (largest block 35–49 KB), so the buffer is **5 minutes (about 15 KB)**,
  allocated once at boot. That still covers Chrome's once-a-minute throttling of hidden
  tabs five times over.
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
  after 5 minutes, which matters while Zwift is full screen. The 5-minute buffer
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
| Browser throttling or a closed tab loses data | Sequence numbers and a 5-minute buffer; the page reports gaps; logger state on the status page. |

## Decisions (2026-11-22)

- Zwift PC has Chrome and Edge: the logger writes straight to a file.
- 1 Hz sampling; faster rates later only if diagnostics need them.
- A head unit is available to record the Power2Max over ANT+ during test rides.
- The power2max app login fails, so the bridge reads the meter's firmware version itself
  over Bluetooth (Device Information Service) in the phase 0 probe.

## Phase 0 diagnostic build (`v2.0.0-alpha.0-ftms`)

- `/probe` page (linked as *Diagnostics* at the bottom of the status page):
  - *Take Zwift snapshot*: every characteristic Zwift uses, with subscriptions,
    notifications sent to Zwift, writes and reads, plus mode and heap.
  - *Probe power meter*: 12 s scan, picks the Power2Max (name, else strongest Cycling
    Power device that isn't the trainer), connects as a second connection, reads device
    information, CP features, sensor location and battery, lists all attributes, records
    30 s of measurements (raw and decoded: power, balance, torque, crank data, cadence),
    disconnects. Reports heap before/connected/after and whether the trainer stayed
    connected.
  - *Copy report*: both results as one JSON to paste back.
- Full Cycling Power Measurement parser (balance, torque, offset indicator, energy) and
  CP feature names in `lib/TrainerProtocol`, with tests.
- Fix: the trainer reconnect created a new Bluetooth client every time without freeing
  the old one.
- Footer credits Jürgen Leber and links this fork for bug reports.

## Phase 0 results (2026-11-22)

From the diagnostics report (`v2.0.0-alpha.0-ftms`, Hammer 07268, Zwift in SIM + virtual
shifting):

**Power2Max**

| | |
|---|---|
| Name / address | `P2MNG 109052`, `cb:57:07:63:7e:6c` |
| Device info | Saxonar GmbH, model P0004-8-D, hardware HW_REV_D1, software 01.01.00/f0/537, bootloader D013/38, battery 100 % |
| Sensor location | 15 = spider |
| CP features (0x110789) | pedal balance, crank revolutions, accumulated energy, offset compensation indicator, **offset compensation**, **enhanced offset compensation**, content masking, torque based |
| Not offered | accumulated torque, wheel data |
| Measurements | 1 Hz. Flags 0x0023 (balance, left reference, crank data), every other packet 0x0823 (adds accumulated energy). When no new crank event happened it repeats the previous crank data. |
| Other services | Nordic UART (6E400001…, probably the power2max app's configuration channel), Nordic DFU (00001530…, firmware update: never touched), proprietary 0x4558 in the CPS service |
| Connection | Failed twice with 574 (HCI 0x3E, connection failed to be established) before connecting in 190 ms; RSSI −55 dBm |

Consequences:
- Zeroing over Bluetooth is supported (CP Control Point op 0x0C, enhanced 0x10), so phase 5
  can have a working *Zero Power2Max* button.
- Cadence comes from crank events (1/1024 s), steady at 84.9–85.3 rpm in the sample.
- Connection attempts need automatic retries (alpha.1 retries 4 times).

**Zwift**

Power and cadence reach Zwift by **three** paths at the same time, all at about 1 Hz:

| Path | Characteristic | Source |
|---|---|---|
| Zwift protocol riding data | 00000002-19ca-4651-86e5-fa29dcdd09d1 | Generated by the bridge from `TrainerControl` |
| FTMS Indoor Bike Data | 0x2AD2 | Forwarded unchanged from the trainer |
| Cycling Power Measurement | 0x2A63 | Forwarded unchanged from the trainer |

Phase 4 must replace power (and cadence) consistently in all three, or Zwift may mix
sources. Zwift is also subscribed to FTMS Status (0x2ADA) and the Control Point (0x2AD9),
so it receives the trainer's responses to the bridge's own commands (about 300 each per
ride segment). Harmless so far, noted for cleanup. Zwift subscribes to the Hammer's Wahoo
extension characteristic (A026E005…) but nothing arrives on it.

**Bridge**

- Second connection costs about 6 KB of heap; the trainer stayed connected and kept
  sending at 1 Hz during the 45 s probe.
- Free heap: 90 KB at the Zwift snapshot, 75.5 KB before the probe, 67 KB afterwards
  (largest block 35 KB). The probe's own results stay in memory (about 8 KB) until the
  next reboot, which explains part of it. Phase 1 logs free heap every second, so any
  slow leak in long rides shows up.

**Fixed in alpha.1:** *Copy report* didn't work (the browser clipboard API needs https;
now falls back to the older copy method, plus a *Download report* button); connection
retries.

## Phase 1: logger (`v2.0.0-alpha.2-ftms`)

**Bridge** (`src/Telemetry.cpp`):
- 1 Hz samples, 76 bytes each, 240 kept (4 minutes, 18 KB), plus 96 events × 48 bytes (5 KB).
  Both are allocated at boot, before Bluetooth starts.
- `GET /log/info`: version, boot id, buffer sizes, column names.
- `GET /log?s=<next sample>&e=<next event>`: chunked text, at most 120 rows of each per
  response, with `G` lines for overwritten rows and `M,1` when more is waiting.
- `POST /log/marker` (`text=…`).
- Events: boot (with reset reason), trainer connected/disconnected, FTMS control
  granted/refused/lost, failed trainer commands, Zwift connected/disconnected, mode
  changes, shifts, ERG targets, settings saved, probe results, markers.
- Status page shows whether a logger is fetching.

**Browser** (`/logger`): nothing to install.
- A Web Worker ticks every second, so Chrome's background-tab throttling doesn't apply.
- Rows go into IndexedDB as they arrive, so a reload or crash loses nothing that was
  already fetched; a recording that was running resumes when the page reopens.
- A bridge reboot and any gap are written into the log as events.
- *Download CSV* gives one file per recording: `pc_time,row,<41 sample columns>,event,detail`.
  `row` is `sample` or `event`, so a filter in Excel separates them.
- The File System Access API (writing straight to a file) needs https, which the bridge
  doesn't have, so the page doesn't use it.

## Phase 2: Power2Max link (`v2.0.0-alpha.3-ftms`)

- `src/PowerMeter.cpp`: one FreeRTOS task (core 0) does all radio work for the meter.
  - Connects directly to the stored address (3 s attempt, 3 s pause), no scanning, and
    only while the trainer is connected. NimBLE can't scan and initiate at the same time,
    so the trainer's reconnect scans keep priority.
  - Relaxed connection interval (80–120 ms) for the 1 Hz meter.
  - Subscribes to Cycling Power Measurement with a confirmed CCCD write; reads battery
    on connect and every 10 min; RSSI every 5 s.
  - Power, crank-event cadence (drops to 0 after 3 s without a revolution) and left-leg
    balance (converted from the meter's reference pedal).
- Pairing in *Settings > Power meter*: *Find power meters* (10 s scan for Cycling Power
  devices other than the trainer), *Use*, *Forget*, or enter the address. Stored in NVS
  (`shiftr-v2` namespace), applied without a reboot; existing settings are untouched.
- Status page: *Power meter* row (power, cadence, L/R balance, battery, signal) and
  *Trainer ÷ meter*, the 30 s ratio while both read over 50 W.
- Logger: `pm_*` columns filled; events for paired/forgotten, searching, connected (with
  attempts and battery), disconnected.
- The diagnostics probe pauses the meter task and refuses to run while the meter is
  connected.
- Power to Zwift is unchanged (still the trainer) until phase 4.

## First log (2026-11-22 16:00, 13 min, `alpha.3`)

- Data integrity: no missing samples, trainer and Power2Max both 1 packet/s, meter data
  never older than 1.1 s, power aligned at lag 0 (correlation 0.98).
- Heap: about 60 KB free with trainer, meter and Zwift connected; largest block steady at
  31.7 KB. No sign of a leak in this short ride.
- One 5.4 s main-loop stall, at the trainer (re)connect: v1 discovers the trainer's
  services in the main loop. Only on reconnects; candidate for moving into a task later.
- Event text over 39 characters is cut ("battery 1"): shortened in alpha.4.
- Cadence: trainer and Power2Max agree, median difference 0.2 rpm.
- Power, 10 s steady windows (n = 109, 83-501 W): trainer = 1.005 × Power2Max + 9.4 W.
  Close to a constant offset, not a scale error, and it grows with flywheel speed:
  −0.5 W at 20-30 km/h, +7.5 W at 30-35, +12 W at 35-40, +16 W at 40-45. That pattern
  fits the trainer's internal loss model (spindown calibration) better than a Power2Max
  scale error. Expected with perfect meters: trainer 2-4 % *below* the Power2Max.
- Second-to-second spread is mostly flywheel energy: the trainer reports brake power,
  so it reads high while the flywheel slows (ratio 1.23) and low while it speeds up
  (1.00). About 3.8 kg equivalent at flywheel speed.

Test plan for the next ride: spindown in the Saris app after a 10 min warm-up; coast 3 s
before each block (Power2Max auto zero); ERG blocks at the same power with two
different cadences (same power, different flywheel speed) to separate a power-dependent
error from a speed-dependent one.

## Comparison test (2026-11-23, Saris spindown, Power2Max 52/36 bike then Red 50/36 bike with Quarq)

ERG blocks from `tools/workouts`, 95 s per half-block after 25 s settling. The Hammerhead Karoo
recorded each meter over ANT+; after correcting its clock (+9 s) it matches the bridge's
Bluetooth readings within 0.0–0.9 %, so both radios read the meters identically.

| Block | Flywheel km/h | Trainer ÷ Power2Max | Trainer ÷ Quarq | Quarq ÷ Power2Max |
|---|---|---|---|---|
| 160 W, 70 rpm, 36T | 25 | 1.038 | 0.937 | 1.108 |
| 160 W, 95 rpm, 36T | 33 | 1.023 | 0.945 | 1.083 |
| 205 W, 70 rpm, 36T | 25 | 1.021 | 0.926 | 1.103 |
| 205 W, 95 rpm, 36T | 33 | 1.007 | 0.930 | 1.083 |
| 250 W, 70 rpm, 36T | 24-25 | 1.039 | 0.906 | 1.147 |
| 250 W, 95 rpm, 36T | 33-34 | 1.018 | 0.899 | 1.132 |
| 205 W, big ring, low cadence | 35-36 | 1.043 | 0.959 | 1.088 |
| 205 W, big ring, high cadence | 45-46 | 1.074 | 0.969 | 1.108 |

- **Quarq reads 10.6 % above the Power2Max on average (8–15 %)**, using the trainer as a
  go-between. Matches the rider's experience outdoors.
- The trainer reads relatively higher as flywheel speed rises: about +0.15–0.2 % per
  km/h against *both* meters, so that's the trainer, not either meter.
- With 2–4 % chain loss the trainer should read 0.96–0.98 × crank power. Against that,
  the Power2Max is about 4–6 % low or the Quarq about 4–8 % high, or a mix. A second
  trainer (KICKR CORE) would split it; ride the Red bike first next time to cancel
  trainer warm-up drift.
- Pedal balance: Power2Max 51–53 % left, Quarq 50–53 %; both are spider estimates, and the
  Karoo shows the same values as the bridge.
- Logging issues fixed in alpha.5: Zwift re-sends the ERG target every second, which
  logged 2,797 events and overflowed the event buffer (two gaps); now only changes are
  logged. The Power2Max took up to 7 minutes to reconnect while being pedalled: it only
  advertises for about a minute after waking. The search now listens 5 s per attempt with
  0.5 s pauses, and the status page explains letting it sleep and waking it.
- Bridge health over 70 min: heap 82–87 KB free, no leak; the only pause is the known
  5 s trainer reconnect.

Phase 4 direction: the rider's reference is the Quarq, so Zwift should get
*Power2Max × ~1.10* (Quarq-equivalent), falling back to the trainer with a speed-aware
factor when the Power2Max is missing.

## Race log (2026-11-24, 67 min pedalling, `alpha.5`)

- Steady stretches: trainer ÷ Power2Max 1.032 (1.045 at 20–30 km/h, 1.028 at 30–35 km/h), in
  line with the comparison test. Whole race: trainer 226 W, Power2Max 220 W, so
  Power2Max × 1.106 (Quarq scale) would have been 243 W.
- No missing samples, no loop stalls, heap 76–85 KB, Power2Max connected 97.6 % of the time
  the trainer was.
- **The bridge rebooted with a crash ("reset panic") at 17:07**, two hours before the race,
  with nobody riding. Cause unknown. Core dumps to flash are enabled in this framework, so
  alpha.6 adds `/crash` (summary), `/crash/dump` (ELF core dump) and `/crash/clear`, a crash
  section on the diagnostics page, and a boot event naming the crashed task. The alpha.5 and
  alpha.6 ELF files are kept for decoding.
