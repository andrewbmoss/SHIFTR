# Changelog: SHIFTR for Saris (FTMS fork)

Changes in this fork compared to [JuergenLeber/SHIFTR](https://github.com/JuergenLeber/SHIFTR)
v1.4.0 by Jürgen Leber, on which it is based. Firmware versions carry an `-ftms` suffix.
Developed and tested with a Saris H1 (Hammer), Zwift on Windows, a Zwift Cog and a Zwift
Click, through a WT32-ETH01 on Ethernet.

## v1.6.2-ftms (2026-11-25)

### Fixed
- **Crash after many hours with the trainer off** (inherited from upstream SHIFTR). While it
  looks for the trainer, the bridge scans without end and added every Bluetooth advertisement
  to a list, with no de-duplication and no limit. Phones and watches change their advertising
  data constantly, so the list kept growing until memory ran out and the ESP32 aborted
  (found from a core dump: `operator new` failing in the NimBLE task). The list now keeps only
  named devices, each once, at most 24, with its memory reserved up front; the trainer always
  gets a slot.
- Each trainer reconnect created a new Bluetooth client and never freed the old one.
- A new scan callback object was allocated on every scan start and never freed.

## v1.6.1-ftms (2026-11-22)

### Added
- **Calibration** section on the status page: measured power next to the power the
  simulated bike needs, a 30 s average of both (restarts when you shift or the grade
  changes by more than 1 %), and a suggested *Trainer weight* from a steady climb. It
  says why it is blank (not pedalling, descending, not in virtual-shifting mode).
- `/live` reports the trainer-side drag power and the weights used.

### Fixed
- README had the trainer weight correction inverted. If the trainer is 15 % too hard,
  the trainer weight goes up by 15 %, not down.

### Docs
- Credit to Jürgen Leber as the original author in README.md and NOTICE.txt.
- This changelog.

## v1.6.0-ftms (2026-11-15)

### Added
- **SRAM AXS 2×12 gear table (50/36 × 10-32)**: all 24 chainring/cog combinations,
  sorted by ratio from 36×32 (1.13) to 50×10 (5.00) and mapped one-to-one to Zwift's 24
  gears. The status page shows e.g. `36x15 (11/24)`.
- Status page shows the smoothed grade and the Cw sent to the trainer.

### Changed
- **Air drag is computed by the trainer.** The bridge sends Cw × ρ³ (ρ = virtual ÷
  physical ratio), which the trainer applies from its own flywheel speed. That is the
  same force as the drag at virtual speed × ρ, without the bridge's 1 Hz loop. Anything
  above the protocol limit (FTMS 2.55 kg/m, FE-C 1.86 kg/m) is added to the grade.
- **Grade smoothing is a time constant in seconds** (default 4 s, 0 = off) instead of a
  2-sample average, to stand in for the rider's momentum. Old on/off settings map to 4 s.
- Settings stay as they are on update.

### Fixed
- Resistance oscillated with a 6-8 s period in the biggest gears (from v1.5.0). All
  air drag went through the grade from cadence once a second, which formed a delayed
  feedback loop.

## v1.5.0-ftms (2026-11-14)

### Added
- Gear table setting: Zwift's 24 gears, or an SRAM AXS 1×12 50 × 10-32 table (replaced
  by the 2×12 table in v1.6.0).
- *Trainer weight* setting to match the weight the trainer uses to turn a grade into force.
- Ride panel shows the simulated bike's power and the gear label.

### Changed
- Pedal torque now equals road force × gear ratio for the whole road force. Upstream
  geared only gravity and rolling resistance, so shifts did too little on the flat and
  grades felt 2-3× too strong compared with a shift.
- Zwift's own Cw and Crr, sent with every grade update, are used.
- *Difficulty* scales the grade only.
- Settings reset once on update.

### Fixed
- Removed the −1 % grade offset that made every road about 60-90 W too easy.
- On descents the hill's pull was divided by the gear ratio instead of multiplied.

## v1.4.2-ftms (2026-11-10)

### Fixed
- No power or cadence from Saris trainers. NimBLE wrote the notification descriptor
  (CCCD) without response, and the trainer ignored it. Now written with response and
  checked to exist.
- Trainer dropouts. The Wi-Fi setup access point stayed on permanently and shared the
  radio with Bluetooth. Wi-Fi is now off while Ethernet is up, unless Wi-Fi is
  configured or RECOVERY was held at boot. The access point returns after 30 s without
  Ethernet.

## v1.4.1-ftms (2026-11-10)

### Added
- Cycling Power Service fallback: power and crank cadence from 0x2A63 when FTMS Indoor
  Bike Data has none.
- FTMS Start/Resume after control is granted.
- Raw packet logging for the first packets per source, a 10 s data summary, and packet
  counts on the status page.

### Changed
- Virtual shifting assumes 85 rpm when the trainer sends no data, instead of 0 rpm.

## v1.4.0-ftms (2026-11-08 to 2026-11-12)

### Added
- **FTMS trainer backend**: Request Control, Indoor Bike Simulation, Target Power,
  Target Resistance, Indoor Bike Data parsing. The FE-C backend stays. *Settings >
  Trainer protocol* picks one; *Auto* uses FTMS when available.
- `lib/TrainerProtocol`: FTMS and FE-C encoders and decoders with PC unit tests.
- One writer for all trainer commands: coalesced, at most every 250 ms, re-sent every
  5 s, with FTMS control re-requested if lost.
- Automatic reconnect: after a Bluetooth drop, all of Zwift's subscriptions and the
  last command are restored without re-pairing in Zwift.
- Cadence estimated from flywheel speed when the trainer reports none.
- Safety: 0 % grade when Zwift disconnects; direct writes to the trainer's control
  characteristic are blocked while Zwift drives it through the Zwift service.
- Live *Ride* panel on the status page.
- Function button on IO4 (short press re-pairs the trainer, hold 5 s reboots), ArduinoOTA.
- 3D-printable case (`case/`) with labelled FUNCTION/RESET/RECOVERY buttons, LED hole,
  RJ45 slot and a DC-005 barrel jack window.
- WIRING.md with a wiring diagram, flashing via TX0/RX0 and upload troubleshooting.

### Changed
- `min_spiffs.csv` partitions (the firmware no longer fits 1.25 MB OTA slots).
- Pinned library versions.

### Fixed
- Setup access point password was a dangling pointer ("passphrase too short"), so the
  recovery button couldn't start the access point.
- Host name read the MAC during static initialisation.
- FE-C user configuration packed the bike weight wrong.
- Difficulty on the settings page was never saved.
- FTMS emulation passed Crr in the wrong units.
- Grade calculation could return NaN at extreme gears.
