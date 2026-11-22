# SHIFTR for Saris (FTMS fork)

> **Credit:** SHIFTR was created by **Jürgen Leber**
> ([@JuergenLeber](https://github.com/JuergenLeber)). The original project is
> [JuergenLeber/SHIFTR](https://github.com/JuergenLeber/SHIFTR): the Direct Connect
> bridge, the Zwift protocol and virtual-shifting groundwork, the web interface and the
> FE-C backend are all his. This fork builds on that work, keeps its full commit history
> and the GPL-3.0 license, and lists its own changes below. It is not affiliated with
> Saris, Zwift or the upstream project. If you use a trainer that upstream already
> supports, use the original.

This fork adds a Bluetooth **FTMS** trainer backend, so trainers like the Saris
H1/H2/H3 get Zwift virtual shifting through a WT32-ETH01 bridge. The bridge connects
to the trainer over Bluetooth, shows up in Zwift as a Direct Connect (Ethernet)
trainer that speaks the Zwift protocol, and turns Zwift's gear, grade and ERG
commands into FTMS (or FE-C) commands for the trainer.

Wiring and the first flash are in [WIRING.md](WIRING.md). Release notes for each
version are in [CHANGELOG.md](CHANGELOG.md).

## What changed compared to upstream SHIFTR

- **FTMS backend** (`src/TrainerControl.cpp`, `lib/TrainerProtocol`): Request Control,
  Set Indoor Bike Simulation, Set Target Power and Set Target Resistance, plus Indoor
  Bike Data parsing. The FE-C backend is still there. *Settings > Trainer protocol*
  picks one: *Auto* uses FTMS when the trainer has it.
- **One writer, coalesced commands**: all trainer writes happen in the main loop, at
  most every 250 ms, newest command wins, and the last command is re-sent every 5 s.
  If the trainer reports that FTMS control was lost, the bridge requests it again
  and re-sends.
- **Reconnects recover on their own**: after a Bluetooth drop the bridge reconnects,
  re-subscribes everything Zwift was subscribed to, and re-sends the last command.
  Upstream needed Zwift to re-pair.
- **Cadence fallback**: if the trainer doesn't report cadence, it is estimated from
  flywheel speed. That works because the Cog keeps the physical gear fixed.
- **Safety**: when Zwift disconnects, the trainer is set to a flat 0 % grade. While
  Zwift drives the trainer through the Zwift service, any direct write to the
  trainer's FTMS/FE-C control characteristic is blocked, so two controllers can't
  fight. The *Blocked writes* counter on the status page shows if that ever happens.
- **Web status page** now has a live *Ride* panel (power, cadence and its source,
  virtual gear, Zwift grade, what was sent to the trainer).
- **Lifelike virtual shifting** (v1.5, v1.6): see "Virtual shifting feel" below.
- **Function button** on IO4: short press re-pairs the trainer, hold 5 s reboots.
- **ArduinoOTA** in addition to the web firmware upload (`pio run -e wt32-eth01_ota -t upload`).
- Bigger app partitions (`min_spiffs.csv`). The firmware is about 1.3 MB and no longer
  fits the default 1.25 MB OTA slots.
- Fixes: the FE-C user configuration packed the bike weight wrong, the difficulty
  field in the settings page was never saved, FTMS emulation passed Crr in the wrong
  units, and grade calculation could produce NaN at extreme gears.
- Uses the maintained `esp32async/AsyncTCP` fork, and all library versions are pinned.

## Virtual shifting feel (v1.5, v1.6)

The goal is that every gear and every grade feels like a real bike: pedal torque at a
given cadence equals *road force × gear ratio*, where road force is gravity, rolling
resistance and air drag at the speed the virtual bike would go.

### v1.6

- **Gear table: SRAM AXS 2×12, 50/36 × 10-32.** All 24 chainring/cog combinations,
  sorted from 36×32 (1.13) to 50×10 (5.00), one per Zwift gear: Zwift gear 1 is 36×32,
  gear 12 (Zwift's default) is 36×14, gear 24 is 50×10. The status page shows e.g.
  `36x15 (11/24)`. Because it's every combination in ratio order, a few neighbours are
  close together (36×24 → 50×32 is +4 %, 36×15 → 36×14 is +0.8 %), the same as on the
  real bike if you used every gear sequentially.
- **Air drag is computed by the trainer again.** v1.5 put all of it into the grade,
  computed from cadence once a second. That forms a feedback loop (faster → more drag →
  slower → less drag) with about 2 s of delay, and in the biggest gears it oscillated
  with a period of 6–8 s. v1.6 sends the trainer *Cw × ρ³* (ρ = virtual ratio / physical
  ratio). The trainer applies 0.5 × Cw × v² from its own flywheel speed many times a
  second, which is exactly *drag at virtual speed × ρ*, so the physics is unchanged but
  the loop is gone. Gravity and rolling resistance still go in the grade, so the grade no
  longer depends on cadence. FTMS can carry Cw up to 2.55 kg/m, FE-C up to 1.86 kg/m;
  anything above that is added to the grade as before (the *Air drag* line on the
  status page shows it).
- **Grade smoothing is a time constant.** On the road, rider + bike (~85 kg) carry you
  into a ramp and your speed, and with it the force you feel, settles over 5–12 s. A
  direct-drive flywheel is equivalent to only ~20–25 kg, so a grade that reaches it
  instantly feels sharper than the real thing. *Grade smoothing* (default 4 s, 0 = off)
  lets the grade follow Zwift with that lag. The status page shows Zwift's grade and the
  smoothed one.

### Physical gear (important in v1.6)

Put the chain on the **big ring** and set *Chainring* to match (*Sprocket* stays 14 for
the Zwift Cog). The trainer's Cw limit only covers every virtual gear if the physical
ratio is at least about 2.9 (FTMS, chainring ≥ 42) or 3.25 (FE-C, chainring ≥ 46):

| Chainring on the 14T Cog | ρ at 50×10 | Cw needed at 50×10 | FTMS (max 2.55) | FE-C (max 1.86) |
|---|---|---|---|---|
| 34 | 2.06 | 4.45 | 43 % of the drag via grade | no |
| 36 | 1.94 | 3.75 | 32 % via grade | no |
| 39 | 1.79 | 2.95 | 14 % via grade | no |
| 42 | 1.67 | 2.36 | fits | no |
| 46 | 1.52 | 1.80 | fits | fits |
| 50 | 1.40 | 1.40 | fits | fits |
| 52 | 1.35 | 1.24 | fits | fits |

The part that doesn't fit is handled like v1.5 did: it still works, it's just the part
of the force that lags behind cadence changes. A bigger ring also spins the flywheel
faster, which gives it more inertia and a more road-like feel. If the easiest gears on
descents all feel the same, the trainer is at its minimum resistance at that flywheel
speed: try the small ring.

### Calibrating to the trainer

The FTMS standard has no field for rider weight, so the trainer turns the grade into
force using a weight of its own. Since v1.6 that weight only affects gravity and rolling
resistance (air drag is sent separately), so calibrate **on a steady climb of 4 % or
more**.

The status page has a **Calibration** section under *Ride*:

1. In Zwift, ride a free ride or route (not ERG) with virtual shifting, and find a
   steady climb of 4 % or more.
2. Stay in one gear at a steady cadence. *Simulated bike needs* shows what a real bike
   in that gear would take; *30 s average* fills up after 20 s and resets when you shift
   or the grade changes by more than 1 %.
3. *Suggestion* then says either *Matches* or *Set Trainer weight to … kg*. Enter that
   in *Settings > Trainer weight*, save, and check again on the next climb.

The maths behind the suggestion: if the trainer is 15 % too hard on the climb, it is
using a weight 15 % higher than the bridge assumed, so the bridge must assume that
higher weight too (84 kg × 1.15 ≈ 97 kg). The air drag part is subtracted first.

On the flat the *30 s average* compares mainly air drag: within about 10 % means the
trainer applies the Cw it is sent.

*Difficulty* scales only the grade (like Zwift's trainer difficulty). Leave it at 100 %
for real-world hills, or lower it if you want climbs gentler than the road.

### v1.5

- **Air drag scales with the gear.** Upstream multiplied only gravity and rolling
  resistance by the gear ratio, so in the upper gears a shift on the flat did about 15 %
  instead of 19 %.
- **No more −1 % grade offset.** It made every road about 8 N (60–90 W) too easy, so a
  hill was a much bigger jump from that lowered baseline than it should be.
- **Downhill gearing fixed.** On a descent the hill's pull was divided by the gear ratio
  instead of multiplied, so a harder gear passed less of the pull to the pedals.
- **Zwift's own Cw and Crr are used.** Zwift sends them with every grade update; upstream
  ignored them.

## Build and first flash (Windows)

1. Install [VS Code](https://code.visualstudio.com/) and the **PlatformIO IDE** extension.
2. Install the Silicon Labs **CP210x driver** if Windows doesn't recognise the adapter (Device Manager shows a COM port when it works).
3. Open this folder in VS Code. PlatformIO downloads the toolchain and libraries on the first build.
4. Wire the CP2102 and put the board in flash mode ([WIRING.md](WIRING.md)).
5. PlatformIO sidebar > `wt32-eth01` > **Upload and Monitor**. You should see the boot log and, once Ethernet is up, a line like
   `Ethernet DHCP successful with IP 192.168.1.50`.

Prebuilt binaries are in `firmware/` if you'd rather not build. Flash
`shiftr-saris-merged.bin` at address `0x0` with
[esptool-js](https://espressif.github.io/esptool-js/) in Chrome or Edge, or with
`esptool.py write_flash 0x0 shiftr-saris-merged.bin`.

## First setup

1. Disconnect the CP2102, connect Ethernet and the 5 V adapter.
2. Open `http://SHIFTR-xxxxxx.local` (the name is in the serial log), or use the IP address.
3. Go to *Device settings*. User `admin`, password = the device name, e.g. `SHIFTR-123456`. Change it under *Configuration* afterwards.
4. Set these, then *Save* (the device reboots):
   - **Trainer device**: your Saris. Close QZ or any other app first: the trainer accepts only one Bluetooth connection, and anything still connected hides it from the bridge.
   - **Trainer protocol**: *Auto (FTMS)*.
   - **Virtual shifting**: on. **VS mode**: *Track Resistance* (recommended for FTMS).
   - **Chainring**: the teeth of the chainring your chain sits on (use the big ring). **Sprocket**: 14 for the Zwift Cog. These set the physical ratio the virtual gears are computed against.
5. On the status page the *Ride* panel should show the trainer *via FTMS* and power/speed when you pedal.

## Pairing in Zwift (Windows)

- The Windows network profile must be **Private**, and Zwift must be allowed through
  the firewall. Direct Connect discovery uses mDNS, which Windows blocks on Public networks.
- In the pairing screen pick **SHIFTR xxxxxx** for *Power*, *Controllable* and
  *Cadence*. It shows a Direct Connect/Ethernet icon. Don't pair the Saris itself.
- Pair the Zwift Click (or Play) as usual over Bluetooth. Zwift offers virtual shifting
  once the controllable trainer speaks the Zwift protocol.

## What to check on the first ride

| Check | Where | Expected |
|---|---|---|
| Trainer link | *Ride* panel | `... via FTMS (control granted)` after the first command |
| Shifting | Press the Click | *Virtual gear* changes, *Sent to trainer* grade moves, resistance follows within about 0.5 s |
| Climbs | Ride a hill | *Zwift grade* rises, sent grade rises with it |
| ERG | Start a workout | *Sent to trainer* shows `Target power ... W` |
| Cadence | *Ride* panel | Source says `trainer` or `estimated from speed`. If it is estimated and reads wrong, correct the chainring setting. |

If climbs feel too strong, lower *Difficulty* or calibrate *Trainer weight* (see above). If
*Track Resistance* feels laggy on your trainer, try *Target Power*, which uses ERG on
the trainer. If FTMS misbehaves, try the *FE-C* protocol. The Saris advertised both.

## Updating

Over Ethernet, without the CP2102:

- Web page > *Firmware update*, upload `.pio/build/wt32-eth01/firmware.bin`; or
- edit `upload_port`/`--auth` in the `[env:wt32-eth01_ota]` section of `platformio.ini`, then run `pio run -e wt32-eth01_ota -t upload`.

## Tests

`pio test -e native` runs the PC unit tests for the FTMS/FE-C encoders and the
virtual-shifting physics (`test/test_protocol`).

## Credits and license

- **Jürgen Leber**, author of [SHIFTR](https://github.com/JuergenLeber/SHIFTR), on
  which everything here is based. His original README follows below, unchanged.
- The third-party libraries and projects listed in [NOTICE.txt](NOTICE.txt).
- Licensed under the [GNU GPL v3](LICENSE), same as upstream. The changes in this fork
  are described in "What changed compared to upstream SHIFTR" above and in the commit
  history.

---

# Upstream README

*The original SHIFTR README by Jürgen Leber, kept as is. Its CI badge and instructions
refer to the upstream repository.*

[![License: GPL v3](https://img.shields.io/badge/License-GPLv3-blue.svg)](https://www.gnu.org/licenses/gpl-3.0)
[![CI](https://github.com/JuergenLeber/SHIFTR/actions/workflows/build.yml/badge.svg)](https://github.com/JuergenLeber/SHIFTR/actions/workflows/build.yml)
# SHIFTR 
A BLE to Direct Connect bridge for bike trainers using a WT32-ETH01 module based on ESP32. Increases connection stability and additionally adding Zwift "virtual shifting" functionality to any device supporting FE-C over BLE. Also adding FTMS emulation via DirCon for non-FE-C compatible software like MyWhoosh.

Currently tested with Garmin/Tacx NEO 2T and Vortex but should also work on other trainers supporting FE-C over BLE.

Tested bike trainer software so far:
- &#x2611; Zwift (Pass-through + virtual shifting mode)
- &#x2611; MyWhoosh (Pass-through mode only / TACX Neo 2T and Vortex only with FTMS emulation enabled) 

Disclaimer: Even if everything is working fine this project is still work in progress. Feel free to provide feedback if you encounter any issues.

## Overview
Bike trainers heavily evolved over the last few years. Tacx (acquired by Garmin some years ago) is one of the market leaders producing bike trainers of a very high quality with special features like e.g. "Road Feeling" to simulate different road surfaces. 
Other vendors like Zwift came up with cool new features like "virtual shifting" using the "Zwift Cog" and a controller device (Zwift Click/Play) to have a non-mechanical shifting solution which is setting the gears via software that adjusts the resistance in the trainer. 
Wahoo came up with a technology called "Direct Connect" that allows the trainer devices to be connected via Ethernet (or even WiFi) instead of bluetooth to have a more stable connection.

While other vendors like e.g. Elite or JetBlack are very fast in implementing things like virtual shifting to their newer models Garmin is very slow and unresponsive when it comes to such feature requests. Probably also because they have their own ecosystem that competes with Zwift. 

Using Zwift for many years and owning a "Tacx Vortex" and a "Tacx Neo 2T" bike trainer I have been jealous of features like virtual shifting and the possibility to connect the trainer device via ethernet. So the idea for this project, SHIFTR, was born by having a device between Zwift and the Tacx supporting all this cool new functionality.

After many days and nights of searching the internet for a specification of the protocols I was only able to find small parts for "Direct Connect" that have already been reverse engineered by Roberto Viola in his [QDomyos-Zwift](https://github.com/cagnulein/qdomyos-zwift) project. The code helped me a lot to understand the protocol.
Unfortunately there was absolutely no information about the "virtual shifting" to be found. So a very long session of decompiling Zwift apps and sniffing BLE traffic between Zwift and capable trainers (which fortunately a friend had one of) started. It took some weeks but I was able to reverse engineer the used protocols.

As I wanted to have a convenient solution that doesn't require a special program or app to be started to have that functionality I decided to implement everything on an ESP32 microcontroller that already features WiFi and Bluetooth onboard and also optionally ethernet. The perfect module for that was the [WT32-ETH01](https://en.wireless-tag.com/product-item-2.html) module that comes with everything I needed. The device will be placed very close to the trainer in a 3D printed case that also has a place for a step down converter to be able to use the existing power supply from the trainer.

Now I have a great setup with a Garmin/Tacx NEO 2T with a Zwift Cog installed and communicating to Zwift via ethernet while still having the "Road Feeling" feature: 
![The whole setup](images/tacx_neo_2t_and_zwift_cog_and_device.jpg)

This setup even works great with Zwift running on an Apple TV. Apple TV only supports two simultaneous bluetooth connections and those can now be used for the Zwift Play controllers while the trainer itself is being connected through ethernet.

## How it works
The SHIFTR is working in two modes, "Pass-through" and "Pass-through + virtual shifting":
### Pass-through mode
In this mode the SHIFTR just takes all services from the BLE trainer devices and provides them 1:1 via Direct Connect. It supports SIM and ERG mode as if the device would be connected via bluetooth.
### Pass-through + virtual shifting mode
This mode provides the pass through features as mentioned before and additionally offers a special Zwift service via Direct Connect that allows it to behave like a Zwift certified device offering virtual shifting, too. All necessary information like incline, bicycle and user weight, etc. are transmitted by Zwift and used for calculations. SIM and ERG mode are supported but of course virtual shifting only works in SIM mode.

Details about the virtual shifting function and calculations can be found [here](./VirtualShifting.md).

***Note***: Virtual shifting will be disabled if neither Zwift Play controllers nor a Zwift Click are connected which results in the standard SIM mode with a track resistance of 0%. Then you'd have to shift with your bike but with a Zwift Cog installed this doesn't make sense of course. If the controllers disconnect during a training then the fallback will be this normal SIM mode but as soon as the controller(s) are re-connected the virtual shifting will be enabled again.

### Pass-through + FTMS emulation
This mode provides all services from the BLE trainer and additionally offers a FTMS emulation that internally sends the commands to the BLE FE-C trainer. This makes software like MyWhoosh also working. Doesn't (at least not yet) work with virtual shifting as this is a Zwift-only feature at the moment.

## Needed hardware
- WT32-ETH01 ESP32 board (e.g. from [Amazon](https://www.amazon.de/WT32-ETH01-Embedded-Schnittstelle-Bluetooth-Entwicklungsplatine/dp/B0CW3DDWZ4))
- Step-Down converter 5-80V (the Tacx Neo supplies 48V) to 5V (e.g. from [Amazon](https://www.amazon.de/dp/B0CMC7Y3DJ))
- Y-Splitter for the Tacx power cable (e.g. from [Amazon](https://www.amazon.de/dp/B09FHHN9T5))
- Power connector for the case (e.g. from [Amazon](https://www.amazon.de/gp/product/B09PD6J4BN))
- 3D printed case that fits the WT32-ETH01. Luckily I was able to find a perfect match that also fits the step-down converter: [WT32-ETH01 Enclosure](https://www.thingiverse.com/thing:5621092)
- Two countersunk screws M2x7 to hold the lid on the case


## Hardware installation
- ***IMPORTANT***: Don't solder the pin headers that came with the WT32-ETH01. With the longer pins on the bottom side the module won't fit the case anymore.
- ***IMPORTANT #2***: Connect the step-down converter first to a power supply (e.g. the original one with 48V) and a multimeter and use a screwdriver to adjust the output voltage to 5V before connecting it to the WT32-ETH01 module!
- Use a standard USB<->TTL converter (e.g. with FTDI232 or similar). Make sure that it can provide **3,3V and not 5V** and connect as follows (see also pinout above):
  | Converter | -> | ETH01-EVO | 
  |-|-|-|
  | RX | -> | TXD |
  | TX | -> | RXD |
  | 3V3 | -> | 3V3 |
  | GND | -> | GND |

  It's best to use some breadboard jumper wires that can just be connected without soldering. Use a rubber band to hold them in contact.

  To start the WT32-ETH01 in boot mode it is necessary to connect "IO0" with GND and then to reset the board, shortly connect "EN" to GND for a quarter of a second.

## Software installation
- Open the project in [PlatformIO](https://platformio.org) and let it install the dependencies
- Connect the programmer and upload via the task wt32-eth01 -> Upload and Monitor
- Connect an ethernet cable and make sure a DHCP server exists in your network
- After a few seconds you should be able to see the IP address and the hostname (like e.g. "SHIFTR-123456.local") in the monitor log
- If you don't use the Ethernet interface you can also use the WiFi functionality. For that the device opens an AP with the device name (e.g. "SHIFTR-123456") and the password is the same but of course can be changed later. Please note that WiFi is provided by an external library called [IoTWebConf](https://github.com/prampec/IotWebConf) and is more or less untested
- After that you can configure the device and its network settings in the web interface on e.g. http://SHIFTR-123456.local
- The status page is accessible without credentials, the settings and firmware update pages need the username "admin" and the password is the configured AP password or just the device name (e.g. "SHIFTR-123456") as default. Note that both are case sensitive!
- Further updates can be made over ethernet or WiFi so you can disconnect the programming adapter now

## Finalization
- Finally you can solder some wires to the power connector to the step down module and from there to the WT32-ETH01. Then mount everything in the 3D printed case:

  ![Mounted adapter](images/wt32-eth01_case.jpg)

- Use the Y-Power-Splitter to connect the Tacx trainer and the device:
  
  ![Connected to Tacx](images/connected_to_tacx_neo_2t.jpg)

- Power the Trainer and go to the web interface
- Go to "Settings", select the trainer device and "Save"
- Make sure the "Virtual shifting" option is enabled if you want to use it. Virtual shifting only works in Zwift when a Zwift Click or Zwift Play controllers are connected. Otherwise it will fall back to the normal ERG and SIM modes
- In Zwift you need to connect to the "SHIFTR 123456" device instead of your old device now. Have fun!

## Thank you!
You've made it to the end, hopefully you'll have fun rebuilding the whole thing. Please tell me if you like it and if it works as expected.
In case of any questions feel free to contact me!

## Disclaimer
Please always check the documentation of the hardware you bought as there are sometimes small changes in pin assignments, voltage and so on. I don't take any liability for damages or injuries. Build this project at your own risk. The linked products and pages are for reference only. I don't get any money from the manufacturers or the (re)sellers.




