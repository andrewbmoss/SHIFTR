# Wiring and first flash (WT32-ETH01 + Saris H1)

![Wiring diagram](docs/wiring/wiring.svg)

Pin names below match the silkscreen on the WT32-ETH01. The board has a 20-pin
main header (two rows of 10) plus 6 programming pins at the antenna end, three at
the end of each row: `EN`, `GND`, `3V3` on one side and `IO0`, `RX0`, `TX0` on the
other. If your board shipped without pins there, solder them in, or hold Dupont
leads in place with a rubber band for the one-time flash.

> **Watch out:** the main header also has pins printed `TXD` and `RXD` (IO17 and
> IO5). Those belong to a second serial port that can't be used for flashing. For
> flashing, use the pins printed **`TX0`** and **`RX0`** (with a zero).

## Permanent connections

| From | To | Notes |
|---|---|---|
| USB charger 5V | `5V` | Any phone charger, 1 A or more. The board has no USB port, so use a USB breakout or a cut cable. |
| USB charger GND | `GND` | Any GND pin. |
| Reset button | `EN` and `GND` | Tap to reboot. |
| Function button | `IO4` and `GND` | Short press: drop and re-pair the trainer. Hold 5 s: reboot. |
| Recovery button | `IO12` and `GND` | Hold while powering on: Wi-Fi access point with the *default* password. Use it if you lock yourself out of the web page. |
| Status LED (optional) | `3V3` → 330 Ω → LED anode, LED cathode → `IO14` | Active low. On while booting, blinks while Wi-Fi connects, off once online. |
| Ethernet | RJ45 jack → your router or switch | Same network as the Zwift PC. |

Tactile switches from the kit have four legs that are joined in pairs. Wire across
diagonal legs and you can't get the pairing wrong.

## First flash only (CP2102)

| CP2102 pin | WT32-ETH01 pin |
|---|---|
| TXD | `RX0` (IO3, programming pins) |
| RXD | `TX0` (IO1, programming pins) |
| GND | `GND` |
| +5V | `5V` (main header) |

Do **not** power the board from the CP2102's 3V3 pin. That regulator only supplies
about 100 mA, and the ESP32 with the Ethernet PHY and Bluetooth needs more, so you
get random resets or failed flashes. The CP2102's TX/RX lines are 3.3 V logic and
are safe for the ESP32.

Unplug the USB charger lead while the CP2102 is connected. Two 5 V sources on the
same pin can back-feed each other.

The `IO0` pin is also the Ethernet clock input, so it only goes to GND for
flashing. Use a jumper wire, not a permanent button.

### Entering flash mode

The 5-pin CP2102 has no DTR/RTS lines, so PlatformIO can't put the board into
flash mode itself (that's the long row of dots after `Connecting`). Do it by hand:

1. Unplug the CP2102 from the PC.
2. Connect `IO0` to `GND` with a jumper wire.
3. Plug the CP2102 back in. The ESP32 powers up straight into flash mode.
4. Start the upload in PlatformIO.
5. When it finishes, remove the jumper and unplug/replug the CP2102 to start the firmware.

If you have a reset button wired to `EN`, you can instead hold the jumper in
place, tap reset, and start the upload.

### If the upload fails with "No serial data received"

Run *PlatformIO > Monitor* only (no upload), at 115200 baud, then power-cycle
the board and read what appears:

| Monitor shows | Meaning | Fix |
|---|---|---|
| Nothing at all | No data reaches the PC | Check that adapter `RXD` goes to board `TX0` (not `TXD`), that GND is shared, and that the board is powered (red LED on) |
| `boot:0x13 (SPI_FAST_FLASH_BOOT)` | Board starts normally, not in flash mode | `IO0` wasn't on GND at power-up |
| `boot:0x3 (DOWNLOAD_BOOT...)` `waiting for download` | Flash mode works | Board receives nothing: check adapter `TXD` → board `RX0`. Close the monitor before uploading |
| Garbage characters | Baud rate is wrong | Set the monitor to 115200 |

After this first flash, every update goes over Ethernet (see README, "Updating").
