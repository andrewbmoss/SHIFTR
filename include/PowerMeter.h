#ifndef POWERMETER_H
#define POWERMETER_H

// v2 phase 2: a Bluetooth power meter (Power2Max NG) as a second, independent source of
// power, cadence and pedal balance, connected next to the trainer.
//
// All radio work for the meter (connect attempts, the pairing scan) runs in one task of
// its own, so the main loop and Zwift are never blocked. It connects straight to the
// stored address without scanning and only while the trainer is connected: NimBLE can't
// scan and initiate a connection at the same time, so this keeps the trainer's own
// reconnect scans free.
//
// The pairing (address, address type, name) lives in NVS (Preferences), separate from
// the IotWebConf settings, so adding it didn't reset anything.

#include <Arduino.h>
#include <WebServer.h>

enum class PowerMeterState : uint8_t {
  NOT_PAIRED = 0,
  WAITING_FOR_TRAINER,  // paired, but the trainer has priority on the radio
  SEARCHING,            // connect attempts running, meter asleep or out of range
  CONNECTED,
  PAUSED                // diagnostics probe running
};

struct PowerMeterReading {
  bool connected = false;
  int16_t power = 0;
  double cadence = 0;
  bool hasBalance = false;
  double balanceLeft = 0;  // % of power from the left leg
  int rssi = 0;
  uint32_t ageMs = UINT32_MAX;  // since the last measurement
  uint32_t packets = 0;         // since boot
  int battery = -1;
};

class PowerMeter {
 public:
  static void begin();
  static PowerMeterState getState();
  static const char* getStateName();
  static PowerMeterReading getReading();
  static String getName();
  static String getAddress();
  static bool isConnected();
  // the diagnostics probe needs the radio to itself
  static void setPaused(bool paused);
  static bool isRadioBusy();

  // HTTP: GET /powermeter (state), POST /powermeter/scan, POST /powermeter/pair, POST /powermeter/forget
  static void handleStatus(WebServer& server);
  static void handleScan(WebServer& server);
  static void handlePair(WebServer& server);
  static void handleForget(WebServer& server);
};

#endif
