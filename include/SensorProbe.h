#ifndef SENSORPROBE_H
#define SENSORPROBE_H

// v2 phase 0: one-off diagnostic of a Bluetooth power meter (Power2Max NG).
// Scans for devices advertising the Cycling Power Service, connects to one as a second
// connection next to the trainer, reads what it offers (device information, CP features,
// services) and records 30 s of power measurements, then disconnects.
// Nothing here is used for riding yet; it tells us what the meter supports and what a
// second connection costs (RAM, trainer link).

#include <Arduino.h>

#include <string>

class SensorProbe {
 public:
  // address empty = pick automatically (name contains "p2m"/"power2max", else strongest CPS device)
  static bool start(const std::string& address, String& error);
  static bool isRunning();
  static String getReport();
};

#endif
