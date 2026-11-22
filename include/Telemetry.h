#ifndef TELEMETRY_H
#define TELEMETRY_H

// v2 phase 1: everything the bridge knows, once a second, plus events, kept in RAM ring
// buffers so a browser page (/logger) on the Zwift PC can fetch and store it.
//
// Samples are written only from the main loop. Events can come from any task
// (NimBLE, AsyncTCP, main loop), so they go through a spinlock.

#include <Arduino.h>
#include <WebServer.h>

enum class TelemetryEvent : uint8_t {
  BOOT = 0,
  MARKER,
  TRAINER_CONNECTED,
  TRAINER_DISCONNECTED,
  CONTROL_GRANTED,
  CONTROL_REFUSED,
  CONTROL_LOST,
  COMMAND_FAILED,
  ZWIFT_CONNECTED,
  ZWIFT_DISCONNECTED,
  MODE,
  SHIFT,
  ERG_TARGET,
  SETTINGS_SAVED,
  PROBE,
  LOGGER,
  POWER_METER,
  COUNT
};

class Telemetry {
 public:
  // allocate the buffers; call early in setup() while the heap is still unfragmented
  static bool begin();
  // call from the main loop; takes a sample once per second
  static void update();
  static void event(TelemetryEvent type, const char* detail);
  static void eventf(TelemetryEvent type, const char* format, ...);

  // HTTP handlers
  static void handleInfo(WebServer& server);
  static void handleLog(WebServer& server);
  static void handleMarker(WebServer& server);

  // for the status page
  static uint32_t getMillisSinceLastFetch();
  static size_t getSampleCapacity();
};

#endif
