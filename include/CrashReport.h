#ifndef CRASHREPORT_H
#define CRASHREPORT_H

// Reads the core dump the ESP32 writes to flash when it crashes (panic, watchdog), so a
// crash can be traced afterwards without a serial cable. The dump survives reboots and
// network firmware updates (only the app partition is rewritten) until the next crash
// replaces it or it is cleared.
//
//   GET  /crash        summary as JSON: task, program counter, exception, backtrace
//   GET  /crash/dump   the whole core dump (ELF) for espcoredump.py
//   POST /crash/clear  erase it

#include <Arduino.h>
#include <WebServer.h>

class CrashReport {
 public:
  // call once at boot, after Telemetry::begin(): logs a crash event if the last reset was one
  static void begin();
  static bool hasDump();
  static void handleSummary(WebServer& server);
  static void handleDump(WebServer& server);
  static void handleClear(WebServer& server);
};

#endif
