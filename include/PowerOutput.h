#ifndef POWEROUTPUT_H
#define POWEROUTPUT_H

// v2 phase 4: what power and cadence Zwift receives.
// Wraps PowerCal::Calibrator (lib/Calculations) with live inputs from the trainer and the
// power meter, NVS settings (namespace "shiftr-v2", no reboot), events and HTTP endpoints.
// Readers in other tasks (NimBLE notifications, DirCon) get the latest values through a spinlock.

#include <Arduino.h>
#include <NimBLEUUID.h>
#include <PowerCalibrator.h>
#include <WebServer.h>

class PowerOutput {
 public:
  static void begin();
  // call from the main loop; works once per second
  static void update();

  static int16_t getPower();
  static double getCadence();
  static PowerCal::Source getSource();
  static const char* getSourceName();
  static PowerCal::Mode getMode();
  static double getFactor();
  static double getBaselineFactor();
  static double getDriftPercent();
  static bool getDriftWarning();
  static uint32_t getLearnedSeconds();
  static double getMeterScale();
  // ERG: what to ask the trainer for so the crank-equivalent power equals target
  static double trainerTargetFor(double target);
  // rewrite a trainer notification on its way to Zwift (IBD / CPS); false = leave as is
  static bool rewriteForZwift(const NimBLEUUID& characteristicUUID, uint8_t* data, size_t length);

  // GET/POST /power/settings, POST /power/reset, POST /trainer/release, POST /trainer/resume
  static void handleSettingsGet(WebServer& server);
  static void handleSettingsPost(WebServer& server);
  static void handleReset(WebServer& server);
  static void handleRelease(WebServer& server);
  static void handleResume(WebServer& server);
};

#endif
