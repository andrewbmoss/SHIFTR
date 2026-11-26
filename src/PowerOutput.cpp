#include <PowerOutput.h>
#include <BTDeviceManager.h>
#include <DirConManager.h>
#include <Preferences.h>
#include <PowerMeter.h>
#include <SettingsManager.h>
#include <Telemetry.h>
#include <TrainerControl.h>
#include <TrainerProtocol.h>

namespace {

const char* const NVS_NAMESPACE = "shiftr-v2";
constexpr uint32_t UPDATE_INTERVAL_MS = 1000;   // learning
constexpr uint32_t REFRESH_INTERVAL_MS = 100;   // output: picks up each new reading within 0.1 s
constexpr uint32_t FRESH_MS = 2500;
constexpr uint32_t SAVE_INTERVAL_MS = 600000;  // save the learned factor at most every 10 min
const NimBLEUUID INDOOR_BIKE_DATA((uint16_t)0x2AD2);
const NimBLEUUID CYCLING_POWER_MEASUREMENT((uint16_t)0x2A63);
const char* const SOURCE_NAMES[] = {"trainer", "power meter", "trainer (corrected)", "none"};

PowerCal::Calibrator calibrator;
portMUX_TYPE lock = portMUX_INITIALIZER_UNLOCKED;
// latest output, read from other tasks
int16_t outPower = 0;
double outCadence = 0;
PowerCal::Source outSource = PowerCal::Source::NONE;
double outFactor = 1.0;
uint32_t lastUpdateAt = 0;
uint32_t lastRefreshAt = 0;
uint32_t lastSaveAt = 0;
double savedFactor = 0;
PowerCal::Source lastLoggedSource = PowerCal::Source::NONE;
bool lastWarning = false;
bool wasReleased = false;

PowerCal::Config loadConfig() {
  PowerCal::Config config;
  Preferences preferences;
  preferences.begin(NVS_NAMESPACE, true);
  config.mode = (PowerCal::Mode)preferences.getUChar("po_mode", (uint8_t)PowerCal::Mode::POWER_METER);
  config.meterScale = preferences.getDouble("po_scale", 1.055);
  config.chainLossPercent = preferences.getDouble("po_loss", 3.0);
  config.driftWarnPercent = preferences.getDouble("po_drift", 4.0);
  savedFactor = preferences.getDouble("po_factor", 0);
  preferences.end();
  return config;
}

void saveConfig(const PowerCal::Config& config) {
  Preferences preferences;
  preferences.begin(NVS_NAMESPACE, false);
  preferences.putUChar("po_mode", (uint8_t)config.mode);
  preferences.putDouble("po_scale", config.meterScale);
  preferences.putDouble("po_loss", config.chainLossPercent);
  preferences.putDouble("po_drift", config.driftWarnPercent);
  preferences.end();
}

void saveFactor(double factor) {
  Preferences preferences;
  preferences.begin(NVS_NAMESPACE, false);
  if (factor > 0) {
    preferences.putDouble("po_factor", factor);
  } else {
    preferences.remove("po_factor");
  }
  preferences.end();
  savedFactor = factor;
  lastSaveAt = millis();
}

bool requireAuth(WebServer& server);

}  // namespace

void PowerOutput::begin() {
  calibrator.configure(loadConfig());
  calibrator.setPriorFactor(savedFactor);
  log_i("Power output: mode %d, meter scale %.3f, chain loss %.1f %%, saved trainer factor %.4f", (int)calibrator.getConfig().mode,
        calibrator.getConfig().meterScale, calibrator.getConfig().chainLossPercent, savedFactor);
}

namespace {
PowerCal::Input readInputs() {
  PowerCal::Input input;
  input.trainerFresh = BTDeviceManager::isConnected() && (TrainerControl::getMillisSinceLastData() < FRESH_MS);
  input.trainerPower = TrainerControl::getPower();
  input.trainerCadence = TrainerControl::getEffectiveCadence(DirConManager::getDefaultGearRatio());
  input.trainerSpeedKph = TrainerControl::getSpeedMps() * 3.6;
  PowerMeterReading meter = PowerMeter::getReading();
  input.meterFresh = meter.connected && (meter.ageMs < FRESH_MS);
  input.meterPower = meter.power;
  input.meterCadence = meter.cadence;
  return input;
}

void publish(const PowerCal::Output& output) {
  portENTER_CRITICAL(&lock);
  outPower = (int16_t)lround(output.power);
  outCadence = output.cadence;
  outSource = output.source;
  outFactor = calibrator.factor();
  portEXIT_CRITICAL(&lock);
}
}  // namespace

void PowerOutput::update() {
  uint32_t now = millis();
  if ((lastUpdateAt != 0) && (now - lastUpdateAt < UPDATE_INTERVAL_MS)) {
    // between learning steps, keep the output current: alpha.8 refreshed it only once a
    // second, so Zwift could get a reading up to a second old
    if (now - lastRefreshAt >= REFRESH_INTERVAL_MS) {
      lastRefreshAt = now;
      publish(calibrator.compute(readInputs()));
    }
    return;
  }
  lastUpdateAt = now;
  lastRefreshAt = now;

  // a trainer that comes back after being released was probably just spun down: relearn
  bool released = BTDeviceManager::isReleased();
  if (wasReleased && !released) {
    calibrator.reset();
    Telemetry::event(TelemetryEvent::POWER_OUTPUT, "trainer back; relearning factor");
  }
  wasReleased = released;

  PowerCal::Output output = calibrator.update(readInputs());
  publish(output);

  if (output.source != lastLoggedSource) {
    // NONE comes and goes with every stop; only log real switches
    if (output.source != PowerCal::Source::NONE) {
      Telemetry::eventf(TelemetryEvent::POWER_OUTPUT, "source: %s", SOURCE_NAMES[(uint8_t)output.source]);
    }
    lastLoggedSource = output.source;
  }
  if (calibrator.driftWarning() != lastWarning) {
    lastWarning = calibrator.driftWarning();
    Telemetry::eventf(TelemetryEvent::POWER_OUTPUT, lastWarning ? "drift %+.1f%%: spindown advised" : "drift back to %+.1f%%", calibrator.driftPercent());
  }
  if ((calibrator.learnedSeconds() >= 300) && (fabs(calibrator.factor() - savedFactor) >= 0.003) &&
      ((lastSaveAt == 0) || (now - lastSaveAt > SAVE_INTERVAL_MS))) {
    saveFactor(calibrator.factor());
    Telemetry::eventf(TelemetryEvent::POWER_OUTPUT, "factor %.4f saved", calibrator.factor());
  }
}

int16_t PowerOutput::getPower() {
  portENTER_CRITICAL(&lock);
  int16_t value = outPower;
  portEXIT_CRITICAL(&lock);
  return value;
}

double PowerOutput::getCadence() {
  portENTER_CRITICAL(&lock);
  double value = outCadence;
  portEXIT_CRITICAL(&lock);
  return value;
}

PowerCal::Source PowerOutput::getSource() { return outSource; }
const char* PowerOutput::getSourceName() { return SOURCE_NAMES[(uint8_t)outSource < 4 ? (uint8_t)outSource : 3]; }
PowerCal::Mode PowerOutput::getMode() { return calibrator.getConfig().mode; }

double PowerOutput::getFactor() {
  portENTER_CRITICAL(&lock);
  double value = outFactor;
  portEXIT_CRITICAL(&lock);
  return value;
}

double PowerOutput::getBaselineFactor() { return calibrator.baselineFactor(); }
double PowerOutput::getDriftPercent() { return (getFactor() / calibrator.baselineFactor() - 1.0) * 100.0; }
bool PowerOutput::getDriftWarning() { return lastWarning; }
uint32_t PowerOutput::getLearnedSeconds() { return calibrator.learnedSeconds(); }
double PowerOutput::getMeterScale() { return calibrator.getConfig().meterScale; }

double PowerOutput::trainerTargetFor(double target) {
  if (calibrator.getConfig().mode != PowerCal::Mode::POWER_METER) return target;
  return target / getFactor();
}

bool PowerOutput::rewriteForZwift(const NimBLEUUID& characteristicUUID, uint8_t* data, size_t length) {
  if (calibrator.getConfig().mode == PowerCal::Mode::TRAINER) return false;
  portENTER_CRITICAL(&lock);
  int16_t power = outPower;
  double cadence = outCadence;
  PowerCal::Source source = outSource;
  portEXIT_CRITICAL(&lock);
  if (source == PowerCal::Source::NONE) return false;
  if (characteristicUUID.equals(INDOOR_BIKE_DATA)) {
    return TrainerProtocol::rewriteIndoorBikeData(data, length, power, cadence);
  }
  if (characteristicUUID.equals(CYCLING_POWER_MEASUREMENT)) {
    return TrainerProtocol::rewriteCyclingPowerMeasurement(data, length, power);
  }
  return false;
}

void PowerOutput::handleSettingsGet(WebServer& server) {
  const PowerCal::Config& config = calibrator.getConfig();
  char buffer[640];
  snprintf(buffer, sizeof(buffer),
           "{\"mode\": \"%s\", \"meter_scale\": %.3f, \"chain_loss\": %.1f, \"drift_warn\": %.1f, \"factor\": %.4f, \"baseline\": %.4f, "
           "\"drift\": %.2f, \"drift_warning\": %s, \"learned_s\": %lu, \"saved_factor\": %.4f, \"source\": \"%s\", \"power\": %d, "
           "\"trainer_released\": %s, \"spindown_supported\": %s}",
           config.mode == PowerCal::Mode::POWER_METER ? "power_meter" : "trainer", config.meterScale, config.chainLossPercent,
           config.driftWarnPercent, getFactor(), calibrator.baselineFactor(), getDriftPercent(), lastWarning ? "true" : "false",
           (unsigned long)calibrator.learnedSeconds(), savedFactor, getSourceName(), getPower(), BTDeviceManager::isReleased() ? "true" : "false",
           TrainerControl::supportsSpinDown() ? "true" : "false");
  server.send(200, "application/json", buffer);
}

void PowerOutput::handleSettingsPost(WebServer& server) {
  if (!requireAuth(server)) return;
  PowerCal::Config config = calibrator.getConfig();
  if (server.hasArg("mode")) config.mode = (server.arg("mode") == "trainer") ? PowerCal::Mode::TRAINER : PowerCal::Mode::POWER_METER;
  if (server.hasArg("meter_scale")) {
    double value = server.arg("meter_scale").toDouble();
    if ((value >= 0.8) && (value <= 1.25)) config.meterScale = value;
  }
  if (server.hasArg("chain_loss")) {
    double value = server.arg("chain_loss").toDouble();
    if ((value >= 0) && (value <= 10)) config.chainLossPercent = value;
  }
  if (server.hasArg("drift_warn")) {
    double value = server.arg("drift_warn").toDouble();
    if ((value >= 1) && (value <= 20)) config.driftWarnPercent = value;
  }
  bool scaleChanged = config.meterScale != calibrator.getConfig().meterScale;
  calibrator.configure(config);
  saveConfig(config);
  if (scaleChanged) {
    // the learned factor was measured against the old scale
    calibrator.reset();
    saveFactor(0);
  }
  Telemetry::eventf(TelemetryEvent::POWER_OUTPUT, "settings: %s x%.3f loss %.1f%%", config.mode == PowerCal::Mode::POWER_METER ? "meter" : "trainer",
                    config.meterScale, config.chainLossPercent);
  handleSettingsGet(server);
}

void PowerOutput::handleReset(WebServer& server) {
  if (!requireAuth(server)) return;
  calibrator.reset();
  saveFactor(0);
  Telemetry::event(TelemetryEvent::POWER_OUTPUT, "learning reset");
  handleSettingsGet(server);
}

void PowerOutput::handleRelease(WebServer& server) {
  if (!requireAuth(server)) return;
  BTDeviceManager::release();
  Telemetry::event(TelemetryEvent::POWER_OUTPUT, "trainer released");
  handleSettingsGet(server);
}

void PowerOutput::handleResume(WebServer& server) {
  if (!requireAuth(server)) return;
  BTDeviceManager::resume();
  Telemetry::event(TelemetryEvent::POWER_OUTPUT, "trainer reconnecting");
  handleSettingsGet(server);
}

namespace {
bool requireAuth(WebServer& server) {
  if (!server.authenticate(SettingsManager::getUsername().c_str(), SettingsManager::getAPPassword().c_str())) {
    server.requestAuthentication();
    return false;
  }
  return true;
}
}  // namespace
