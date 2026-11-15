#include <BTDeviceManager.h>
#include <Calculations.h>
#include <Config.h>
#include <SettingsManager.h>
#include <TrainerControl.h>
#include <UUIDs.h>

using namespace TrainerProtocol;

portMUX_TYPE TrainerControl::lock = portMUX_INITIALIZER_UNLOCKED;
volatile TrainerProtocolType TrainerControl::protocol = TrainerProtocolType::NONE;
TrainerControl::Command TrainerControl::pendingCommand;
TrainerControl::Command TrainerControl::lastSentCommand;
volatile bool TrainerControl::commandDirty = false;
volatile bool TrainerControl::controlGranted = false;
volatile bool TrainerControl::controlRequestPending = false;
uint32_t TrainerControl::controlRequestedAt = 0;
uint32_t TrainerControl::lastCommandSentAt = 0;
volatile bool TrainerControl::userConfigurationDirty = false;
double TrainerControl::userBicycleWeightKg = 10;
double TrainerControl::userWeightKg = 75;
double TrainerControl::userGearRatio = 34.0 / 14.0;
bool TrainerControl::resistanceRangeKnown = false;
int16_t TrainerControl::resistanceRangeMinimum = 0;
int16_t TrainerControl::resistanceRangeMaximum = 0;

volatile uint16_t TrainerControl::power = 0;
volatile uint32_t TrainerControl::speedMilliMps = 0;
volatile uint8_t TrainerControl::cadence = 0;
volatile bool TrainerControl::cadenceMeasured = false;
volatile uint16_t TrainerControl::maximumResistance = 0;
volatile uint32_t TrainerControl::lastDataAt = 0;

static volatile bool capabilitiesRequestPending = false;

// diagnostics and data source selection
static volatile uint32_t packetsIndoorBikeData = 0;
static volatile uint32_t packetsCyclingPower = 0;
static volatile uint32_t packetsFEC = 0;
static volatile uint32_t ibdPowerAt = 0;     // last non-zero power from FTMS Indoor Bike Data
static volatile uint32_t ibdCadenceAt = 0;   // last non-zero cadence from FTMS Indoor Bike Data
static CrankCadence cyclingPowerCadence;
static bool startSent = false;
static uint32_t lastSummaryAt = 0;
static const uint32_t SOURCE_TIMEOUT = 3000;
static const uint32_t RAW_DUMP_PACKETS = 3;

static void dumpPacket(const char* source, volatile uint32_t& counter, const uint8_t* data, size_t length) {
  counter++;
  if (counter <= RAW_DUMP_PACKETS) {
    log_i("Trainer %s packet %lu: %s", source, (unsigned long)counter, toHex(data, length).c_str());
  }
}

// ---------------------------------------------------------------------------
// Connection lifecycle
// ---------------------------------------------------------------------------

void TrainerControl::onConnected(bool hasFTMS, bool hasFEC) {
  TrainerProtocolPreference preference = SettingsManager::getTrainerProtocolPreference();
  TrainerProtocolType selected = TrainerProtocolType::NONE;
  if (preference == TrainerProtocolPreference::FEC) {
    selected = hasFEC ? TrainerProtocolType::FEC : (hasFTMS ? TrainerProtocolType::FTMS : TrainerProtocolType::NONE);
  } else {
    // AUTO and FTMS both prefer FTMS: it is the trainer's native protocol on Saris
    selected = hasFTMS ? TrainerProtocolType::FTMS : (hasFEC ? TrainerProtocolType::FEC : TrainerProtocolType::NONE);
  }
  log_i("Trainer services: FTMS=%d FE-C=%d, preference=%d -> using %s", hasFTMS, hasFEC, (int)preference,
        selected == TrainerProtocolType::FTMS ? "FTMS" : (selected == TrainerProtocolType::FEC ? "FE-C" : "nothing (pass-through only)"));

  resistanceRangeKnown = false;
  if (selected == TrainerProtocolType::FTMS) {
    // keep these subscribed for the whole connection, regardless of what Zwift subscribes to
    BTDeviceManager::subscribeRemote(NimBLEUUID(FITNESS_MACHINE_SERVICE_UUID), NimBLEUUID(INDOOR_BIKE_DATA_CHARACTERISTIC_UUID));
    BTDeviceManager::subscribeRemote(NimBLEUUID(FITNESS_MACHINE_SERVICE_UUID), NimBLEUUID(FITNESS_MACHINE_CONTROL_POINT_CHARACTERISTIC_UUID));
    BTDeviceManager::subscribeRemote(NimBLEUUID(FITNESS_MACHINE_SERVICE_UUID), NimBLEUUID(FITNESS_MACHINE_STATUS_CHARACTERISTIC_UUID));
    std::vector<uint8_t> range = BTDeviceManager::readBLECharacteristic(NimBLEUUID(FITNESS_MACHINE_SERVICE_UUID), NimBLEUUID(SUPPORTED_RESISTANCE_LEVEL_RANGE_CHARACTERISTIC_UUID));
    ResistanceLevelRange parsedRange;
    if (parseResistanceLevelRange(range.data(), range.size(), parsedRange)) {
      resistanceRangeKnown = true;
      resistanceRangeMinimum = parsedRange.minimum;
      resistanceRangeMaximum = parsedRange.maximum;
      log_i("FTMS resistance level range: %d..%d (0.1 units)", parsedRange.minimum, parsedRange.maximum);
    }
  } else if (selected == TrainerProtocolType::FEC) {
    BTDeviceManager::subscribeRemote(NimBLEUUID(TACX_FEC_PRIMARY_SERVICE_UUID), NimBLEUUID(TACX_FEC_READ_CHARACTERISTIC_UUID));
    capabilitiesRequestPending = true;
  }
  // the power meter service is a second source for power and cadence (crank revolutions)
  BTDeviceManager::subscribeRemote(NimBLEUUID(CYCLING_POWER_SERVICE_UUID), NimBLEUUID(CYCLING_POWER_MEASUREMENT_CHARACTERISTIC_UUID));

  packetsIndoorBikeData = 0;
  packetsCyclingPower = 0;
  packetsFEC = 0;
  ibdPowerAt = 0;
  ibdCadenceAt = 0;
  cyclingPowerCadence.reset();
  startSent = false;
  lastSummaryAt = millis();

  portENTER_CRITICAL(&lock);
  protocol = selected;
  controlGranted = false;
  controlRequestPending = false;
  controlRequestedAt = 0;
  // re-send whatever Zwift asked for last, the trainer forgot it while disconnected
  commandDirty = (pendingCommand.type != CommandType::NONE);
  userConfigurationDirty = true;
  portEXIT_CRITICAL(&lock);
  lastSentCommand = Command();
}

void TrainerControl::onDisconnected() {
  portENTER_CRITICAL(&lock);
  protocol = TrainerProtocolType::NONE;
  controlGranted = false;
  controlRequestPending = false;
  commandDirty = (pendingCommand.type != CommandType::NONE);
  portEXIT_CRITICAL(&lock);
  power = 0;
  speedMilliMps = 0;
  cadence = 0;
  cadenceMeasured = false;
}

// ---------------------------------------------------------------------------
// Desired state (safe to call from any task)
// ---------------------------------------------------------------------------

void TrainerControl::setCommand(const Command& command) {
  portENTER_CRITICAL(&lock);
  pendingCommand = command;
  commandDirty = true;
  portEXIT_CRITICAL(&lock);
}

void TrainerControl::setSimulation(double gradePercent, double crr, double cw) {
  Command command;
  command.type = CommandType::SIMULATION;
  command.value1 = gradePercent;
  command.value2 = crr;
  command.value3 = cw;
  setCommand(command);
}

void TrainerControl::setTargetPower(double watts) {
  Command command;
  command.type = CommandType::TARGET_POWER;
  command.value1 = watts;
  setCommand(command);
}

void TrainerControl::setResistancePercent(double percent) {
  Command command;
  command.type = CommandType::RESISTANCE;
  command.value1 = percent;
  setCommand(command);
}

void TrainerControl::clearCommand() {
  setCommand(Command());
}

void TrainerControl::setUserConfiguration(double bicycleWeightKg, double weightKg, double gearRatio) {
  portENTER_CRITICAL(&lock);
  userBicycleWeightKg = bicycleWeightKg;
  userWeightKg = weightKg;
  userGearRatio = gearRatio;
  userConfigurationDirty = true;
  portEXIT_CRITICAL(&lock);
}

// ---------------------------------------------------------------------------
// Main loop: the only place that writes to the trainer
// ---------------------------------------------------------------------------

void TrainerControl::update() {
  if ((protocol == TrainerProtocolType::NONE) || !BTDeviceManager::isConnected()) {
    return;
  }
  uint32_t now = millis();

  if (now - lastSummaryAt > 10000) {
    lastSummaryAt = now;
    log_i("Trainer data: FTMS %lu, CPS %lu, FE-C %lu packets | power %u W, cadence %u rpm (%s), speed %.1f km/h | control %s | last command: %s",
          (unsigned long)packetsIndoorBikeData, (unsigned long)packetsCyclingPower, (unsigned long)packetsFEC,
          power, cadence, cadenceMeasured ? "measured" : "none", getSpeedMps() * 3.6,
          hasControl() ? "yes" : "no", describe(lastSentCommand).c_str());
  }

  if (protocol == TrainerProtocolType::FEC) {
    if (capabilitiesRequestPending) {
      capabilitiesRequestPending = false;
      sendFEC(fecCapabilitiesRequest());
    }
    if (userConfigurationDirty) {
      userConfigurationDirty = false;
      sendFEC(fecUserConfiguration(userBicycleWeightKg, userWeightKg, Calculations::wheelDiameter, userGearRatio));
    }
  }

  Command command;
  bool dirty;
  portENTER_CRITICAL(&lock);
  command = pendingCommand;
  dirty = commandDirty;
  portEXIT_CRITICAL(&lock);

  if (command.type == CommandType::NONE) {
    return;
  }

  // FTMS needs "Request Control" granted before it accepts commands
  if ((protocol == TrainerProtocolType::FTMS) && !controlGranted) {
    if (!controlRequestPending) {
      if ((controlRequestedAt != 0) && (now - controlRequestedAt < TRAINER_CONTROL_RETRY_INTERVAL)) {
        return;  // refused or failed recently, try again a bit later
      }
      log_i("Requesting FTMS control");
      controlRequestedAt = now;
      controlRequestPending = sendFTMS(ftmsRequestControl());
      return;
    }
    if (now - controlRequestedAt > TRAINER_CONTROL_INDICATION_TIMEOUT) {
      // some firmwares never indicate the response, carry on and find out from the next result
      log_w("No FTMS control response, assuming control was granted");
      controlGranted = true;
      controlRequestPending = false;
    } else {
      return;
    }
  }

  // rate limit, coalesce and keep alive
  if (now - lastCommandSentAt < TRAINER_COMMAND_MIN_INTERVAL) {
    return;
  }
  // FTMS "Start or Resume" once per connection: some trainers only stream data / apply sim after it
  if ((protocol == TrainerProtocolType::FTMS) && !startSent) {
    startSent = true;
    lastCommandSentAt = now;
    log_i("Sending FTMS start/resume");
    sendFTMS(ftmsStartOrResume());
    return;
  }
  bool keepAliveDue = (now - lastCommandSentAt) > TRAINER_COMMAND_KEEPALIVE_INTERVAL;
  if (!dirty && !keepAliveDue) {
    return;
  }
  portENTER_CRITICAL(&lock);
  if (sameCommand(command, pendingCommand)) {
    commandDirty = false;
  }
  portEXIT_CRITICAL(&lock);

  lastCommandSentAt = now;
  if (sendCommand(command)) {
    lastSentCommand = command;
  } else {
    log_e("Sending '%s' to trainer failed, will retry", describe(command).c_str());
    portENTER_CRITICAL(&lock);
    commandDirty = true;
    portEXIT_CRITICAL(&lock);
  }
}

bool TrainerControl::sendCommand(const Command& command) {
  if (protocol == TrainerProtocolType::FTMS) {
    switch (command.type) {
      case CommandType::SIMULATION:
        return sendFTMS(ftmsIndoorBikeSimulation(command.value1, command.value2, command.value3));
      case CommandType::TARGET_POWER:
        return sendFTMS(ftmsTargetPower(command.value1));
      case CommandType::RESISTANCE:
        return sendFTMS(ftmsTargetResistanceLevel(command.value1, resistanceRangeKnown, resistanceRangeMinimum, resistanceRangeMaximum));
      default:
        return true;
    }
  }
  if (protocol == TrainerProtocolType::FEC) {
    switch (command.type) {
      case CommandType::SIMULATION:
        // set the trainer's air drag explicitly, otherwise it keeps its own default (0.51 kg/m)
        sendFEC(fecWindResistance(command.value3));
        return sendFEC(fecTrackResistance(command.value1, command.value2));
      case CommandType::TARGET_POWER:
        return sendFEC(fecTargetPower(command.value1));
      case CommandType::RESISTANCE:
        return sendFEC(fecBasicResistance(command.value1));
      default:
        return true;
    }
  }
  return false;
}

bool TrainerControl::sendFTMS(const std::vector<uint8_t>& data) {
  std::vector<uint8_t> copy = data;
  return BTDeviceManager::writeBLECharacteristic(NimBLEUUID(FITNESS_MACHINE_SERVICE_UUID), NimBLEUUID(FITNESS_MACHINE_CONTROL_POINT_CHARACTERISTIC_UUID), &copy, true);
}

bool TrainerControl::sendFEC(const std::vector<uint8_t>& data) {
  std::vector<uint8_t> copy = data;
  return BTDeviceManager::writeBLECharacteristic(NimBLEUUID(TACX_FEC_PRIMARY_SERVICE_UUID), NimBLEUUID(TACX_FEC_WRITE_CHARACTERISTIC_UUID), &copy, false);
}

// ---------------------------------------------------------------------------
// Data from the trainer (runs in the NimBLE task)
// ---------------------------------------------------------------------------

void TrainerControl::onNotify(const NimBLEUUID& characteristicUUID, const uint8_t* data, size_t length) {
  uint32_t now = millis();
  if (characteristicUUID.equals(NimBLEUUID(INDOOR_BIKE_DATA_CHARACTERISTIC_UUID))) {
    dumpPacket("FTMS bike data", packetsIndoorBikeData, data, length);
    IndoorBikeData bikeData;
    if (parseIndoorBikeData(data, length, bikeData)) {
      if (bikeData.hasSpeed) speedMilliMps = (uint32_t)(bikeData.speedKph / 3.6 * 1000.0);
      if (bikeData.hasCadence && (bikeData.cadenceRpm > 0)) {
        cadence = (uint8_t)(bikeData.cadenceRpm > 255 ? 255 : bikeData.cadenceRpm);
        cadenceMeasured = true;
        ibdCadenceAt = now;
      } else if (bikeData.hasCadence && (now - ibdCadenceAt > SOURCE_TIMEOUT) && (cyclingPowerCadence.current(now) == 0)) {
        cadence = 0;
      }
      if (bikeData.hasPower && (bikeData.powerWatts > 0)) {
        power = (uint16_t)bikeData.powerWatts;
        ibdPowerAt = now;
      } else if (bikeData.hasPower && (now - ibdPowerAt > SOURCE_TIMEOUT) && (packetsCyclingPower == 0)) {
        power = 0;
      }
      lastDataAt = now;
    } else {
      log_w("Could not parse FTMS bike data: %s", toHex(data, length).c_str());
    }
  } else if (characteristicUUID.equals(NimBLEUUID(CYCLING_POWER_MEASUREMENT_CHARACTERISTIC_UUID))) {
    dumpPacket("power meter", packetsCyclingPower, data, length);
    CyclingPowerData powerData;
    if (parseCyclingPowerMeasurement(data, length, powerData)) {
      // FTMS values win while they are non-zero, the power meter fills the gaps
      if (now - ibdPowerAt > SOURCE_TIMEOUT) {
        power = (uint16_t)(powerData.powerWatts < 0 ? 0 : powerData.powerWatts);
      }
      if (powerData.hasCrank) {
        double crankCadence = cyclingPowerCadence.update(powerData.crankRevolutions, powerData.crankEventTime, now);
        if (now - ibdCadenceAt > SOURCE_TIMEOUT) {
          cadence = (uint8_t)round(crankCadence);
          cadenceMeasured = true;
        }
      }
      lastDataAt = now;
    }
  } else if (characteristicUUID.equals(NimBLEUUID(TACX_FEC_READ_CHARACTERISTIC_UUID))) {
    dumpPacket("FE-C", packetsFEC, data, length);
    FECData fecData;
    if (parseFECMessage(data, length, fecData)) {
      if (fecData.hasSpeed) speedMilliMps = (uint32_t)(fecData.speedMps * 1000.0);
      if (fecData.hasCadence) {
        cadence = fecData.cadenceRpm;
        cadenceMeasured = true;
      }
      if (fecData.hasPower) power = fecData.powerWatts;
      if (fecData.hasMaximumResistance) maximumResistance = fecData.maximumResistanceNewton;
      lastDataAt = now;
    }
  } else if (characteristicUUID.equals(NimBLEUUID(FITNESS_MACHINE_CONTROL_POINT_CHARACTERISTIC_UUID))) {
    handleControlPointResponse(data, length);
  } else if (characteristicUUID.equals(NimBLEUUID(FITNESS_MACHINE_STATUS_CHARACTERISTIC_UUID))) {
    if ((length > 0) && ((data[0] == FTMS_STATUS_CONTROL_PERMISSION_LOST) || (data[0] == FTMS_STATUS_RESET))) {
      log_w("FTMS control lost (status 0x%02X), will request it again", data[0]);
      portENTER_CRITICAL(&lock);
      startSent = false;
      controlGranted = false;
      controlRequestPending = false;
      commandDirty = (pendingCommand.type != CommandType::NONE);
      portEXIT_CRITICAL(&lock);
    }
  }
}

void TrainerControl::handleControlPointResponse(const uint8_t* data, size_t length) {
  ControlPointResponse response;
  if (!parseControlPointResponse(data, length, response)) {
    return;
  }
  if (response.requestOpCode == FTMS_REQUEST_CONTROL) {
    controlRequestPending = false;
    controlGranted = (response.resultCode == FTMS_RESULT_SUCCESS);
    log_i("FTMS control %s (result %d)", controlGranted ? "granted" : "refused", response.resultCode);
    return;
  }
  if (response.resultCode == FTMS_RESULT_CONTROL_NOT_PERMITTED) {
    log_w("FTMS command 0x%02X refused: control not permitted, requesting control again", response.requestOpCode);
    portENTER_CRITICAL(&lock);
    controlGranted = false;
    controlRequestPending = false;
    commandDirty = true;
    portEXIT_CRITICAL(&lock);
  } else if (response.resultCode != FTMS_RESULT_SUCCESS) {
    log_w("FTMS command 0x%02X failed with result %d", response.requestOpCode, response.resultCode);
  }
}

// ---------------------------------------------------------------------------
// Getters
// ---------------------------------------------------------------------------

TrainerProtocolType TrainerControl::getProtocol() {
  return protocol;
}

const char* TrainerControl::getProtocolName() {
  switch (protocol) {
    case TrainerProtocolType::FTMS:
      return "FTMS";
    case TrainerProtocolType::FEC:
      return "FE-C";
    default:
      return "none";
  }
}

bool TrainerControl::hasControl() {
  return (protocol == TrainerProtocolType::FEC) || controlGranted;
}

String TrainerControl::getLastCommandDescription() {
  return describe(lastSentCommand);
}

String TrainerControl::describe(const Command& command) {
  char buffer[48];
  switch (command.type) {
    case CommandType::SIMULATION:
      snprintf(buffer, sizeof(buffer), "Grade %.2f %%", command.value1);
      break;
    case CommandType::TARGET_POWER:
      snprintf(buffer, sizeof(buffer), "Target power %.0f W", command.value1);
      break;
    case CommandType::RESISTANCE:
      snprintf(buffer, sizeof(buffer), "Resistance %.1f %%", command.value1);
      break;
    default:
      snprintf(buffer, sizeof(buffer), "-");
      break;
  }
  return String(buffer);
}

bool TrainerControl::sameCommand(const Command& a, const Command& b) {
  return (a.type == b.type) && (a.value1 == b.value1) && (a.value2 == b.value2) && (a.value3 == b.value3);
}

uint16_t TrainerControl::getPower() {
  return power;
}

double TrainerControl::getSpeedMps() {
  return speedMilliMps / 1000.0;
}

uint8_t TrainerControl::getMeasuredCadence() {
  return cadence;
}

bool TrainerControl::isCadenceMeasured() {
  return cadenceMeasured && (cadence > 0);
}

uint8_t TrainerControl::getEffectiveCadence(double physicalGearRatio) {
  if (isCadenceMeasured()) {
    return cadence;
  }
  double estimated = Calculations::estimateCadenceFromSpeed(getSpeedMps(), physicalGearRatio, Calculations::wheelDiameter);
  if (estimated > 255) estimated = 255;
  return (uint8_t)round(estimated);
}

uint16_t TrainerControl::getMaximumResistance() {
  return maximumResistance;
}

uint32_t TrainerControl::getMillisSinceLastData() {
  if (lastDataAt == 0) {
    return UINT32_MAX;
  }
  return millis() - lastDataAt;
}

String TrainerControl::getPacketSummary() {
  char buffer[64];
  snprintf(buffer, sizeof(buffer), "FTMS %lu / power meter %lu / FE-C %lu",
           (unsigned long)packetsIndoorBikeData, (unsigned long)packetsCyclingPower, (unsigned long)packetsFEC);
  return String(buffer);
}
