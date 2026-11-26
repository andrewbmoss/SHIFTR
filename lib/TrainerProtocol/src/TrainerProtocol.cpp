#include <TrainerProtocol.h>

#include <cmath>

namespace TrainerProtocol {

namespace {

double clamp(double value, double minimum, double maximum) {
  if (std::isnan(value)) {
    return 0.0;
  }
  if (value < minimum) {
    return minimum;
  }
  if (value > maximum) {
    return maximum;
  }
  return value;
}

void pushUInt16(std::vector<uint8_t>& data, uint16_t value) {
  data.push_back((uint8_t)(value & 0xFF));
  data.push_back((uint8_t)(value >> 8));
}

void pushSInt16(std::vector<uint8_t>& data, int16_t value) {
  pushUInt16(data, (uint16_t)value);
}

uint16_t readUInt16(const uint8_t* data) {
  return (uint16_t)(data[0] | (data[1] << 8));
}

int16_t readSInt16(const uint8_t* data) {
  return (int16_t)readUInt16(data);
}

// Wraps 8 bytes of FE-C page data into a FE-C over BLE message.
std::vector<uint8_t> fecMessage(uint8_t messageId, const uint8_t page[8]) {
  std::vector<uint8_t> message;
  message.push_back(0xA4);       // SYNC
  message.push_back(0x09);       // MSG_LEN
  message.push_back(messageId);  // 0x4E broadcast, 0x4F acknowledged
  message.push_back(0x05);       // channel
  for (int index = 0; index < 8; index++) {
    message.push_back(page[index]);
  }
  message.push_back(fecChecksum(message));
  return message;
}

}  // namespace

// ---------------------------------------------------------------------------
// FTMS
// ---------------------------------------------------------------------------

std::vector<uint8_t> ftmsRequestControl() {
  return {FTMS_REQUEST_CONTROL};
}

std::vector<uint8_t> ftmsStartOrResume() {
  return {FTMS_START_OR_RESUME};
}

std::vector<uint8_t> ftmsReset() {
  return {FTMS_RESET};
}

std::vector<uint8_t> ftmsTargetPower(double watts) {
  std::vector<uint8_t> data{FTMS_SET_TARGET_POWER};
  pushSInt16(data, (int16_t)std::lround(clamp(watts, 0.0, 4000.0)));
  return data;
}

std::vector<uint8_t> ftmsIndoorBikeSimulation(double gradePercent, double crr, double cw, double windSpeed) {
  std::vector<uint8_t> data{FTMS_SET_INDOOR_BIKE_SIMULATION};
  pushSInt16(data, (int16_t)std::lround(clamp(windSpeed, -32.0, 32.0) * 1000.0));
  pushSInt16(data, (int16_t)std::lround(clamp(gradePercent, MIN_GRADE_PERCENT, MAX_GRADE_PERCENT) * 100.0));
  data.push_back((uint8_t)std::lround(clamp(crr / 0.0001, 0.0, 255.0)));
  data.push_back((uint8_t)std::lround(clamp(cw / 0.01, 0.0, 255.0)));
  return data;
}

std::vector<uint8_t> ftmsTargetResistanceLevel(double percent, bool rangeKnown, int16_t minRaw, int16_t maxRaw) {
  percent = clamp(percent, 0.0, 100.0);
  double raw = percent;
  if (rangeKnown && (maxRaw > minRaw)) {
    raw = minRaw + (maxRaw - minRaw) * (percent / 100.0);
  }
  std::vector<uint8_t> data{FTMS_SET_TARGET_RESISTANCE_LEVEL};
  data.push_back((uint8_t)std::lround(clamp(raw, 0.0, 255.0)));
  return data;
}

bool parseIndoorBikeData(const uint8_t* data, size_t length, IndoorBikeData& result) {
  result = IndoorBikeData();
  if ((data == nullptr) || (length < 2)) {
    return false;
  }
  uint16_t flags = readUInt16(data);
  size_t offset = 2;

  // bit 0 "More Data": 0 means the instantaneous speed field IS present
  if ((flags & 0x0001) == 0) {
    if (offset + 2 > length) return false;
    result.speedKph = readUInt16(data + offset) / 100.0;
    result.hasSpeed = true;
    offset += 2;
  }
  if (flags & 0x0002) {  // average speed
    offset += 2;
  }
  if (flags & 0x0004) {  // instantaneous cadence, 0.5 rpm
    if (offset + 2 > length) return false;
    result.cadenceRpm = readUInt16(data + offset) / 2.0;
    result.hasCadence = true;
    offset += 2;
  }
  if (flags & 0x0008) {  // average cadence
    offset += 2;
  }
  if (flags & 0x0010) {  // total distance (UINT24)
    offset += 3;
  }
  if (flags & 0x0020) {  // resistance level
    offset += 2;
  }
  if (flags & 0x0040) {  // instantaneous power
    if (offset + 2 > length) return false;
    result.powerWatts = readSInt16(data + offset);
    result.hasPower = true;
    offset += 2;
  }
  return true;
}

bool parseControlPointResponse(const uint8_t* data, size_t length, ControlPointResponse& result) {
  if ((data == nullptr) || (length < 3) || (data[0] != FTMS_RESPONSE_CODE)) {
    return false;
  }
  result.requestOpCode = data[1];
  result.resultCode = data[2];
  return true;
}

bool parseResistanceLevelRange(const uint8_t* data, size_t length, ResistanceLevelRange& result) {
  if ((data == nullptr) || (length < 6)) {
    return false;
  }
  result.minimum = readSInt16(data);
  result.maximum = readSInt16(data + 2);
  result.increment = readUInt16(data + 4);
  return result.maximum > result.minimum;
}

// ---------------------------------------------------------------------------
// Cycling Power
// ---------------------------------------------------------------------------

bool parseCyclingPowerMeasurement(const uint8_t* data, size_t length, CyclingPowerData& result) {
  result = CyclingPowerData();
  if ((data == nullptr) || (length < 4)) {
    return false;
  }
  uint16_t flags = readUInt16(data);
  result.flags = flags;
  result.powerWatts = readSInt16(data + 2);
  result.balanceReferenceLeft = (flags & 0x0002) != 0;
  result.torqueFromCrank = (flags & 0x0008) != 0;
  result.offsetCompensationIndicator = (flags & 0x1000) != 0;
  size_t offset = 4;
  // fields follow in flag order; a truncated packet still has a valid power value
  if (flags & 0x0001) {  // pedal power balance, 1/2 %
    if (offset + 1 > length) return true;
    result.balancePercent = data[offset] / 2.0;
    result.hasBalance = (data[offset] != 0xFF);
    offset += 1;
  }
  if (flags & 0x0004) {  // accumulated torque, 1/32 Nm
    if (offset + 2 > length) return true;
    result.accumulatedTorque = readUInt16(data + offset);
    result.hasAccumulatedTorque = true;
    offset += 2;
  }
  if (flags & 0x0010) {  // wheel revolution data
    if (offset + 6 > length) return true;
    result.wheelRevolutions = (uint32_t)data[offset] | ((uint32_t)data[offset + 1] << 8) | ((uint32_t)data[offset + 2] << 16) | ((uint32_t)data[offset + 3] << 24);
    result.wheelEventTime = readUInt16(data + offset + 4);
    result.hasWheel = true;
    offset += 6;
  }
  if (flags & 0x0020) {  // crank revolution data
    if (offset + 4 > length) return true;
    result.crankRevolutions = readUInt16(data + offset);
    result.crankEventTime = readUInt16(data + offset + 2);
    result.hasCrank = true;
    offset += 4;
  }
  if (flags & 0x0040) offset += 4;  // extreme force magnitudes
  if (flags & 0x0080) offset += 4;  // extreme torque magnitudes
  if (flags & 0x0100) offset += 3;  // extreme angles
  if (flags & 0x0200) offset += 2;  // top dead spot angle
  if (flags & 0x0400) offset += 2;  // bottom dead spot angle
  if (flags & 0x0800) {             // accumulated energy, kJ
    if (offset + 2 > length) return true;
    result.accumulatedEnergyKJ = readUInt16(data + offset);
    result.hasAccumulatedEnergy = true;
    offset += 2;
  }
  return true;
}

std::string describeCyclingPowerFeatures(uint32_t features) {
  struct Bit {
    uint8_t bit;
    const char* name;
  };
  static const Bit bits[] = {{0, "pedal_balance"}, {1, "accumulated_torque"}, {2, "wheel_revolutions"}, {3, "crank_revolutions"},
                             {4, "extreme_magnitudes"}, {5, "extreme_angles"}, {6, "dead_spot_angles"}, {7, "accumulated_energy"},
                             {8, "offset_compensation_indicator"}, {9, "offset_compensation"}, {10, "content_masking"},
                             {11, "multiple_sensor_locations"}, {12, "crank_length_adjustment"}, {13, "chain_length_adjustment"},
                             {14, "chain_weight_adjustment"}, {15, "span_length_adjustment"}, {16, "torque_based"},
                             {19, "factory_calibration_date"}, {20, "enhanced_offset_compensation"}};
  std::string text;
  for (const Bit& entry : bits) {
    if (features & (1u << entry.bit)) {
      if (!text.empty()) text += ", ";
      text += entry.name;
    }
  }
  uint32_t direction = (features >> 17) & 0x3;
  if (direction != 0) {
    if (!text.empty()) text += ", ";
    text += "instantaneous_direction_" + std::to_string(direction);
  }
  return text;
}

double CrankCadence::update(uint16_t revolutions, uint16_t eventTime1024, uint32_t nowMs) {
  if (!initialised) {
    initialised = true;
    lastRevolutions = revolutions;
    lastEventTime = eventTime1024;
    lastChangeMs = nowMs;
    return cadence;
  }
  uint16_t deltaRevolutions = (uint16_t)(revolutions - lastRevolutions);
  uint16_t deltaTime = (uint16_t)(eventTime1024 - lastEventTime);
  if ((deltaRevolutions > 0) && (deltaTime > 0) && (deltaRevolutions < 100)) {
    cadence = deltaRevolutions * 60.0 * 1024.0 / deltaTime;
    if (cadence > 250) {
      cadence = 250;
    }
    lastRevolutions = revolutions;
    lastEventTime = eventTime1024;
    lastChangeMs = nowMs;
  } else if (deltaRevolutions >= 100) {
    // implausible jump (e.g. sensor restart), resynchronise
    lastRevolutions = revolutions;
    lastEventTime = eventTime1024;
    lastChangeMs = nowMs;
  }
  return current(nowMs);
}

double CrankCadence::current(uint32_t nowMs) const {
  if (!initialised || (nowMs - lastChangeMs > timeoutMs)) {
    return 0;
  }
  return cadence;
}

void CrankCadence::reset() {
  initialised = false;
  cadence = 0;
}

bool rewriteIndoorBikeData(uint8_t* data, size_t length, int16_t powerWatts, double cadenceRpm) {
  if ((data == nullptr) || (length < 2)) return false;
  uint16_t flags = data[0] | (data[1] << 8);
  size_t offset = 2;
  if (!(flags & 0x0001)) offset += 2;  // instantaneous speed (present when "more data" is 0)
  if (flags & 0x0002) offset += 2;     // average speed
  if (flags & 0x0004) {                // instantaneous cadence, 0.5 rpm
    if (offset + 2 > length) return false;
    long value = lround(cadenceRpm * 2.0);
    if (value < 0) value = 0;
    if (value > 0xFFFF) value = 0xFFFF;
    data[offset] = value & 0xFF;
    data[offset + 1] = (value >> 8) & 0xFF;
    offset += 2;
  }
  if (flags & 0x0008) offset += 2;  // average cadence
  if (flags & 0x0010) offset += 3;  // total distance
  if (flags & 0x0020) offset += 2;  // resistance level
  if (flags & 0x0040) {             // instantaneous power
    if (offset + 2 > length) return false;
    data[offset] = (uint16_t)powerWatts & 0xFF;
    data[offset + 1] = ((uint16_t)powerWatts >> 8) & 0xFF;
  }
  return true;
}

bool rewriteCyclingPowerMeasurement(uint8_t* data, size_t length, int16_t powerWatts) {
  if ((data == nullptr) || (length < 4)) return false;
  data[2] = (uint16_t)powerWatts & 0xFF;
  data[3] = ((uint16_t)powerWatts >> 8) & 0xFF;
  return true;
}

std::vector<uint8_t> ftmsSpinDownControl(uint8_t parameter) { return {FTMS_SPIN_DOWN_CONTROL, parameter}; }

bool parseSpinDownResponse(const uint8_t* data, size_t length, uint8_t& result, double& lowKph, double& highKph) {
  if ((data == nullptr) || (length < 3) || (data[0] != FTMS_RESPONSE_CODE) || (data[1] != FTMS_SPIN_DOWN_CONTROL)) return false;
  result = data[2];
  lowKph = 0;
  highKph = 0;
  if (length >= 7) {
    lowKph = (data[3] | (data[4] << 8)) / 100.0;
    highKph = (data[5] | (data[6] << 8)) / 100.0;
  }
  return true;
}

void SpinDown::start(uint32_t now) {
  state = State::STARTING;
  startedAt = now;
  stateAt = now;
  targetLow = 0;
  targetHigh = 0;
  requestedByTrainer = false;
  message = "asking the trainer";
}

void SpinDown::fail(const char* reason, uint32_t now) {
  state = State::FAILED;
  stateAt = now;
  message = reason;
}

void SpinDown::onResponse(uint8_t result, double lowKph, double highKph, uint32_t now) {
  if (state != State::STARTING) return;
  if (result != FTMS_RESULT_SUCCESS) {
    static const char* const reasons[] = {"start refused", "start refused", "start refused: the trainer doesn't support it", "start refused: invalid parameter",
                                          "start refused: the trainer couldn't start it", "start refused: no control (another app connected?)"};
    fail((result < 6) ? reasons[result] : "start refused", now);
    return;
  }
  state = State::SPEED_UP;
  stateAt = now;
  targetLow = lowKph;
  targetHigh = highKph;
  message = "speed up";
}

void SpinDown::onStatus(uint8_t status, uint32_t now) {
  if (!active()) {
    if ((status == FTMS_SPIN_DOWN_STATUS_REQUESTED) && (state == State::IDLE)) requestedByTrainer = true;
    return;
  }
  switch (status) {
    case FTMS_SPIN_DOWN_STATUS_STOP_PEDALING:
      // some trainers skip the response's speed range and go straight here
      state = State::COAST;
      stateAt = now;
      message = "stop pedalling";
      break;
    case FTMS_SPIN_DOWN_STATUS_SUCCESS:
      state = State::SUCCESS;
      stateAt = now;
      message = "done";
      requestedByTrainer = false;
      break;
    case FTMS_SPIN_DOWN_STATUS_ERROR:
      fail("the trainer reported an error (pedalled during the coast, or the speed was too low)", now);
      break;
    default:
      break;
  }
}

void SpinDown::tick(uint32_t now) {
  if ((state == State::STARTING) && (now - stateAt > ANSWER_TIMEOUT_MS)) {
    fail("no answer from the trainer", now);
  } else if (((state == State::SPEED_UP) || (state == State::COAST)) && (now - startedAt > PROCEDURE_TIMEOUT_MS)) {
    fail("timed out after 2 minutes", now);
  } else if (((state == State::SUCCESS) || (state == State::FAILED)) && (now - stateAt > RESULT_SHOWN_MS)) {
    state = State::IDLE;
    message = "";
  }
}

void SpinDown::cancel(uint32_t now) {
  if (active()) fail("cancelled", now);
}

const char* SpinDown::getStateName() const {
  static const char* const names[] = {"idle", "starting", "speed_up", "coast", "success", "failed"};
  return names[(uint8_t)state < 6 ? (uint8_t)state : 0];
}

std::string toHex(const uint8_t* data, size_t length) {
  static const char digits[] = "0123456789ABCDEF";
  std::string text;
  for (size_t index = 0; index < length; index++) {
    if (index > 0) text += ' ';
    text += digits[data[index] >> 4];
    text += digits[data[index] & 0x0F];
  }
  return text;
}

// ---------------------------------------------------------------------------
// FE-C over BLE
// ---------------------------------------------------------------------------

uint8_t fecChecksum(const std::vector<uint8_t>& message) {
  uint8_t checksum = 0;
  for (uint8_t value : message) {
    checksum ^= value;
  }
  return checksum;
}

std::vector<uint8_t> fecTargetPower(double watts) {
  uint16_t value = (uint16_t)std::lround(clamp(watts, 0.0, 4000.0) * 4.0);
  const uint8_t page[8] = {0x31, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, (uint8_t)(value & 0xFF), (uint8_t)(value >> 8)};
  return fecMessage(0x4E, page);
}

std::vector<uint8_t> fecTrackResistance(double gradePercent, double crr) {
  uint16_t grade = (uint16_t)std::lround((clamp(gradePercent, MIN_GRADE_PERCENT, MAX_GRADE_PERCENT) + 200.0) * 100.0);
  uint8_t crrValue = (uint8_t)std::lround(clamp(crr / 0.00005, 0.0, 254.0));
  const uint8_t page[8] = {0x33, 0xFF, 0xFF, 0xFF, 0xFF, (uint8_t)(grade & 0xFF), (uint8_t)(grade >> 8), crrValue};
  return fecMessage(0x4E, page);
}

std::vector<uint8_t> fecWindResistance(double cw) {
  uint8_t coefficient = (uint8_t)std::lround(clamp(cw / 0.01, 0.0, 186.0));
  // wind speed 127 = 0 km/h, drafting factor 100 = no drafting
  const uint8_t page[8] = {0x32, 0xFF, 0xFF, 0xFF, 0xFF, coefficient, 127, 100};
  return fecMessage(0x4E, page);
}

std::vector<uint8_t> fecBasicResistance(double percent) {
  uint8_t value = (uint8_t)std::lround(clamp(percent, 0.0, 100.0) * 2.0);
  const uint8_t page[8] = {0x30, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, value};
  return fecMessage(0x4E, page);
}

std::vector<uint8_t> fecUserConfiguration(double bicycleWeightKg, double userWeightKg, double wheelDiameterMeters, double gearRatio) {
  uint16_t userWeight = (uint16_t)std::lround(clamp(userWeightKg, 0.0, 655.0) * 100.0);   // 0.01 kg
  uint16_t bicycleWeight = (uint16_t)std::lround(clamp(bicycleWeightKg, 0.0, 50.0) * 20.0);  // 0.05 kg, 12 bit
  uint8_t wheelDiameter = (uint8_t)std::lround(clamp(wheelDiameterMeters, 0.0, 2.54) * 100.0);  // 0.01 m
  uint8_t gear = (uint8_t)std::lround(clamp(gearRatio, 0.03, 7.65) / 0.03);  // 0.03
  const uint8_t page[8] = {
      0x37,
      (uint8_t)(userWeight & 0xFF),
      (uint8_t)(userWeight >> 8),
      0xFF,
      (uint8_t)((bicycleWeight & 0x0F) << 4),  // low nibble of bike weight, wheel offset 0
      (uint8_t)(bicycleWeight >> 4),
      wheelDiameter,
      gear};
  return fecMessage(0x4F, page);
}

std::vector<uint8_t> fecCapabilitiesRequest() {
  const uint8_t page[8] = {0x46, 0xFF, 0xFF, 0xFF, 0xFF, 0x80, 0x36, 0x01};
  return fecMessage(0x4F, page);
}

bool parseFECMessage(const uint8_t* data, size_t length, FECData& result) {
  result = FECData();
  if ((data == nullptr) || (length != 13) || (data[0] != 0xA4)) {
    return false;
  }
  result.page = data[4];
  switch (result.page) {
    case 0x10:  // page 16, general FE data: speed in 0.001 m/s
      result.speedMps = ((data[9] << 8) | data[8]) / 1000.0;
      result.hasSpeed = true;
      break;
    case 0x19:  // page 25, stationary bike data
      result.cadenceRpm = data[6];
      result.hasCadence = (data[6] != 0xFF);
      result.powerWatts = (uint16_t)(((data[10] & 0x0F) << 8) | data[9]);
      result.hasPower = true;
      break;
    case 0x36:  // page 54, capabilities: maximum resistance in N
      result.maximumResistanceNewton = (uint16_t)((data[10] << 8) | data[9]);
      result.hasMaximumResistance = (result.maximumResistanceNewton != 0xFFFF);
      break;
    default:
      break;
  }
  return true;
}

}  // namespace TrainerProtocol
