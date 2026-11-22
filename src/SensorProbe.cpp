#include <NimBLEDevice.h>
#include <SensorProbe.h>
#include <BTDeviceManager.h>
#include <TrainerControl.h>
#include <TrainerProtocol.h>
#include <esp_heap_caps.h>

#include <mutex>
#include <vector>

namespace {

constexpr uint32_t SCAN_SECONDS = 12;
constexpr uint32_t LISTEN_MS = 30000;
constexpr size_t MAX_SCAN_ENTRIES = 30;
constexpr size_t MAX_PACKETS = 40;
constexpr int CONNECT_ATTEMPTS = 4;

const NimBLEUUID CPS_SERVICE((uint16_t)0x1818);
const NimBLEUUID CPS_MEASUREMENT((uint16_t)0x2A63);
const NimBLEUUID CPS_FEATURE((uint16_t)0x2A65);
const NimBLEUUID CPS_SENSOR_LOCATION((uint16_t)0x2A5D);
const NimBLEUUID CPS_CONTROL_POINT((uint16_t)0x2A66);
const NimBLEUUID DIS_SERVICE((uint16_t)0x180A);
const NimBLEUUID BATTERY_SERVICE((uint16_t)0x180F);
const NimBLEUUID BATTERY_LEVEL((uint16_t)0x2A19);

struct ScanEntry {
  NimBLEAddress address;
  std::string name;
  int rssi = 0;
  std::string services;
  std::string manufacturerData;
  bool cyclingPower = false;
  bool isTrainer = false;
};

struct HeapSnapshot {
  uint32_t free = 0;
  uint32_t minimum = 0;
  uint32_t largestBlock = 0;
};

struct Packet {
  uint32_t ms;
  std::string hex;
  TrainerProtocol::CyclingPowerData data;
  double cadence;
};

enum class State { IDLE, SCANNING, CONNECTING, LISTENING, DONE, FAILED };

std::mutex lock;
State state = State::IDLE;
String failure;
std::string requestedAddress;
std::string trainerAddress;
std::vector<ScanEntry> scanEntries;
std::string chosenAddress;
std::string chosenName;
int connectionRssi = 0;
uint32_t connectMs = 0;
int connectAttempts = 0;
std::string connectErrors;
std::vector<std::string> attributeLines;  // "service/characteristic props"
std::vector<std::pair<std::string, std::string>> deviceInfo;
uint32_t cpFeatures = 0;
bool cpFeaturesRead = false;
int sensorLocation = -1;
int batteryLevel = -1;
bool hasControlPoint = false;
bool subscribed = false;
std::vector<Packet> packets;
uint32_t packetCount = 0;
uint32_t listenStartMs = 0;
TrainerProtocol::CrankCadence crankCadence(4000);
HeapSnapshot heapBefore, heapConnected, heapAfter;
String trainerBefore, trainerAfter;
bool trainerConnectedBefore = false, trainerConnectedAfter = false;
TaskHandle_t taskHandle = nullptr;

HeapSnapshot snapshotHeap() {
  HeapSnapshot snapshot;
  snapshot.free = ESP.getFreeHeap();
  snapshot.minimum = ESP.getMinFreeHeap();
  snapshot.largestBlock = heap_caps_get_largest_free_block(MALLOC_CAP_8BIT);
  return snapshot;
}

std::string clean(const std::string& text) {
  std::string out;
  for (char c : text) {
    if ((c >= 0x20) && (c < 0x7F) && (c != '"') && (c != '\\')) out += c;
  }
  return out;
}

std::string hex(const uint8_t* data, size_t length) { return TrainerProtocol::toHex(data, length); }

void setState(State newState) {
  std::lock_guard<std::mutex> guard(lock);
  state = newState;
}

void fail(const String& message) {
  std::lock_guard<std::mutex> guard(lock);
  failure = message;
  state = State::FAILED;
  log_e("Probe failed: %s", message.c_str());
}

class ProbeScanCallbacks : public NimBLEAdvertisedDeviceCallbacks {
  void onResult(NimBLEAdvertisedDevice* device) override {
    std::lock_guard<std::mutex> guard(lock);
    for (ScanEntry& entry : scanEntries) {
      if (entry.address == device->getAddress()) {
        entry.rssi = device->getRSSI();
        if (device->haveName() && entry.name.empty()) entry.name = clean(device->getName());
        return;
      }
    }
    if (scanEntries.size() >= MAX_SCAN_ENTRIES) return;
    ScanEntry entry;
    entry.address = device->getAddress();
    entry.name = device->haveName() ? clean(device->getName()) : "";
    entry.rssi = device->getRSSI();
    for (int index = 0; index < device->getServiceUUIDCount(); index++) {
      if (!entry.services.empty()) entry.services += " ";
      entry.services += device->getServiceUUID(index).toString();
    }
    if (device->haveManufacturerData()) {
      std::string data = device->getManufacturerData();
      entry.manufacturerData = hex((const uint8_t*)data.data(), data.size());
    }
    entry.cyclingPower = device->isAdvertisingService(CPS_SERVICE);
    entry.isTrainer = (!trainerAddress.empty()) && (entry.address.toString() == trainerAddress);
    scanEntries.push_back(entry);
  }
};
ProbeScanCallbacks scanCallbacks;

bool containsIgnoreCase(const std::string& text, const char* part) {
  std::string lower;
  for (char c : text) lower += (char)tolower(c);
  return lower.find(part) != std::string::npos;
}

void onMeasurement(NimBLERemoteCharacteristic* characteristic, uint8_t* data, size_t length, bool isNotify) {
  std::lock_guard<std::mutex> guard(lock);
  packetCount++;
  Packet packet;
  packet.ms = millis() - listenStartMs;
  packet.hex = hex(data, length);
  TrainerProtocol::parseCyclingPowerMeasurement(data, length, packet.data);
  packet.cadence = packet.data.hasCrank ? crankCadence.update(packet.data.crankRevolutions, packet.data.crankEventTime, millis()) : -1;
  if (packets.size() < MAX_PACKETS) packets.push_back(packet);
}

std::string readString(NimBLEClient* client, const NimBLEUUID& service, const NimBLEUUID& characteristic) {
  NimBLERemoteService* remoteService = client->getService(service);
  if (remoteService == nullptr) return "";
  NimBLERemoteCharacteristic* remoteCharacteristic = remoteService->getCharacteristic(characteristic);
  if ((remoteCharacteristic == nullptr) || !remoteCharacteristic->canRead()) return "";
  std::string value = remoteCharacteristic->readValue();
  return value;
}

std::string properties(NimBLERemoteCharacteristic* characteristic) {
  std::string text;
  if (characteristic->canRead()) text += "R";
  if (characteristic->canWrite()) text += "W";
  if (characteristic->canWriteNoResponse()) text += "w";
  if (characteristic->canNotify()) text += "N";
  if (characteristic->canIndicate()) text += "I";
  return text;
}

void probeTask(void*) {
  heapBefore = snapshotHeap();
  trainerBefore = TrainerControl::getPacketSummary();
  trainerConnectedBefore = BTDeviceManager::isConnected();

  // --- scan ---
  NimBLEScan* scan = NimBLEDevice::getScan();
  if (scan->isScanning()) {
    fail("Bluetooth is busy looking for the trainer. Wait until the trainer is connected, then try again.");
    taskHandle = nullptr;
    vTaskDelete(nullptr);
    return;
  }
  setState(State::SCANNING);
  scan->setAdvertisedDeviceCallbacks(&scanCallbacks, false);
  scan->setActiveScan(true);
  scan->setInterval(97);
  scan->setWindow(37);
  scan->setMaxResults(0);
  scan->start(SCAN_SECONDS, false);

  // --- pick the meter ---
  ScanEntry target;
  bool found = false;
  {
    std::lock_guard<std::mutex> guard(lock);
    int bestScore = -1000;
    for (const ScanEntry& entry : scanEntries) {
      if (entry.isTrainer) continue;
      if (!requestedAddress.empty()) {
        if (entry.address.toString() == requestedAddress) {
          target = entry;
          found = true;
          break;
        }
        continue;
      }
      if (!entry.cyclingPower) continue;
      int score = entry.rssi;
      if (containsIgnoreCase(entry.name, "p2m") || containsIgnoreCase(entry.name, "power2max")) score += 1000;
      if (score > bestScore) {
        bestScore = score;
        target = entry;
        found = true;
      }
    }
  }
  if (!found) {
    fail(requestedAddress.empty() ? "No power meter found. Spin the cranks to wake the Power2Max (green LED), then start the probe within a minute."
                                  : "That address wasn't seen in the scan. Spin the cranks and try again.");
    taskHandle = nullptr;
    vTaskDelete(nullptr);
    return;
  }

  // --- connect as a second client ---
  {
    std::lock_guard<std::mutex> guard(lock);
    chosenAddress = target.address.toString();
    chosenName = target.name;
    state = State::CONNECTING;
  }
  NimBLEClient* client = NimBLEDevice::createClient();
  if (client == nullptr) {
    fail("No free Bluetooth connection slot (NimBLE limit reached)");
    taskHandle = nullptr;
    vTaskDelete(nullptr);
    return;
  }
  client->setConnectTimeout(10);
  // error 574 (HCI 0x3E, connection failed to be established) is common right after the
  // meter wakes up; a couple of retries usually get through
  bool connectedOk = false;
  std::string errors;
  int attempt = 0;
  for (attempt = 1; attempt <= CONNECT_ATTEMPTS; attempt++) {
    uint32_t connectStart = millis();
    if (client->connect(target.address)) {
      connectMs = millis() - connectStart;
      connectedOk = true;
      break;
    }
    if (!errors.empty()) errors += " ";
    errors += std::to_string(client->getLastError());
    log_w("Power meter connect attempt %d failed, error %d", attempt, client->getLastError());
    vTaskDelay(pdMS_TO_TICKS(700));
  }
  {
    std::lock_guard<std::mutex> guard(lock);
    connectAttempts = connectedOk ? attempt : CONNECT_ATTEMPTS;
    connectErrors = errors;
  }
  if (!connectedOk) {
    fail(String("Couldn't connect after ") + CONNECT_ATTEMPTS + " attempts (errors " + errors.c_str() + "). Spin the cranks and try again.");
    NimBLEDevice::deleteClient(client);
    taskHandle = nullptr;
    vTaskDelete(nullptr);
    return;
  }

  // --- discover and read ---
  std::vector<NimBLERemoteService*>* services = client->getServices(true);
  std::vector<std::string> lines;
  for (NimBLERemoteService* service : *services) {
    std::vector<NimBLERemoteCharacteristic*>* characteristics = service->getCharacteristics(true);
    for (NimBLERemoteCharacteristic* characteristic : *characteristics) {
      lines.push_back(service->getUUID().toString() + " / " + characteristic->getUUID().toString() + " " + properties(characteristic));
    }
  }
  static const struct {
    uint16_t uuid;
    const char* name;
  } disFields[] = {{0x2A29, "manufacturer"}, {0x2A24, "model"}, {0x2A25, "serial"}, {0x2A27, "hardware"}, {0x2A26, "firmware"}, {0x2A28, "software"}};
  std::vector<std::pair<std::string, std::string>> info;
  for (const auto& field : disFields) {
    std::string value = readString(client, DIS_SERVICE, NimBLEUUID(field.uuid));
    if (!value.empty()) info.push_back({field.name, clean(value)});
  }
  std::string featureValue = readString(client, CPS_SERVICE, CPS_FEATURE);
  std::string locationValue = readString(client, CPS_SERVICE, CPS_SENSOR_LOCATION);
  std::string batteryValue = readString(client, BATTERY_SERVICE, BATTERY_LEVEL);
  NimBLERemoteService* cps = client->getService(CPS_SERVICE);
  NimBLERemoteCharacteristic* measurement = (cps != nullptr) ? cps->getCharacteristic(CPS_MEASUREMENT) : nullptr;
  bool controlPoint = (cps != nullptr) && (cps->getCharacteristic(CPS_CONTROL_POINT) != nullptr);
  {
    std::lock_guard<std::mutex> guard(lock);
    attributeLines = lines;
    deviceInfo = info;
    if (featureValue.size() >= 4) {
      cpFeatures = (uint8_t)featureValue[0] | ((uint32_t)(uint8_t)featureValue[1] << 8) | ((uint32_t)(uint8_t)featureValue[2] << 16) | ((uint32_t)(uint8_t)featureValue[3] << 24);
      cpFeaturesRead = true;
    }
    if (!locationValue.empty()) sensorLocation = (uint8_t)locationValue[0];
    if (!batteryValue.empty()) batteryLevel = (uint8_t)batteryValue[0];
    hasControlPoint = controlPoint;
    connectionRssi = client->getRssi();
  }
  heapConnected = snapshotHeap();

  // --- listen ---
  bool ok = false;
  if ((measurement != nullptr) && (measurement->getDescriptor(NimBLEUUID((uint16_t)0x2902)) != nullptr)) {
    listenStartMs = millis();
    ok = measurement->subscribe(true, onMeasurement, true);
  }
  {
    std::lock_guard<std::mutex> guard(lock);
    subscribed = ok;
    state = State::LISTENING;
  }
  if (ok) {
    uint32_t start = millis();
    while ((millis() - start < LISTEN_MS) && client->isConnected()) {
      vTaskDelay(pdMS_TO_TICKS(200));
    }
    if (client->isConnected()) measurement->unsubscribe(true);
  }

  // --- disconnect ---
  if (client->isConnected()) client->disconnect();
  uint32_t waitStart = millis();
  while (client->isConnected() && (millis() - waitStart < 3000)) vTaskDelay(pdMS_TO_TICKS(50));
  NimBLEDevice::deleteClient(client);
  vTaskDelay(pdMS_TO_TICKS(500));
  heapAfter = snapshotHeap();
  trainerAfter = TrainerControl::getPacketSummary();
  trainerConnectedAfter = BTDeviceManager::isConnected();
  if (ok) {
    setState(State::DONE);
  } else {
    fail("Connected, but couldn't subscribe to power measurements (no Cycling Power Measurement with a notification descriptor)");
  }
  log_i("Probe finished");
  taskHandle = nullptr;
  vTaskDelete(nullptr);
}

const char* stateName(State value) {
  switch (value) {
    case State::IDLE: return "idle";
    case State::SCANNING: return "scanning";
    case State::CONNECTING: return "connecting";
    case State::LISTENING: return "listening";
    case State::DONE: return "done";
    case State::FAILED: return "failed";
  }
  return "?";
}

String heapJson(const HeapSnapshot& snapshot) {
  return String("{\"free\": ") + snapshot.free + ", \"min_free\": " + snapshot.minimum + ", \"largest_block\": " + snapshot.largestBlock + "}";
}

}  // namespace

bool SensorProbe::start(const std::string& address, String& error) {
  std::lock_guard<std::mutex> guard(lock);
  if (taskHandle != nullptr) {
    error = "A probe is already running";
    return false;
  }
  if (!BTDeviceManager::isConnected()) {
    error = "Connect the trainer first, so the probe also shows the effect of a second connection";
    return false;
  }
  state = State::SCANNING;
  failure = "";
  requestedAddress = address;
  trainerAddress = BTDeviceManager::getConnectedAddress();
  scanEntries.clear();
  chosenAddress = "";
  chosenName = "";
  connectAttempts = 0;
  connectErrors = "";
  attributeLines.clear();
  deviceInfo.clear();
  cpFeatures = 0;
  cpFeaturesRead = false;
  sensorLocation = -1;
  batteryLevel = -1;
  hasControlPoint = false;
  subscribed = false;
  packets.clear();
  packetCount = 0;
  crankCadence.reset();
  if (xTaskCreatePinnedToCore(probeTask, "probe", 8192, nullptr, 1, &taskHandle, 1) != pdPASS) {
    taskHandle = nullptr;
    state = State::FAILED;
    error = "Couldn't start the probe task";
    return false;
  }
  return true;
}

bool SensorProbe::isRunning() { return taskHandle != nullptr; }

String SensorProbe::getReport() {
  std::lock_guard<std::mutex> guard(lock);
  String json = "{";
  json += "\"state\": \"" + String(stateName(state)) + "\"";
  json += ", \"error\": \"" + failure + "\"";
  json += ", \"trainer_address\": \"" + String(trainerAddress.c_str()) + "\"";
  json += ", \"scan\": [";
  for (size_t index = 0; index < scanEntries.size(); index++) {
    const ScanEntry& entry = scanEntries[index];
    if (index > 0) json += ",";
    json += "{\"address\": \"" + String(entry.address.toString().c_str()) + "\", \"name\": \"" + String(entry.name.c_str()) +
            "\", \"rssi\": " + entry.rssi + ", \"cycling_power\": " + (entry.cyclingPower ? "true" : "false") +
            ", \"trainer\": " + (entry.isTrainer ? "true" : "false") + ", \"services\": \"" + String(entry.services.c_str()) +
            "\", \"manufacturer_data\": \"" + String(entry.manufacturerData.c_str()) + "\"}";
  }
  json += "]";
  if (!chosenAddress.empty()) {
    json += ", \"meter\": {\"address\": \"" + String(chosenAddress.c_str()) + "\", \"name\": \"" + String(chosenName.c_str()) + "\"";
    json += ", \"connect_ms\": " + String(connectMs) + ", \"connect_attempts\": " + connectAttempts + ", \"connect_errors\": \"" + String(connectErrors.c_str()) + "\", \"rssi\": " + connectionRssi;
    for (const auto& field : deviceInfo) {
      json += ", \"" + String(field.first.c_str()) + "\": \"" + String(field.second.c_str()) + "\"";
    }
    json += ", \"battery\": " + String(batteryLevel);
    json += ", \"sensor_location\": " + String(sensorLocation);
    json += ", \"cp_features_raw\": \"0x" + String(cpFeatures, HEX) + "\"";
    json += ", \"cp_features\": \"" + String(cpFeaturesRead ? TrainerProtocol::describeCyclingPowerFeatures(cpFeatures).c_str() : "not readable") + "\"";
    json += ", \"control_point\": " + String(hasControlPoint ? "true" : "false");
    json += ", \"offset_compensation\": " + String((cpFeatures & TrainerProtocol::CP_FEATURE_OFFSET_COMPENSATION) ? "true" : "false");
    json += ", \"subscribed\": " + String(subscribed ? "true" : "false");
    json += ", \"packets_received\": " + String(packetCount);
    json += ", \"attributes\": [";
    for (size_t index = 0; index < attributeLines.size(); index++) {
      if (index > 0) json += ",";
      json += "\"" + String(attributeLines[index].c_str()) + "\"";
    }
    json += "], \"packets\": [";
    for (size_t index = 0; index < packets.size(); index++) {
      const Packet& packet = packets[index];
      if (index > 0) json += ",";
      json += "{\"ms\": " + String(packet.ms) + ", \"raw\": \"" + String(packet.hex.c_str()) + "\", \"power\": " + packet.data.powerWatts;
      if (packet.data.hasBalance) json += ", \"balance\": " + String(packet.data.balancePercent, 1) + ", \"balance_ref_left\": " + (packet.data.balanceReferenceLeft ? "true" : "false");
      if (packet.data.hasAccumulatedTorque) json += ", \"acc_torque_32nm\": " + String(packet.data.accumulatedTorque);
      if (packet.data.hasCrank) json += ", \"crank_revs\": " + String(packet.data.crankRevolutions) + ", \"crank_time\": " + packet.data.crankEventTime + ", \"cadence\": " + String(packet.cadence, 1);
      if (packet.data.hasWheel) json += ", \"wheel_revs\": " + String(packet.data.wheelRevolutions);
      if (packet.data.offsetCompensationIndicator) json += ", \"offset_indicator\": true";
      json += "}";
    }
    json += "]}";
  }
  json += ", \"heap\": {\"before\": " + heapJson(heapBefore) + ", \"connected\": " + heapJson(heapConnected) + ", \"after\": " + heapJson(heapAfter) + ", \"now\": " + heapJson(snapshotHeap()) + "}";
  json += ", \"trainer\": {\"connected_before\": " + String(trainerConnectedBefore ? "true" : "false") + ", \"connected_after\": " + String(trainerConnectedAfter ? "true" : "false") +
          ", \"packets_before\": \"" + trainerBefore + "\", \"packets_after\": \"" + trainerAfter + "\"}";
  json += "}";
  return json;
}
