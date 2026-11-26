#include <PowerMeter.h>
#include <BTDeviceManager.h>
#include <NimBLEDevice.h>
#include <Preferences.h>
#include <Telemetry.h>
#include <TrainerProtocol.h>

#include <mutex>
#include <vector>

namespace {

// The Power2Max advertises for only about a minute after it wakes up, so listen almost
// continuously while searching. Attempts only run while the trainer is connected, so they
// never hold up the trainer's own reconnect scans.
constexpr uint32_t CONNECT_TIMEOUT_S = 5;      // one attempt listens this long for the meter's advertisement
constexpr uint32_t ATTEMPT_PAUSE_MS = 500;     // between attempts
constexpr uint32_t RSSI_INTERVAL_MS = 5000;
constexpr uint32_t BATTERY_INTERVAL_MS = 600000;
constexpr uint32_t SCAN_SECONDS = 10;
constexpr size_t MAX_FOUND = 10;
const char* const NVS_NAMESPACE = "shiftr-v2";

const NimBLEUUID CPS_SERVICE((uint16_t)0x1818);
const NimBLEUUID CPS_MEASUREMENT((uint16_t)0x2A63);
const NimBLEUUID BATTERY_SERVICE((uint16_t)0x180F);
const NimBLEUUID BATTERY_LEVEL((uint16_t)0x2A19);

struct Found {
  std::string address;
  uint8_t type;
  std::string name;
  int rssi;
};

portMUX_TYPE dataLock = portMUX_INITIALIZER_UNLOCKED;
// pairing (written by HTTP handlers, read by the task), guarded by pairingMutex
std::mutex pairingMutex;
String pairedAddress;
uint8_t pairedType = BLE_ADDR_RANDOM;
String pairedName;
volatile bool pairingChanged = false;
// state
volatile PowerMeterState state = PowerMeterState::NOT_PAIRED;
volatile bool paused = false;
volatile bool radioBusy = false;
volatile bool scanRequested = false;
volatile bool scanRunning = false;
std::vector<Found> found;
String scanError;
// data (written from the NimBLE notify callback)
int16_t power = 0;
bool hasBalance = false;
double balanceLeft = 0;
uint32_t lastDataAt = 0;
uint32_t packets = 0;
int rssi = 0;
int battery = -1;
TrainerProtocol::CrankCadence crankCadence(3000);
uint32_t lastCrankAt = 0;
NimBLEClient* client = nullptr;
volatile bool disconnectedFlag = false;
volatile int disconnectReason = 0;

std::string clean(const std::string& text) {
  std::string out;
  for (char c : text) {
    if ((c >= 0x20) && (c < 0x7F) && (c != '"') && (c != '\\')) out += c;
  }
  return out;
}

class MeterClientCallbacks : public NimBLEClientCallbacks {
  // NimBLE-Arduino 1.4 doesn't pass the reason
  void onDisconnect(NimBLEClient* pClient) override {
    disconnectReason = pClient->getLastError();
    disconnectedFlag = true;
  }
  bool onConnParamsUpdateRequest(NimBLEClient* pClient, const ble_gap_upd_params* params) override { return true; }
};
MeterClientCallbacks clientCallbacks;

void onMeasurement(NimBLERemoteCharacteristic* characteristic, uint8_t* data, size_t length, bool isNotify) {
  TrainerProtocol::CyclingPowerData parsed;
  if (!TrainerProtocol::parseCyclingPowerMeasurement(data, length, parsed)) return;
  uint32_t now = millis();
  portENTER_CRITICAL(&dataLock);
  power = parsed.powerWatts;
  if (parsed.hasBalance) {
    hasBalance = true;
    // the meter states which pedal the percentage refers to
    balanceLeft = parsed.balanceReferenceLeft ? parsed.balancePercent : 100.0 - parsed.balancePercent;
  }
  if (parsed.hasCrank) {
    crankCadence.update(parsed.crankRevolutions, parsed.crankEventTime, now);
  }
  lastDataAt = now;
  packets++;
  portEXIT_CRITICAL(&dataLock);
}

void loadPairing() {
  Preferences preferences;
  preferences.begin(NVS_NAMESPACE, true);
  pairedAddress = preferences.getString("pm_addr", "");
  pairedType = preferences.getUChar("pm_type", BLE_ADDR_RANDOM);
  pairedName = preferences.getString("pm_name", "");
  preferences.end();
}

void savePairing(const String& address, uint8_t type, const String& name) {
  Preferences preferences;
  preferences.begin(NVS_NAMESPACE, false);
  if (address.length() == 0) {
    preferences.remove("pm_addr");
    preferences.remove("pm_type");
    preferences.remove("pm_name");
  } else {
    preferences.putString("pm_addr", address);
    preferences.putUChar("pm_type", type);
    preferences.putString("pm_name", name);
  }
  preferences.end();
}

void dropConnection() {
  if ((client != nullptr) && client->isConnected()) {
    client->disconnect();
    uint32_t start = millis();
    while (client->isConnected() && (millis() - start < 3000)) vTaskDelay(pdMS_TO_TICKS(50));
  }
}

void runScan() {
  scanRunning = true;
  std::vector<Found> results;
  String error;
  NimBLEScan* scan = NimBLEDevice::getScan();
  if (!BTDeviceManager::isConnected() || scan->isScanning()) {
    error = "Connect the trainer first (while it's searching, Bluetooth is busy)";
  } else {
    radioBusy = true;
    scan->setAdvertisedDeviceCallbacks(nullptr);
    scan->setActiveScan(true);
    scan->setInterval(97);
    scan->setWindow(37);
    scan->setMaxResults(30);
    NimBLEScanResults scanResults = scan->start(SCAN_SECONDS, false);
    std::string trainer = BTDeviceManager::getConnectedAddress();
    for (int index = 0; index < scanResults.getCount(); index++) {
      NimBLEAdvertisedDevice device = scanResults.getDevice(index);
      if (!device.isAdvertisingService(CPS_SERVICE)) continue;
      if (device.getAddress().toString() == trainer) continue;
      if (results.size() >= MAX_FOUND) break;
      results.push_back({device.getAddress().toString(), device.getAddress().getType(), device.haveName() ? clean(device.getName()) : "", device.getRSSI()});
    }
    scan->clearResults();
    scan->setMaxResults(0);
    radioBusy = false;
    if (results.empty()) error = "No power meter found. Spin the cranks backwards until the LED blinks, then search again within a minute.";
  }
  portENTER_CRITICAL(&dataLock);
  found = results;
  portEXIT_CRITICAL(&dataLock);
  scanError = error;
  scanRunning = false;
}

bool connectOnce(const String& address, uint8_t type) {
  if (client == nullptr) {
    client = NimBLEDevice::createClient();
    if (client == nullptr) return false;
    client->setClientCallbacks(&clientCallbacks, false);
    // the meter sends once a second; a relaxed interval leaves more airtime for the trainer
    client->setConnectionParams(64, 96, 0, 400);
  }
  client->setConnectTimeout(CONNECT_TIMEOUT_S);
  radioBusy = true;
  bool ok = client->connect(NimBLEAddress(std::string(address.c_str()), type), true);
  radioBusy = false;
  if (!ok) return false;

  NimBLERemoteService* service = client->getService(CPS_SERVICE);
  NimBLERemoteCharacteristic* measurement = (service != nullptr) ? service->getCharacteristic(CPS_MEASUREMENT) : nullptr;
  if ((measurement == nullptr) || (measurement->getDescriptor(NimBLEUUID((uint16_t)0x2902)) == nullptr)) {
    log_e("Power meter has no Cycling Power Measurement");
    Telemetry::event(TelemetryEvent::POWER_METER, "no cycling power measurement");
    dropConnection();
    return false;
  }
  portENTER_CRITICAL(&dataLock);
  crankCadence.reset();
  hasBalance = false;
  power = 0;
  lastDataAt = 0;
  portEXIT_CRITICAL(&dataLock);
  // with response: the CCCD write must be confirmed
  if (!measurement->subscribe(true, onMeasurement, true)) {
    log_e("Power meter subscription failed");
    dropConnection();
    return false;
  }
  NimBLERemoteService* batteryService = client->getService(BATTERY_SERVICE);
  if (batteryService != nullptr) {
    NimBLERemoteCharacteristic* level = batteryService->getCharacteristic(BATTERY_LEVEL);
    if ((level != nullptr) && level->canRead()) {
      std::string value = level->readValue();
      if (!value.empty()) battery = (uint8_t)value[0];
    }
  }
  rssi = client->getRssi();
  return true;
}

void meterTask(void*) {
  bool wasConnected = false;
  bool announcedSearch = false;
  uint32_t lastAttemptAt = 0;
  uint32_t lastRssiAt = 0;
  uint32_t lastBatteryAt = 0;
  uint32_t attempts = 0;
  while (true) {
    if (scanRequested) {
      scanRequested = false;
      if (!(client != nullptr && client->isConnected())) {
        runScan();
      } else {
        scanError = "Forget the connected meter first";
        scanRunning = false;
      }
    }
    if (pairingChanged) {
      pairingChanged = false;
      dropConnection();
      wasConnected = false;
      announcedSearch = false;
      attempts = 0;
    }
    bool connectedNow = (client != nullptr) && client->isConnected();
    if (wasConnected && !connectedNow) {
      Telemetry::eventf(TelemetryEvent::POWER_METER, "disconnected (last error %d)", disconnectReason);
      log_w("Power meter disconnected, last error %d", disconnectReason);
      wasConnected = false;
      announcedSearch = false;
      attempts = 0;
    }

    String address, name;
    uint8_t type;
    {
      std::lock_guard<std::mutex> guard(pairingMutex);
      address = pairedAddress;
      name = pairedName;
      type = pairedType;
    }
    if (address.length() == 0) {
      state = PowerMeterState::NOT_PAIRED;
    } else if (connectedNow) {
      state = PowerMeterState::CONNECTED;
      uint32_t now = millis();
      if (now - lastRssiAt > RSSI_INTERVAL_MS) {
        int value = client->getRssi();
        portENTER_CRITICAL(&dataLock);
        rssi = value;
        portEXIT_CRITICAL(&dataLock);
        lastRssiAt = now;
      }
      if (now - lastBatteryAt > BATTERY_INTERVAL_MS) {
        NimBLERemoteService* batteryService = client->getService(BATTERY_SERVICE);
        NimBLERemoteCharacteristic* level = (batteryService != nullptr) ? batteryService->getCharacteristic(BATTERY_LEVEL) : nullptr;
        if ((level != nullptr) && level->canRead()) {
          std::string value = level->readValue();
          if (!value.empty()) battery = (uint8_t)value[0];
        }
        lastBatteryAt = now;
      }
    } else if (paused) {
      state = PowerMeterState::PAUSED;
    } else if (!BTDeviceManager::isConnected() && !BTDeviceManager::isReleased()) {
      state = PowerMeterState::WAITING_FOR_TRAINER;
    } else {
      state = PowerMeterState::SEARCHING;
      if (!announcedSearch) {
        Telemetry::eventf(TelemetryEvent::POWER_METER, "searching for %s", name.length() ? name.c_str() : address.c_str());
        announcedSearch = true;
      }
      if ((lastAttemptAt == 0) || (millis() - lastAttemptAt > ATTEMPT_PAUSE_MS)) {
        attempts++;
        if (connectOnce(address, type)) {
          wasConnected = true;
          lastRssiAt = millis();
          lastBatteryAt = millis();
          Telemetry::eventf(TelemetryEvent::POWER_METER, "connected; tries %lu; battery %d%%", (unsigned long)attempts, battery);
          log_i("Power meter %s connected after %lu attempts", address.c_str(), (unsigned long)attempts);
        }
        lastAttemptAt = millis();
      }
    }
    vTaskDelay(pdMS_TO_TICKS(200));
  }
}

const char* const STATE_NAMES[] = {"not paired", "waiting for the trainer", "searching", "connected", "paused"};

}  // namespace

void PowerMeter::begin() {
  {
    std::lock_guard<std::mutex> guard(pairingMutex);
    loadPairing();
  }
  state = (pairedAddress.length() > 0) ? PowerMeterState::WAITING_FOR_TRAINER : PowerMeterState::NOT_PAIRED;
  log_i("Power meter: %s", pairedAddress.length() ? (pairedName + " " + pairedAddress).c_str() : "not paired");
  // core 0 alongside the NimBLE host; it mostly waits on NimBLE anyway
  xTaskCreatePinnedToCore(meterTask, "powermeter", 6144, nullptr, 1, nullptr, 0);
}

PowerMeterState PowerMeter::getState() { return state; }

const char* PowerMeter::getStateName() { return STATE_NAMES[(uint8_t)state < 5 ? (uint8_t)state : 0]; }

PowerMeterReading PowerMeter::getReading() {
  PowerMeterReading reading;
  uint32_t now = millis();
  portENTER_CRITICAL(&dataLock);
  reading.connected = (state == PowerMeterState::CONNECTED);
  reading.power = power;
  reading.cadence = crankCadence.current(now);
  reading.hasBalance = hasBalance;
  reading.balanceLeft = balanceLeft;
  reading.rssi = rssi;
  reading.ageMs = (lastDataAt == 0) ? UINT32_MAX : now - lastDataAt;
  reading.packets = packets;
  reading.battery = battery;
  portEXIT_CRITICAL(&dataLock);
  if (!reading.connected) {
    reading.power = 0;
    reading.cadence = 0;
  }
  return reading;
}

String PowerMeter::getName() {
  std::lock_guard<std::mutex> guard(pairingMutex);
  return pairedName;
}
String PowerMeter::getAddress() {
  std::lock_guard<std::mutex> guard(pairingMutex);
  return pairedAddress;
}
bool PowerMeter::isConnected() { return state == PowerMeterState::CONNECTED; }
void PowerMeter::setPaused(bool value) { paused = value; }
bool PowerMeter::isRadioBusy() { return radioBusy || scanRunning; }

void PowerMeter::handleStatus(WebServer& server) {
  PowerMeterReading reading = getReading();
  String json = "{\"state\": \"" + String(getStateName()) + "\", \"name\": \"" + getName() + "\", \"address\": \"" + getAddress() + "\"";
  json += ", \"battery\": " + String(reading.battery) + ", \"rssi\": " + String(reading.rssi);
  json += ", \"scan_running\": " + String(scanRunning ? "true" : "false") + ", \"scan_error\": \"" + scanError + "\", \"found\": [";
  portENTER_CRITICAL(&dataLock);
  std::vector<Found> copy = found;
  portEXIT_CRITICAL(&dataLock);
  for (size_t index = 0; index < copy.size(); index++) {
    if (index > 0) json += ",";
    json += "{\"address\": \"" + String(copy[index].address.c_str()) + "\", \"type\": " + String(copy[index].type) + ", \"name\": \"" + String(copy[index].name.c_str()) + "\", \"rssi\": " + String(copy[index].rssi) + "}";
  }
  json += "]}";
  server.send(200, "application/json", json);
}

void PowerMeter::handleScan(WebServer& server) {
  if (scanRunning || scanRequested) {
    server.send(200, "application/json", "{\"started\": false, \"error\": \"already searching\"}");
    return;
  }
  portENTER_CRITICAL(&dataLock);
  found.clear();
  portEXIT_CRITICAL(&dataLock);
  scanError = "";
  scanRunning = true;  // until the task picks it up
  scanRequested = true;
  server.send(200, "application/json", "{\"started\": true}");
}

void PowerMeter::handlePair(WebServer& server) {
  String address = server.arg("address");
  address.trim();
  address.toLowerCase();
  if (address.length() != 17) {
    server.send(400, "application/json", "{\"ok\": false, \"error\": \"address must look like cb:57:07:63:7e:6c\"}");
    return;
  }
  uint8_t type = server.hasArg("type") ? (uint8_t)server.arg("type").toInt() : BLE_ADDR_RANDOM;
  String name = server.arg("name");
  name.replace("\"", "");
  {
    std::lock_guard<std::mutex> guard(pairingMutex);
    savePairing(address, type, name);
    pairedAddress = address;
    pairedType = type;
    pairedName = name;
  }
  pairingChanged = true;
  Telemetry::eventf(TelemetryEvent::POWER_METER, "paired %s", name.length() ? name.c_str() : address.c_str());
  server.send(200, "application/json", "{\"ok\": true}");
}

void PowerMeter::handleForget(WebServer& server) {
  {
    std::lock_guard<std::mutex> guard(pairingMutex);
    savePairing("", 0, "");
    pairedAddress = "";
    pairedName = "";
  }
  pairingChanged = true;
  Telemetry::event(TelemetryEvent::POWER_METER, "forgotten");
  server.send(200, "application/json", "{\"ok\": true}");
}
