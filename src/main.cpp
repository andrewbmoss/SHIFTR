#include <Arduino.h>
#include <AsyncTCP.h>
#include <BTDeviceManager.h>
#include <Config.h>
#include <DirConManager.h>
#include <ESPmDNS.h>
#include <ETH.h>
#include <IotWebConf.h>
#include <IotWebConfESP32HTTPUpdateServer.h>
#include <IotWebConfUsing.h>
#include <NimBLEDevice.h>
#include <SettingsManager.h>
#include <UUIDs.h>
#include <Utils.h>
#include <Version.h>
#include <WiFi.h>
#include <ArduinoOTA.h>
#include <TrainerControl.h>
#include <SensorProbe.h>
#include <Telemetry.h>
#include <PowerMeter.h>
#include <CrashReport.h>
#include <esp_heap_caps.h>

#define STR_HELPER(x) #x
#define STR(x) STR_HELPER(x)

void networkEvent(WiFiEvent_t event);
void handleWebServerFile(const String& fileName);
void handleWebServerStatus();
void handleWebServerSettings();
void handleWebServerSettingsPost();
void handleWebServerDebug();
void handleWebServerLive();
void handleWebServerZwiftSnapshot();
void handleFunctionButton();
void startArduinoOTA();

bool isMDNSStarted = false;
bool isBLEConnected = false;
bool isEthernetConnected = false;
bool isWiFiConnected = false;
bool isRecoveryBoot = false;         // RECOVERY button held at power-up: keep the access point
uint32_t ethernetDownSince = 0;
void manageWiFiRadio();
bool isArduinoOTAStarted = false;

DNSServer dnsServer;
WebServer webServer(WEB_SERVER_PORT);
HTTPUpdateServer updateServer;
// IotWebConf keeps the pointers it is given, so the strings must outlive it
// (passing Utils::getHostName().c_str() directly left them dangling and broke the setup access point)
std::string hostName = Utils::getHostName();
IotWebConf iotWebConf(hostName.c_str(), &dnsServer, &webServer, hostName.c_str(), WIFI_CONFIG_VERSION);

ServiceManager serviceManager;

void setup() {
  log_i(DEVICE_NAME_PREFIX " " VERSION " starting...");
  log_i("Device name: %s, host name: %s", Utils::getDeviceName().c_str(), Utils::getFQDN().c_str());

  // logger buffers first, while the heap is one big block
  Telemetry::begin();
  CrashReport::begin();

  // initialize bluetooth device manager
  BTDeviceManager::setLocalDeviceName(Utils::getDeviceName());
  BTDeviceManager::setServiceManager(&serviceManager);
  if (!BTDeviceManager::start()) {
    log_e("Startup failed: Unable to start bluetooth device manager");
    ESP.restart();
  }
  log_i("Bluetooth device manager initialized");
  PowerMeter::begin();

  // function button (short press: reconnect trainer, long press: reboot)
  pinMode(FUNCTION_BUTTON_PIN, INPUT_PULLUP);

  // initialize network events
  WiFi.onEvent(networkEvent);
  log_i("Network events initialized");

  // initialize ethernet interface
  ETH.begin();
  log_i("Ethernet interface initialized");

  // initialize settings manager
  SettingsManager::initialize(&iotWebConf);

  // initialize wifi manager and web server
  iotWebConf.setStatusPin(WIFI_STATUS_PIN);
  iotWebConf.setConfigPin(WIFI_CONFIG_PIN);
  pinMode(WIFI_CONFIG_PIN, INPUT_PULLUP);
  delay(5);
  isRecoveryBoot = (digitalRead(WIFI_CONFIG_PIN) == LOW);
  if (isRecoveryBoot) {
    log_i("Recovery button held: access point stays on with the default password");
  }
  iotWebConf.addParameterGroup(SettingsManager::getIoTWebConfSettingsParameterGroup());
  iotWebConf.setupUpdateServer(
      [](const char* updatePath) { updateServer.setup(&webServer, updatePath, SettingsManager::getUsername().c_str(), SettingsManager::getAPPassword().c_str()); },
      [](const char* userName, char* password) { updateServer.updateCredentials(userName, password); });
  iotWebConf.init();

  // workaround for missing thing name
  strncpy(iotWebConf.getThingNameParameter()->valueBuffer, Utils::getHostName().c_str(), iotWebConf.getThingNameParameter()->getLength());

  webServer.on("/debug", handleWebServerDebug);
  webServer.on("/status", handleWebServerStatus);
  webServer.on("/live", handleWebServerLive);
  webServer.on("/favicon.ico", [] { handleWebServerFile("favicon.ico"); });
  webServer.on("/style.css", [] { handleWebServerFile("style.css"); });
  webServer.on("/", [] { handleWebServerFile("index.html"); });
  webServer.on("/probe", HTTP_GET, [] { handleWebServerFile("probe.html"); });
  webServer.on("/logger", HTTP_GET, [] { handleWebServerFile("logger.html"); });
  webServer.on("/log", HTTP_GET, [] { Telemetry::handleLog(webServer); });
  webServer.on("/log/info", HTTP_GET, [] { Telemetry::handleInfo(webServer); });
  webServer.on("/log/marker", HTTP_POST, [] { Telemetry::handleMarker(webServer); });
  webServer.on("/powermeter", HTTP_GET, [] { PowerMeter::handleStatus(webServer); });
  webServer.on("/crash", HTTP_GET, [] { CrashReport::handleSummary(webServer); });
  webServer.on("/crash/dump", HTTP_GET, [] { CrashReport::handleDump(webServer); });
  webServer.on("/crash/clear", HTTP_POST, [] {
    if (!webServer.authenticate(SettingsManager::getUsername().c_str(), SettingsManager::getAPPassword().c_str())) {
      return webServer.requestAuthentication();
    }
    CrashReport::handleClear(webServer); });
  webServer.on("/powermeter/scan", HTTP_POST, [] {
    if (!webServer.authenticate(SettingsManager::getUsername().c_str(), SettingsManager::getAPPassword().c_str())) {
      return webServer.requestAuthentication();
    }
    PowerMeter::handleScan(webServer); });
  webServer.on("/powermeter/pair", HTTP_POST, [] {
    if (!webServer.authenticate(SettingsManager::getUsername().c_str(), SettingsManager::getAPPassword().c_str())) {
      return webServer.requestAuthentication();
    }
    PowerMeter::handlePair(webServer); });
  webServer.on("/powermeter/forget", HTTP_POST, [] {
    if (!webServer.authenticate(SettingsManager::getUsername().c_str(), SettingsManager::getAPPassword().c_str())) {
      return webServer.requestAuthentication();
    }
    PowerMeter::handleForget(webServer); });
  webServer.on("/probe/report", HTTP_GET, [] { webServer.send(200, "application/json", SensorProbe::getReport()); });
  webServer.on("/probe/zwift", HTTP_GET, handleWebServerZwiftSnapshot);
  webServer.on("/probe/start", HTTP_POST, [] {
    if (!webServer.authenticate(SettingsManager::getUsername().c_str(), SettingsManager::getAPPassword().c_str())) {
      return webServer.requestAuthentication();
    }
    String error;
    bool started = SensorProbe::start(std::string(webServer.arg("address").c_str()), error);
    error.replace("\"", "'");
    webServer.send(200, "application/json", String("{\"started\": ") + (started ? "true" : "false") + ", \"error\": \"" + error + "\"}");
  });
  webServer.on("/settings", HTTP_GET, [] { 
    if (!webServer.authenticate(SettingsManager::getUsername().c_str(), SettingsManager::getAPPassword().c_str())) {
      return webServer.requestAuthentication();
    }
    handleWebServerFile("settings.html"); });
  webServer.on("/settings", HTTP_POST, [] { 
    if (!webServer.authenticate(SettingsManager::getUsername().c_str(), SettingsManager::getAPPassword().c_str())) {
      return webServer.requestAuthentication();
    }
    handleWebServerSettingsPost(); });
  webServer.on("/devicesettings", [] { 
    if (!webServer.authenticate(SettingsManager::getUsername().c_str(), SettingsManager::getAPPassword().c_str())) {
      return webServer.requestAuthentication();
    }
    handleWebServerSettings(); });
  webServer.on("/config", [] { 
    if (!webServer.authenticate(SettingsManager::getUsername().c_str(), SettingsManager::getAPPassword().c_str())) {
      return webServer.requestAuthentication();
    }
    iotWebConf.handleConfig(); });
  
  webServer.onNotFound([]() { iotWebConf.handleNotFound(); });
  log_i("WiFi manager and web server initialized");

  // initialize service manager internal service if enabled
  if (SettingsManager::isVirtualShiftingEnabled()) {
    Service* zwiftCustomService = new Service(NimBLEUUID(ZWIFT_CUSTOM_SERVICE_UUID), true, true);
    zwiftCustomService->addCharacteristic(new Characteristic(NimBLEUUID(ZWIFT_ASYNC_CHARACTERISTIC_UUID), NOTIFY));
    zwiftCustomService->addCharacteristic(new Characteristic(NimBLEUUID(ZWIFT_SYNCRX_CHARACTERISTIC_UUID), WRITE));
    zwiftCustomService->addCharacteristic(new Characteristic(NimBLEUUID(ZWIFT_SYNCTX_CHARACTERISTIC_UUID), INDICATE));
    serviceManager.addService(zwiftCustomService);
  }

  // initialize service manager FTMS service if enabled
  if (SettingsManager::isFTMSEnabled()) {
    Service* fitnessMachineService = new Service(NimBLEUUID(FITNESS_MACHINE_SERVICE_UUID), true, true);
    fitnessMachineService->addCharacteristic(new Characteristic(NimBLEUUID(FITNESS_MACHINE_FEATURE_CHARACTERISTIC_UUID), READ));
    fitnessMachineService->addCharacteristic(new Characteristic(NimBLEUUID(INDOOR_BIKE_DATA_CHARACTERISTIC_UUID), NOTIFY));
    fitnessMachineService->addCharacteristic(new Characteristic(NimBLEUUID(TRAINING_STATUS_CHARACTERISTIC_UUID), READ | NOTIFY));
    fitnessMachineService->addCharacteristic(new Characteristic(NimBLEUUID(FITNESS_MACHINE_CONTROL_POINT_CHARACTERISTIC_UUID), WRITE | INDICATE));
    fitnessMachineService->addCharacteristic(new Characteristic(NimBLEUUID(FITNESS_MACHINE_STATUS_CHARACTERISTIC_UUID), NOTIFY));
    serviceManager.addService(fitnessMachineService);
  }
  log_i("Service manager initialized");

  // set BTDeviceManager selected trainer device
  BTDeviceManager::setRemoteDeviceNameFilter(SettingsManager::getTrainerDeviceName());

  // initialize MDNS
  if (!MDNS.begin(Utils::getHostName().c_str())) {
    log_e("Startup failed: Unable to start MDNS");
    ESP.restart();
  }
  MDNS.setInstanceName(Utils::getDeviceName().c_str());
  isMDNSStarted = true;
  log_i("MDNS initialized");

  // initialize DirCon manager
  DirConManager::setServiceManager(&serviceManager);
  if (!DirConManager::start()) {
    log_e("Startup failed: Unable to start DirCon manager");
    ESP.restart();
  }
  log_i("DirCon Manager initialized");

  log_i("Startup finished");
}

void loop() {
  Telemetry::update();
  BTDeviceManager::update();
  TrainerControl::update();
  DirConManager::update();
  iotWebConf.doLoop();
  manageWiFiRadio();
  handleFunctionButton();
  if (!isArduinoOTAStarted && (isEthernetConnected || isWiFiConnected)) {
    startArduinoOTA();
  }
  if (isArduinoOTAStarted) {
    ArduinoOTA.handle();
  }
}

void startArduinoOTA() {
  ArduinoOTA.setPort(ARDUINO_OTA_PORT);
  ArduinoOTA.setHostname(Utils::getHostName().c_str());
  ArduinoOTA.setPassword(SettingsManager::getAPPassword().c_str());
  // our own MDNS instance carries the DirCon records, don't let ArduinoOTA restart it
  ArduinoOTA.setMdnsEnabled(false);
  ArduinoOTA.onStart([]() {
    log_i("ArduinoOTA update starting, stopping bluetooth");
    BTDeviceManager::stop();
  });
  ArduinoOTA.onError([](ota_error_t error) { log_e("ArduinoOTA error %u", error); });
  ArduinoOTA.begin();
  isArduinoOTAStarted = true;
  log_i("ArduinoOTA ready on port %d", ARDUINO_OTA_PORT);
}

void handleFunctionButton() {
  static bool lastReading = HIGH;
  static bool stableState = HIGH;
  static uint32_t lastChange = 0;
  static uint32_t pressedAt = 0;
  bool reading = digitalRead(FUNCTION_BUTTON_PIN);
  uint32_t now = millis();
  if (reading != lastReading) {
    lastChange = now;
    lastReading = reading;
  }
  if ((now - lastChange) < 50) {
    return;  // debounce
  }
  if (reading != stableState) {
    stableState = reading;
    if (stableState == LOW) {
      pressedAt = now;
    } else if ((now - pressedAt) < FUNCTION_BUTTON_LONG_PRESS) {
      log_i("Function button: reconnecting trainer");
      BTDeviceManager::forceReconnect();
    }
  }
  if ((stableState == LOW) && ((now - pressedAt) >= FUNCTION_BUTTON_LONG_PRESS)) {
    log_i("Function button held: rebooting");
    delay(100);
    ESP.restart();
  }
}

void networkEvent(WiFiEvent_t event) {
  switch (event) {
    case ARDUINO_EVENT_ETH_START:
      log_d("Ethernet started");
      ETH.setHostname(Utils::getHostName().c_str());
      break;
    case ARDUINO_EVENT_ETH_CONNECTED:
      log_i("Ethernet connected");
      break;
    case ARDUINO_EVENT_ETH_GOT_IP:
      log_i("Ethernet DHCP successful with IP %u.%u.%u.%u", ETH.localIP()[0], ETH.localIP()[1], ETH.localIP()[2], ETH.localIP()[3]);
      isEthernetConnected = true;
      break;
    case ARDUINO_EVENT_ETH_DISCONNECTED:
      log_i("Ethernet disconnected");
      isEthernetConnected = false;
      break;
    case ARDUINO_EVENT_ETH_STOP:
      log_d("Ethernet stopped");
      isEthernetConnected = false;
      break;
    case ARDUINO_EVENT_WIFI_STA_GOT_IP:
      log_i("WiFi DHCP successful with IP %u.%u.%u.%u", WiFi.localIP()[0], WiFi.localIP()[1], WiFi.localIP()[2], WiFi.localIP()[3]);
      isWiFiConnected = true;
      break;
    case ARDUINO_EVENT_WIFI_STA_DISCONNECTED:
      log_i("WiFi disconnected");
      isWiFiConnected = false;
      break;
    case ARDUINO_EVENT_WIFI_STA_STOP:
      log_d("WiFi stopped");
      isWiFiConnected = false;
      break;
    default:
      break;
  }
}

extern const uint8_t favicon_ico_start[] asm("_binary_src_web_favicon_ico_start");
extern const uint8_t favicon_ico_end[] asm("_binary_src_web_favicon_ico_end");

extern const uint8_t index_html_start[] asm("_binary_src_web_index_html_start");
extern const uint8_t index_html_end[] asm("_binary_src_web_index_html_end");

extern const uint8_t settings_html_start[] asm("_binary_src_web_settings_html_start");
extern const uint8_t settings_html_end[] asm("_binary_src_web_settings_html_end");

extern const uint8_t logger_html_start[] asm("_binary_src_web_logger_html_start");
extern const uint8_t logger_html_end[] asm("_binary_src_web_logger_html_end");

extern const uint8_t probe_html_start[] asm("_binary_src_web_probe_html_start");
extern const uint8_t probe_html_end[] asm("_binary_src_web_probe_html_end");

extern const uint8_t style_css_start[] asm("_binary_src_web_style_css_start");
extern const uint8_t style_css_end[] asm("_binary_src_web_style_css_end");

void handleWebServerFile(const String& fileName) {
  if (iotWebConf.handleCaptivePortal()) {
    return;
  }

  if (fileName.equals("index.html")) {
    webServer.send_P(200, "text/html", (char*)index_html_start, (index_html_end - index_html_start));
  }
  if (fileName.equals("settings.html")) {
    webServer.send_P(200, "text/html", (char*)settings_html_start, (settings_html_end - settings_html_start));
  }
  if (fileName.equals("logger.html")) {
    webServer.send_P(200, "text/html", (char*)logger_html_start, (logger_html_end - logger_html_start));
  }
  if (fileName.equals("probe.html")) {
    webServer.send_P(200, "text/html", (char*)probe_html_start, (probe_html_end - probe_html_start));
  }
  if (fileName.equals("style.css")) {
    webServer.send_P(200, "text/css", (char*)style_css_start, (style_css_end - style_css_start));
  }
  if (fileName.equals("favicon.ico")) {
    webServer.send_P(200, "image/x-icon", (char*)favicon_ico_start, (favicon_ico_end - favicon_ico_start));
  }
}

void handleWebServerSettings() {
  String json = "{";
  json += "\"device_name\": \"";
  json += Utils::getDeviceName().c_str();
  json += "\",";

  json += "\"trainer_device\": \"";
  json += SettingsManager::getTrainerDeviceName().c_str();
  json += "\",";

  String devices_json = "\"trainer_devices\": [";
  devices_json += "\"\",";
  for (size_t deviceIndex = 0; deviceIndex < BTDeviceManager::getScannedDevices()->size(); deviceIndex++) {
    if (BTDeviceManager::getScannedDevices()->at(deviceIndex).haveName()) {
      devices_json += "\"";
      devices_json += BTDeviceManager::getScannedDevices()->at(deviceIndex).getName().c_str();
      devices_json += "\",";
    }
  }

  if (devices_json.endsWith(",")) {
    devices_json.remove(devices_json.length() - 1);
  }
  devices_json += "],";

  json += devices_json;

  json += "\"virtual_shifting\": ";
  if (SettingsManager::isVirtualShiftingEnabled()) {
    json += "true";
  } else {
    json += "false";
  }
  json += ",";

  json += "\"virtual_shifting_mode\": ";
  json += SettingsManager::getVirtualShiftingMode();
  json += ",";

  std::map<size_t, std::string> modes = SettingsManager::getVirtualShiftingModes();
  String modes_json = "\"virtual_shifting_modes\": [";
  for (auto mode = modes.begin(); mode != modes.end(); mode++) {
      modes_json += "{\"name\": \"";
      modes_json += mode->second.c_str();
      modes_json += "\", \"value\": ";
      modes_json += mode->first;
      modes_json += "},";
  }
  if (modes_json.endsWith(",")) {
    modes_json.remove(modes_json.length() - 1);
  }
  modes_json += "],";

  json += modes_json;

  json += "\"chainring_teeth\": ";
  json += SettingsManager::getChainringTeeth();
  json += ",";

  json += "\"sprocket_teeth\": ";
  json += SettingsManager::getSprocketTeeth();
  json += ",";

  json += "\"grade_smoothing\": ";
  json += String(SettingsManager::getGradeSmoothingSeconds(), 1);
  json += ",";

  json += "\"difficulty\": ";
  json += SettingsManager::getDifficulty();
  json += ",";

  json += "\"trainer_weight\": ";
  json += SettingsManager::getTrainerWeight();
  json += ",";
  
  json += "\"ftms_emulation\": ";
  if (SettingsManager::isFTMSEnabled()) {
    json += "true";
  } else {
    json += "false";
  }
  json += ",";

  json += "\"trainer_protocol\": ";
  json += (int)SettingsManager::getTrainerProtocolPreference();
  json += ",";

  json += "\"gear_table\": ";
  json += (int)SettingsManager::getGearTable();
  json += ",";

  std::map<size_t, std::string> gearTables = SettingsManager::getGearTables();
  String gear_tables_json = "\"gear_tables\": [";
  for (auto gearTable = gearTables.begin(); gearTable != gearTables.end(); gearTable++) {
    gear_tables_json += "{\"name\": \"";
    gear_tables_json += gearTable->second.c_str();
    gear_tables_json += "\", \"value\": ";
    gear_tables_json += gearTable->first;
    gear_tables_json += "},";
  }
  if (gear_tables_json.endsWith(",")) {
    gear_tables_json.remove(gear_tables_json.length() - 1);
  }
  gear_tables_json += "],";
  json += gear_tables_json;

  std::map<size_t, std::string> protocols = SettingsManager::getTrainerProtocolPreferences();
  String protocols_json = "\"trainer_protocols\": [";
  for (auto protocol = protocols.begin(); protocol != protocols.end(); protocol++) {
    protocols_json += "{\"name\": \"";
    protocols_json += protocol->second.c_str();
    protocols_json += "\", \"value\": ";
    protocols_json += protocol->first;
    protocols_json += "},";
  }
  if (protocols_json.endsWith(",")) {
    protocols_json.remove(protocols_json.length() - 1);
  }
  protocols_json += "]";
  json += protocols_json;

  json += "}";

  webServer.send(200, "application/json", json);
}

void handleWebServerSettingsPost() {
  if (webServer.args() > 0) {
    if (webServer.hasArg("trainer_device")) {
      SettingsManager::setTrainerDeviceName(webServer.arg("trainer_device").c_str());
    }
    if (webServer.hasArg("virtual_shifting")) {
      SettingsManager::setVirtualShiftingEnabled(true);
    } else {
      SettingsManager::setVirtualShiftingEnabled(false);
    }
    if (webServer.hasArg("virtual_shifting_mode")) {
      SettingsManager::setVirtualShiftingMode((VirtualShiftingMode)std::atoi(webServer.arg("virtual_shifting_mode").c_str()));
    }
    if (webServer.hasArg("chainring_teeth")) {
      SettingsManager::setChainringTeeth(std::atoi(webServer.arg("chainring_teeth").c_str()));
    }
    if (webServer.hasArg("sprocket_teeth")) {
      SettingsManager::setSprocketTeeth(std::atoi(webServer.arg("sprocket_teeth").c_str()));
    }
    if (webServer.hasArg("grade_smoothing")) {
      SettingsManager::setGradeSmoothingSeconds(std::atof(webServer.arg("grade_smoothing").c_str()));
    }
    if (webServer.hasArg("trainer_weight")) {
      SettingsManager::setTrainerWeight(std::atoi(webServer.arg("trainer_weight").c_str()));
    }
    if (webServer.hasArg("difficulty")) {
      SettingsManager::setDifficulty(std::atoi(webServer.arg("difficulty").c_str()));
    }
    if (webServer.hasArg("gear_table")) {
      SettingsManager::setGearTable((VirtualGears::GearTable)std::atoi(webServer.arg("gear_table").c_str()));
    }
    if (webServer.hasArg("trainer_protocol")) {
      SettingsManager::setTrainerProtocolPreference((TrainerProtocolPreference)std::atoi(webServer.arg("trainer_protocol").c_str()));
    }
    if (webServer.hasArg("ftms_emulation")) {
      SettingsManager::setFTMSEnabled(true);
      SettingsManager::setVirtualShiftingEnabled(false);
    } else {
      SettingsManager::setFTMSEnabled(false);
    }
    iotWebConf.saveConfig();
    Telemetry::event(TelemetryEvent::SETTINGS_SAVED, "");
    delay(500);
    ESP.restart();
  }
}

void handleWebServerStatus() {
  String json = "{";
  json += "\"device_name\": \"";
  json += Utils::getDeviceName().c_str();
  json += "\",";

  json += "\"version\": \"";
  json += VERSION;
  json += "\",";

  json += "\"build_timestamp\": \"";
  json += VERSION_TIMESTAMP;
  json += "\",";

  json += "\"hostname\": \"";
  json += Utils::getFQDN().c_str();
  json += "\",";

  json += "\"ethernet_status\": \"";
  if (isEthernetConnected) {
    json += "Connected";
    json += ", IP: ";
    json += ETH.localIP().toString();
  } else {
    json += "Not connected";
  }
  json += "\",";

  json += "\"wifi_status\": \"";
  switch (iotWebConf.getState()) {
    case iotwebconf::NetworkState::ApMode:
      json += "Access-Point mode";
      break;
    case iotwebconf::NetworkState::Boot:
      json += "Booting";
      break;
    case iotwebconf::NetworkState::Connecting:
      json += "Connecting";
      break;
    case iotwebconf::NetworkState::NotConfigured:
      json += "Not configured";
      break;
    case iotwebconf::NetworkState::OffLine:
      json += "Disconnected";
      break;
    case iotwebconf::NetworkState::OnLine:
      json += "Connected";
      json += ", SSID: ";
      json += WiFi.SSID();
      json += ", IP: ";
      json += WiFi.localIP().toString();
      break;
    default:
      json += "Unknown";
      break;
  }
  json += "\",";

  json += "\"service_status\": \"";
  json += serviceManager.getStatusMessage().c_str();
  json += "\",";

  json += "\"dircon_status\": \"";
  json += DirConManager::getStatusMessage().c_str();
  json += "\",";

  json += "\"ble_status\": \"";
  json += BTDeviceManager::getStatusMessage().c_str();
  json += "\",";

  json += "\"trainer_protocol\": \"";
  json += TrainerControl::getProtocolName();
  json += "\",";

  json += "\"mode\": \"";
  json += "Pass-through";
  if (SettingsManager::isFTMSEnabled()) {
    json += " + FTMS emulation";
  }
  if (SettingsManager::isVirtualShiftingEnabled()) {
    json += " + virtual shifting";
  }
  json += "\",";

  json += "\"free_heap\": ";
  json += ESP.getFreeHeap();
  json += "}";

  webServer.send(200, "application/json", json);
}

void handleWebServerDebug() {

  String json = "{";
  json += "\"zwift_trainer_mode\": \"";
  switch (DirConManager::getZwiftTrainerMode()) {
    case TrainerMode::SIM_MODE:
      json += "SIM mode";
      break;
    case TrainerMode::SIM_MODE_VIRTUAL_SHIFTING:
      json += "SIM + VS mode";
      break;
    default:
      json += "ERG mode";
      break;
  }
  json += "\",";

  String services_json = "\"ble_services\": {";
  for (Service* service : serviceManager.getServices()) {
    services_json += "\"";
    services_json += service->UUID.to128().toString().c_str();
    services_json += "\": {";
    for (Characteristic* characteristic : service->getCharacteristics()) {
      services_json += "\"";
      services_json += characteristic->UUID.to128().toString().c_str();
      services_json += "\": ";
      services_json += characteristic->getSubscriptions().size();
      services_json += ",";
    }
    if (services_json.endsWith(",")) {
      services_json.remove(services_json.length() - 1);
    }
    services_json += "},";
  }
  if (services_json.endsWith(",")) {
    services_json.remove(services_json.length() - 1);
  }
  services_json += "}";

  json += services_json;

  json += "}";

  webServer.send(200, "application/json", json);
}
// Phase 0: what Zwift subscribes to and which characteristics carry data to it
void handleWebServerZwiftSnapshot() {
  const char* zwiftMode = "SIM";
  switch (DirConManager::getZwiftTrainerMode()) {
    case TrainerMode::SIM_MODE_VIRTUAL_SHIFTING:
      zwiftMode = "SIM + virtual shifting";
      break;
    case TrainerMode::ERG_MODE:
      zwiftMode = "ERG";
      break;
    default:
      break;
  }
  String json = "{\"mode\": \"";
  json += zwiftMode;
  json += "\", \"clients\": ";
  json += DirConManager::getConnectedClientCount();
  json += ", \"uptime_s\": ";
  json += millis() / 1000;
  json += ", \"trainer\": \"";
  json += BTDeviceManager::getConnecedDeviceName().c_str();
  json += "\", \"trainer_protocol\": \"";
  json += TrainerControl::getProtocolName();
  json += "\", \"trainer_packets\": \"";
  json += TrainerControl::getPacketSummary();
  json += "\", \"heap\": {\"free\": ";
  json += ESP.getFreeHeap();
  json += ", \"min_free\": ";
  json += ESP.getMinFreeHeap();
  json += ", \"largest_block\": ";
  json += heap_caps_get_largest_free_block(MALLOC_CAP_8BIT);
  json += "}, \"traffic\": ";
  json += DirConManager::getZwiftTrafficReport();
  json += "}";
  webServer.send(200, "application/json", json);
}

// Live ride data for the status page, polled once per second
void handleWebServerLive() {
  char buffer[1280];
  PowerMeterReading meter = PowerMeter::getReading();
  const char* zwiftMode = "SIM";
  switch (DirConManager::getZwiftTrainerMode()) {
    case TrainerMode::SIM_MODE_VIRTUAL_SHIFTING:
      zwiftMode = "SIM + virtual shifting";
      break;
    case TrainerMode::ERG_MODE:
      zwiftMode = "ERG";
      break;
    default:
      break;
  }
  double defaultGearRatio = DirConManager::getDefaultGearRatio();
  uint32_t dataAge = TrainerControl::getMillisSinceLastData();
  const char* cadenceSource = TrainerControl::isCadenceMeasured() ? "trainer" : "estimated from speed";
  if (dataAge > TRAINER_DATA_TIMEOUT) {
    cadenceSource = "no trainer data, shifting assumes " STR(NOMINAL_CADENCE) " rpm";
  }
  snprintf(buffer, sizeof(buffer),
           "{\"trainer_connected\": %s, \"trainer_name\": \"%s\", \"protocol\": \"%s\", \"control\": %s, "
           "\"power\": %u, \"cadence\": %u, \"cadence_source\": \"%s\", \"speed_kph\": %.1f, \"data_age_ms\": %ld, "
           "\"zwift_clients\": %u, \"zwift_mode\": \"%s\", \"virtual_gear\": \"%s\", \"gear_ratio\": %.2f, \"model_power\": %.0f, \"drag_power\": %.0f, \"total_weight\": %.1f, \"trainer_weight\": %.1f, "
           "\"zwift_grade\": %.2f, \"smoothed_grade\": %.2f, \"trainer_cw\": %.2f, \"residual_drag\": %.1f, \"trainer_command\": \"%s\", \"blocked_writes\": %lu, \"packets\": \"%s\", \"logger_age_ms\": %ld, "
           "\"pm_state\": \"%s\", \"pm_name\": \"%s\", \"pm_power\": %d, \"pm_cadence\": %.0f, \"pm_balance\": %.1f, \"pm_battery\": %d, \"pm_rssi\": %d, \"pm_age_ms\": %ld}",
           BTDeviceManager::isConnected() ? "true" : "false",
           BTDeviceManager::getConnecedDeviceName().c_str(),
           TrainerControl::getProtocolName(),
           TrainerControl::hasControl() ? "true" : "false",
           TrainerControl::getPower(),
           TrainerControl::getEffectiveCadence(defaultGearRatio),
           cadenceSource,
           TrainerControl::getSpeedMps() * 3.6,
           dataAge == UINT32_MAX ? -1L : (long)dataAge,
           (unsigned)DirConManager::getConnectedClientCount(),
           zwiftMode,
           DirConManager::getVirtualGearLabel().c_str(),
           DirConManager::getSimulatedGearRatio(),
           DirConManager::getModelPower(),
           DirConManager::getTrainerDragPower(),
           DirConManager::getTotalWeight(),
           (double)SettingsManager::getTrainerWeight(),
           DirConManager::getZwiftGradePercent(),
           DirConManager::getSmoothedGradePercent(),
           DirConManager::getTrainerWindCoefficient(),
           DirConManager::getResidualDragForce(),
           TrainerControl::getLastCommandDescription().c_str(),
           (unsigned long)DirConManager::getBlockedPassthroughWrites(),
           TrainerControl::getPacketSummary().c_str(),
           Telemetry::getMillisSinceLastFetch() == UINT32_MAX ? -1L : (long)Telemetry::getMillisSinceLastFetch(),
           PowerMeter::getStateName(), PowerMeter::getName().c_str(), meter.power, meter.cadence, meter.hasBalance ? meter.balanceLeft : -1.0,
           meter.battery, meter.rssi, meter.ageMs == UINT32_MAX ? -1L : (long)meter.ageMs);
  webServer.send(200, "application/json", buffer);
}

// The ESP32 has one 2.4 GHz radio for Wi-Fi and Bluetooth. IotWebConf keeps its setup access point
// running forever when no Wi-Fi network is configured, which costs Bluetooth airtime and made the
// trainer connection drop. With Ethernet up and no Wi-Fi wanted, switch Wi-Fi off and serve the web
// pages ourselves (IotWebConf only does that in its Wi-Fi states).
void manageWiFiRadio() {
  bool wifiConfigured = iotWebConf.getWifiSsidParameter()->valueBuffer[0] != '\0';
  iotwebconf::NetworkState state = iotWebConf.getState();
  uint32_t now = millis();

  if (isEthernetConnected) {
    ethernetDownSince = 0;
    if (!wifiConfigured && !isRecoveryBoot &&
        ((state == iotwebconf::NetworkState::ApMode) || (state == iotwebconf::NetworkState::NotConfigured))) {
      log_i("Ethernet is up and no Wi-Fi is configured: turning the Wi-Fi access point off");
      iotWebConf.goOffLine();
    }
  } else if (state == iotwebconf::NetworkState::OffLine) {
    if (ethernetDownSince == 0) {
      ethernetDownSince = now;
    } else if (now - ethernetDownSince > 30000) {
      // no Ethernet for a while: bring the access point back so the device stays reachable
      log_w("Ethernet down for 30 s: turning the Wi-Fi access point back on");
      ethernetDownSince = 0;
      iotWebConf.goOnLine();
    }
  }

  if (iotWebConf.getState() == iotwebconf::NetworkState::OffLine) {
    webServer.handleClient();
  }
}
