#include <BTAdvertisedDeviceCallbacks.h>
#include <BTClientCallbacks.h>
#include <BTDeviceManager.h>
#include <Config.h>
#include <DirConManager.h>
#include <TrainerControl.h>
#include <UUIDs.h>
#include <Utils.h>
#include <arduino-timer.h>

std::vector<NimBLEUUID> BTDeviceManager::remoteDeviceFilterUUIDs{};
std::vector<NimBLEAdvertisedDevice> BTDeviceManager::scannedDevices{};
ServiceManager* BTDeviceManager::serviceManager{};
std::string BTDeviceManager::deviceName;
std::string BTDeviceManager::remoteDeviceName;
std::string BTDeviceManager::connectedDeviceName;
NimBLEClient* BTDeviceManager::nimBLEClient{};
bool BTDeviceManager::connected = false;
bool BTDeviceManager::started = false;
Timer<> BTDeviceManager::scanTimer = timer_create_default();
Timer<> BTDeviceManager::connectTimer = timer_create_default();
String BTDeviceManager::statusMessage = "";
std::vector<NimBLEUUID> BTDeviceManager::pinnedCharacteristics{};

class BTDeviceServiceManagerCallbacks : public ServiceManagerCallbacks {
  void onCharacteristicSubscriptionChanged(Characteristic* characteristic, bool removed) {
    if (!characteristic->getService()->isInternal()) {
      if (BTDeviceManager::isPinned(characteristic->UUID)) {
        // the bridge itself needs this one, it stays subscribed for the whole connection
        return;
      }
      if (removed && (characteristic->getSubscriptions().size() == 0)) {
        BTDeviceManager::changeBLENotify(characteristic, true);
      } else {
        if (characteristic->getSubscriptions().size() > 0) {
          BTDeviceManager::changeBLENotify(characteristic, false);
        }
      }
    }
  };
};

void BTDeviceManager::onBLENotify(BLERemoteCharacteristic* pBLERemoteCharacteristic, uint8_t* pData, size_t length, bool isNotify) {
  TrainerControl::onNotify(pBLERemoteCharacteristic->getUUID(), pData, length);
  DirConManager::notifyDirConCharacteristic(pBLERemoteCharacteristic->getUUID(), pData, length);
}

std::vector<NimBLEAdvertisedDevice>* BTDeviceManager::getScannedDevices() {
  return &scannedDevices;
}

bool BTDeviceManager::start() {
  if (!started) {
    scannedDevices.clear();
    if (serviceManager == nullptr) {
      return false;
    }
    serviceManager->subscribeCallbacks(new BTDeviceServiceManagerCallbacks);
    NimBLEDevice::setScanFilterMode(CONFIG_BTDM_SCAN_DUPL_TYPE_DATA_DEVICE);
    NimBLEDevice::setScanDuplicateCacheSize(200);
    NimBLEDevice::init(deviceName);
    started = true;
    startScan();
    return true;
  }
  return false;
}

void BTDeviceManager::stop() {
  connected = false;
  started = false;
  connectTimer.cancel();
  stopScan();
}

void BTDeviceManager::update() {
  scanTimer.tick();
  connectTimer.tick();
  if (connected) {
    if (nimBLEClient != nullptr) {
      if (!nimBLEClient->isConnected()) {
        connected = false;
        connectedDeviceName = "";
        TrainerControl::onDisconnected();
        startScan();
      }
    }
  }
}

bool BTDeviceManager::isConnected() {
  return connected;
}

std::string BTDeviceManager::getConnecedDeviceName() {
  return connectedDeviceName;
}

std::string BTDeviceManager::getConnectedAddress() {
  if (connected && (nimBLEClient != nullptr) && nimBLEClient->isConnected()) {
    return nimBLEClient->getPeerAddress().toString();
  }
  return "";
}

int BTDeviceManager::getRssi() {
  if (connected && (nimBLEClient != nullptr) && nimBLEClient->isConnected()) {
    return nimBLEClient->getRssi();
  }
  return 0;
}

bool BTDeviceManager::isStarted() {
  return started;
}

void BTDeviceManager::setLocalDeviceName(const std::string localDeviceName) {
  deviceName = localDeviceName;
}

void BTDeviceManager::setRemoteDeviceNameFilter(const std::string remoteDeviceNameFilter) {
  remoteDeviceName = remoteDeviceNameFilter;
  if (started) {
    if (NimBLEDevice::getScan() != nullptr) {
      if (!NimBLEDevice::getScan()->isScanning()) {
        startScan();
      } else {
        if (BTDeviceManager::getRemoteDevice() != nullptr) {
          stopScan();
        }
      }
    }
  }
}

void BTDeviceManager::setServiceManager(ServiceManager* serviceManager) {
  BTDeviceManager::serviceManager = serviceManager;
}

void BTDeviceManager::addRemoteDeviceFilter(const NimBLEUUID& serviceUUID) {
  remoteDeviceFilterUUIDs.push_back(serviceUUID);
}

NimBLEAdvertisedDevice* BTDeviceManager::getRemoteDevice() {
  if (remoteDeviceName != "") {
    for (size_t deviceIndex = 0; deviceIndex < scannedDevices.size(); deviceIndex++) {
      NimBLEAdvertisedDevice* scannedDevice = &scannedDevices.at(deviceIndex);
      if (scannedDevice->haveName()) {
        if (scannedDevice->getName().rfind(remoteDeviceName, 0) == 0) {
          return &scannedDevices.at(deviceIndex);
        }
      }
    }
  }
  return nullptr;
}

bool BTDeviceManager::connectRemoteDevice(NimBLEAdvertisedDevice* remoteDevice) {
  if (remoteDevice == nullptr) {
    log_e("No trainer selected or trainer not found in scan results");
    statusMessage = "No trainer selected";
    startScan();
    return false;
  }
  pinnedCharacteristics.clear();
  log_i("Connecting to BLE device %s (%s)", remoteDevice->getAddress().toString().c_str(), remoteDevice->getName().c_str());
  stopScan();
  if (nimBLEClient != nullptr) {
    // v1 created a new client on every reconnect and never freed the old one (a few KB each)
    NimBLEDevice::deleteClient(nimBLEClient);
    nimBLEClient = nullptr;
  }
  nimBLEClient = NimBLEDevice::createClient();
  nimBLEClient->setClientCallbacks(new BTClientCallbacks(), true);
  nimBLEClient->setConnectTimeout(BLE_CONNECT_TIMEOUT);
  if (!nimBLEClient->connect(remoteDevice)) {
    log_e("Connection to BLE device failed, error %d", nimBLEClient->getLastError());
    statusMessage = "Connection failed";
    nimBLEClient->disconnect();
    connectedDeviceName = "";
    return false;
  }
  connected = true;
  connectedDeviceName = remoteDevice->getName();
  statusMessage = "Connected to '";
  statusMessage += connectedDeviceName.c_str();
  statusMessage += "'";

  if (serviceManager != nullptr) {
    std::vector<NimBLERemoteService*>* remoteServices = nimBLEClient->getServices(true);
    for (auto remoteService = remoteServices->begin(); remoteService != remoteServices->end(); remoteService++) {
      // if FTMS emulation is enabled skip the FTMS service
      if (!(SettingsManager::isFTMSEnabled() && (*remoteService)->getUUID().equals(NimBLEUUID(FITNESS_MACHINE_SERVICE_UUID)))) {
        Service* service = serviceManager->getService((*remoteService)->getUUID());
        bool isAdvertising = remoteDevice->isAdvertisingService((*remoteService)->getUUID());
        // little hack for service advertising that gets truncated
        if (!isAdvertising) {
          if ((*remoteService)->getUUID().equals(NimBLEUUID(TACX_FEC_PRIMARY_SERVICE_UUID))) {
            isAdvertising = true;
          }
        }
        if (service == nullptr) {
          service = new Service((*remoteService)->getUUID(), isAdvertising, false);
          serviceManager->addService(service);
        }
        service->Advertise = isAdvertising;
        std::vector<NimBLERemoteCharacteristic*>* remoteCharacteristics = (*remoteService)->getCharacteristics(true);
        for (auto remoteCharacterisic = remoteCharacteristics->begin(); remoteCharacterisic != remoteCharacteristics->end(); remoteCharacterisic++) {
          Characteristic* characteristic = service->getCharacteristic((*remoteCharacterisic)->getUUID());
          uint32_t properties = 0;
          if (characteristic == nullptr) {
            characteristic = new Characteristic((*remoteCharacterisic)->getUUID(), getProperties((*remoteCharacterisic)));
            service->addCharacteristic(characteristic);
          }
          characteristic->setProperties(getProperties((*remoteCharacterisic)));
        }
      }
    }
    TrainerControl::onConnected(nimBLEClient->getService(NimBLEUUID(FITNESS_MACHINE_SERVICE_UUID)) != nullptr,
                                nimBLEClient->getService(NimBLEUUID(TACX_FEC_PRIMARY_SERVICE_UUID)) != nullptr);
    // after a reconnect the trainer forgot all notification subscriptions, restore the ones Zwift still has
    for (Service* service : serviceManager->getServices()) {
      if (service->isInternal()) {
        continue;
      }
      for (Characteristic* characteristic : service->getCharacteristics()) {
        if ((characteristic->getSubscriptions().size() > 0) && !isPinned(characteristic->UUID)) {
          changeBLENotify(characteristic, false);
        }
      }
    }
  } else {
    log_e("Connection to BLE device failed, service manager not available");
    nimBLEClient->disconnect();
    connectedDeviceName = "";
    return false;
  }
  return true;
}

void BTDeviceManager::startScan() {
  if (started) {
    connectTimer.cancel();
    scanTimer.every(BLE_SCAN_INTERVAL, BTDeviceManager::doScan);
  } else {
    scanTimer.cancel();
  }
}

void BTDeviceManager::stopScan() {
  log_i("Stopping BLE scan...");
  statusMessage = "Disconnected";
  scanTimer.cancel();
  NimBLEDevice::getScan()->stop();
}

bool BTDeviceManager::doScan(void* argument) {
  if (started) {
    NimBLEScan* nimBLEScanner = NimBLEDevice::getScan();
    if (!nimBLEScanner->isScanning()) {
      log_i("Starting BLE scan...");
      if (nimBLEClient != nullptr) {
        if (nimBLEClient->isConnected()) {
          nimBLEClient->disconnect();
          connectedDeviceName = "";
          connected = false;
        }
      }
      nimBLEScanner->setAdvertisedDeviceCallbacks(new BTAdvertisedDeviceCallbacks(), false);
      nimBLEScanner->setActiveScan(true);
      nimBLEScanner->setInterval(97);
      nimBLEScanner->setWindow(37);
      nimBLEScanner->setMaxResults(0);
      scannedDevices.clear();
      if (nimBLEScanner->start(0, BTDeviceManager::onScanEnd, false)) {
        statusMessage = "Scanning...";
        log_i("BLE scan started successfully");
        return false;
      }
    } else {
      log_e("BLE scan already running");
      return false;
    }
  }
  log_e("BLE scan start failed");
  return true;
}

void BTDeviceManager::onScanEnd(NimBLEScanResults scanResults) {
  log_i("BLE scan finished with %d devices", scannedDevices.size());
  if (scannedDevices.size() <= 0) {
    if (!connected) {
      startScan();
    }
  } else {
    if (!connected) {
      connectTimer.every(BLE_CONNECT_INTERVAL, BTDeviceManager::doConnect);
    }
  }
}

bool BTDeviceManager::doConnect(void* argument) {
  log_i("Starting BLE connection...");
  if (started) {
    statusMessage = "Connecting...";
    if (connectRemoteDevice(getRemoteDevice())) {
      return false;
    }
  }
  log_e("BLE connection failed: Bluetooth device manager not started");
  return true;
}

uint32_t BTDeviceManager::getProperties(NimBLERemoteCharacteristic* remoteCharacteristic) {
  uint32_t returnValue = 0x00;
  if (remoteCharacteristic->canRead()) {
    returnValue = returnValue | READ;
  }
  if (remoteCharacteristic->canWrite() || remoteCharacteristic->canWriteNoResponse()) {
    returnValue = returnValue | WRITE;
  }
  if (remoteCharacteristic->canIndicate() || remoteCharacteristic->canNotify() || remoteCharacteristic->canBroadcast()) {
    returnValue = returnValue | NOTIFY;
  }
  return returnValue;
}

bool BTDeviceManager::writeBLECharacteristic(const NimBLEUUID& serviceUUID, const NimBLEUUID& characteristicUUID, std::vector<uint8_t>* data, bool withResponse) {
  if (!connected) {
    log_e("BLE characteristic write failed: Not connected");
    return false;
  }
  NimBLERemoteService* remoteService = nullptr;
  NimBLERemoteCharacteristic* remoteCharacteristic = nullptr;
  remoteService = nimBLEClient->getService(serviceUUID);
  if (remoteService) {
    remoteCharacteristic = remoteService->getCharacteristic(characteristicUUID);
    if (remoteCharacteristic) {
      if (remoteCharacteristic->canWrite() || remoteCharacteristic->canWriteNoResponse()) {
        // value needs to be copied to this scope as it will be handled asynchronously by another core and NimBLE will only write garbage
        std::vector<uint8_t> valueData;
        for (auto currentByte = data->begin(); currentByte != data->end(); currentByte++) {
          valueData.push_back(*currentByte);
        }
        bool response = withResponse && remoteCharacteristic->canWrite();
        if (!remoteCharacteristic->canWriteNoResponse()) {
          response = true;
        }
        if (remoteCharacteristic->writeValue(valueData, response)) {
          return true;
        }
      } else {
        log_e("BLE characteristic write failed: Remote characteristic not writable");
      }
    } else {
      log_e("BLE characteristic write failed: Remote characteristic not found");
    }
  } else {
    log_e("BLE characteristic write failed: Remote service not found");
  }
  return false;
}

std::vector<uint8_t> BTDeviceManager::readBLECharacteristic(const NimBLEUUID& serviceUUID, const NimBLEUUID& characteristicUUID) {
  std::vector<uint8_t> returnData;
  if (!connected) {
    log_e("BLE characteristic read failed: Not connected");
    return returnData;
  }
  NimBLERemoteService* remoteService = nullptr;
  NimBLERemoteCharacteristic* remoteCharacteristic = nullptr;
  remoteService = nimBLEClient->getService(serviceUUID);
  if (remoteService) {
    remoteCharacteristic = remoteService->getCharacteristic(characteristicUUID);
    if (remoteCharacteristic) {
      if (remoteCharacteristic->canRead()) {
        returnData = remoteCharacteristic->readValue();
      } else {
        log_e("BLE characteristic read failed: Remote characteristic not readable");
      }
    } else {
      log_e("BLE characteristic read failed: Remote characteristic not found");
    }
  } else {
    log_e("BLE characteristic read failed: Remote service not found");
  }
  return returnData;
}

bool BTDeviceManager::changeBLENotify(Characteristic* characteristic, bool remove) {
  if (!connected) {
    log_e("BLE characteristic change notify failed: Not connected");
    return false;
  }
  NimBLERemoteService* remoteService = nullptr;
  NimBLERemoteCharacteristic* remoteCharacteristic = nullptr;
  remoteService = nimBLEClient->getService(characteristic->getService()->UUID);
  if (remoteService) {
    remoteCharacteristic = remoteService->getCharacteristic(characteristic->UUID);
    if (remoteCharacteristic) {
      if (remoteCharacteristic->canNotify() || remoteCharacteristic->canIndicate()) {
        if (!remove) {
          // indicate-only characteristics (e.g. FTMS control point) need an indication subscription;
          // the CCCD write must be confirmed (Saris ignores unconfirmed ones)
          return remoteCharacteristic->subscribe(remoteCharacteristic->canNotify(), onBLENotify, true);
        } else {
          return remoteCharacteristic->unsubscribe(true);
        }
      } else {
        log_e("BLE characteristic change notify failed: Remote characteristic %s can't notify nor indicate", characteristic->UUID.to128().toString().c_str());
      }
    } else {
      log_e("BLE characteristic change notify failed: Remote characteristic %s not found", characteristic->UUID.to128().toString().c_str());
    }
  } else {
    log_e("BLE characteristic change notify failed: Remote service for characteristic %s not found", characteristic->UUID.to128().toString().c_str());
  }
  return false;
}

bool BTDeviceManager::subscribeRemote(const NimBLEUUID& serviceUUID, const NimBLEUUID& characteristicUUID) {
  if (!connected || (nimBLEClient == nullptr)) {
    return false;
  }
  NimBLERemoteService* remoteService = nimBLEClient->getService(serviceUUID);
  if (remoteService == nullptr) {
    log_w("Trainer has no service %s", serviceUUID.toString().c_str());
    return false;
  }
  NimBLERemoteCharacteristic* remoteCharacteristic = remoteService->getCharacteristic(characteristicUUID);
  if (remoteCharacteristic == nullptr) {
    log_w("Trainer has no characteristic %s", characteristicUUID.toString().c_str());
    return false;
  }
  if (remoteCharacteristic->getDescriptor(NimBLEUUID((uint16_t)0x2902)) == nullptr) {
    // NimBLE would report success without subscribing anything
    log_e("Trainer characteristic %s has no notification descriptor (CCCD)", characteristicUUID.toString().c_str());
    return false;
  }
  bool result = false;
  // write the CCCD *with* response: the spec requires it and Saris trainers ignore it otherwise
  if (remoteCharacteristic->canNotify()) {
    result = remoteCharacteristic->subscribe(true, onBLENotify, true);
  } else if (remoteCharacteristic->canIndicate()) {
    result = remoteCharacteristic->subscribe(false, onBLENotify, true);
  }
  if (result) {
    if (!isPinned(characteristicUUID)) {
      pinnedCharacteristics.push_back(characteristicUUID);
    }
    log_i("Subscribed to trainer characteristic %s", characteristicUUID.toString().c_str());
  } else {
    log_e("Subscribing to trainer characteristic %s failed", characteristicUUID.toString().c_str());
  }
  return result;
}

bool BTDeviceManager::isPinned(const NimBLEUUID& characteristicUUID) {
  for (const NimBLEUUID& uuid : pinnedCharacteristics) {
    if (uuid.equals(characteristicUUID)) {
      return true;
    }
  }
  return false;
}

void BTDeviceManager::forceReconnect() {
  log_i("Forcing trainer reconnect");
  if ((nimBLEClient != nullptr) && nimBLEClient->isConnected()) {
    // the disconnect callback starts a new scan, which reconnects to the selected trainer
    nimBLEClient->disconnect();
  } else {
    stopScan();
    startScan();
  }
}

String BTDeviceManager::getStatusMessage() {
  return statusMessage;
}