#ifndef BTDEVICEMANAGER_H
#define BTDEVICEMANAGER_H

#include <Arduino.h>
#include <NimBLEDevice.h>
#include <ServiceManager.h>
#include <arduino-timer.h>

class BTDeviceManager {
 public:
  static bool start();
  static void stop();
  static void update();
  static void setLocalDeviceName(const std::string localDeviceName);
  static void setRemoteDeviceNameFilter(const std::string remoteDeviceNameFilter);
  static void setServiceManager(ServiceManager* serviceManager);
  static void addRemoteDeviceFilter(const NimBLEUUID& serviceUUID);
  static bool isConnected();
  static bool isStarted();
  static std::string getConnecedDeviceName();
  static std::string getConnectedAddress();
  // adds a named device once per address; false when it was already listed or the list is full
  static bool addScannedDevice(NimBLEAdvertisedDevice* device);
  // names of the devices found so far, for the settings page
  static std::vector<std::string> getScannedDeviceNames();
  // signal strength of the trainer connection, 0 when not connected
  static int getRssi();
  static bool writeBLECharacteristic(const NimBLEUUID& serviceUUID, const NimBLEUUID& characteristicUUID, std::vector<uint8_t>* data, bool withResponse = false);
  static std::vector<uint8_t> readBLECharacteristic(const NimBLEUUID& serviceUUID, const NimBLEUUID& characteristicUUID);
  // subscribe for the whole connection (used by TrainerControl for its own data)
  static bool subscribeRemote(const NimBLEUUID& serviceUUID, const NimBLEUUID& characteristicUUID);
  static bool isPinned(const NimBLEUUID& characteristicUUID);
  static void forceReconnect();
  // let go of the trainer so another app (Saris/Wahoo spindown) can connect; no scanning until resumed
  static void release();
  static void resume();
  static bool isReleased();
  static std::vector<NimBLEAdvertisedDevice>* getScannedDevices();
  static String getStatusMessage();

 private:
  friend class BTAdvertisedDeviceCallbacks;
  friend class BTClientCallbacks;
  friend class BTDeviceServiceManagerCallbacks;
  static std::string deviceName;
  static std::string remoteDeviceName;
  static std::string connectedDeviceName;
  static bool connected;
  static bool started;
  static bool released;
  static ServiceManager* serviceManager;
  static NimBLEClient* nimBLEClient;
  static std::vector<NimBLEUUID> remoteDeviceFilterUUIDs;
  static std::vector<NimBLEAdvertisedDevice> scannedDevices;

  static Timer<> scanTimer;
  static Timer<> connectTimer;
  static NimBLEAdvertisedDevice* getRemoteDevice();
  static bool connectRemoteDevice(NimBLEAdvertisedDevice* remoteDevice);
  static void onScanEnd(NimBLEScanResults scanResults);
  static void startScan();
  static void stopScan();
  static bool doScan(void* argument);
  static bool doConnect(void* argument);
  static uint32_t getProperties(NimBLERemoteCharacteristic* remoteCharacteristic);
  static void onBLENotify(BLERemoteCharacteristic* pBLERemoteCharacteristic, uint8_t* pData, size_t length, bool isNotify);
  static bool changeBLENotify(Characteristic* characteristic, bool remove);
  static std::vector<NimBLEUUID> pinnedCharacteristics;
  static String statusMessage;
};

#endif