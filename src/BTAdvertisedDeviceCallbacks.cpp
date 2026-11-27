#include <BTAdvertisedDeviceCallbacks.h>
#include <BTDeviceManager.h>
#include <NimBLEDevice.h>

// Runs in the NimBLE host task for every advertisement while the bridge looks for the trainer.
//
// Up to v1.6.1 / v2.0.0-alpha.6 every report was appended to scannedDevices. The scan runs
// without end while the trainer is off, and phones and watches change their advertising
// data constantly, so each change counted as a new device: the list grew without limit
// until operator new failed in this task and the bridge aborted ("reset panic", core dump
// in nimble_host). Now only named devices are kept (the trainer is chosen by name), each
// address once, and the list never grows past its reserved size, so it never reallocates.
void BTAdvertisedDeviceCallbacks::onResult(NimBLEAdvertisedDevice* advertisedDevice) {
  for (NimBLEUUID uuid : BTDeviceManager::remoteDeviceFilterUUIDs) {
    if (!advertisedDevice->isAdvertisingService(uuid)) {
      return;
    }
  }
  if (!advertisedDevice->haveName() || advertisedDevice->getName().empty()) {
    return;
  }
  bool added = BTDeviceManager::addScannedDevice(advertisedDevice);
  if (added) {
    log_d("Adding %s (%s) to device list", advertisedDevice->getAddress().toString().c_str(), advertisedDevice->getName().c_str());
  }
  if (BTDeviceManager::getRemoteDevice() != nullptr) {
    // only stop the radio scan here (NimBLE host task); BTDeviceManager::stopScan() also touches
    // the timers, which belong to the main loop. The scan-end flag takes it from there.
    NimBLEDevice::getScan()->stop();
  }
}
