#ifndef DIRCONMANAGER_H
#define DIRCONMANAGER_H

#define DIRCON_MDNS_SERVICE_NAME "_wahoo-fitness-tnp"
#define DIRCON_MDNS_SERVICE_PROTOCOL "tcp"

#include <AsyncTCP.h>
#include <Config.h>
#include <DirConMessage.h>
#include <ServiceManager.h>
#include <UUIDs.h>
#include <arduino-timer.h>
#include <SettingsManager.h>
#include <deque>

typedef enum TrainerMode {
  ERG_MODE = 0,
  SIM_MODE = 1,
  SIM_MODE_VIRTUAL_SHIFTING = 2
} trainerModeType;

class DirConManager {
 public:
  static bool start();
  static void stop();
  static void update();
  static void setServiceManager(ServiceManager* serviceManager);
  static void notifyDirConCharacteristic(const NimBLEUUID& characteristicUUID, uint8_t* pData, size_t length);
  static String getStatusMessage();
  static TrainerMode getZwiftTrainerMode();
  static double getZwiftGradePercent();
  // grade after smoothing (what the virtual-shifting model uses)
  static double getSmoothedGradePercent() { return filteredGrade; }
  // Cw handed to the trainer and the part of the air drag that didn't fit under its limit
  static double getTrainerWindCoefficient() { return trainerWindCoefficient; }
  static double getResidualDragForce() { return residualDragForce; }
  // power the trainer should be absorbing through that Cw (the part trainer weight doesn't affect)
  static double getTrainerDragPower() { return trainerDragPower; }
  // Zwift rider + bike weight in kg
  static double getTotalWeight() { return (zwiftBicycleWeight + zwiftUserWeight) / 100.0; }
  static double getZwiftGearRatio();
  static double getSimulatedGearRatio();
  static String getVirtualGearLabel();
  // power the simulated bike needs at the current cadence, gear and grade
  static double getModelPower();
  static size_t getConnectedClientCount();
  static uint32_t getBlockedPassthroughWrites();
  static double getDefaultGearRatio();
  static String getZwiftTrafficReport();

 private:
  friend class DirConServiceManagerCallbacks;
  static bool doNotifications(void* arg);
  static void handleNewClient(void* arg, AsyncClient* client);
  static void handleDirConData(void* arg, AsyncClient* client, void* data, size_t len);
  static void handleDirConError(void* arg, AsyncClient* client, int8_t error);
  static void handleDirConDisconnect(void* arg, AsyncClient* client);
  static void handleDirConTimeOut(void* arg, AsyncClient* client, uint32_t time);
  static void removeSubscriptions(AsyncClient* client);
  static size_t getDirConClientIndex(AsyncClient* client);
  static bool processDirConMessage(DirConMessage* dirConMessage, AsyncClient* client, size_t clientIndex);
  static uint8_t getDirConProperties(uint32_t characteristicProperties);
  static bool processZwiftSyncRequest(Service* service, Characteristic* characteristic, std::vector<uint8_t>* requestData);
  static void notifyDirConCharacteristic(Characteristic* characteristic, uint8_t* pData, size_t length);
  static void notifyInternalCharacteristics();
  static std::vector<uint8_t> generateZwiftAsyncNotificationData(int64_t power, int64_t cadence, int64_t unknown1, int64_t unknown2, int64_t unknown3, int64_t unknown4 = 25714LL);
  static std::map<uint8_t, uint64_t> getZwiftDataValues(std::vector<uint8_t>* requestData);
  static void sendDirConCharacteristicNotification(const NimBLEUUID& characteristicUUID, uint8_t* pData, size_t length, bool onlySubscribers);
  static void sendDirConCharacteristicNotification(Characteristic* characteristic, uint8_t* pData, size_t length, bool onlySubscribers);
  static void updateStatusMessage();
  static void resetValues();
  static void updateSIMModeResistance();
  static bool isTrainerControlCharacteristic(const NimBLEUUID& characteristicUUID);
  static std::vector<uint8_t> processFTMSReadRequest(Service* service, Characteristic* characteristic, std::vector<uint8_t>* requestData); 
  static std::vector<uint8_t> processFTMSWriteRequest(Service* service, Characteristic* characteristic, std::vector<uint8_t>* requestData);
  static std::vector<uint8_t> generateIndoorBikeDataNotificationData(uint16_t instantaneousSpeed, uint16_t instantaneousCadence, int16_t instantaneousPower);
  
  static ServiceManager* serviceManager;
  static Timer<> notificationTimer;
  static AsyncServer* dirConServer;
  static AsyncClient* dirConClients[DIRCON_MAX_CLIENTS];
  static bool started;
  static String statusMessage;
  static uint8_t zwiftAsyncRideOnAnswer[];
  static uint8_t zwiftSyncRideOnAnswer[];

  static double defaultGearRatio;
  static VirtualShiftingMode virtualShiftingMode;

  static TrainerMode zwiftTrainerMode;
  static uint64_t zwiftPower;
  static int64_t zwiftGrade;
  static uint64_t zwiftGearRatio;
  static uint16_t zwiftBicycleWeight;
  static uint16_t zwiftUserWeight;

  static double filteredGrade;
  static uint32_t filteredGradeAt;
  static bool filteredGradeValid;
  static double trainerWindCoefficient;
  static double residualDragForce;
  static double trainerDragPower;
  static double getFilteredGradePercent();

  static bool zwiftProtocolActive;
  static uint32_t blockedPassthroughWrites;

  static uint16_t difficulty;

  static int16_t ftmsPower;
  static int16_t ftmsWindSpeed;
  static int16_t ftmsGrade;
  static uint8_t ftmsCrr;
  static uint8_t ftmsCw;

};

#endif
