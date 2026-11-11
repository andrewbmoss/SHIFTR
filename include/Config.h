#ifndef CONFIG_H
#define CONFIG_H

#define ST(A) #A
#define STR(A) ST(A)

#define DEFAULT_USERNAME "admin"

#define DEVICE_NAME_PREFIX "SHIFTR"

#define WEB_SERVER_PORT 80

#define DIRCON_TCP_PORT 8080
#define DIRCON_MAX_CLIENTS 3
#define DIRCON_NOTIFICATION_INTERVAL 1000

#define BLE_SCAN_INTERVAL 1000
#define BLE_CONNECT_INTERVAL 1000
#define BLE_CONNECT_TIMEOUT 10

// Trainer command scheduling (see TrainerControl)
#define TRAINER_COMMAND_MIN_INTERVAL 250        // ms between two commands to the trainer
#define TRAINER_COMMAND_KEEPALIVE_INTERVAL 5000 // ms after which the last command is re-sent
#define TRAINER_CONTROL_RETRY_INTERVAL 2000     // ms between FTMS "request control" attempts
#define TRAINER_CONTROL_INDICATION_TIMEOUT 1000 // ms to wait for the FTMS control response
#define TRAINER_DATA_TIMEOUT 3000               // ms without trainer data before NOMINAL_CADENCE is used
#define NOMINAL_CADENCE 85                      // rpm assumed for virtual shifting when the trainer sends no data

// Function button (re-pair / reboot), active low with internal pull-up
#define FUNCTION_BUTTON_PIN 4
#define FUNCTION_BUTTON_LONG_PRESS 5000         // ms held for a reboot

// ArduinoOTA (upload from PlatformIO over the network)
#define ARDUINO_OTA_PORT 3232

#endif
