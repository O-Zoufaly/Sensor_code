#include <Arduino.h>
#include <esp_now.h>
#include <WiFi.h>
#include <NimBLEDevice.h>
#include <M5GFX.h>
#include <M5Unified.h>

// If other module is not connected send special value to let PC now
// the other module is not connected
#define NOT_CONNECTED 112

// BLE Service and characteristtic UUIDs
#define SERVICE_UUID        "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHARACTERISTIC_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a8"

// Address of the other sensor module
uint8_t broadcastAddress[] = {0xF0, 0x24, 0xF9, 0x97, 0x49, 0x98};


// Simple variables and bool flags
NimBLECharacteristic* pCharacteristic    = nullptr;
bool                  deviceConnected    = false;
uint8_t bleCommand = 0;
bool newMsg = false;
volatile bool newPkt = false;
bool startPktTransmission = false;
uint8_t batteryPkt[2] = {0, 0};
bool sendBattery = false;
volatile bool batteryReceived = false;
uint8_t battery = 0;
uint32_t startTime = 0;
volatile bool sensorConnected = false;
volatile bool sensorConnectionConfirm = false;
volatile bool checkSent = false;
volatile bool flagChanged = false;
uint32_t timerValue = 0;
volatile bool* flagPtr = NULL;

// Timers for checkeing connection and power saving mode
hw_timer_t *hwTimer = NULL;
hw_timer_t *connectionTimer = NULL;

/* temp **************************/
float angle = 0;
uint32_t raw = 0;
uint32_t count = 0;

//Packed structure for IMU packet
#pragma pack(push,1)
struct ImuPacket
{
    uint8_t  sessionId{};
    uint32_t timestamp{};

    int16_t ax{}, ay{}, az{};
    int16_t gx{}, gy{}, gz{};
};

#pragma pack(pop)

//Enum class for communication
enum class Command : uint8_t
{
    Stop = 0,
    Start = 1,
    Battery = 2,
    ConnectionSuccess = 3,
    ConnectionCeased = 4
};

// IMU packet array
ImuPacket pkt[2];

// ESP-Now variable for storing info about the other sensor module
esp_now_peer_info_t peerInfo;

/* Flag changed function
//  Input: flag - reference to bool flag to be changed
//         value - value to set
//  Function -  if the value is different from current value of the flag, 
//            the flag is changed flagChanged variable gets set and the
//             flagPtr is stored, otherwise nothing happens
*/
void changeFlag(volatile bool& flag, bool value){
  if (flag == value) return;
  else{
    flagPtr = &flag;
    flag = value;
    flagChanged = true;
  }
}

/* OnDataRecv callback function
//  Input: mac - address of the sender
//         incomingData - pointer to the first byte of incoming data
//         len - lenght of the incoming data in bytes
//  Function - sets the sensorConnected flag to true to indicate the 
//             connection between sensors is OK, and sets the 
//             sensorConnectionConfirm to false to start connection 
//             verification timer, lastly based on the length of 
//             incoming data copies the data into new IMU packet or 
//             into new battery packet
*/
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  changeFlag(sensorConnected, true);
  sensorConnectionConfirm = false;
  if (deviceConnected){
    if (startPktTransmission && (len == 17)){
      memcpy(&pkt[0], incomingData, sizeof(pkt[0]));
      newPkt = true;
    } else if(len == 1){
      memcpy(&batteryPkt[0], incomingData, sizeof(batteryPkt[0]));
      batteryReceived = true;
    }
  }
}

/* OnDataSent callback function
//  Input: mac - address of the sender
//         status - status of sending ESP-Now data
//  Function - no function
*/
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
}


/* BLE Server callback class
//  Callbacks: onConnect - runs when device connects to the server
//             onDisconnect - runs when device disconnects from the server
*/
class ServerCallbacks : public NimBLEServerCallbacks {

/*  onConnect callback function
//  Input: pServer - pointer to the server
//         connInfo - information about the connection
//  Function - change device connected flag to true to indicate device has connected
//             sends ConnectionSuccess command to other module via ESP-Now and
//             updates connection parameters to enable high speed data transfer
*/
    void onConnect(NimBLEServer* pServer, NimBLEConnInfo& connInfo) override 
    { 
      changeFlag(deviceConnected, true);
      Command cmd = Command::ConnectionSuccess;
      esp_now_send(broadcastAddress, (uint8_t *) &cmd, sizeof(cmd));
      Serial.println("**************Device connected**************");
      pServer->updateConnParams(connInfo.getConnHandle(),
                          6,  // min interval (7.5ms)
                          12, // max interval (15ms)
                          0,
                          60);
    };

/*  onDisconnect callback function
//  Input: pServer - pointer to the server
//         connInfo - information about the connection
//         reason - reason the disconnec happened
//  Function - adds the device that disconnected to the the whitelist to minimize 
//             reconnection times, changes the deviceConnected flag to false and 
//             sends ConnectionCeased command to the other module via ESP-Now
*/
    void onDisconnect(NimBLEServer* pServer, NimBLEConnInfo& connInfo, int reason) override {
        NimBLEDevice::whiteListAdd(connInfo.getAddress());
        changeFlag(deviceConnected, false);
        Command cmd = Command::ConnectionCeased;
        esp_now_send(broadcastAddress, (uint8_t *) &cmd, sizeof(cmd));
    }
} serverCallbacks;

/* BLE Characteristic callback class
//  Callbacks: onWrite - runs when device writes to the characterisic
*/
class CharacteristicCallbacks : public NimBLECharacteristicCallbacks {

/*  onWrite callback function
//  Input:  pCharacteristic - pointer to the server characteristic
//         connInfo - information about the connection
//  Function - receives the incoming data and if they are just one byte as expected  
//             casts to value to Command Enum and based on the 
//             Command sets flags and variables 
*/
    void onWrite(NimBLECharacteristic* pCharacteristic, NimBLEConnInfo& connInfo) override {
      if (pCharacteristic->getLength() == 1){
        Command cmd = static_cast<Command>((uint8_t)pCharacteristic->getValue()[0]);
        
        switch (cmd)
        {
          case Command::Start:
              bleCommand = 1;
              startPktTransmission = true;
              newMsg = true;
              startTime = millis();
              pkt[1].sessionId++;
              break;

          case Command::Stop:
              bleCommand = 0;
              newMsg = true;
              startPktTransmission = false;
              break;

          case Command::Battery:
              bleCommand = 2;
              newMsg = true;
              sendBattery = true;
              batteryReceived = false;
              break;

          case Command::ConnectionSuccess:
              bleCommand = 3;
              newMsg = true;
              break;
        }
      }
    }
}chrCallbacks;

/*  onAdvComplete callback function
//  Input:  pAdvertising - pointer to the server advvertising
//  Function - if the advertising ended by device connection nothing is done, else
//             the advertising is restarted
*/
void onAdvComplete(NimBLEAdvertising* pAdvertising) {
    Serial.println("*************Advertising stopped*************");
    if (deviceConnected) {
        return;
    }
    
    pAdvertising->setScanFilter(false, false);
    pAdvertising->start();
}

/*  drawBattery function
//  Input:  x - x position of the battery
//          y - y position of the battery
//          w - width of the battery
//          h - height of the battery
//          lvl - battery level
//  Function - draws the battery with the level to the display
*/
void drawBattery(int x,int y,int w, int h,uint8_t lvl){
  int color = (lvl>30) ? (DARKGREEN) : (ORANGE);
  M5.Display.drawRect(x, y, w, h, color);
  M5.Display.fillRect(x + 1, y + 1, w * (((float)lvl) / 100.0f), h - 1, color);
}

/*  updateDisplay function
//  Input:  none
//  Function - updates the display based on the value of the flags
*/
void updateDisplay(){

  M5.Display.fillScreen(BLACK);

  //Battery
  M5.Display.setTextSize(1.3f);
  M5.Display.setTextColor(BLUE);
  M5.Display.drawString("Battery:",M5.Display.width() / 2,M5.Display.height() - 60);
  drawBattery(30, M5.Display.height() - 40, 70,25,battery);

  //Module connection
  M5.Display.setTextColor(BLUE);
  M5.Display.drawString("Sensor status:",M5.Display.width() / 2,M5.Display.height() - 120);
  if (sensorConnected){
    M5.Display.setTextColor(DARKGREEN);
    M5.Display.drawString("Connected",M5.Display.width() / 2,M5.Display.height() - 90);
  } else {
    M5.Display.setTextColor(ORANGE);
    M5.Display.drawString("Waiting",M5.Display.width() / 2,M5.Display.height() - 90);
  }
  //PC connection
  M5.Display.setTextColor(BLUE);
  M5.Display.drawString("PC status:",M5.Display.width() / 2,M5.Display.height() - 180);
  if (deviceConnected){
    M5.Display.setTextColor(DARKGREEN);
    M5.Display.drawString("Connected",M5.Display.width() / 2,M5.Display.height() - 150);
  } else {
    M5.Display.setTextColor(ORANGE);
    M5.Display.drawString("Waiting",M5.Display.width() / 2,M5.Display.height() - 150);
  }
  M5.Display.setTextSize(1.8f);
  M5.Display.setTextColor(MAGENTA);
  M5.Display.drawString("ANKLE",M5.Display.width() / 2,M5.Display.height() - 210);
  flagChanged = false;
}

/*  onTimer callback function
//  Input:  none
//  Function - runs when timer hwTimer reaches the timer value
//             sets the display brightness to lowest value for power saving
*/
void ARDUINO_ISR_ATTR onTimer() {
  M5.Display.setBrightness(0);
}

/*  onConnTimer callback function
//  Input:  none
//  Function - runs when timer connectionTimer reaches the timer value
//             changes Sensor connected flag to false and sensorConnectionConfirm 
//             to false to indicate the time for device connection confirm has run out.
*/
void ARDUINO_ISR_ATTR onConnTimer() {
  changeFlag(sensorConnected, false);
  pkt[1].sessionId = 0;
  sensorConnectionConfirm = false;
  
}

/*  checkSensorConnection function
//  Input:  none
//  Function - checks the sensor connection before the timer finishes
//             by sending battery packet request
*/
void checkSensorConnection(){
  Command cmd = Command::Battery;
  esp_now_send(broadcastAddress, (uint8_t *) &cmd, sizeof(cmd));
}


 
void setup() {
  // Init Serial Monitor
  Serial.begin(115200);
  // Init M5 class
  auto cfg = M5.config();
  M5.begin(cfg);
  // Get battery level
  battery = M5.Power.getBatteryLevel();

  // Init timer for display power saving
  hwTimer = timerBegin(0, 80, true);
  timerAttachInterrupt(hwTimer, &onTimer,false);
  timerAlarmWrite(hwTimer, 7000000, true);
  timerAlarmEnable(hwTimer);

  // Init timer for checking other module connection
  connectionTimer = timerBegin(1, 80, true);
  timerAttachInterrupt(connectionTimer, &onConnTimer,false);
  timerAlarmWrite(connectionTimer, 3000000, true);

  // Init display
  M5.Display.setRotation(0);
  M5.Display.setTextDatum(middle_center);
  M5.Display.setFont(&fonts::DejaVu12);
  M5.Display.setTextSize(1.3f);
  updateDisplay();

  // Set device as a Wi-Fi Station for ESP-Now
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();

  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("*************Error initializing ESP-NOW*************");
    return;
  }

  
  // Register ESP-Now callbacks after it was initialized
  esp_now_register_send_cb(OnDataSent);
  esp_now_register_recv_cb(esp_now_recv_cb_t(OnDataRecv));
  
  // Register ESP-Now peer
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;
  
  // Add ESP-Now peer        
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("*************Failed to add peer*************");
    return;
  }

  checkSensorConnection();
  
  // Initialize NIMBLE device class instance
  Serial.println("*************Start of NImble*************");
  NimBLEDevice::init("KneeOneB");
  // Set maximal transmission unit to 128 bytes
  NimBLEDevice::setMTU(128);
  NimBLEDevice::setOwnAddrType(BLE_OWN_ADDR_PUBLIC);
  // Create server and set callbacks and set advertising on disconnect to true
  NimBLEServer* pServer = NimBLEDevice::createServer();
  pServer->setCallbacks(&serverCallbacks);
  pServer->advertiseOnDisconnect(true);

  // Create a service and its characteristics
  NimBLEService* pService = pServer->createService(SERVICE_UUID);
  pCharacteristic =
      pService->createCharacteristic(CHARACTERISTIC_UUID,
                                      NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::WRITE |
                                       NIMBLE_PROPERTY::NOTIFY);
  pCharacteristic->setCallbacks(&chrCallbacks);
  pService->start();

  // Initialize and start advertising
  NimBLEAdvertising* pAdvertising = NimBLEDevice::getAdvertising();
  pAdvertising->setName("KneeOneB");
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->enableScanResponse(false);
  pAdvertising->setScanFilter(false, false);
  pAdvertising->setAdvertisingCompleteCallback(onAdvComplete);
  pAdvertising->start();
  Serial.println("Waiting a client connection to notify...");
}
 
void loop() {
  // Poll the state of the buttons
  M5.update();
  // Send message via ESP-NOW if there is new one
  if (newMsg)
  {
    esp_now_send(broadcastAddress, (uint8_t *) &bleCommand, sizeof(bleCommand));
    newMsg = false;
  } 
  // Start IMU measurement if there is new incoming packet and packet transmission is enabled
  if (startPktTransmission && newPkt)
  {
    auto imu_update = M5.Imu.update();
    uint32_t time = millis();
    if (imu_update) 
      { 
        pkt[1].ax = M5.Imu.getRawData(0);
        pkt[1].ay = M5.Imu.getRawData(1);
        pkt[1].az = M5.Imu.getRawData(2);
        pkt[1].gx = M5.Imu.getRawData(3);
        pkt[1].gy = M5.Imu.getRawData(4);
        pkt[1].gz = M5.Imu.getRawData(5);
        pkt[1].timestamp = time - startTime;
        pCharacteristic->setValue((uint8_t*)&pkt, sizeof(pkt));
        pCharacteristic->notify();
        newPkt = false;
        /*angle += pkt[0].gz * (2000.0f / 32768.0f) * pkt[0].timestamp;
        raw += pkt[0].gz;
        if (count<500) Serial.println(pkt[0].gz);
        count++;*/
      }
  }
  // Set display brightness to max for 7 seconds for reading the display
  if (M5.BtnA.wasPressed()){
    M5.Display.setBrightness(100);
    timerAlarmDisable(hwTimer);
    timerWrite(hwTimer,0);
    timerAlarmEnable(hwTimer);
    raw = 0;
    Serial.println(count);
    count = 0;
  }
  // Send battery if sendBattery command was received and battery packet was received
  // or if the other sensor is disconnected send special value to indicate the other 
  // sensor is disconnected
  if(sendBattery){
    if (sensorConnected && batteryReceived){
      batteryPkt[1] = battery;
    }else if (!sensorConnected){
      batteryPkt[0] = NOT_CONNECTED;
    }
    if ((sensorConnected && batteryReceived) || (!sensorConnected)){
      pCharacteristic->setValue((uint8_t*)&batteryPkt, sizeof(batteryPkt));
      pCharacteristic->notify();
      sendBattery = false;
    }
  }
  // Start the timer for connection check and set flags
  if(!sensorConnectionConfirm){
    timerValue += timerRead(connectionTimer);
    timerAlarmDisable(connectionTimer);
    timerWrite(connectionTimer, 0);
    timerAlarmEnable(connectionTimer);
    sensorConnectionConfirm = true;
    checkSent = false;
  }
  // Check connection before timeout
  if(((timerValue > 2500000) || (timerRead(connectionTimer)> 2500000) ) && !checkSent){
    checkSensorConnection();
    checkSent = true;
    timerValue = 0;
  };
  // Check if flag controlling the display was changed
  if (flagChanged){
    updateDisplay();
    if (flagPtr == &sensorConnected){
      Command cmd = (deviceConnected)?(Command::ConnectionSuccess):(Command::ConnectionCeased);
      esp_now_send(broadcastAddress, (uint8_t *) &cmd, sizeof(cmd));
    }
  }
  // Update battery level if the level has changed more than 5% since last update
  uint8_t newBattery = M5.Power.getBatteryLevel();
  if (abs(newBattery - battery) > 5){
    battery = newBattery;
    updateDisplay();
  }
}