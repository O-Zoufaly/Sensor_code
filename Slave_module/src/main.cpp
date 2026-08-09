#include <Arduino.h>
#include <M5GFX.h>
#include <M5Unified.h>
#include <esp_now.h>
#include <WiFi.h>

// Address of the other sensor module
uint8_t broadcastAddress[] = {0x00, 0x4B, 0x12, 0xC4, 0x87, 0xA0};

// Simple variables
bool start = false;
bool sensorConnected = false;
bool sensorConnectionConfirm = false;
bool pcConnected = false;
bool flagChanged = false;
uint32_t startTime = 0;
uint8_t battery = 0;
bool sendBattery = false;
uint32_t lastSent = 0;
uint32_t presentTime = 0;

// Timers for checkeing connection and power saving mode
hw_timer_t *hwTimer = NULL;
hw_timer_t *connectionTimer = NULL;

//Enum class for communication
enum class Command : uint8_t
{
    Stop = 0,
    Start = 1,
    Battery = 2,
    ConnectionSuccess = 3,
    ConnectionCeased = 4
};

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

// IMU packet definition
ImuPacket pkt;

// ESP-Now variable for storing info about the other sensor module
esp_now_peer_info_t peerInfo;

/* Flag changed function
//  Input: flag - reference to bool flag to be changed
//         value - value to set
//  Function -  if the value is different from current value 
//            of the flag, the flag is changed and flagChanged
//            variable gets set, otherwise nothing happens
*/
void changeFlag(bool& flag, bool value){
  if (flag == value) return;
  else{

    flag = value;
    flagChanged = true;
  }
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
  if (pcConnected){
    M5.Display.setTextColor(DARKGREEN);
    M5.Display.drawString("Connected",M5.Display.width() / 2,M5.Display.height() - 150);
  } else {
    M5.Display.setTextColor(ORANGE);
    M5.Display.drawString("Waiting",M5.Display.width() / 2,M5.Display.height() - 150);
  }
  M5.Display.setTextSize(1.8f);
  M5.Display.setTextColor(MAGENTA);
  M5.Display.drawString("THIGH",M5.Display.width() / 2,M5.Display.height() - 210);
  flagChanged = false;
}

/* OnDataRecv callback function
//  Input: mac - address of the sender
//         incomingData - pointer to the first byte of incoming data
//         len - lenght of the incoming data in bytes
//  Function - receives the incomming ESP-Now data and changes the 
//             sensorConnected flag to true, sets the sensorConnectionConfirm 
//             flag to true and reads the command and based 
//             on its value it sets variables and flags
*/
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  Command cmd;
  changeFlag(sensorConnected, true);
  sensorConnectionConfirm = false;
  memcpy(&cmd, incomingData, sizeof(cmd));
  switch (cmd)
  {
    case Command::Start:
        start = true;
        startTime = millis();
        pkt.sessionId++;
        break;

    case Command::Stop:
        start = false;
        break;

    case Command::Battery:
        sendBattery = true;
        break;

    case Command::ConnectionSuccess:
        changeFlag(pcConnected,true);
        break;

    case Command::ConnectionCeased:
        changeFlag(pcConnected,false);
        break;
  }
}

/* OnDataSent callback function
//  Input: mac - address of the sender
//         status - status of sending ESP-Now data
//  Function - no function
*/
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
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
  changeFlag(pcConnected, false);
  sensorConnectionConfirm = false;
  pkt.sessionId = 0;
}

void setup() {
  // Initialize Serial Monitor
  Serial.begin(115200);
  // Init M5 class
  auto cfg = M5.config();
  M5.begin(cfg);
  
  // Get battery level
  battery = M5.Power.getBatteryLevel();

  // Init timer for display power saving
  hwTimer = timerBegin(0, 80, true);;
  timerAttachInterrupt(hwTimer, &onTimer,false);
  timerAlarmWrite(hwTimer, 7000000, true);
  timerAlarmEnable(hwTimer);

  // Init timer for checking other module connection
  connectionTimer = timerBegin(1, 80, true);
  timerAttachInterrupt(connectionTimer, &onConnTimer,false);
  timerAlarmWrite(connectionTimer, 3200000, true);

  // Init display
  M5.Display.setRotation(0);
  M5.Display.setTextDatum(middle_center);
  M5.Display.setFont(&fonts::DejaVu12);
  M5.Display.setTextSize(1.3f);

  // Set device as a Wi-Fi Station for ESP-Now
  updateDisplay();
  WiFi.mode(WIFI_STA);

  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
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
    Serial.println("Failed to add peer");
    return;
  }
}
 
void loop() {
  M5.update();
  // Start IMU measurement if the packet transmission is enabled
  presentTime = millis();
  if (start & ((presentTime - lastSent)>12)) //Limits the sampling rate to approx. 80 Hz
  {
    auto imu_update = M5.Imu.update();
    if (imu_update) 
      {
        pkt.ax = M5.Imu.getRawData(0);
        pkt.ay = M5.Imu.getRawData(1);
        pkt.az = M5.Imu.getRawData(2);
        pkt.gx = M5.Imu.getRawData(3);
        pkt.gy = M5.Imu.getRawData(4);
        pkt.gz = M5.Imu.getRawData(5);
        pkt.timestamp = presentTime - startTime;
        lastSent = presentTime;
        esp_now_send(broadcastAddress, (uint8_t *) &pkt, sizeof(pkt));
      }
  }
  // Set display brightness to max for 7 seconds for reading the display
  if (M5.BtnA.wasPressed()){
    M5.Display.setBrightness(100);
    delay(100);
    timerAlarmDisable(hwTimer);
    timerWrite(hwTimer,0);
    timerAlarmEnable(hwTimer);
  }
  // Start the timer for connection check and set flags
  if(!sensorConnectionConfirm){
    timerAlarmDisable(connectionTimer);
    timerWrite(connectionTimer, 0);
    timerAlarmEnable(connectionTimer);
    sensorConnectionConfirm = true;
  }
  // Send battery if sendBattery command was received
  if (sendBattery){
    esp_now_send(broadcastAddress, (uint8_t *) &battery, sizeof(battery));
    sendBattery = false;
  }
  // Check if flag controlling the display was changed
  if (flagChanged){
    updateDisplay();
  }
  // Update battery level if the level has changed more than 5% since last update
  uint8_t newBattery = M5.Power.getBatteryLevel();
  if (abs(newBattery - battery) > 5){
    battery = newBattery;
    updateDisplay();
  }
}