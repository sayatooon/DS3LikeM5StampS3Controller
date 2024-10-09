#include <Arduino.h>
#include <esp_now.h>
#include <WiFi.h>
#include <PacketSerial.h>

// UART setting
PacketSerial myPacketSerial;

// ESP-NOW & UART data
union SendData // 8 byte (10 byte when encoded using COBS )
{
  struct // for ESP-NOW sending data
  {
    int motorState[2];
    int code; // 0:None, 1:init IMU, ...
  };
  uint8_t bin[sizeof(uint) * 3]; // for UART sending data
};
union SendData sendData;

union RecieveData // 120 byte (122 byte when encoded using COBS )
{
  struct // for ESP-NOW recieving data
  {
    int buttonState[17];
    int joystickState[4];
    float acc[3];
    float gyro[3];
    float mag[3];
  };
  uint8_t bin[sizeof(int) * 21 + sizeof(float) * 9]; // for UART sending data
};
union RecieveData recieveData;

// ESP-NOW setting
esp_now_peer_info_t controller;
uint8_t controllerMacAddress[] = {0x48, 0x27, 0xE2, 0xE3, 0xC7, 0x30}; // change to your controller MAC address

// callback function for sending over ESP-NOW
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status)
{
  // USBSerial.print("Last Packet Send Status: ");
  // USBSerial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}
// callback function for reciving over ESP-NOW
void OnDataRecv(const uint8_t *mac_addr, const uint8_t *data, int data_len)
{
  memcpy(&recieveData, data, sizeof(recieveData));
  myPacketSerial.send(recieveData.bin, sizeof(recieveData));

  /* for sending ASCII over UART
  USBSerial.print("Button: ");
  for(int i = 0; i< 17; i++){
    USBSerial.print(recieveData.buttonState[i]);
    if(i == 16){
        USBSerial.println("");
    }else{
        USBSerial.print(", ");
    }
  }
  USBSerial.print("Joystick: ");
  for(int i = 0; i< 4; i++){
    USBSerial.print(recieveData.joystickState[i]);
    if(i == 3){
        USBSerial.println("");
    }else{
        USBSerial.print(", ");
    }
  }
  USBSerial.print("Acc: ");
  for(int i = 0; i< 3; i++){
    USBSerial.print(recieveData.acc[i]);
    if(i == 2){
        USBSerial.println("");
    }else{
        USBSerial.print(", ");
    }
  }
  USBSerial.print("Gyro: ");
  for(int i = 0; i< 3; i++){
    USBSerial.print(recieveData.gyro[i]);
    if(i == 2){
        USBSerial.println("");
    }else{
        USBSerial.print(", ");
    }
  }
  USBSerial.print("Mag: ");
  for(int i = 0; i< 3; i++){
    USBSerial.print(recieveData.mag[i]);
    if(i == 2){
        USBSerial.println("");
    }else{
        USBSerial.print(", ");
    }
  }

  */
}

// callback function for reciving over UART
void onPacketReceived(const uint8_t *buffer, size_t size)
{
  memcpy(&sendData, buffer, sizeof(sendData));
  esp_now_send(controller.peer_addr, (uint8_t *)&sendData, sizeof(sendData));
}

void setup()
{
  USBSerial.begin(115200);
  myPacketSerial.setStream(&USBSerial);
  myPacketSerial.setPacketHandler(&onPacketReceived);

  // init ESP-NOW
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  if (esp_now_init() == ESP_OK)
  {
    //USBSerial.println("ESPNow Init Success");
    
  }
  else
  {
    //USBSerial.println("ESPNow Init Failed");
    ESP.restart();
  }
  // Register peer
  memset(&controller, 0, sizeof(controller));
  memcpy(controller.peer_addr, controllerMacAddress, 6);
  controller.channel = 0; // The range of the channel of paired controllers is from 0 to 14. If the channel is set to 0, data will be sent on the current channel.

  if (esp_now_add_peer(&controller) == ESP_OK)
  { // Pair success
    USBSerial.println("Pair success");
  }
  else
  {
    USBSerial.println("Failed to add peer");
  }
  // Register callback function
  esp_now_register_send_cb(OnDataSent);
  esp_now_register_recv_cb(OnDataRecv);
}
void loop()
{
  myPacketSerial.update();

  /* for sending ASCII over UART
  String strMotor, tmpData;
  if (USBSerial.available() > 0)
  {
    strMotor = USBSerial.readStringUntil('\n');
    tmpData = strMotor.substring(0, strMotor.indexOf(','));
    // tmpData = tmpData.trim();
    sendData.motorState[0] = tmpData.toInt();
    tmpData = strMotor.substring(strMotor.indexOf(',') + 1);
    // tmpData = tmpData.trim();
    sendData.motorState[1] = tmpData.toInt();

    // USBSerial.print("Send Data: ");
    // USBSerial.print(sendData.motorState[0]);
    // USBSerial.print(", ");
    // USBSerial.println(sendData.motorState[1]);

    esp_now_send(controller.peer_addr, (uint8_t *)&sendData, sizeof(sendData));
  }
  */

  delay(10);
}
