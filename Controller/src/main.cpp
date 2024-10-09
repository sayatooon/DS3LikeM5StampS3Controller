#include <Arduino.h>
#include <FastLED.h>
#include <Wire.h>
#include <SparkFun_TCA9534.h>
#include <MPU9250.h>
#include <esp_now.h>
#include <WiFi.h>
#include "Joystick.hpp"
#include "PushButton.hpp"

// I2C address
#define I2CADDR_TCA9534 0x38
#define I2CADDR_MPU9250 0x68
#define I2CADDR_TIMPOWR 0x51

// M5StampS3 pin setting:
const int PIN_HOLD = 9;
const int PIN_LED = 21;               // M5StampS3 defalut LED WS2812
const int PIN_3S[] = {40, 0, 3};      // PS_SW, START_SW & G0 button, SELECT_SW
const int PIN_JOYS[] = {2, 1, 14, 4}; // Joystick RX, RY, LX, LY
const int PIN_ROW[] = {7, 12, 46, 43, 11, 42};
const int PIN_COL[] = {39, 44, 41};
const int PIN_SDA = 13, PIN_SCL = 15;
const int PIN_MT[] = {8, 10, 5, 6}; // Motor RM1, RM2, LM1, LM2

// MUX pin setting:
const int PIN_MUX_LED[] = {0, 1, 2, 3}; // LEDs
const int PIN_MUX_DRV = 4;              // DRV8833 nSLEEP

// vairables
const int NUM_LEDS = 1;            // for M5StampS3 defalut LED WS2812
const int ledcCH[] = {0, 1, 2, 3}; // channales of LEDC PWM for motors
const int ledcFreq = 5000;         // [Hz], frequenc of LEDC PWM for motors
const int ledcRes = 8;             // [bit], max 14bit at 5000Hz
const int maxDuty = 255;
const int joyRange = 10; // joystick output max range
String buttonName_3S[] = {"PS", "START", "SELECT"};
String buttonName_ROW[3][6] = {{"L2", "L1", "UP", "LEFT", "DOWN", "RIGHT"},
                               {"R1", "R2", "TRI", "CIR", "CRS", "SQR"},
                               {"R3", "L3", "", "", "", ""}};
String joyStickName[] = {"RX", "RY", "LX", "LY"};
bool mpu_use = false; // true: use MPU, false: no use MPU

unsigned long lastChangedTime;       // last time any button was pushed
unsigned long shutdownTime = 600000; // [ms], shutdown after 10min idle
int lastSendButtonState[17];         // for check

// device setting:
CRGB leds[NUM_LEDS];
TCA9534 mux;
MPU9250 mpu;

PushButton button3S[3], buttonROW[14];
Joystick joy[2]; // joy[0] for R, joy[1] for L

// function prototype:
void doIMUInit();
void doESPNowInit();
bool checkIdle();
void doShutdown();

// ESP-NOW setting
struct SendData
{ // for sending data
  int buttonState[17];
  int joystickState[4];
  float acc[3];
  float gyro[3];
  float mag[3];   
};
struct SendData sendData;

struct RecieveData
{ // for recieving data
  int motorState[2];
  int code; // 0:None, 1:Init IMU, ...
};
struct RecieveData recieveData;

esp_now_peer_info_t device;
uint8_t deviceMacAddress[] = {0xF4, 0x12, 0xFA, 0x66, 0x74, 0xC0}; // change to your device address

// callback function for sending
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status)
{
  // USBSerial.print("Last Packet Send Status: ");
  // USBSerial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}

// callback function for reciving
void OnDataRecv(const uint8_t *mac_addr, const uint8_t *data, int data_len)
{
  memcpy(&recieveData, data, sizeof(recieveData));

  for (int i = 0; i < 2; i++)
  {
    if (recieveData.motorState[i] >= 0)
    {
      if (recieveData.motorState[i] > maxDuty)
      {
        recieveData.motorState[i] = maxDuty;
      }
      ledcWrite(ledcCH[i * 2], recieveData.motorState[i]);
      ledcWrite(ledcCH[i * 2 + 1], 0);
    }
    else
    {
      if (recieveData.motorState[i] < -maxDuty)
      {
        recieveData.motorState[i] = -maxDuty;
      }
      ledcWrite(ledcCH[i * 2], 0);
      ledcWrite(ledcCH[i * 2 + 1], -recieveData.motorState[i]);
    }
  }
  if (recieveData.code == 1){
    mpu_use = true;
    doIMUInit();
    mux.digitalWrite(PIN_MUX_LED[0], LOW); // light only LED0
  }
}



void setup()
{
  USBSerial.begin(115200);
  // set pins
  pinMode(PIN_HOLD, OUTPUT);
  digitalWrite(PIN_HOLD, HIGH); // to keep turning on

  pinMode(PIN_3S[0], INPUT_PULLUP);
  for (int i = 1; i < 3; i++)
  {
    pinMode(PIN_3S[i], INPUT); // G0 and G3 are pulled up in M5StampS3
  }
  for (int i = 0; i < 6; i++)
  {
    pinMode(PIN_ROW[i], INPUT_PULLUP);
  }
  for (int i = 0; i < 3; i++)
  {
    pinMode(PIN_COL[i], OUTPUT);
    digitalWrite(PIN_COL[i], HIGH);
  }

  for (int i = 0; i < 4; i++)
  {
    pinMode(PIN_JOYS[i], INPUT_PULLUP);
    // pinMode(PIN_MT[i], OUTPUT);
    ledcSetup(ledcCH[i], ledcFreq, ledcRes);
    ledcAttachPin(PIN_MT[i], ledcCH[i]);
  }

  // init button classes
  for (int i = 0; i < 3; i++)
  {
    button3S[i].init(PIN_3S[i], buttonName_3S[i]);
  }
  for (int i = 0; i < 3; i++)
  {
    for (int j = 0; j < 6; j++)
    {
      if (i * 6 + j < 14)
      {
        buttonROW[i * 6 + j].init(PIN_ROW[j], buttonName_ROW[i][j]);
      }
    }
  }

  // init joystick classes
  for (int i = 0; i < 2; i++)
  {
    joy[i].setPins(PIN_JOYS[i * 2], PIN_JOYS[i * 2 + 1]);
    joy[i].setRange(-joyRange, joyRange, -joyRange, joyRange);
    joy[i].updateOffset();
  }

  // M5stampS3's LED setting
  // FastLED.addLeds<WS2812, PIN_LED, GRB>(leds, NUM_LEDS);

  // I2C setting
  Wire.setPins(PIN_SDA, PIN_SCL);
  Wire.begin();
  // i2cCheck();
  if (mux.begin(Wire, I2CADDR_TCA9534) == false)
  {
    USBSerial.println("failed to connect TCA9534");
    while (1)
      delay(1000);
  }

  mux.pinMode(PIN_MUX_DRV, GPIO_OUT);
  mux.digitalWrite(PIN_MUX_DRV, LOW); // Motor SLEEP MODE on

  for (int i = 0; i < 4; i++)
  {
    mux.pinMode(PIN_MUX_LED[i], GPIO_OUT);
    mux.digitalWrite(PIN_MUX_LED[i], HIGH); // LEDs off
  }

  // All LEDs blink 0.8s for the turning on signal
  for (int i = 0; i < 4; i++)
  {
    mux.digitalWrite(PIN_MUX_LED[i], LOW);
  }
  delay(800);
  for (int i = 0; i < 4; i++)
  {
    mux.digitalWrite(PIN_MUX_LED[i], HIGH);
  }

  // initialize IMU
  //doIMUInit();

  // initialize ESP Now
  doESPNowInit();

  // Motor nSLEEP off (SLEEP MODE off)
  mux.digitalWrite(PIN_MUX_DRV, HIGH);

  // idle shurdown/sleep mode setting
  gpio_wakeup_enable((gpio_num_t)PIN_3S[0], GPIO_INTR_LOW_LEVEL); // PS button is wake-up trigger of light sleep mode when using USB power
  esp_sleep_enable_gpio_wakeup();
  lastChangedTime = millis();
}

void loop()
{

  for (int i = 0; i < 3; i++)
  {
    sendData.buttonState[i] = button3S[i].getChangeState() | !button3S[i].getState() << 2;
    switch (sendData.buttonState[i])
    {
    case 0: // off && no changed
      break;
    case 2: // OFF && released
      USBSerial.print(buttonName_3S[i]);
      USBSerial.println(" is released.");
      if (i == 0 && button3S[i].getLongPressState())
      {
        USBSerial.print(buttonName_3S[i]);
        USBSerial.println(" is long pressed.");
        doShutdown();
      }
      break;
    case 4: // ON && no changed
      break;
    case 5: // ON && pushed
      USBSerial.print(buttonName_3S[i]);
      USBSerial.println(" is pushed.");
      break;
    default:
      break;
    }
  }

  for (int i = 0; i < 3; i++)
  {
    digitalWrite(PIN_COL[i], LOW);
    for (int j = 0; j < 6; j++)
    {
      if (i * 6 + j < 14)
      {
        sendData.buttonState[i * 6 + j + 3] = buttonROW[i * 6 + j].getChangeState() | !buttonROW[i * 6 + j].getState() << 2;
        switch (sendData.buttonState[i * 6 + j + 3])
        {
        case 0: // off && no changed
          break;
        case 2: // OFF && released
          USBSerial.print(buttonName_ROW[i][j]);
          USBSerial.println(" is released.");
          // leds[0] = CRGB::Black;
          // FastLED.show();
          break;
        case 4: // ON && no changed
          break;
        case 5: // ON && pushed
          USBSerial.print(buttonName_ROW[i][j]);
          USBSerial.println(" is pushed.");
          // leds[0] = CRGB::Blue;
          // FastLED.show();
          break;
        default:
          break;
        }
      }
    }
    digitalWrite(PIN_COL[i], HIGH);
  }

  for (int i = 0; i < 2; i++)
  {
    joy[i].updateValue();
    sendData.joystickState[i * 2] = joy[i].getX();
    sendData.joystickState[i * 2 + 1] = joy[i].getY();
  }

  if (joy[0].getValueChangeFlag() || joy[1].getValueChangeFlag())
  {
    USBSerial.print("joyR:");
    USBSerial.print(joy[0].getX());
    USBSerial.print(", ");
    USBSerial.print(joy[0].getY());
    USBSerial.print(", joyL:");
    USBSerial.print(joy[1].getX());
    USBSerial.print(", ");
    USBSerial.println(joy[1].getY());
  }

  if (mpu_use)
  {
    if (mpu.update())
    {
      // USBSerial.print(mpu.getYaw()); USBSerial.print(", ");
      // USBSerial.print(mpu.getPitch()); USBSerial.print(", ");
      // USBSerial.println(mpu.getRoll());
      for (int i = 0; i < 3; i++)
      {
        sendData.acc[i] = mpu.getAcc(i);
        sendData.gyro[i] = mpu.getGyro(i);
        sendData.mag[i] = mpu.getMag(i);
      }
      /* USBSerial.print("ACC:"); USBSerial.print(mpu.getAccX()); USBSerial.print(", "); USBSerial.print(mpu.getAccY()); USBSerial.print(", "); USBSerial.println(mpu.getAccZ());
      USBSerial.print("GYRO:"); USBSerial.print(mpu.getGyroX()); USBSerial.print(", "); USBSerial.print(mpu.getGyroY()); USBSerial.print(", "); USBSerial.println(mpu.getGyroZ());
      USBSerial.print("Mag:"); USBSerial.print(mpu.getMagX()); USBSerial.print(", "); USBSerial.print(mpu.getMagY()); USBSerial.print(", "); USBSerial.println(mpu.getMagZ()); */
    }
  }

  if (checkIdle())
  {
    doShutdown();
  }

  esp_now_send(device.peer_addr, (uint8_t *)&sendData, sizeof(sendData));

  delay(10);
}

void doIMUInit()
{
  // MPU setting
  MPU9250Setting mpu_setting;
  mpu_setting.accel_fs_sel = ACCEL_FS_SEL::A16G;
  mpu_setting.gyro_fs_sel = GYRO_FS_SEL::G2000DPS;
  mpu_setting.mag_output_bits = MAG_OUTPUT_BITS::M16BITS;
  mpu_setting.fifo_sample_rate = FIFO_SAMPLE_RATE::SMPL_200HZ;
  mpu_setting.gyro_fchoice = 0x03;
  mpu_setting.gyro_dlpf_cfg = GYRO_DLPF_CFG::DLPF_41HZ;
  mpu_setting.accel_fchoice = 0x01;
  mpu_setting.accel_dlpf_cfg = ACCEL_DLPF_CFG::DLPF_45HZ;
  mpu.setMagneticDeclination(-12.30); // check the magnetic declination in your city https://www.magnetic-declination.com/

  if (mpu_use)
  {
    if (mpu.setup(0x68, mpu_setting) == false)
    {
      USBSerial.println("failed to connect MPU9250 or MPU9250 isn't mounted.");
      mpu_use = false;
    }
    else
    {
      // MPU calibration
      USBSerial.println("Start Accle & Gyro Calibration! Please leave the device still on the flat plane.");
      mux.digitalWrite(PIN_MUX_LED[0], LOW); // light LED 0 and LED 2 during Acc & Gyro calibration.
      mux.digitalWrite(PIN_MUX_LED[2], LOW);
      mpu.calibrateAccelGyro();
      USBSerial.println("Start Mag Calibration! Please Wave device in a figure eight until done.");
      mux.digitalWrite(PIN_MUX_LED[0], HIGH);
      mux.digitalWrite(PIN_MUX_LED[2], HIGH);
      mux.digitalWrite(PIN_MUX_LED[1], LOW); // light LED 1 and LED 3 during Mag calibration.
      mux.digitalWrite(PIN_MUX_LED[3], LOW);
      mpu.calibrateMag();
      USBSerial.println("Calibration Done!!");
      mux.digitalWrite(PIN_MUX_LED[1], HIGH);
      mux.digitalWrite(PIN_MUX_LED[3], HIGH);

      USBSerial.print("ACC_BIAS:");
      USBSerial.print(mpu.getAccBiasX());
      USBSerial.print(", ");
      USBSerial.print(mpu.getAccBiasY());
      USBSerial.print(", ");
      USBSerial.println(mpu.getAccBiasZ());
      USBSerial.print("GYRO_BIAS:");
      USBSerial.print(mpu.getGyroBiasX());
      USBSerial.print(", ");
      USBSerial.print(mpu.getGyroBiasY());
      USBSerial.print(", ");
      USBSerial.println(mpu.getGyroBiasZ());
      USBSerial.print("Mag_BIAS:");
      USBSerial.print(mpu.getMagBiasX());
      USBSerial.print(", ");
      USBSerial.print(mpu.getMagBiasY());
      USBSerial.print(", ");
      USBSerial.println(mpu.getMagBiasZ());
      USBSerial.print("Mag_SCALE:");
      USBSerial.print(mpu.getMagScaleX());
      USBSerial.print(", ");
      USBSerial.print(mpu.getMagScaleY());
      USBSerial.print(", ");
      USBSerial.println(mpu.getMagScaleZ());
    }
  }

  for (int i = 0; i < 4; i++)
  { // turn off all LEDs
    mux.digitalWrite(PIN_MUX_LED[i], HIGH);
  }
}

void doESPNowInit()
{
  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA); // Starts the wifi
  WiFi.disconnect();
  if (esp_now_init() == ESP_OK)
  {
    USBSerial.println("ESPNow Init Success");
  }
  else
  {
    USBSerial.println("ESPNow Init Failed");
    ESP.restart();
  }

  // Register peer
  memset(&device, 0, sizeof(device));
  memcpy(device.peer_addr, deviceMacAddress, 6);
  device.channel = 0; // The range of the channel of paired devices is from 0 to 14. If the channel is set to 0, data will be sent on the current channel.

  if (esp_now_add_peer(&device) == ESP_OK)
  { // Pair success
    USBSerial.println("Pair success");
    mux.digitalWrite(PIN_MUX_LED[0], LOW); // light only LED0
  }
  else
  {
    USBSerial.println("Failed to add peer");
  }
  // Register callback function
  esp_now_register_send_cb(OnDataSent);
  esp_now_register_recv_cb(OnDataRecv);
}

bool checkIdle()
{

  if (memcmp(lastSendButtonState, sendData.buttonState, sizeof(lastSendButtonState)) == 0 && !joy[0].getValueChangeFlag() && !joy[1].getValueChangeFlag())
  {
    if (millis() - lastChangedTime > shutdownTime)
    {
      return true;
    }
  }
  else
  {
    lastChangedTime = millis();
  }

  memcpy(lastSendButtonState, sendData.buttonState, sizeof(lastSendButtonState));
  return false;
}

void doShutdown()
{
  USBSerial.println("Shutdown or go to sleep mode");
  for (int i = 0; i < 4; i++)
  { // stop motors
    ledcWrite(ledcCH[i], 0);
  }
  mux.digitalWrite(PIN_MUX_DRV, LOW); // Motor SLEEP MODE on

  for (int i = 0; i < 4; i++)
  { // turn off all LEDs
    mux.digitalWrite(PIN_MUX_LED[i], HIGH);
  }
  digitalWrite(PIN_HOLD, LOW); // turn off power when using battery
  // only when using USB power
  // esp_sleep_enable_gpio_wakeup();
  esp_light_sleep_start();
  ESP.restart(); // start here after wakeup
}