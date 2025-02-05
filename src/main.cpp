#include <Arduino.h>
#include <Wire.h>
#include <WiFi.h>
#include <nvs.h>
#include <nvs_flash.h>
#include <esp_wifi.h>
#include "PAC1710.hpp"

// PAC1710 Config
#define DEVID_R430 PAC1710::ADDR::R430 // Resistor OPEN (N.C.) at ADDR_SEL pin
#define DEVID_OPEN PAC1710::ADDR::OPEN // Resistor 0 Ohm at ADDR_SEL pin
const int VSHUNT_mOHM = 10;
int c1cnf[] = {0B01000110}; // Current-Config, Sample time: 20ms, Range: ±40mV
int c2cnf[] = {0B00001101}; // Voltage-Config, sample time: 20ms

// アクセスポイントのSSIDとパスワード
const char *ssid = "ESP32-Access-Point";
const char *password = "12345678";

// サーバーオブジェクトを作成
WiFiServer Server(12345);

// MotorDriver Config
const int M1A = 12;
const int M1B = 13;
const int M2A = 14;
const int M2B = 15;
const int ChannelA_1 = 0;
const int ChannelB_1 = 1;
const int ChannelA_2 = 2;
const int ChannelB_2 = 3;
const int PWMFreq = 5000;
const int PWMResolution = 8;

/** I2C sending data to device
 */
void datasend(int id, int reg, int *data, int datasize)
{
  Wire.beginTransmission(id);
  Wire.write(reg);
  Serial.println(datasize);
  for (int i = 0; i < datasize; i++)
  {
    Wire.write(data[i]);
  }
  Wire.endTransmission();
}

/** I2C reading data from device
 */
void dataread(int id, int reg, uint16_t *data, int datasize)
{
  Wire.beginTransmission(id);
  Wire.write(reg);
  Wire.endTransmission(false);
  Wire.requestFrom(id, datasize);
  Serial.println(Wire.available());
  int i = 0;
  while ((i < datasize) && Wire.available())
  {
    data[i] = Wire.read();
    i++;
  }

  // return Wire.endTransmission(true);
}

void setup()
{
  Serial.begin(115200);

  WiFi.softAP(ssid, password);
  IPAddress IP = WiFi.softAPIP();

  Server.begin();

  Serial.print("IP Address: ");
  Serial.println(IP);

  Wire.begin();

  ledcSetup(ChannelA_1, PWMFreq, PWMResolution);
  ledcSetup(ChannelB_1, PWMFreq, PWMResolution);
  ledcSetup(ChannelA_2, PWMFreq, PWMResolution);
  ledcSetup(ChannelB_2, PWMFreq, PWMResolution);
  ledcAttachPin(M1A, ChannelA_1);
  ledcAttachPin(M1B, ChannelB_1);
  ledcAttachPin(M2A, ChannelA_2);
  ledcAttachPin(M2B, ChannelB_2);

  // Serial.print("Reading PID and MID... ");
  // Serial.println(getID(), HEX);                           // expected "585D" in PAC1710
  datasend(DEVID_R430, PAC1710::REG::C1_VSAMP_CFG, c1cnf, 1);
  datasend(DEVID_OPEN, PAC1710::REG::C1_VSAMP_CFG, c1cnf, 1); // send config
  datasend(DEVID_R430, PAC1710::REG::VSO_CFG, c2cnf, 1);
  datasend(DEVID_OPEN, PAC1710::REG::VSO_CFG, c2cnf, 1);

  // Initialize PAC1710 here...
  // Default sample rate 80ms => Denominator: 2047
  // Default sample range => +-80 mV
}

void loop()
{
  WiFiClient client = Server.available();

  ledcWrite(ChannelA_1, 50);
  ledcWrite(ChannelB_1, 0);
  ledcWrite(ChannelA_2, 50);
  ledcWrite(ChannelB_2, 0);

  if (client)
  {
    Serial.println("Client connected");

    while (client.connected())
    {
      // WiFiClient client = Server.available(); // クライアントからの接続を待機
      uint16_t ch1Vsense[2] = {0};
      uint16_t ch1Vsource[2] = {0};
      uint16_t ch2Vsense[2] = {0};
      uint16_t ch2Vsource[2] = {0};

      dataread(DEVID_OPEN, PAC1710::REG::C1_SVRES_H, ch1Vsense, 2); // CHANNEL 1 VSENSE RESULT REGISTER
      dataread(DEVID_OPEN, PAC1710::REG::C1_VVRES_H, ch1Vsource, 2);
      dataread(DEVID_R430, PAC1710::REG::C1_SVRES_H, ch2Vsense, 2);
      dataread(DEVID_R430, PAC1710::REG::C1_VVRES_H, ch2Vsource, 2);

      float Ibus_OPEN_mA = (((int16_t(ch1Vsense[0] << 8 | (ch1Vsense[1])) >> 4) * 3.9100684262));
      float Ibus_R430_mA = (((int16_t(ch2Vsense[0] << 8 | (ch2Vsense[1])) >> 4) * 3.9100684262));
      //  7.8278 is magic value in default denominator 511, Rsenes 10mOhm, Measure range += 40mV
      //  see "4.4 Current Measurement"
      //  FSC = 4A

      float measuredVsource_OPEN = (int16_t((ch1Vsource[0] << 3) | (ch1Vsource[1] >> 5)) * 0.0390625);
      float measuredVsource_R430 = (int16_t((ch2Vsource[0] << 3) | (ch2Vsource[1] >> 5)) * 0.0390625);
      // 19.531 is magic value in default denominator 2047
      // see "4.5 Voltage Measurement"

      String data = String(Ibus_OPEN_mA) + "," + String(Ibus_R430_mA) + "," + String(measuredVsource_OPEN) + "," + String(measuredVsource_R430);
      client.println(data);

      Serial.print("Raw ch1Vsense: ");
      Serial.print(ch2Vsense[0], BIN);
      Serial.print(" ");
      Serial.println(ch2Vsense[1], BIN);

      Serial.print("\tIbus_OPEN: ");
      Serial.print(Ibus_OPEN_mA);
      Serial.print(" mA,");

      Serial.print("\tIbus_R430: ");
      Serial.print(Ibus_R430_mA);
      Serial.println(" mA, ");

      Serial.print("All_Current: ");
      Serial.print(Ibus_OPEN_mA + Ibus_R430_mA);
      Serial.println(" mA");

      Serial.print("Vsource_OPEN: ");
      Serial.print(measuredVsource_OPEN);
      Serial.print(" V");

      Serial.print("\tVsource_R430: ");
      Serial.print(measuredVsource_R430);
      Serial.println(" V");

      delay(1000);
    }
    client.stop();
    Serial.println("Client disconnected");
  }
}
