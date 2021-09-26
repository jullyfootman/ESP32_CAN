
// WifiでPCへ受信値を送信
// Pythonアプリで表示　コマンド送信
// MAC値生成まで

#include <Arduino.h>

#include <ESP32CAN.h>
#include <CAN_config.h>

#include <Wire.h>
#include <LiquidCrystal.h> //#include <Wire.h>が必要
const int RS_PIN = 23;
//const int RW_PIN = GND;
const int E_PIN = 22;
const int DB4_PIN = 21;
const int DB5_PIN = 17;
const int DB6_PIN = 19;
const int DB7_PIN = 18;
LiquidCrystal lcd = LiquidCrystal(RS_PIN, E_PIN, DB4_PIN, DB5_PIN, DB6_PIN, DB7_PIN);

CAN_device_t CAN_cfg;
CAN_frame_t tx_frame; //送信用フレーム

// 受信したデータをPCへシリアル送信する
unsigned long pre_time, now_time;
void SerialOut(CAN_frame_t rx_frame)
{
  if (rx_frame.FIR.B.RTR == CAN_RTR)
  {
    Serial.println("RTR");
  }
  else
  {
    Serial.print("MsgID : ");
    Serial.print(rx_frame.MsgID, HEX);
    Serial.print(' ');
    Serial.print("DLC : ");
    Serial.print('[');
    Serial.print(rx_frame.FIR.B.DLC, HEX);
    Serial.print(']');
    Serial.print(' ');

    lcd.clear();
    lcd.print("ID: ");
    lcd.printf("%02X", rx_frame.MsgID);
    lcd.print(" DLC: ");
    lcd.print(rx_frame.FIR.B.DLC, HEX);
    lcd.setCursor(0, 2);

    for (int i = 0; i < 8; i++)
    {
      Serial.print(int(rx_frame.data.u8[i]), HEX);
      Serial.print(' ');
      lcd.print(int(rx_frame.data.u8[i]), HEX);
    }
    // Serial.println();
  }
  now_time = millis();
  Serial.print(now_time - pre_time);
  Serial.print(" ");
  Serial.println("ms");
  pre_time = now_time;
}

// CANへデータ書き込み

void sendData(uint16_t ID, uint8_t data[])
{
  if (ID <= 0x7ff)
  {
    CAN_frame_t tx_frame;
    tx_frame.MsgID = ID;
    tx_frame.FIR.B.FF = CAN_frame_std;
    tx_frame.FIR.B.DLC = 8;
    for (int i = 0; i < 8; i++)
    {
      tx_frame.data.u8[i] = data[i];
    }
    ESP32Can.CANWriteFrame(&tx_frame);
  }
}

void setup()
{
  Serial.begin(9600);
  while (!Serial)
  {
  }
  lcd.begin(16, 2);
  CAN_cfg.speed = CAN_SPEED_500KBPS;
  CAN_cfg.tx_pin_id = GPIO_NUM_27;
  CAN_cfg.rx_pin_id = GPIO_NUM_26;
  CAN_cfg.rx_queue = xQueueCreate(10, sizeof(CAN_frame_t));
  ESP32Can.CANInit();
}

int volatile flag_ISR = 0; //<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<ADD
void loop()
{
  CAN_frame_t rx_frame, tx_frame;

  if (flag_ISR >= 1)
  { 
    flag_ISR = 0;
    if (xQueueReceive(CAN_cfg.rx_queue, &rx_frame, 3 * portTICK_PERIOD_MS) == pdTRUE)
    {
      SerialOut(rx_frame);
    }
  }

  // if (xQueueReceive(CAN_cfg.rx_queue, &rx_frame, 3 * portTICK_PERIOD_MS) == pdTRUE)
  // {
  //   // SerialOut(rx_frame);

  //   //++++++++++++++++++++++++++++++++++
  //   // float temp ;
  //   // temp = rx_frame.data.u8[2] * 0.2;
  //   // Serial.print(temp);
  //   // lcd.print(temp);
  //   //++++++++++++++++++++++++++++++++++
  // }
  // else
  // {
    // tx_frame.FIR.B.FF = CAN_frame_std;
    // tx_frame.MsgID = 0xff;
    // tx_frame.FIR.B.DLC = 8;
    // tx_frame.data.u8[0] = 0x10;
    // tx_frame.data.u8[1] = 0x20;
    // tx_frame.data.u8[2] = 0x30;
    // tx_frame.data.u8[3] = 0x40;
    // tx_frame.data.u8[4] = 0x50;
    // tx_frame.data.u8[5] = 0x60;
    // tx_frame.data.u8[6] = 0x70;
    // tx_frame.data.u8[7] = 0x80;
    // ESP32Can.CANWriteFrame(&tx_frame);
  // }

  // delay(100);
}