#include <Arduino.h>

#include <ESP32CAN.h>
#include <CAN_config.h>

#include <Wire.h>
#include <LiquidCrystal.h> //#include <Wire.h>が必要
#define RS_PIN 23
//define RW_PIN  GND
#define E_PIN 22
#define DB4_PIN 21
#define DB5_PIN 17
#define DB6_PIN 19
#define DB7_PIN 18
LiquidCrystal lcd = LiquidCrystal(RS_PIN, E_PIN, DB4_PIN, DB5_PIN, DB6_PIN, DB7_PIN);

CAN_device_t CAN_cfg;

hw_timer_t *timer1 = NULL;
volatile int interruptCounter1;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;

void IRAM_ATTR INTRUPT_1ms()
{
  portENTER_CRITICAL_ISR(&timerMux);
  interruptCounter1++;
  portEXIT_CRITICAL_ISR(&timerMux);
}

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
  }
  now_time = millis();
  Serial.print(now_time - pre_time);
  Serial.print(" ");
  Serial.println("ms");
  pre_time = now_time;
}

// CANへデータ書き込み
void send_stdCanFrame(uint16_t ID, uint8_t *data)
{
  const uint8_t DLC_NUM = 8;
  if (ID <= 0x7ff)
  {
    CAN_frame_t tx_frame;
    tx_frame.MsgID = ID;
    tx_frame.FIR.B.FF = CAN_frame_std;
    tx_frame.FIR.B.DLC = DLC_NUM;
    for (int i = 0; i < DLC_NUM; i++)
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
  // 割り込み
  timer1 = timerBegin(1, 80, true); //timer=1us
  timerAttachInterrupt(timer1, &INTRUPT_1ms, true);
  timerAlarmWrite(timer1, 100000, true); //100ms
  timerAlarmEnable(timer1);

  //LCD init
  lcd.begin(16, 2);
  
  //CAN init
  CAN_cfg.speed = CAN_SPEED_500KBPS;
  CAN_cfg.tx_pin_id = GPIO_NUM_27;
  CAN_cfg.rx_pin_id = GPIO_NUM_26;
  CAN_cfg.rx_queue = xQueueCreate(10, sizeof(CAN_frame_t));
  ESP32Can.CANInit();
}

int volatile flag_ISR = 0; //<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<ADD
void loop()
{
  //recieve
  CAN_frame_t rx_frame;
  if (flag_ISR >= 1)
  {
    if (xQueueReceive(CAN_cfg.rx_queue, &rx_frame, 3 * portTICK_PERIOD_MS) == pdTRUE)
    {
      SerialOut(rx_frame);
    }
    flag_ISR = 0;
    Serial.println("RECIEVE");
  }

  //send
  if (interruptCounter1 > 0)
  {
    uint8_t data[8];
    portENTER_CRITICAL(&timerMux);
    interruptCounter1 = 0;
    portEXIT_CRITICAL(&timerMux);
    data[0] = 0x10;
    data[1] = 0x20;
    data[2] = 0x30;
    data[3] = 0x40;
    data[4] = 0x50;
    data[5] = 0x60;
    data[6] = 0x70;
    data[7] = 0x80;
    uint8_t send_ID = 0xff;
    send_stdCanFrame(send_ID, data);
    Serial.println("SEND");
  }
}