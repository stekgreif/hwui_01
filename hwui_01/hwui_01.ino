/******************************************************************************/
/** @file   hwui_01
    @date   2016-10-13
    @author dan@stekgreif.com
    @brief  mosaik mvier 10 hwui
            - 1 encoder
            - 10 buttons
            - 2 arcade buttons
            - 1 distance sensor

  get ID via Pitchbend Midi Msg, Value = ID
*******************************************************************************/
#include "MIDIUSB.h"


uint32_t cur_tick = 0;
uint32_t prev_tick = 0;
uint8_t  sched_cnt  = 0;
uint8_t  usb_midi_msg_cnt = 0;


// actual button pin mapping
#define ARCADE_TOP 4
#define ARCADE_BOT 5
#define BTN_01  A4
#define BTN_02  A3
#define BTN_03  A2
#define BTN_04  A1
#define BTN_05  11
#define BTN_06  12
#define BTN_07  A0
#define BTN_08  8
#define BTN_09  9
#define BTN_10  10

#define BTN_DEAD_TIME 100

#define _ID 0x02




typedef struct {
  uint8_t   hw_pin;
  uint16_t  debounce_ticks;
  uint8_t   cur_state;
  uint8_t   prev_state;
} btn_t;

btn_t ba[12] = {};


uint16_t distance_val = 0;
uint8_t serial_flag = 0;

String inputString = "";         // a string to hold incoming data
boolean stringComplete = false;  // whether the string is complete

void setup()
{
  Serial.begin(115200);

  // reserve 200 bytes for the inputString:
  inputString.reserve(8);

  pinMode(13, OUTPUT);

  ba[0].hw_pin  = BTN_01;
  ba[1].hw_pin  = BTN_02;
  ba[2].hw_pin  = BTN_03;

  ba[3].hw_pin  = BTN_04;
  ba[4].hw_pin  = BTN_05;
  ba[5].hw_pin  = BTN_06;

  ba[6].hw_pin  = BTN_07;
  ba[7].hw_pin  = BTN_08;
  ba[8].hw_pin  = BTN_09;
  ba[9].hw_pin  = BTN_10;

  ba[10].hw_pin = ARCADE_TOP;
  ba[11].hw_pin = ARCADE_BOT;



  for ( uint8_t cnt = 0; cnt < 12; cnt++ )
  {
    pinMode(ba[cnt].hw_pin, INPUT_PULLUP);
  }
}



void buttons_read(void)
{
  for ( uint8_t id = 0; id < 12; id++ )
  {
    ba[id].cur_state = digitalRead(ba[id].hw_pin);

    if ( ba[id].cur_state == 0)
    {
      if ( ba[id].debounce_ticks == 0 )
      {
        midiEventPacket_t event = {0x09, 0x90, id, 1};
        MidiUSB.sendMIDI(event);
        usb_midi_msg_cnt++;

        ba[id].debounce_ticks = BTN_DEAD_TIME;
      }
    }
  }
}



void buttons_detect_changes(void)
{

}


void buttons_debounce(void)
{
  for ( uint8_t id = 0; id < 12; id++ )
  {
    if ( ba[id].debounce_ticks > 0 )
    {
      ba[id].debounce_ticks--;
    }
  }
}


uint16_t b_cnt = 0;
uint8_t tgl = 0;
void led_blink(void)
{
  b_cnt++;
  if (b_cnt > 75)
  {
    if ( tgl )
    {
      digitalWrite(13, HIGH);
      tgl = 0;
    }
    else
    {
      digitalWrite(13, LOW);
      tgl = 1;
    }
    b_cnt = 0;
  }
}


uint8_t d_cnt = 0;

void read_distance(void)
{
  d_cnt++;

  if ( d_cnt > 100 )
  {
    distance_val = analogRead(A5);
    d_cnt = 0;

    midiEventPacket_t event = {0x0B, 0xB0, 5, (uint16_t) (distance_val >> 3)};
    MidiUSB.sendMIDI(event);
    usb_midi_msg_cnt++;
  }
}



void usb_midi_read()
{
  midiEventPacket_t rx;


  rx = MidiUSB.read();

  if (rx.header == 0xE) // MIDI SYSTEM
  {
    if ( rx.byte2 == _ID)
    {
      midiEventPacket_t event = {0x0E, 0xE0, 0, _ID};
      MidiUSB.sendMIDI(event);
      usb_midi_msg_cnt++;
    }
  }
}



void loop()
{
  cur_tick = millis();

  //Atmega32U4
  if (Serial.available()) serialEvent();

  if ( cur_tick != prev_tick )
  {
    sched_cnt++;

    switch ( sched_cnt )
    {
      case 1:
        {
          buttons_read();
          break;
        }
      case 2:
        {
          buttons_debounce();
          break;
        }
      case 3:
        {
          //read_distance();
          led_blink();
          break;
        }
      case 4:
        {
          usb_midi_read();
          //Serial.println("alive");
          break;
        }
      case 5:
        {
          //Serial.println("i");
          if ( usb_midi_msg_cnt > 0 )
          {
            MidiUSB.flush();
          }
          if ( stringComplete )
          {
            Serial.println(_ID);
            // clear the string:
            inputString = "";
            stringComplete = false;
          }
          sched_cnt = 0;
        }
    }
  }
  prev_tick = cur_tick;
}



void serialEvent() {
  while (Serial.available()) {
    char inChar = (char)Serial.read();
    inputString += inChar;
    //if (inChar == '\n') {
    stringComplete = true;
    //}
  }
}

