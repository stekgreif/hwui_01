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
#include <ClickEncoder.h>


#define ENC_HALFSTEP
#define ENC_DECODER (1 << 2)

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

#define _ID 0x01



ClickEncoder *encoder;
int16_t enc_last_val = -1;
int16_t enc_value = 0;


typedef struct {
  uint8_t   hw_pin;
  uint16_t  debounce_ticks;
  uint8_t   cur_state;
  uint8_t   prev_state;
} btn_t;

btn_t ba[12] = {};


uint16_t distance_val = 0;


void setup()
{
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

  encoder = new ClickEncoder(3, 2, -1, 4);
  encoder->setAccelerationEnabled(true);

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



void enc_check_changes(void)
{
  enc_value += encoder->getValue();
  
  if (enc_value != enc_last_val) 
  {
    int16_t diff = enc_value - enc_last_val;
    uint8_t send_val = diff + 64;
    
    enc_last_val = enc_value;
    
    midiEventPacket_t event = {0x0B, 0xB0, 5, send_val};
    MidiUSB.sendMIDI(event);
    usb_midi_msg_cnt++;
  }
}


void loop()
{
  cur_tick = millis();

  if ( cur_tick != prev_tick )
  {
    encoder->service();
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
          enc_check_changes();
          break;
        }
      case 4:
        {
          usb_midi_read();
          break;
        }
      case 10:
        {
          //Serial.println("i");
          if ( usb_midi_msg_cnt > 0 )
          {
            MidiUSB.flush();
          }
          sched_cnt = 0;
          break;
        }
        default:
          break;
    }
  }
  prev_tick = cur_tick;
}




