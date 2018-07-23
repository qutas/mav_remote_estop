#include <SoftwareSerial.h>

#define MAVLINK_USE_CONVENIENCE_FUNCTIONS

#include "include/mavlink/mavlink_types.h"
void comm_send_ch(mavlink_channel_t chan, uint8_t ch);
mavlink_system_t mavlink_system;
#include "include/mavlink/common/mavlink.h"

typedef enum {
  MODE_NORMAL = 0,
  MODE_FAILSAFE
} mode_t;

mode_t mode = MODE_NORMAL;

bool mav_state_armed = false;

const int buttonRed = 4;
const int buttonGreen = 5;
const int ledRed = 2;
const int ledGreen = 3;

int ledStateRed = LOW;
int ledStateGreen = HIGH;
int buttonGreenState = HIGH;
int lastGreenButtonState = HIGH;
int buttonRedState = HIGH;
int lastRedButtonState = HIGH;

unsigned long lastDebounceTimeRed = 0;
unsigned long lastDebounceTimeGreen = 0;
unsigned long debounceDelay = 50;
unsigned long blinkdelay = 250;
unsigned long lastBlink = 0;

SoftwareSerial Serials(10, 11);

void system_failsafe();
void set_leds_failsafe();
void set_leds_normal();
void check_buttons();
void communication_decode(uint8_t port, uint8_t c);

void setup() {
  pinMode(buttonGreen, INPUT_PULLUP);
  pinMode(buttonRed, INPUT_PULLUP);
  pinMode(ledGreen, OUTPUT);
  pinMode(ledRed, OUTPUT);
  
  set_leds_normal();

  mavlink_set_proto_version(MAVLINK_COMM_0, 1);
  mavlink_set_proto_version(MAVLINK_COMM_1, 1);
  mavlink_system.sysid = 255;
  mavlink_system.compid = 0;
  
  Serial.begin(115200);
  Serials.begin(115200);
}

void loop() {
  switch(mode) {
    case MODE_NORMAL: {
      set_leds_normal();
  
      break;
    }
    case MODE_FAILSAFE: {
      set_leds_failsafe();

      break;
    }
    default: {
      //Something is really wrong, system failsafe;
      system_failsafe();
    }
  }

  if(Serial.available()) {
    communication_decode( MAVLINK_COMM_0, Serial.read() );
  }

  if(Serials.available()) {
    communication_decode( MAVLINK_COMM_1, Serials.read() );
  }
}

void comm_send_ch(mavlink_channel_t chan, uint8_t ch) {
    if( chan == MAVLINK_COMM_0 )
      Serial.write(ch);
}

void communication_decode(uint8_t port, uint8_t c) {
  mavlink_message_t msg;
  mavlink_status_t status;

  //Enable forwarding by default for primary port
  bool forward_message = (port == MAVLINK_COMM_0) ? true : false;

  // Try to get a new message
  if(mavlink_parse_char(port, c, &msg, &status)) {
    //XXX: This may happen automatically in the MAVLINK backend
    //If we detected a mavlink v2 status from GCS, and we're still in v1, switch
    if( ( !(mavlink_get_channel_status(port)->flags & MAVLINK_STATUS_FLAG_IN_MAVLINK1) ) &&
      ( mavlink_get_proto_version(port) == 1) ) {
      mavlink_set_proto_version(port, 2);
    }
    
    // Handle message
    switch(msg.msgid) {
      case MAVLINK_MSG_ID_HEARTBEAT: {
        //If we are recieving this from the drone
        if(port == MAVLINK_COMM_1) {
          if( (mode == MODE_FAILSAFE) && (mavlink_msg_heartbeat_get_system_status(&msg)) ){
            mavlink_msg_command_long_send(MAVLINK_COMM_0,
                                          msg.sysid, msg.compid,
                                          MAV_CMD_COMPONENT_ARM_DISARM,
                                          0,
                                          false,
                                          0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
           }
        }
        
        break;
      }
      case MAVLINK_MSG_ID_COMMAND_LONG: {
        uint16_t command = mavlink_msg_command_long_get_command(&msg);

        switch(command) {
          /*
          // XXX: May need to switch dynamically if we get this
          case MAV_CMD_REQUEST_PROTOCOL_VERSION: {
            uint8_t ver = mavlink_get_proto_version(port);
            mavlink_set_proto_version(port, 2); //Switch to v2

            const uint8_t blank_array[8] = {0,0,0,0,0,0,0,0};
            mavlink_msg_protocol_version_send(port,
                              MAVLINK_VERSION_MAX,
                              MAVLINK_VERSION_MIN,
                              MAVLINK_VERSION_MAX,
                              &blank_array[0],
                              (uint8_t*)GIT_VERSION_MAVLINK_STR);

            mavlink_set_proto_version(port, ver); //Switch back

            break;
          }
          */
          case MAV_CMD_COMPONENT_ARM_DISARM: {
            //If we're in failsafe and the request is to arm, ignore it
            if( (mode == MODE_FAILSAFE) && ((bool)mavlink_msg_command_long_get_param1(&msg)) ){
              forward_message = false;
            }

            break;
          }
        }

        break;
      }
      default:
        //TODO: Error?
        //Do nothing
        break;
    }
  }
  
  if(forward_message)
    _mavlink_resend_uart(port, &msg);
}

void check_buttons() {
  int readingRed = digitalRead(buttonRed);
  int readingGreen = digitalRead(buttonGreen);

  // If the switch changed, due to noise or pressing:
  if (readingRed != lastRedButtonState) {
    // reset the debouncing timer
    lastDebounceTimeRed = millis();
  }

  // If the switch changed, due to noise or pressing:
  if (readingGreen != lastGreenButtonState) {
    // reset the debouncing timer
    lastDebounceTimeGreen = millis();
  }

  if ((millis() - lastDebounceTimeRed) > debounceDelay) {
    if (readingRed != buttonRedState) {
      buttonRedState = readingRed;
      if (buttonRedState == LOW) {
        if (mode != MODE_FAILSAFE) {
          mode = MODE_FAILSAFE;
          Serial.println("Switching to failsafe mode");
        }
      }
    }
  }

  if ((millis() - lastDebounceTimeGreen) > debounceDelay) {
    if (readingGreen != buttonGreenState) {
      buttonGreenState = readingGreen;
      if (buttonGreenState == LOW) {
        if (mode != MODE_NORMAL) {
          mode = MODE_NORMAL;
          Serial.println("Switching to normal mode");
        }
      }
    }
  }
 
  lastRedButtonState = readingRed;
  lastGreenButtonState = readingGreen;
  
}

void system_failsafe() {
  while(true) {
    ledStateGreen = !ledStateGreen;
    ledStateRed = !ledStateRed;

    digitalWrite(ledGreen, ledStateGreen);
    digitalWrite(ledRed, ledStateRed);

    delay(500);
  }
}

void set_leds_normal() {
  ledStateRed = LOW;
  ledStateGreen = HIGH;
 
  digitalWrite(ledGreen, ledStateGreen);
  digitalWrite(ledRed, ledStateRed); 
}

void set_leds_failsafe() {
  if ( (millis() - lastBlink) > blinkdelay) {
    lastBlink = millis();
    ledStateRed = !ledStateRed;
  }

  ledStateGreen = LOW;
  
  digitalWrite(ledGreen, ledStateGreen);
  digitalWrite(ledRed, ledStateRed);
}

