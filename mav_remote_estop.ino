#define MAVLINK_USE_CONVENIENCE_FUNCTIONS

#include "include/mavlink/mavlink_types.h"
void comm_send_ch(mavlink_channel_t chan, uint8_t ch);
mavlink_system_t mavlink_system;

#ifndef MAVLINK_MESSAGE_CRCS
#define MAVLINK_MESSAGE_CRCS {{0, 50, 9, 0, 0, 0}, {1, 124, 31, 0, 0, 0}, {2, 137, 12, 0, 0, 0}, {4, 237, 14, 3, 12, 13}, {5, 217, 28, 1, 0, 0}, {6, 104, 3, 0, 0, 0}, {7, 119, 32, 0, 0, 0}, {11, 89, 6, 1, 4, 0}, {20, 214, 20, 3, 2, 3}, {21, 159, 2, 3, 0, 1}, {22, 220, 25, 0, 0, 0}, {23, 168, 23, 3, 4, 5}, {24, 24, 30, 0, 0, 0}, {25, 23, 101, 0, 0, 0}, {26, 170, 22, 0, 0, 0}, {27, 144, 26, 0, 0, 0}, {28, 67, 16, 0, 0, 0}, {29, 115, 14, 0, 0, 0}, {30, 39, 28, 0, 0, 0}, {31, 246, 32, 0, 0, 0}, {32, 185, 28, 0, 0, 0}, {33, 104, 28, 0, 0, 0}, {34, 237, 22, 0, 0, 0}, {35, 244, 22, 0, 0, 0}, {36, 222, 21, 0, 0, 0}, {37, 212, 6, 3, 4, 5}, {38, 9, 6, 3, 4, 5}, {39, 254, 37, 3, 32, 33}, {40, 230, 4, 3, 2, 3}, {41, 28, 4, 3, 2, 3}, {42, 28, 2, 0, 0, 0}, {43, 132, 2, 3, 0, 1}, {44, 221, 4, 3, 2, 3}, {45, 232, 2, 3, 0, 1}, {46, 11, 2, 0, 0, 0}, {47, 153, 3, 3, 0, 1}, {48, 41, 13, 1, 12, 0}, {49, 39, 12, 0, 0, 0}, {50, 78, 37, 3, 18, 19}, {51, 196, 4, 3, 2, 3}, {54, 15, 27, 3, 24, 25}, {55, 3, 25, 0, 0, 0}, {61, 167, 72, 0, 0, 0}, {62, 183, 26, 0, 0, 0}, {63, 119, 181, 0, 0, 0}, {64, 191, 225, 0, 0, 0}, {65, 118, 42, 0, 0, 0}, {66, 148, 6, 3, 2, 3}, {67, 21, 4, 0, 0, 0}, {69, 243, 11, 0, 0, 0}, {70, 124, 18, 3, 16, 17}, {73, 38, 37, 3, 32, 33}, {74, 20, 20, 0, 0, 0}, {75, 158, 35, 3, 30, 31}, {76, 152, 33, 3, 30, 31}, {77, 143, 3, 3, 8, 9}, {81, 106, 22, 0, 0, 0}, {82, 49, 39, 3, 36, 37}, {83, 22, 37, 0, 0, 0}, {84, 143, 53, 3, 50, 51}, {85, 140, 51, 0, 0, 0}, {86, 5, 53, 3, 50, 51}, {87, 150, 51, 0, 0, 0}, {89, 231, 28, 0, 0, 0}, {90, 183, 56, 0, 0, 0}, {91, 63, 42, 0, 0, 0}, {92, 54, 33, 0, 0, 0}, {93, 47, 81, 0, 0, 0}, {100, 175, 26, 0, 0, 0}, {101, 102, 32, 0, 0, 0}, {102, 158, 32, 0, 0, 0}, {103, 208, 20, 0, 0, 0}, {104, 56, 32, 0, 0, 0}, {105, 93, 62, 0, 0, 0}, {106, 138, 44, 0, 0, 0}, {107, 108, 64, 0, 0, 0}, {108, 32, 84, 0, 0, 0}, {109, 185, 9, 0, 0, 0}, {110, 84, 254, 3, 1, 2}, {111, 34, 16, 0, 0, 0}, {112, 174, 12, 0, 0, 0}, {113, 124, 36, 0, 0, 0}, {114, 237, 44, 0, 0, 0}, {115, 4, 64, 0, 0, 0}, {116, 76, 22, 0, 0, 0}, {117, 128, 6, 3, 4, 5}, {118, 56, 14, 0, 0, 0}, {119, 116, 12, 3, 10, 11}, {120, 134, 97, 0, 0, 0}, {121, 237, 2, 3, 0, 1}, {122, 203, 2, 3, 0, 1}, {123, 250, 113, 3, 0, 1}, {124, 87, 35, 0, 0, 0}, {125, 203, 6, 0, 0, 0}, {126, 220, 79, 0, 0, 0}, {127, 25, 35, 0, 0, 0}, {128, 226, 35, 0, 0, 0}, {129, 46, 22, 0, 0, 0}, {130, 29, 13, 0, 0, 0}, {131, 223, 255, 0, 0, 0}, {132, 85, 14, 0, 0, 0}, {133, 6, 18, 0, 0, 0}, {134, 229, 43, 0, 0, 0}, {135, 203, 8, 0, 0, 0}, {136, 1, 22, 0, 0, 0}, {137, 195, 14, 0, 0, 0}, {138, 109, 36, 0, 0, 0}, {139, 168, 43, 3, 41, 42}, {140, 181, 41, 0, 0, 0}, {141, 47, 32, 0, 0, 0}, {142, 72, 243, 0, 0, 0}, {143, 131, 14, 0, 0, 0}, {144, 127, 93, 0, 0, 0}, {146, 103, 100, 0, 0, 0}, {147, 154, 36, 0, 0, 0}, {148, 178, 60, 0, 0, 0}, {149, 200, 30, 0, 0, 0}, {230, 163, 42, 0, 0, 0}, {231, 105, 40, 0, 0, 0}, {232, 151, 63, 0, 0, 0}, {233, 35, 182, 0, 0, 0}, {234, 150, 40, 0, 0, 0}, {235, 179, 42, 0, 0, 0}, {241, 90, 32, 0, 0, 0}, {242, 104, 52, 0, 0, 0}, {243, 85, 53, 1, 52, 0}, {244, 95, 6, 0, 0, 0}, {245, 130, 2, 0, 0, 0}, {246, 184, 38, 0, 0, 0}, {247, 81, 19, 0, 0, 0}, {248, 8, 254, 3, 3, 4}, {249, 204, 36, 0, 0, 0}, {250, 49, 30, 0, 0, 0}, {251, 170, 18, 0, 0, 0}, {252, 44, 18, 0, 0, 0}, {253, 83, 51, 0, 0, 0}, {254, 46, 9, 0, 0, 0}, {256, 71, 42, 3, 8, 9}, {257, 131, 9, 0, 0, 0}, {258, 187, 32, 3, 0, 1}, {259, 92, 235, 0, 0, 0}, {260, 146, 5, 0, 0, 0}, {261, 179, 27, 0, 0, 0}, {262, 12, 18, 0, 0, 0}, {263, 133, 255, 0, 0, 0}, {264, 49, 28, 0, 0, 0}, {265, 26, 16, 0, 0, 0}, {266, 193, 255, 3, 2, 3}, {267, 35, 255, 3, 2, 3}, {268, 14, 4, 3, 2, 3}, {269, 58, 246, 0, 0, 0}, {270, 232, 247, 3, 14, 15}, {299, 19, 96, 0, 0, 0}, {300, 217, 22, 0, 0, 0}, {310, 28, 17, 0, 0, 0}, {311, 95, 116, 0, 0, 0}, {320, 243, 20, 3, 2, 3}, {321, 88, 2, 3, 0, 1}, {322, 243, 149, 0, 0, 0}, {323, 78, 147, 3, 0, 1}, {324, 132, 146, 0, 0, 0}, {330, 23, 158, 0, 0, 0}, {331, 58, 230, 0, 0, 0}, {332, 91, 229, 0, 0, 0}, {333, 231, 109, 0, 0, 0}}
#endif

#include "include/mavlink/minimal/mavlink.h"

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
  mavlink_system.sysid = 255;
  mavlink_system.compid = 0;
  
  Serial.begin(115200);
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
          
          //TODO: Send disarm command if in failsafe mode

        }
        
        break;
      }
      case MAVLINK_MSG_ID_COMMAND_LONG: {
        //A command should always have an acknowledge
        bool need_ack = false;
        uint16_t command = mavlink_msg_command_long_get_command(&msg);
        uint8_t command_result = MAV_RESULT_FAILED;

        switch(command) {
          /* XXX: May need to switch dynamically if we get this
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

