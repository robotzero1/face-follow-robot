#include <Wire.h>
#include "PinChangeInt.h"
#include "Pins.h"
#include "mode.h"
#include "Command.h"
#include "BalanceCar.h"
#include "Rgb.h"
#include "voltage.h"


const byte numChars = 32;
char receivedChars[numChars];
boolean newData = false;

unsigned long start_prev_time = 0;
unsigned long serial_data_received_time;
boolean carInitialize_en = true;

int new_setting_turn_speed;
int new_setting_car_speed;
long check_time;

void setMotionState()
{
  switch (motion_mode)
  {
    case STANDBY:
      setting_car_speed = 0;
      setting_turn_speed = 0;
      break;
    case STOP:
      if (millis() - start_prev_time > 1000)
      {
        function_mode = IDLE;
        if (balance_angle_min <= kalmanfilter_angle && kalmanfilter_angle <= balance_angle_max)
        {
          motion_mode = STANDBY;
          rgb.lightOff();
        }
      }
      break;
    case START:
      if (millis() - start_prev_time > 2000)
      {
        if (balance_angle_min <= kalmanfilter_angle && kalmanfilter_angle <= balance_angle_max)
        {
          car_speed_integeral = 0;
          setting_car_speed = 0;
          motion_mode = STANDBY;
          rgb.lightOff();
        }
        else
        {
          motion_mode = STOP;
          carStop();
          rgb.brightRedColor();
        }
      }
      break;
    default:
      break;
  }
}

void setup() {
  delay(8000); //delay 8 seconds before starting Serial
  Serial.begin(9600);
  keyInit();
  rgb.initialize();
  voltageInit();
  start_prev_time = millis();
  carInitialize();
  motion_mode = STOP;
}

void loop() {
  Serial.println(motion_mode);
  Serial.println(new_setting_turn_speed);

  recvWithStartEndMarkers();
  showNewData();

  if (millis() - serial_data_received_time > 500) { // no new face detections for .5sec then reduce by x each loop

    if (millis() - check_time > 100) {

      if (new_setting_turn_speed > 0) {
        new_setting_turn_speed = new_setting_turn_speed - 1;
      }
       if (new_setting_turn_speed < 0) {
        new_setting_turn_speed = new_setting_turn_speed + 1;
      }
      if (new_setting_car_speed > 0) {
        new_setting_car_speed = new_setting_car_speed - 1;
      }
      if (new_setting_car_speed < 0) {
        new_setting_car_speed = new_setting_car_speed + 1;
      }    
      check_time = millis();
    }
  }


  if ( new_setting_turn_speed == 0 && new_setting_car_speed == 0) {
    motion_mode = STANDBY;
  }
  if ( new_setting_turn_speed < 0 ) {
    motion_mode = TURNLEFT;
  }  
  if ( new_setting_turn_speed > 0 ) {
    motion_mode = TURNRIGHT;
  }
  if ( new_setting_car_speed > 0 && new_setting_turn_speed == 0) {
    motion_mode = FORWARD;
  }
  if ( new_setting_car_speed < 0 && new_setting_turn_speed == 0) {
    motion_mode = BACKWARD;
  }

  setting_turn_speed = new_setting_turn_speed;
  setting_car_speed = new_setting_car_speed;

  voltageMeasure();
  setMotionState();

  rgb.blink(500);

  static unsigned long print_time;
  if (millis() - print_time > 100)
  {
    print_time = millis();
    //Serial.println(kalmanfilter.angle);
  }
  static unsigned long start_time;
  if (millis() - start_time < 10)
  {
    //      function_mode = IDLE;
    //      motion_mode = STOP;
    //      carStop();
  }
  if (millis() - start_time == 2000) // Enter the pendulum, the car balances...
  {
    key_value = '5';
  }
}

void recvWithStartEndMarkers() {
  static boolean recvInProgress = false;
  static byte ndx = 0;
  char startMarker = '<';
  char endMarker = '>';
  char rc;

  while (Serial.available() > 0 && newData == false) {
    rc = Serial.read();

    if (recvInProgress == true) {
      if (rc != endMarker) {
        receivedChars[ndx] = rc;
        ndx++;
        if (ndx >= numChars) {
          ndx = numChars - 1;
        }
      }
      else {
        receivedChars[ndx] = '\0'; // terminate the string
        recvInProgress = false;
        ndx = 0;
        newData = true;
      }
    }

    else if (rc == startMarker) {
      recvInProgress = true;
    }
  }
}

void showNewData() {
  if (newData == true) {
    rgb.flashYellowColorback();
    serial_data_received_time = millis();
    int pan, tilt, distance;

    int result = sscanf(receivedChars, "%i,%i,%i", &pan, &tilt, &distance);

    new_setting_turn_speed = map(pan, 0, 320, 40, -40); // face location on camera,  mapped to left or right turn speed
    new_setting_car_speed = map(distance, 10, 1200, -20, 20); // face distance from camera,  mapped to forward/reverse speed

    if (pan < 160) {
      rgb.flashBlueColorLeft();
    }
    if (pan > 160) {
      rgb.flashBlueColorRight();
    }
    newData = false;
    //rgb.lightOff();
  }
}
