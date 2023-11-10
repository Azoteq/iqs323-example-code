/******************************************************************************
 *                                                                            *
 *                                Copyright by                                *
 *                                                                            *
 *                              Azoteq (Pty) Ltd                              *
 *                          Republic of South Africa                          *
 *                                                                            *
 *                           Tel: +27(0)21 863 0033                           *
 *                           E-mail: info@azoteq.com                          *
 *                                                                            *
 * ========================================================================== *
 * @file        iqs323-example-code.ino                                       *
 * @brief       IQS323 EV-KIT Variants Example code                           *
 *              This example demonstrates how to write the desired settings   *
 *              to the IQS323 in order to use the three IQS323 EV-Kit variants*
 *                                                                            *
 *              EV-KIT Options:                                               *
 *              0: Inductive Options EV-Kit (AZP1212A4).                      *
 *              1: Slider EV-Kit (AZP1209A4).                                 *
 *              2: 3-Projected Buttons EV-Kit (AZP1210A4).                    *
 *                                                                            *
 *              All data is displayed over serial communication with the      *
 *              following outputs:                                            *
 *                  - Slider coordinates (Slider EV-Kit)                      *
 *                  - Gesture events (Slider EV-Kit)                          *
 *                  - Channel states (Inductive Options and Projected Buttons)* 
 *                  - Power Mode Feedback (All EV-Kits)                       *
 *                  - Forced Communication (All EV-Kits)                      *
 *                                                                            *
 * @author      Azoteq PTY Ltd                                                *
 * @version     v1.5.2-pl2                                                    *
 * @date        2023-11-10                                                    *
 *****************************************************************************/

#include <Arduino.h>
#include "src\IQS323\IQS323.h"

/*** Defines ***/
#define DEMO_IQS323_ADDR                      0x44
#define DEMO_IQS323_POWER_PIN                 4
#define DEMO_IQS323_RDY_PIN                   7

/*** Instances ***/
IQS323 iqs323;

/*** Global Variables ***/
iqs323_ch_states button_states[3];
bool show_data = false;
iqs323_power_modes running_power_state = IQS323_NORMAL_POWER;
uint16_t slider_position = 65535;
iqs323_gesture_events slider_event = IQS323_GESTURE_NONE;

void setup()
{
  /* Start Serial Communication */
  Serial.begin(115200);
  //while(!Serial);
  Serial.println("Start Serial communication");
  delay(200);

  /* Power On IQS323 */
  pinMode(DEMO_IQS323_POWER_PIN, OUTPUT);
  delay(200);
  digitalWrite(DEMO_IQS323_POWER_PIN, LOW);
  delay(200);
  digitalWrite(DEMO_IQS323_POWER_PIN, HIGH);

  #if IQS323_EV_KIT == 0
    Serial.println("IQS323 Inductive EV-Kit Selected");
  #elif IQS323_EV_KIT == 1
    Serial.println("IQS323 Slider EV-Kit Selected");
  #elif IQS323_EV_KIT == 2
    Serial.println("IQS323 Projected Buttons EV-Kit Selected");
  #endif

  /* Initialize the IQS323 with input parameters device address and RDY pin */
  iqs323.begin(DEMO_IQS323_ADDR, DEMO_IQS323_RDY_PIN);
  Serial.println("IQS323 Ready");
  delay(1);
}

void loop()
{
  iqs323.run(); // Runs the IQS323 program loop

  force_comms_and_reset(); // function to initialize a force communication window.

  /* Process data read from IQS323 when new data is available (RDY Line Low) */
  if(iqs323.new_data_available)
  {
    check_power_mode();         // Verify if a power mode change occurred
    show_IQS323_data();         // Show data if a forced communication window was requested

    #if IQS323_EV_KIT == 1
    read_slider_coordinates();  // Check the current slider coordinates
    read_gesture_event();       // Check if a slider gesture state change has occurred
    #else
    check_channel_states();     // Check if a channel state change has occurred
    #endif

    iqs323.new_data_available = false;
  }
}

/* Function to check when the current power mode of the IQS323 changed */
void check_power_mode(void)
{
  iqs323_power_modes buffer = iqs323.get_PowerMode(); // read current power mode

  /* Only display power mode when the state has changed */
  if(running_power_state != buffer)
  {
    switch(buffer)
    {
      case IQS323_NORMAL_POWER:
        Serial.println("NORMAL POWER Activated!");
        break;
      case IQS323_LOW_POWER:
        Serial.println("LOW POWER Activated!");
        break;
      case IQS323_ULP:
        Serial.println("ULP Activated!");
        break;
      case IQS323_HALT:
        Serial.println("Halt Activated!");
      break;
      default:
      break;
    }

    /* Update the running state */
    running_power_state = buffer;
  }
}

/* Function to check the proximity and touch states of the three IQS323 channels*/
void check_channel_states(void)
{
  /* Loop through all the active channels */
  for(uint8_t i = 0; i < 3; i++)
  {
    /* Check if the touch state bit is set */
    if(iqs323.channel_touchState((iqs323_channel_e)(i)))
    {
      if(button_states[i] != IQS323_CH_TOUCH)
      {
        Serial.print("CH");
        Serial.print(i);
        Serial.println(": Touch");
        button_states[i] = IQS323_CH_TOUCH;
      }
    }
    /* Check if the proximity state bit is set */
    else if (iqs323.channel_proxState((iqs323_channel_e)(i)))
    {
      if(button_states[i] != IQS323_CH_PROX)
      {
        Serial.print("CH");
        Serial.print((i));
        Serial.println(": Prox");
        button_states[i] = IQS323_CH_PROX;
      }
    }
    /* None state if neither the touch nor proximity bit was set to 1 */
    else
    {
      if(button_states[i] != IQS323_CH_NONE)
      {
        Serial.print("CH");
        Serial.print((i));
        Serial.println(": None");
        button_states[i] = IQS323_CH_NONE;
      }
    }
  }
}

/* Read the coordinate of a finger on the IQS323 Slider */
void read_slider_coordinates(void)
{
  /* read slider coordinates from memory */
  uint16_t buffer = iqs323.sliderCoordinate();

  if(buffer != slider_position)
  {
    slider_position = buffer;
  }
}

/* Function to process Slider gesture events */
void read_gesture_event(void)
{
  /* Read slider bit to check if a slider event occurred */
  bool gesture_event = iqs323.getSliderEvent();
  if(gesture_event)
  {
    /* returns slider event that occurred (tap, swipe or flick) by reading event bits from MM */
    iqs323_gesture_events buffer = iqs323.getGestureType();
    if(slider_event != buffer)
    {
      slider_event = buffer;
      switch(slider_event)
      {
        case IQS323_GESTURE_TAP:
          Serial.println("SLIDER: Tap");
          break;
        case IQS323_GESTURE_SWIPE_NEGATIVE:
          Serial.println("SLIDER: Swipe <-");
          break;
        case IQS323_GESTURE_SWIPE_POSITIVE:
          Serial.println("SLIDER: Swipe ->");
          break;
        case IQS323_GESTURE_FLICK_NEGATIVE:
          Serial.println("SLIDER: Flick <-");
          break;
        case IQS323_GESTURE_FLICK_POSITIVE:
          Serial.println("SLIDER: Flick ->");
          break;
        case IQS323_GESTURE_HOLD:
          Serial.println("SLIDER: Hold");
          break;
        case IQS323_GESTURE_NONE:
          Serial.println("SLIDER: None");
          break;
      }

      /* Clear event if a finger is removed from slider after the event was processed */
      if(slider_position == 65535)
      {
        slider_event = IQS323_GESTURE_NONE;
      }
    }
  }
}

/* Force the IQS323 to open a RDY window and read the current state of the
 * device or request a software reset */
void force_comms_and_reset(void)
{
  char message = read_message();

  /* If an 'f' was received over serial, open a forced communication window and
   * print the new data received */
  if(message == 'f')
  {
    iqs323.force_I2C_communication(); // Prompt the IQS323
    show_data = true;
  }

  /* If an 'r' was received over serial, request a software (SW) reset */
  if(message == 'r')
  {
    Serial.println("Software Reset Requested!");
    iqs323.force_I2C_communication(); // Request a RDY window
    iqs323.iqs323_state.state = IQS323_STATE_SW_RESET;
  }
}

/* Read message sent over serial communication */
char read_message(void)
{
  while (Serial.available())
  {
    if(Serial.available() > 0)
    {
      return  Serial.read();
    }
  }

  return '\n';
}

/* Shows the current IQS323 data */
void show_IQS323_data()
{
  if(show_data)
  {
    Serial.println("***** IQS323 DATA *****");
    Serial.println("Forced Communication");

    /* Clear existing states */
    for(uint8_t i = 0; i < 3; i++)
    {
      button_states[i] = IQS323_CH_UNKNOWN;
    }
    /* Retrieve new values directly from the IQS323 device */
    running_power_state = IQS323_POWER_UNKNOWN;
    slider_event = IQS323_GESTURE_UNKNOWN;

    check_power_mode();         // Verify if a power mode change occurred
    read_slider_coordinates();  // Check the current slider coordinates
    read_gesture_event();       // Check if a slider gesture state change has occurred
    check_channel_states();     // Check if a channel state change has occurred

    Serial.println("*************************");
    show_data = false;
  }
}
