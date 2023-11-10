/*
* This file contains all the necessary settings for the IQS323 and this file can
* be changed from the GUI or edited here
* File:   IQS323_init.h
* Author: Azoteq
*/

#ifndef IQS323_SETTINGS_H
#define IQS323_SETTINGS_H

/* Change the Sensor 0 Settings */
/* Memory Map Position 0x30 - 0x39 */
#define S0_SETUP                                 0x29
#define S0_TX_SELECT                             0x04
#define S0_CONV_FREQ_FRAC                        0x7F
#define S0_CONV_FREQ_PERIOD                      0x05
#define S0_PRX_CTRL_0                            0xBD
#define S0_PRX_CTRL_1                            0x12
#define S0_TG_CTRL                               0x8F
#define S0_RX_SELECT                             0x02
#define S0_CALCAP_INACTIVE_RX                    0x0A
#define S0_PATTERN_SETUP                         0x0B
#define S0_PATTERN_SELECT                        0x00
#define S0_BIAS_CURRENT                          0x00
#define S0_ATI_SETUP_0                           0x0C
#define S0_ATI_SETUP_1                           0x02
#define S0_ATI_BASE_0                            0xC8
#define S0_ATI_BASE_1                            0x00
#define S0_ATI_COARSE                            0x44
#define S0_ATI_FINE                              0x58
#define S0_COMPENSATION_0                        0x10
#define S0_COMPENSATION_1                        0xFB

/* Change the Sensor 1 Settings */
/* Memory Map Position 0x40 - 0x49 */
#define S1_SETUP                                 0x29
#define S1_TX_SELECT                             0x08
#define S1_CONV_FREQ_FRAC                        0x7F
#define S1_CONV_FREQ_PERIOD                      0x05
#define S1_PRX_CTRL_0                            0xBD
#define S1_PRX_CTRL_1                            0x12
#define S1_TG_CTRL                               0x8F
#define S1_RX_SELECT                             0x01
#define S1_CALCAP_INACTIVE_RX                    0x0A
#define S1_PATTERN_SETUP                         0x0B
#define S1_PATTERN_SELECT                        0x00
#define S1_BIAS_CURRENT                          0x00
#define S1_ATI_SETUP_0                           0x8C
#define S1_ATI_SETUP_1                           0x02
#define S1_ATI_BASE_0                            0xFA
#define S1_ATI_BASE_1                            0x00
#define S1_ATI_COARSE                            0x44
#define S1_ATI_FINE                              0x5E
#define S1_COMPENSATION_0                        0xD5
#define S1_COMPENSATION_1                        0xFA

/* Change the Sensor 2 Settings */
/* Memory Map Position 0x50 - 0x59 */
#define S2_SETUP                                 0x00
#define S2_TX_SELECT                             0x01
#define S2_CONV_FREQ_FRAC                        0x7F
#define S2_CONV_FREQ_PERIOD                      0x05
#define S2_PRX_CTRL_0                            0x90
#define S2_PRX_CTRL_1                            0x12
#define S2_TG_CTRL                               0xCF
#define S2_RX_SELECT                             0x01
#define S2_CALCAP_INACTIVE_RX                    0x0A
#define S2_PATTERN_SETUP                         0x03
#define S2_PATTERN_SELECT                        0x00
#define S2_BIAS_CURRENT                          0x00
#define S2_ATI_SETUP_0                           0x0C
#define S2_ATI_SETUP_1                           0x04
#define S2_ATI_BASE_0                            0x64
#define S2_ATI_BASE_1                            0x00
#define S2_ATI_COARSE                            0x4E
#define S2_ATI_FINE                              0x5A
#define S2_COMPENSATION_0                        0xE9
#define S2_COMPENSATION_1                        0x63

/* Change the Channel 0 settings */
/* Memory Map Position 0x60 - 0x63 */
#define CH0_REF_UI_SETUP                         0x00
#define CH0_FOLLOWER_MASK                        0x00
#define CH0_PROX_THRESHOLD                       0x14
#define CH0_PROX_DEBOUNCE                        0x44
#define CH0_TOUCH_THRESHOLD                      0x28
#define CH0_TOUCH_HYSTERESIS                     0x00
#define CH0_FOLLOWER_WEIGHT_0                    0x00
#define CH0_FOLLOWER_WEIGHT_1                    0x00

/* Change the Channel 1 settings */
/* Memory Map Position 0x70 - 0x75 */
#define CH1_REF_UI_SETUP                         0x00
#define CH1_FOLLOWER_MASK                        0x00
#define CH1_PROX_THRESHOLD                       0x0A
#define CH1_PROX_DEBOUNCE                        0x44
#define CH1_TOUCH_THRESHOLD                      0x0C
#define CH1_TOUCH_HYSTERESIS                     0x00
#define CH1_FOLLOWER_WEIGHT_0                    0x00
#define CH1_FOLLOWER_WEIGHT_1                    0x00

/* Change the Channel 2 settings */
/* Memory Map Position 0x80 - 0x85 */
#define CH2_REF_UI_SETUP                         0x00
#define CH2_FOLLOWER_MASK                        0x00
#define CH2_PROX_THRESHOLD                       0x00
#define CH2_PROX_DEBOUNCE                        0x00
#define CH2_TOUCH_THRESHOLD                      0x00
#define CH2_TOUCH_HYSTERESIS                     0x00
#define CH2_FOLLOWER_WEIGHT_0                    0x00
#define CH2_FOLLOWER_WEIGHT_1                    0x00

/* Change the Slider Configuration */
/* Memory Map Position 0x90 - 0x98 */
#define SLIDER_SETUP                             0x00
#define LOWER_CALIBRATION                        0x00
#define UPPER_CALIBRATION                        0x00
#define BOTTOM_SPEED                             0x00
#define TOP_SPEED_0                              0x00
#define TOP_SPEED_1                              0x00
#define SLIDER_RESOLUTION_0                      0x00
#define SLIDER_RESOLUTION_1                      0x00
#define ENABLE_MASK_0                            0x00
#define ENABLE_MASK_1                            0x00
#define ENABLE_STATUS_POINTER_0                  0x00
#define ENABLE_STATUS_POINTER_1                  0x00
#define DELTA_LINK0_0                            0x00
#define DELTA_LINK0_1                            0x00
#define DELTA_LINK1_0                            0x00
#define DELTA_LINK1_1                            0x00
#define DELTA_LINK2_0                            0x00
#define DELTA_LINK2_1                            0x00

/* Change the Gesture Setup */
/* Memory Map Position 0xA0 - 0xA6 */
#define GESTURE_SELECT                           0x00
#define RESERVED_BYTE                            0x00
#define MINIMUM_TIME_0                           0x00
#define MINIMUM_TIME_1                           0x00
#define MAXIMUM_TAP_TIME_0                       0x00
#define MAXIMUM_TAP_TIME_1                       0x00
#define MAXIMUM_SWIPE_TIME_0                     0x00
#define MAXIMUM_SWIPE_TIME_1                     0x00
#define MINIMUM_HOLD_TIME_0                      0x00
#define MINIMUM_HOLD_TIME_1                      0x00
#define MAXIMUM_TAP_DISTANCE_0                   0x00
#define MAXIMUM_TAP_DISTANCE_1                   0x00
#define MINIMUM_SWIPE_DISTANCE_0                 0x00
#define MINIMUM_SWIPE_DISTANCE_1                 0x00

/* Change the Filter Betas */
/* Memory Map Position 0xB0 - 0xB4 */
#define NP_COUNTS_FILTER                         0x02
#define LP_COUNTS_FILTER                         0x01
#define NP_LTA_FILTER                            0x08
#define LP_LTA_FILTER                            0x07
#define NP_LTA_FAST_FILTER                       0x04
#define LP_LTA_FAST_FILTER                       0x03
#define NP_ACTIVATION_LTA_FILTER                 0x00
#define LP_ACTIVATION_LTA_FILTER                 0x00
#define FAST_FILTER_BAND_0                       0x0F
#define FAST_FILTER_BAND_1                       0x00

/* Change the Power mode & System Settings */
/* Memory Map Position 0xC0 - 0xC5 */
#define SYSTEM_CONTROL                           0x50
#define CHANNEL_TIMEOUT_DISABLE                  0x00
#define NP_REPORT_RATE_0                         0x0A
#define NP_REPORT_RATE_1                         0x00
#define LP_REPORT_RATE_0                         0x50
#define LP_REPORT_RATE_1                         0x00
#define ULP_REPORT_RATE_0                        0xC8
#define ULP_REPORT_RATE_1                        0x00
#define HALT_REPORT_RATE_0                       0xB8
#define HALT_REPORT_RATE_1                       0x0B
#define POWER_MODE_TIMEOUT_0                     0xD0
#define POWER_MODE_TIMEOUT_1                     0x07

/* Change the I2C Settings and Events Mask */
/* Memory Map Position 0xD0 - 0xD4 */
#define OUTA_MASK_0                              0x00
#define OUTA_MASK_1                              0x0A
#define I2C_TIMEOUT_0                            0xC8
#define I2C_TIMEOUT_1                            0x00
#define PROX_EVENT_TIMEOUT                       0x14
#define TOUCH_EVENT_TIMEOUT                      0x0A
#define EVENTS_ENABLE                            0x0B
#define ACTIVATION_THRESHOLD                     0x00
#define RELEASE_DELTA_PERCENTAGE                 0x00
#define DELTA_SNAP_SAMPLE_DELAY                  0x00

/* Change the I2C settings */
/* Memory Map Position 0xE0 - 0xE1 */
#define I2C_SETUP                                0x00
#define RESERVED_BYTE                            0x00
#define HWID_0                                   0xEE
#define HWID_1                                   0xEE

#endif	/* IQS323_INIT_H */
