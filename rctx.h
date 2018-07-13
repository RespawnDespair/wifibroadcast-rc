// Axis number to channel number mapping. Joystick axis 0-7 maps to R/C channel 1-8.
// Axis mapping is AETR1234

//#define JSSWITCHES	16	/// 8 for 8 axis + 8 switches = 16 channels / 16 for 8 + 16 = 24 channels to send

#define ROLL_AXIS      0
#define PITCH_AXIS     1
#define YAW_AXIS       3
#define THROTTLE_AXIS  2
#define AUX1_AXIS      4
#define AUX2_AXIS      5
#define AUX3_AXIS      6
#define AUX4_AXIS      7

// Initial axis settings. Since we cannot determine the stick positions until the
// corresponding axis has been moved, we need to pre-fill initial stick postions.
//
// Set your throttle channel to zero throttle (usually 1000) to make sure motors don't spin unintended!
// Set the other channels to the desired middle stick position (usually 1500)

#define AXIS0_INITIAL 1500
#define AXIS1_INITIAL 1500
#define AXIS2_INITIAL 1020
#define AXIS3_INITIAL 1500
#define AXIS4_INITIAL 1000
#define AXIS5_INITIAL 1000
#define AXIS6_INITIAL 1000
#define AXIS7_INITIAL 1000

#define UPDATE_NTH_TIME 8 // 3 = 6ms (167Hz), 4 = 8ms (125Hz), 5 = 10ms (100Hz), 6 = 12ms (83Hz), 7 = 14ms (71Hz), 8 = 16ms (63Hz), 9 = 18ms (56Hz), 10 = 20ms (50Hz) 
#define TRANSMISSIONS 2
