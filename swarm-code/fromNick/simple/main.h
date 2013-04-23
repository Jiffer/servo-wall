/************************************************************************/
/* xgrid                                                                */
/*                                                                      */
/* main.h                                                               */
/*                                                                      */
/* Alex Forencich <alex@alexforencich.com>                              */
/*                                                                      */
/* Copyright (c) 2011 Alex Forencich                                    */
/*                                                                      */
/* Permission is hereby granted, free of charge, to any person          */
/* obtaining a copy of this software and associated documentation       */
/* files(the "Software"), to deal in the Software without restriction,  */
/* including without limitation the rights to use, copy, modify, merge, */
/* publish, distribute, sublicense, and/or sell copies of the Software, */
/* and to permit persons to whom the Software is furnished to do so,    */
/* subject to the following conditions:                                 */
/*                                                                      */
/* The above copyright notice and this permission notice shall be       */
/* included in all copies or substantial portions of the Software.      */
/*                                                                      */
/* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,      */
/* EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF   */
/* MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND                */
/* NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS  */
/* BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN   */
/* ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN    */
/* CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE     */
/* SOFTWARE.                                                            */
/*                                                                      */
/************************************************************************/

#ifndef __MAIN_H
#define __MAIN_H

#include <math.h>
#include <avr/io.h>
#include <avr/pgmspace.h>
#include <util/delay.h>
#include <avr/interrupt.h>

#include <stdio.h>
//#include <time.h>

#include "board.h"
#include "usart.h"
#include "spi.h"
#include "i2c.h"
#include "eeprom.h"
#include "xgrid.h"
#include "servo.h"
#include "../xboot/xbootapi.h"

// Build information
extern char   __BUILD_DATE;
extern char   __BUILD_NUMBER;

// defines

// typedefs

// Prototypes
void init(void);
int main(void);
uint8_t SP_ReadCalibrationByte( uint8_t index );
uint8_t SP_ReadUserSigRow( uint8_t index );

#ifndef ADCACAL0_offset

#define ADCACAL0_offset 0x20
#define ADCACAL1_offset 0x21
#define ADCBCAL0_offset 0x24
#define ADCBCAL1_offset 0x25

#endif

#endif // __MAIN_H

//##########################################################################
//##########################################################################
//##########################################################################

#define prt_flt3(f)	(int16_t)f,(int16_t)((fabs(f)-(int16_t)(fabs(f)))*1000)	

#define PI 3.14159
#define NUM_NEIGHBORS 6

//input port number					//output 
#define BOTTOM_LEFT				1	//0b00000010
#define BOTTOM_RIGHT			0	//0b00000001
#define LEFT_TOP				3	//0b00001000   
#define LEFT_BOTTOM				2	//0b00000100
#define RIGHT_BOTTOM			4	//0b00010000
#define RIGHT_TOP				5	//0b00100000

#define MESSAGE_NUMDATA 0
#define MESSAGE_COMMAND 1
#define ALL_DIRECTION 0x3F

#define ALL 20
#define NEIGHBOR_DATA 11


//##########################################################################

//===== common =====
volatile unsigned long jiffies = 0, temp_time, cnt4sensor = 0;

bool connected[NUM_NEIGHBORS];
bool sonar_attached = false;
bool dblchk = true, trichk = true;
//uint16_t sensor_value, sensor_value_now, sensor_value_dblchk, sensor_value_trichk;
uint16_t sum_dbl = 0, sum_tri = 0;

// counters
int sec_counter = 0;
int counterTenHz = 0;
int counterFiftyHz = 0;
int presenceTimer = 0;
int neighborPresenceTimer = 0;
int presModeCounter = 0;

// tracking beats
int beatCounterFiftyHz = 0;
int currentBeat = 0;
int lastBeat = 0;
int ticksPerBeat = 12;
int numBeats = 6;

#define MAX_BEATS 8
int beatPattern[MAX_BEATS];

float decay_tim;
float global_amp;

bool communication_on = false;

bool sendmessage_fast = false;
bool sendmessage_slow = false;

bool display = false;
bool display_on = false;

bool reboot_on = false;
bool servo_motor_on = false;

bool speedup_on = false;
bool use_sensor_data_on = false;

bool special = false;	//marker of "Bottom-Left" module
bool bottom = false;

//===== for "Ken's model" =====
//bool sync = true;

//===== for "wave" and "column swing" =====
bool wave_flg = false,  wave_ping = false;
bool column_flg = false;
uint8_t wave_port;

//===== for "rhythm" =====
//bool rhythm_on = false;

// ===== PROTOTYPE DECLARATION =====
void send_message(uint8_t, uint8_t, int, const char[]);
void servo_motor_control(float);

//##### SENSOR RANGE ######
#define RANGE1 50
#define RANGE2 150
#define RANGE3 250
#define IGNORE_DISTANCE 400

// jif's globals //
#define MAX_ANGLE 65.0
#define PRESENCE_THRESH 4000
#define PRESENCE_OFF_THRESH 3500
#define STRENGTH_THRESHOLD 0.1

float curAngle = 0; // from -90 to 90
float transitionAngle = 0;
bool inTransition = false;
float fadeIncrement = 0.01;

float offsetVar[4];
int offsetVarIndex = 0;
float amplitudeScaler = 1.0;

bool presenceDetected = false;
bool neighborPresenceDetected = false;
bool newPresence = false;
bool newNeighborPresence = false;
bool ignorePresence = false;
bool ignoreNeighborPresence = false;
bool presenceDetectedLast = false;
bool neighborPresenceDetectedLast = false;
bool usingNeighborPresence = false;

int presenceTimeOut = 15;
int neighborPresenceTimeOut = 15;
float myStrength;
float lastStrength;
int strengthDir;
int lastStrengthDir;
float randomPeriod = 0.0;
int delayFunction = 0;
float crossFade = 0.0;
bool debugPrint = false;
bool cycleOn = false;
bool servoEnabled = true;

// from ports
#define BELOW   1
#define ABOVE   5
#define LEFT    2
#define RIGHT   4
#define MOOT    0
#define NOTHING    7

// neighbors data
struct NeighborData {
    float angleValue;
    int sensorValue;
    float strength; // use to impart influence on neighbors
    int fromDir;
} ;
NeighborData sendData;
NeighborData neighborData[6];

int numConnected = 0;
float neighborAngles[6];


#define NEIGHBOR_BUFFER_SIZE 100
uint8_t neighborBuffer[6][NEIGHBOR_BUFFER_SIZE];
uint8_t neighborBufferPtr = 0;

#define sensorBufSize 2
int sensor_value = 0;
int light_sensor = 0;
int sensorBuf[sensorBufSize];
int sensorBufPtr = 0;

#define angleBufSize 100
int angleBuffer[angleBufSize];
int angleBufPtr = 0;
bool actionComplete = false;

enum updateInterval {
    NONE,
    SMOOTH,
    ONE_HUNDRED,
    TWO_HUNDRED,
    THREE_HUNDRED,
    FOUR_HUNDRED,
    SIX_HUNDRED,
    EIGHT_HUNDRED,
    };


enum algorithm {
    TOGETHER,
    PERIODIC,
    MESMER,
    SWEEP,
    TWITCH,
    TWITCH_WAVE,
    DELAYED,
    FM_TOGETHER,
    AM_TOGETHER,
    BREAK
    };

enum sensorAlgorithm {
    POINT,
    SHAKE,
    WAVE,
    RANDOM,
    RATE,
    RHYTHM,
    IGNORE
};

enum delayAlgorighm{
    CHOP,
    FM,
    AM
};

int updateRate = SMOOTH; 
int currentMode = DELAYED;
int presMode = POINT;

int lastMode = -1; // first time through this will force initialization to run for whatever mode it starts in
int lastPresMode = -1;

// \jif's globals //

// #################### Synchronize ####################

bool calib_switch;			// whether or not to begin the calibration
int calib_times;			// how many time this is to calibration
bool calib_double_switch;

#define _MAIN_BOARD     13068
#define _DELAY_CALIB    2
#define _TIME_CALIB		10

















