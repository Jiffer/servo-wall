#if PROGMEM_SIZE > 0x010000
#define PGM_READ_BYTE pgm_read_byte_far
#else
#define PGM_READ_BYTE pgm_read_byte_near
#endif

// jif's globals //
long gHundredth = 0; // increment every 1/100th of a second
int gAngle = 0; // from -90 to 90
bool gDirection = 0; // to or fro...
int gNeighborAngles[6];

#define gBufferSize 3
int gAngleBuffer[gBufferSize];
int gPtr = 0;

// \jif's globals //

#include "main.h"
#include "board_init.c"
#include "sonar.c"
#include "swarmfunctions.c"
#include "communication.c"

int ms_sensor_value = 0;

// simple Periodic Motion returns angle based on sinusoidal period (in seconds)
// and angle range (+/- range in degrees)
int periodicMotion(int range, float period){
    int angle;
    gAngle = range * sin(gHundredth * 2.0 * PI /(period * 100)); //
    return angle;
}

// ============================================================================================
// Timer tick ISR (1 kHz)
// ============================================================================================
ISR(TCC0_OVF_vect)
{
	jiffies++;	// Timers

	if(jiffies%10 == 0) // update every 1/100th of a second
	{
        // counter
        gHundredth++;
        
        int range = 45;
        int period = 3;
        
        // sweep sine wave
        //gAngle = range * sin(gHundredth * 2.0 * PI /(period * 100)); //
        
        //fprintf_P(&usart_stream, PSTR("angle: %i \n"), gAngle);
        //gAngle = periodicMotion(45, 8);
        
        // Linear sweep:
        // sweep between -90 and 90
       /* if(gAngle < -90 || gAngle > 90){
            gDirection = !gDirection;
        }
        if (gDirection){
            gAngle++;
        }
        else
            gAngle--;
        */
        
       
        
        // update the servo on this cycle
		servo_motor_on   = true;
		sendmessage_fast = true;	
	}

    // 10 Hz
//    if(jiffies%100 == 0){
//        if (gAngle < 0)
//            gAngle = 70;
//        else
//            gAngle--;
//    }
    
    // check every 5 seconds if it has recieved messages
    if(jiffies%5000){
        if(!connected[1] && !connected[2] && connected[4] && connected[5]) special = true;
        if(!connected[1] && connected[2]) bottom = true;
    }
    
	if(jiffies%200 == 0)
	{
        //////////////////////////
        // Read analog input
        //////////////////////////
        
        if (ADCB.CH0.INTFLAGS) {
            //ADCB.CH0.INTFLAGS = ADC_CH__CHIF_bm;
            ADCB.CH0.INTFLAGS = 0x01;
            ms_sensor_value = ADCB_CH0_RES;
            // print it to serial port
            // range of 200 -> 1900
            
            // if bottom left set angle based on sensor
            if(special){
                // scale sensor value and set angle
                ms_sensor_value = ms_sensor_value - 200;
                float scaled_sensor = (float)ms_sensor_value * 0.2 - 90;
                gAngle = (int)scaled_sensor;
            }
            
            gAngleBuffer[gPtr++] = gAngle;
            gPtr %= gBufferSize;
            
            //fprintf_P(&usart_stream, PSTR("Sensor Value : %i)\r\n"), ms_sensor_value);
            
        }
        //////////////////////////
        
		use_sensor_data_on = true;
		cnt4sensor++;
		
		
			
		display_on = true;


//INDICATOR//INDICATOR//INDICATOR//INDICATOR//INDICATOR//INDICATOR//INDICATOR//INDICATOR
//INDICATOR//INDICATOR//INDICATOR//INDICATOR//INDICATOR//INDICATOR//INDICATOR//INDICATOR
//INDICATOR//INDICATOR//INDICATOR//INDICATOR//INDICATOR//INDICATOR//INDICATOR//INDICATOR

//LED_PORT.OUTTGL = LED_USR_1_PIN_bm;	
//LED_PORT.OUTTGL = LED_USR_2_PIN_bm;	
	}
	
	if(jiffies%1000 == 0)
	{		
		if(communication_on) sec_counter++;
		sync = true;		//synchro bit should be set every 1 sec
		rhythm_on = true;
		sensor_value_now = 0;

		if(!communication_on) LED_PORT.OUT =  LED_USR_0_PIN_bm;
		if(communication_on)  LED_PORT.OUT = !LED_USR_0_PIN_bm;		
	}
	xgrid.process();
}

void StageInit(int StageTime, const char str[])
{
	if(sec_counter == StageTime && special)
	{
		send_message(MESSAGE_COMMAND, ALL_DIRECTION, ALL, str);
		init_variables();
	}
}

// ============================================================================================
// MAIN FUNCTION
// ============================================================================================
int main(void)
{
	float angle = 0;
	
	xgrid.rx_pkt = &rx_pkt;

	_delay_ms(50);

	// ========== INITIALIZATION ==========
    init();				//for board
	init_servo();		//for servo
	init_variables();	//for program
	init_sonar();		//for sensor

	fprintf_P(&usart_stream, PSTR("START (build number : %ld)\r\n"), (unsigned long) &__BUILD_NUMBER);
    fprintf_P(&usart_stream, PSTR("Board ID %i\r\n"), swarm_id);
    

	// ===== SONAR CHECK & Indicated by LED (attached/not = GREEN/RED) =====
	sonar_attached = check_sonar_attached();	//1:attached, 0:no

	if(sonar_attached)	{LED_PORT.OUT = LED_USR_2_PIN_bm; _delay_ms(2000);}
	else				{LED_PORT.OUT = LED_USR_0_PIN_bm; _delay_ms(2000);}

	// ===== Identification of Left Bottom Corner module =====
	// Special module is necessary 
	// 1) as a pace maker in "rhythm" mode,
	// 2) as a messanger of variable-reset signal 
	temp_time = jiffies + 2000;
	while(jiffies < temp_time)
	{
		// send dummy data
		if(sendmessage_fast)
		{
			send_message(MESSAGE_COMMAND, ALL_DIRECTION, ALL, "@");
			sendmessage_fast = false;
		}
	}
	//if(!connected[0] && connected[2] && connected[3] && !connected[4]) special = true;
	if(!connected[1] && !connected[2] && connected[4] && connected[5]) special = true;

	// #################### MAIN LOOP ####################

	while (1)
	{
		// ========== REBOOT PROCESS ==========
		if(reboot_on)
		{
			temp_time = jiffies + 3000;
			while(jiffies < temp_time){LED_PORT.OUTTGL = LED_USR_1_PIN_bm; _delay_ms(100);}
			xboot_reset();	
		}
			
		// ========== KEY INPUT ==========
		key_input();

		// ========== CALCULATION ==========
		if(sonar_attached && !communication_on)
		{
			if(sensor_value_trichk > RANGE1 && sensor_value_trichk < RANGE3)
			{
				send_message(MESSAGE_COMMAND, ALL_DIRECTION, ALL, "R");
				_delay_ms(100);
				send_message(MESSAGE_COMMAND, ALL_DIRECTION, ALL, "1");
				communication_on = true;
			}
		}
        
        
 ////////////////////////////////////////////////////        
        if(servo_motor_on)
        {
            // set the servo position
            if (special){
                set_servo_position(gAngle);
                send_new_message(MOTOR_ANGLE, ALL_DIRECTION, 1, gAngle);
            }
            else{ // use delayed value
                set_servo_position(gAngleBuffer[gPtr]);
                send_new_message(MOTOR_ANGLE, ALL_DIRECTION, 1, gAngleBuffer[gPtr]);
            }
            
            
            

            LED_PORT.OUTTGL = LED_USR_2_PIN_bm;

            servo_motor_on = false;
        }
/////////////////////////////////////////////////////
        
	}
	
	return 0;
}

