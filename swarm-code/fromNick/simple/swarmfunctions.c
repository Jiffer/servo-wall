// ============================================================================================
// variables
// ============================================================================================
float dt = 0.01;		// time step for differential equation (no relation with real time)
float ld=20.0, acc=20.0, cf=100.0;
float gmma1=5.0, ka1=0.1, d1=0.0, rc1=20.0, tau1=1.0;	//for synchro motion
float gmma0=1.0, ka0=1.0, d0=1.0, rc0=25.0, tau0=0.1;	//for chaotic motion
float forcex, forcey;

// ============================================================================================
// for initialization of swarm dynamics function
// ============================================================================================
void init_variables()
{
	int i;

	if(special) 
		send_message(MESSAGE_COMMAND, ALL_DIRECTION, ALL, "i");

	for(i=0;i<NUM_NEIGHBORS;i++)
	{
		agent0.neix[i] = 0.0;	agent0.neiy[i] = 0.0;
		agent1.neix[i] = 0.0;	agent1.neiy[i] = 0.0;
		connected[i] = false;
	}

	// for chaotic motion
	agent0.px = 22.0; agent0.py = 40.0; agent0.vx = 0.186; agent0.vy = -4.8; agent0.hd = 4.75;

	//for ordered motion
	if(sec_counter < STGtime4)
	{
		agent1.px =  2.7; agent1.py = 75.6; agent1.vx = 3.8;   agent1.vy = 1.3;	 agent1.hd = 0.0;
	}
	else
	{
		agent1.px = 22.0; agent1.py = 40.0; agent1.vx = 0.186; agent1.vy = -4.8; agent1.hd = 4.75;
	}

	wave_flg = false; 
	wave_ping = false;
	column_flg = false;
	rhythm_on = false;

	agent2.flg = false;

	agent3.tim2 = 20.0;

	global_amp = 0;
	
	enable_servo();
	
	set_servo_position(0);
}

// ============================================================================================
// simple Periodic Motion returns angle based on sinusoidal period (in seconds)
// and angle range (+/- range in degrees)
// ============================================================================================
float cycle(float period, float range, float offset){
    float angle;
    angle = range * sin(jiffies * 2.0 * PI /(period * 1000.0)) + offset; //
    return angle;
}

// ============================================================================================
/// calculate a linear sweep
// ============================================================================================
float linearSweep(float howFast, float startAngle, float targetAngle){
    static int lastTime;
    float angle;
    float distance = targetAngle - startAngle;
    float step = distance / (1000 * howFast);
    // how long since the last time?
    int timeStep = jiffies - lastTime;
    lastTime = jiffies;
    
    // current angle +
    angle = gAngle + timeStep * step;
    
    if (angle < MAX_ANGLE && angle > -MAX_ANGLE)
        return angle;
    else if (angle > MAX_ANGLE)
        return MAX_ANGLE;
    else if (angle < -MAX_ANGLE)
        return -MAX_ANGLE;
// hmmm
    else
        return angle;
}


// ============================================================================================
// average the neighbors
// ============================================================================================
void mesmer(){
    float myAngle = cycle((float)randomPeriod, 50.0, 0.0);
    float weight = 20.0;
    float average = 0.0;
    
    for(int i = 0; i < 6; i++){
        if (abs(neighborAngles[i]) < 90){
            average += neighborAngles[i];
            
        }

    }
    if (numConnected > 0){
        //fprintf_P(&usart_stream, PSTR("b4_avg: %f\r\n"), average);
        average = average / numConnected;
        // TODO : still not sure why these guys freq out when scale is 2.0 or more...
        float tempAngle = 1.3 * ((myAngle + weight * average) / (weight + 1));
        /*
         if(debugPrint){
            fprintf_P(&usart_stream, PSTR("avg: %f\r\n"), average);
            fprintf_P(&usart_stream, PSTR("myA: %f\r\n"), myAngle);
        }
         */
        gAngle = (tempAngle);
    }
    else{
        gAngle = (int)(myAngle);
        fprintf_P(&usart_stream, PSTR("no neighbors, myAngle: %f\r\n"), myAngle);
    }
    
}

void quickSweep(){
    float newAngle = linearSweep(1.0, 0, 45);
    if ( newAngle > 45)
        newAngle = 0;
    
    gAngle = newAngle;

}



// ============================================================================================
// delayed reaction
// ============================================================================================
void delayedReaction(){
    //            if (special){
    //                set_servo_position(gAngle);
    //                send_new_message(MOTOR_ANGLE, ALL_DIRECTION, 1, gAngle);
    //
    //            }
    //            else{ // use delayed value
    //                set_servo_position(gAngleBuffer[gPtr]);
    //                send_new_message(MOTOR_ANGLE, ALL_DIRECTION, 1, gAngleBuffer[gPtr]);
    //
    //            }
    
}



