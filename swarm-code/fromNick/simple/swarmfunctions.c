
// ============================================================================================
// for initialization of swarm dynamics function
// ============================================================================================
void init_variables()
{
    // initialize array of neighbor arrays
    for(int i; i < 6; i++){
        neighborAngles[i] = 0;
    }
    
    
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
    float myAngle = cycle((float)randomPeriod, MAX_ANGLE/2, 0.0);
    float weight = 20.0;
    float average = 0.0;
    
    for(int i = 0; i < 6; i++){
        if (abs(neighborAngles[i]) < MAX_ANGLE){
            average += neighborAngles[i];
            
        }

    }
    if (numConnected > 0){
        //fprintf_P(&usart_stream, PSTR("b4_avg: %f\r\n"), average);
        average = average / numConnected;
        // TODO : still not sure why these guys freq out when scale is 2.0 or more...
        float tempAngle = 1.3 * ((myAngle + weight * average) / (weight + 1));
   
        //fprintf_P(&usart_stream, PSTR("avg: %f\r\n"), average);
        gAngle = (tempAngle);
    }
    else{
        gAngle = (int)(myAngle);
        //fprintf_P(&usart_stream, PSTR("no neighbors, myAngle: %f\r\n"), myAngle);
    }
    
}

void quickSweep(){
    float newAngle = linearSweep(4.0, 0, MAX_ANGLE - 1);
    if ( newAngle > MAX_ANGLE - 1)
        newAngle = 0;
    
    gAngle = newAngle;

}



// ============================================================================================
// delayed reaction
// ============================================================================================
void delayedReaction(){
    /// TODO: Write this function
    // need to know who to follow and how many samples to delay by
    // this should depend on the updateRate
    
    
                if (special){
    
                }
                else if (bottom){ // use delayed value
                    gAngle = neighborAngles[2]; // enum LEFT, RIGHT, ABOVE, BELOW
    
                }
    
}



