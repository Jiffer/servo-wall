
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
// disable and enable the servo
// ============================================================================================
void setServo(bool on){
    if (on == true && !servoEnabled){
        enable_servo();
        servoEnabled = true;
    }
    else if(on == false && servoEnabled){
        disable_servo();
        servoEnabled = false;
    }
}

// ============================================================================================
// get a random number within specified range
// ============================================================================================
float getRandom(float lo, float high){
    // set a new random seed
    srand(swarm_id * light_sensor + jiffies);
    return ((high - lo) * rand() / RAND_MAX) + lo;
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
    angle = curAngle + timeStep * step;
    
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
// get delayed neighbor
// time is in ms
// ============================================================================================
float getDelNeighbor(int port, int time){
    // sample rate for this buffer is 50ms
    int samples = (int)(time / 50.0);
    //fprintf_P(&usart_stream, PSTR("offset: %i\r\n"), samples);
    int tempPtr = neighborBufferPtr - samples;
    if (tempPtr < 0)
        tempPtr += NEIGHBOR_BUFFER_SIZE;

    return (float)(neighborBuffer[port][tempPtr] - 90); // storing as unsigned so -90
}

// ============================================================================================
// average the neighbors
// ============================================================================================
void mesmer(){
    setServo(true);
    float myAngle = cycle((float)randomPeriod, 45.0, 0.0);
    float weight = 20.0;
    float average = 0.0;
    
    for(int i = 0; i < 6; i++){ 
        if (abs(neighborAngles[i]) < MAX_ANGLE && connected[i]){ // neighborAngles[LEFT]
            average += neighborAngles[i];
        }
    }
    if (numConnected > 0){
        //fprintf_P(&usart_stream, PSTR("b4_avg: %f\r\n"), average);
        average = average / numConnected;
        // TODO : still not sure why these guys freq out when scale is 2.0 or more...
        float tempAngle = ((myAngle + weight * average) / (weight + 1));
   
        curAngle = (tempAngle);
    }
    else{
        curAngle = (myAngle);
    }
    
}

// ============================================================================================
// quick Sweep
// ============================================================================================
void quickSweep(){
    setServo(true);
    float newAngle = linearSweep(4.0, 0, MAX_ANGLE - 1);
    if ( newAngle > MAX_ANGLE - 1)
        newAngle = 0;
    
    curAngle = newAngle;

}


// ============================================================================================
// listener
// ============================================================================================
void listen(){
    setServo(true);
    if (presenceDetected){
        curAngle = 0.0;
    }
    else if(neighborStrength[LEFT] > 0.1 && (neighborStrength[LEFT] > neighborStrength[RIGHT]))
        curAngle = MAX_ANGLE * (1-myStrength);
    else if( neighborStrength[RIGHT] > 0.1)
        curAngle = -MAX_ANGLE * (1-myStrength);
    else
        curAngle = cycle(8, 60, 0);
}

// ============================================================================================
// twitch
// ============================================================================================
void twitch(){
    static float randomTime;
    static bool toFro;
    int lowTime = 2;
    int hiTime = 8;
    float tempAngle = 0.0;
    
    if(lastMode != currentMode){
        setServo(true);
        counterTenHz = 0;
        toFro = true;
        randomTime = getRandom(lowTime, hiTime);
        actionComplete = false;
    }
    
    if (presenceDetected){
        // wiggle
        tempAngle = cycle(0.125, 10, 0);
    }
    else if(neighborStrength[LEFT] > 0.1 && (neighborStrength[LEFT] > neighborStrength[RIGHT]))
    {
        
    }
    else if( neighborStrength[RIGHT] > 0.1)
    {
        
    }
    else
    {
        
        if (counterTenHz > 10.0 * randomTime){
            //fprintf_P(&usart_stream, PSTR("comp: %f, %f\r\n"), (float)counterTenHz, (10.0 * (randomTime + 0.5)));

            if (actionComplete == false){
                if ((float)counterTenHz < (10.0 * (randomTime + 0.25))){
                    tempAngle = cycle(0.25, 5, 0);
                }
                else // wiggle complete
                {
                    tempAngle = 0;
                    actionComplete = true;
                }
            }
            else{
                // reset next random time
                randomTime = getRandom(lowTime, hiTime);
                counterTenHz = 0;
                actionComplete = false;
            }
        }
        // else do nothing
    }
    curAngle = tempAngle;
}

// ============================================================================================
// delayed reaction
// ============================================================================================
void delayedReaction(){
    /// TODO: Write this function
    // need to know who to follow and how many samples to delay by
    // this should depend on the updateRate
    
    
    if (special){
        curAngle = cycle(9, 45 * cycle(12, 0.5, 0.5), cycle(2, 3 * cycle(9, 0.5, 0.5), 0));
    }
    else if (bottom){ // use delayed value
        curAngle = getDelNeighbor(LEFT, 500); // enum LEFT, RIGHT, ABOVE, BELOW
    }
    else
        curAngle = getDelNeighbor(BELOW, 1000);
    
}



