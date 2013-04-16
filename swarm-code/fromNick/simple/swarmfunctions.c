// ============================================================================================
// presence detected functions
// ============================================================================================
void toFro();

// ============================================================================================
// ============================================================================================
// ============================================================================================
// ============================================================================================


// ============================================================================================
// for initialization of swarm dynamics function
// ============================================================================================
void init_variables()
{
    // initialize array of neighbor arrays
    for(int i; i < 6; i++){
        neighborAngles[i] = 0;
    }
    for(int i; i < MAX_BEATS; i++){
        beatPattern[i] = 0;
    }
    beatPattern[0] = 1;
    beatPattern[1] = 1;
    beatPattern[3] = 1;
    
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

// version with just period will go from -1.0 to 1.0
float cycle(float period){
    float angle;
    angle = sin(jiffies * 2.0 * PI /(period * 1000.0)); //
    return angle;
}

// version with just period will go from -1.0 to 1.0
float cycle(float period, float phase){
    float angle;
    angle = sin(jiffies * 2.0 * PI /(period * 1000.0) + phase); //
    return angle;
}

// ============================================================================================
// sinSweep - sweeps from 0 to 1, 1/4 of a sin wave
// reset counter50Hz to 0 before starting a new sweep
// ============================================================================================
float sinSweep(float sweepTime){
    float angle;
    
    // sweepTime * 4 will get through 1/4 of a cycle in the specified time
    // when it gets to pi/2 its 1/4 way through
    float quarterCycle = 50.0 * sweepTime;  // for 50Hz counter
    if (counterFiftyHz <= quarterCycle)
        angle = sin(counterFiftyHz * 2.0 * PI /((sweepTime * 4) * 50));
    else{
        angle = 1.0;
        actionComplete = true;
    }
    
    //fprintf_P(&usart_stream, PSTR("sweepAng: %f\r\n"), angle);
    return angle;
}


// ============================================================================================
// constrain angle functions
// ============================================================================================
float constrainAngle(float angle){
    if (angle > MAX_ANGLE)
        angle = MAX_ANGLE;
    else if (angle < -MAX_ANGLE)
        angle = -MAX_ANGLE;
    
    return angle;
}

float constrainAngle(float angle, float min, float max){
    if (angle > max)
        angle = max;
    else if (angle < min)
        angle = min;
    
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
    // sample rate for this buffer is 20ms
    int samples = (int)(time / 20.0);
    //fprintf_P(&usart_stream, PSTR("offset: %i\r\n"), samples);
    int tempPtr = neighborBufferPtr - samples;
    if (tempPtr < 0)
        tempPtr += NEIGHBOR_BUFFER_SIZE;
    
    if (samples == 0)
        return neighborAngles[port];
    else
        return (float)(neighborBuffer[port][tempPtr] - 90); // storing as unsigned so -90
}

// ============================================================================================
// I'm the Map!
// ============================================================================================
float map(float mapSignal, float loIn, float hiIn, float loOut, float hiOut){
    float scale, offset;
    
    float inputRange = hiIn - loIn;
    float outputRange = hiOut - loOut;
    
    scale = outputRange / inputRange;
    offset = outputRange / 2 + loOut;
    
    return (scale * mapSignal + offset);
}

// ============================================================================================
// presense wave - follow lead of wherever presense is detected by specified delay amount
// ============================================================================================
float presenceWave(float thresh, int delAmount){
    float tempAngle = 0;
    if(neighborStrength[LEFT] > thresh && (neighborStrength[LEFT] > neighborStrength[RIGHT]))
    {
        tempAngle = getDelNeighbor(LEFT, delAmount);
    }
    else if( neighborStrength[RIGHT] > thresh)
    {
        tempAngle = getDelNeighbor(RIGHT, delAmount);
    }
    
    return tempAngle;
}

// ============================================================================================
// average the neighbors
// ============================================================================================
void mesmer(){
    float myAngle = cycle((float)randomPeriod, 45.0, 0.0);
    //float weight = 20.0;
    float weight = cycle(20, 10, 10);
    float average = 0.0;
    float scaling = 6.0;
    static float randAngle;
    
    if (presenceDetected){
        if (newPresence){
            randAngle = getRandom(-MAX_ANGLE, MAX_ANGLE);
        }
        // wiggle
        //curAngle =  cycle(0.25, 10, 0);
        //curAngle =  (2 * cycle(5.3) + 8) * cycle(0.2, cycle(1)) ;
        
        //curAngle = randAngle;
        
        toFro();
    }
    
    else{
    for(int i = 0; i < 6; i++){ 
        if (abs(neighborAngles[i]) < MAX_ANGLE && connected[i]){ // neighborAngles[LEFT]
            average += (neighborAngles[i]);
            }
        }
        if (numConnected > 0){
            //fprintf_P(&usart_stream, PSTR("b4_avg: %f\r\n"), average);
            average = average / numConnected;
            float tempAngle = ((myAngle + weight * average) / (weight + 1)/scaling);
   
            curAngle = constrainAngle(tempAngle * scaling);
        }
        else{
            curAngle = constrainAngle(myAngle);
        }
    }
}

// ============================================================================================
// quick Sweep
// ============================================================================================
void quickSweep(){
    float newAngle = linearSweep(4.0, 0, MAX_ANGLE - 1);
    if ( newAngle > MAX_ANGLE - 1 || newAngle < 0)
        newAngle = 0;
    
    curAngle = newAngle;
}


// ============================================================================================
// listener
// ============================================================================================
void listen(){
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
void twitch(bool wave){
    static float randomTime;
    int lowTime = 2;
    int hiTime = 8;
    float tempAngle = 0.0;
    
    static int startTime;
    static bool reverb;
    
    if(lastMode != currentMode){
        setServo(true);
        
        // random twitch variables
        counterTenHz = 0;
        randomTime = getRandom(lowTime, hiTime);
        actionComplete = false;
        
        // wave still propogating vars
        startTime = 0;
        reverb = false;
    }
    
    if(neighborData[LEFT].strength > 0.01 || neighborData[RIGHT].strength > 0.01){
        startTime = sec_counter;
        reverb = true;
        //fprintf_P(&usart_stream, PSTR("time: %u,\r\n"), startTime);
    }
    else if(startTime + 5 < sec_counter){ // wave lasts 5 seconds after presense is no longer there
        reverb = false;
        //fprintf_P(&usart_stream, PSTR("disabling: %u,\r\n"), sec_counter);
    }
    
    
    // my coloumn is active
    if (presenceDetected){
        // wiggle
        tempAngle = cycle(0.25, 10, 0);
    }
    
    // presense is detected somewhere in the system
    else if(reverb){
        
        if(wave){
            if(lastStrengthDir == RIGHT) // direction it is moving not where it came from
            {
                tempAngle = lastStrength * getDelNeighbor(RIGHT, 1200);
            }
            else if(lastStrengthDir == LEFT)
            {
                tempAngle = lastStrength * getDelNeighbor(LEFT, 1200);
            }
        }
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

    if (special){
        //curAngle = cycle(9, 45 * cycle(12, 0.5, 0.5), 0);

        //curAngle = cycle(9 + tempPeriod, 45, 0);
        //curAngle = 45 * sin((jiffies * 2.0 * PI /(9.0 * 1000.0)) + sin(jiffies * 2.0 * PI /(23.0 * 1000.0)));
        //curAngle = 45 * cycle(9, map(cycle(21), -1, 1, -3, 3));

        curAngle = 45 * cycle(9, cycle(21));
}
    
    else if (bottom){ // use delayed value
        //curAngle = getDelNeighbor(LEFT, 500); // enum LEFT, RIGHT, ABOVE, BELOW
        //float delayAmount = 500 + 500 * sin(jiffies * 2.0 * PI /(20 * 1000.0));
        //fprintf_P(&usart_stream, PSTR("del: %f\r\n"), delayAmount);
        curAngle = getDelNeighbor(LEFT, cycle(20, 500, 500));
    }
    
    else
        curAngle = neighborAngles[BELOW];
    
}

// ============================================================================================
// to and fro with 1/4 sin wave sweep
// ============================================================================================
void toFro(){
    int degreesPerBeat = 20;
    static int direction = 1;
    static float startAngle = 0;
// quantize to something...
    
    if (currentBeat != lastBeat){
        lastBeat = currentBeat;
        counterFiftyHz = 0;
        startAngle = curAngle;
        if (abs(curAngle + direction * degreesPerBeat) > MAX_ANGLE){
            direction *= -1;
        }
    }
    //fprintf_P(&usart_stream, PSTR("beat: %i\r\n"), currentBeat);
    float tempAngle = beatPattern[currentBeat] * direction * degreesPerBeat * sinSweep(0.2);

    curAngle = constrainAngle( startAngle + tempAngle );
    
//    if (counterFiftyHz >=  0.2 * 50){
//        toOrFro = !toOrFro;
//        counterFiftyHz = 0;
//    }
}

// ============================================================================================
// switches between all the servo modes
// ============================================================================================
void servoBehavior(){
    // when new mode starts
    // initialize
    if(lastMode != currentMode){
        if (currentMode == BREAK)
            setServo(false);
        else{
            // start a transition 
            transitionAngle = curAngle;
            crossFade = 0.0;
            inTransition = true;

            // enable servo
            setServo(true);
            updateRate = SMOOTH; // for modes which use other updateRate set below in switch
            
            switch(currentMode)
            {
                case PERIODIC:
                    randomPeriod = getRandom(2.0, 8.0);
                    updateRate = (int)getRandom(SMOOTH, FOUR_HUNDRED);
                    break;
                case MESMER:
                    randomPeriod = getRandom(4.0, 20.0);
                    break;
                    
            }
        }
    }
    
    switch(currentMode)
    {
        case BREAK: // do nothing
            break;
            
        case TOGETHER:
            curAngle = cycle(5, 45.0, 0);
            break;
            
        case PERIODIC:
            curAngle = cycle(randomPeriod, 45.0, 0.0);
            break;
            
        case MESMER:
            mesmer();
            break;
            
        case LISTEN:
            listen();
            break;
            
        case TWITCH:
            twitch(false);
            break;
            
        case TWITCH_WAVE:
            twitch(true);
            break;
            
        case SWEEP:
            quickSweep();
            break;
            
        case DELAYED:
            delayedReaction();
            break;
            
        case FM_TOGETHER:
            curAngle = 45 * cycle(9, cycle(21));
            break;
            
        case AM_TOGETHER:
            curAngle = 45 * cycle(10) * cycle(2);
            break;

    }
    
    if (!inTransition){
        set_servo_position(curAngle);
    }
    else // in transition
    {
        curAngle = crossFade * curAngle + (1.0 - crossFade) * transitionAngle;
        set_servo_position(curAngle);
    }

}


