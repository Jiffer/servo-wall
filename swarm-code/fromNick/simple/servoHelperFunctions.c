//
//  algorithmHelperFunctions.c
//  simple
//
//  Created by Jiffer Harriman on 4/16/13.
//  Copyright (c) 2013 Jiffer Harriman. All rights reserved.
//

void shufflePattern();
// ============================================================================================
// for initialization of swarm dynamics function
// ============================================================================================
void init_variables()
{
    // initialize array of neighbor arrays
    for(int i; i < 6; i++){
        //neighborAngles[i] = 0;
        neighborData[i].angleValue = 0;
    }
    
    for(int i; i < 4; i++){
        offsetVar[i] = 0.0;
    }
    
    shufflePattern();
    
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
    srand(swarm_id * my_sensor_value + jiffies);
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
// randomish shake
// ============================================================================================
float wiggle(){
    // a modulated shake
    return (2 * cycle(5.3 + offsetVar[1]) + 8) * cycle(0.213 + offsetVar[2], cycle(1.03 + offsetVar[3])) ;
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
    static unsigned long lastTime;
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
    // sample rate for this buffer is 30ms
    int samples = (int)(time / 30.0);
    //fprintf_P(&usart_stream, PSTR("offset: %i\r\n"), samples);
    int tempPtr = neighborBufferPtr - samples;
    if (tempPtr < 0)
        tempPtr += NEIGHBOR_BUFFER_SIZE;
    
    if (samples == 0)
        return neighborData[port].angleValue; // neighborAngles[port];
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
// shuffle pattern
// http://en.wikipedia.org/wiki/Fisher-Yates_shuffle
// ============================================================================================
// for i from n − 1 downto 1 do
// j ← random integer with 0 ≤ j ≤ i
// exchange a[j] and a[i]
//
void shufflePattern(){
    // first choose between 1 and 4 beats active
    int numActiveBeats = (int)getRandom(1, 5);
    
    
    for (int i = 0; i < numBeats; i++){
        if (i < numActiveBeats)
            beatPattern[i] = 1;
        else
            beatPattern[i] = 0;
    }
    for (int i = numBeats-1; i > 0; i--){
        int j = (int)getRandom(0, i);
        int tempHolder = beatPattern[j];
        beatPattern[j] = beatPattern[i];
        beatPattern[i] = tempHolder;
        //fprintf_P(&usart_stream, PSTR("i %i, \r\n"), i, beatPattern[i]);
    }
    
}


// ============================================================================================
// start a linear cross fade from current position to some new set of calculations
// fade increment is how much to move the slider (goes from 0.0 to 1.0) each 1/100th of a second
// i.e. startXFade(0.01) will take 1 second to transition
// ============================================================================================
void startXFade(float fadeInc){
    fadeIncrement = fadeInc;
    transitionAngle = curAngle;
    crossFade = 0.0;
    inTransition = true;
}

