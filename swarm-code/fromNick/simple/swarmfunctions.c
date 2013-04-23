#include "servoHelperFunctions.c"
#include "presenceFunctions.c"

// ============================================================================================
// ============================================================================================


// ============================================================================================
// average the neighbors
// ============================================================================================
void mesmer(){
    float myAngle = cycle((float)randomPeriod, 45.0, 0.0);
    //float weight = 20.0;
    float weight = cycle(20, 10, 10);
    float average = 0.0;
    float scaling = 6.0;
    
    if(!sensorBehavior()){
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
    if(!sensorBehavior()){
        float newAngle = linearSweep(4.0 + offsetVar[1], 0, MAX_ANGLE - 1);
        if ( newAngle > MAX_ANGLE - 1 || newAngle < 0)
            newAngle = 0;
    
        curAngle = newAngle;
    }
}


// ============================================================================================
// twitch
// ============================================================================================
void twitch(){
    static float randomTime;
    int lowTime = 2;
    int hiTime = 8;
    float tempAngle = 0.0;
    
    if(lastMode != currentMode){
        setServo(true);
        
        // random twitch variables
        counterTenHz = 0;
        randomTime = getRandom(lowTime, hiTime);
        actionComplete = false;
    }
    

    if(!sensorBehavior())
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
        curAngle = tempAngle;
    }
    
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
/*        switch(delayFunction){
            case FM:
                curAngle = 45 * cycle(9, cycle(21 + offsetVar[0]));
                break;
            case AM:
                curAngle = 45 * cycle(9) * cycle(4.5);
                break;
                
            case CHOP:*/
                float newAngle = linearSweep(4.0 + offsetVar[1], 0, MAX_ANGLE - 1);
                if ( newAngle > MAX_ANGLE - 1 || newAngle < 0)
                    newAngle = 0;
                
                curAngle = newAngle;

                //break;
        //}
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
            startXFade(0.01);
            fprintf_P(&usart_stream, PSTR("newAlgoMode\r\n"));
            // where presence is used ignore when first entering new mode
            newPresence = false;

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
                    
                case DELAYED:
                    curAngle = 0;
                    delayFunction = (int)getRandom(0.0, 0.99);
                    break;
                    
            }
        }
    }
    
    switch(currentMode)
    {
        case BREAK: // do nothing
            break;
            
        case TOGETHER:
            if(!sensorBehavior())
                curAngle = amplitudeScaler * cycle(5 + offsetVar[0], 45.0, 0);
            break;
            
        case PERIODIC:
            if(!sensorBehavior())
                curAngle = amplitudeScaler * cycle(randomPeriod, 45.0, 0.0);
            break;
            
        case MESMER:
            mesmer();
            break;
            
        case TWITCH:
            twitch();
            break;
            
        case TWITCH_WAVE:
            twitch();
            break;
            
        case SWEEP:
            quickSweep();
            break;
            
        case DELAYED:
            if(!sensorBehavior())
                delayedReaction();
            break;
            
        case FM_TOGETHER:
            if(!sensorBehavior())
                curAngle = amplitudeScaler * 45 * cycle(9 + offsetVar[2], cycle(21 + offsetVar[3]));
            break;
            
        case AM_TOGETHER:
            if(!sensorBehavior())
                curAngle = amplitudeScaler * 45 * cycle(10 + offsetVar[2]) * cycle(2 + offsetVar[3]);
            break;

    }
    
    if (!inTransition){
        set_servo_position(curAngle);
    }
    else // in transition
    {
        //fprintf_P(&usart_stream, PSTR("x-fade: %f, %f\r\n"), curAngle, transitionAngle);
        curAngle = crossFade * curAngle + (1.0 - crossFade) * transitionAngle;
        set_servo_position(curAngle);
        

    }

}


