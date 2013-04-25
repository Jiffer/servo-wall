//
//  presenceFunctions.c
//  simple
//
//  Created by Jiffer Harriman on 4/12/13.
//  Copyright (c) 2013 Jiffer Harriman. All rights reserved.
//
#include <stdio.h>

// ============================================================================================
// presence detected functions
// all functions return a bool indicating if they have updated the curAngle variable or not
// based on presence in the system
// ============================================================================================

// ============================================================================================
// point at presence
// ============================================================================================
bool point(){
    bool updated = false;
    if(newPresence){
        newPresence = false;
        fprintf_P(&usart_stream, PSTR("setFalse\r\n"));
    }
    
    if (presenceDetected && !ignorePresence){
        curAngle = 0.0;
        updated = true;
    }
    
    else if(myStrength > STRENGTH_THRESHOLD && (!ignoreNeighborPresence && (neighborPresenceDetected || neighborPresenceDetectedLast)))
    {
        updated = true;
        if(lastStrengthDir == LEFT)
            curAngle = MAX_ANGLE * (1-myStrength);
        else
            curAngle = -MAX_ANGLE * (1-myStrength);
            
    }
    return updated;
}

// ============================================================================================
// jump to random angle when new presence is detected
// ============================================================================================
bool randomAngle(){
    bool updated = false;
    static float randAngle;
    
    if (newPresence){
        randAngle = getRandom(-MAX_ANGLE, MAX_ANGLE);
        newPresence = false;
    }
    
    if (presenceDetected && !ignorePresence)
    {
        curAngle = randAngle;
        updated = true;
    }
    return updated;
}

// ============================================================================================
// back and forth with 1/4 wave sin wave sweep shaped rhythms
// ============================================================================================
bool toFro(){
    bool updated = false;
    int degreesPerBeat;
    static int direction = 1;
    static float startAngle = 0;
    
    if(currentBeat == 0)
        degreesPerBeat = 20;
    else
        degreesPerBeat = 12;
    
    if(newPresence){
        shufflePattern();
        newPresence = false;
    }
    // quantize to something...
    
    if (currentBeat != lastBeat){
        lastBeat = currentBeat;
        // counterFiftyHz = 0;
        startAngle = curAngle;
        if (abs(curAngle + direction * degreesPerBeat * beatPattern[currentBeat]) > MAX_ANGLE){
            direction *= -1;
        }
    }
    //fprintf_P(&usart_stream, PSTR("beat: %i\r\n"), currentBeat);
    if(presenceDetected && !ignorePresence){
        if(bottom){
            curAngle = constrainAngle( startAngle + beatPattern[currentBeat] * direction * degreesPerBeat * sinSweep(0.2));
            updated = true;
        }
        else{
            // curAngle = neighborAngles;
            curAngle = neighborData[BELOW].angleValue;
            updated = true;
        }
    }
    return updated;
}

// ============================================================================================
// follow action at presence column
// ============================================================================================
bool presenceWave(bool wave){
    bool updated = false;
    
    if(newPresence)
        newPresence = false;
    
    // wave still propogating vars
    static int startTime = 0;
    static bool reverb = false;
    

    if(neighborData[LEFT].strength > STRENGTH_THRESHOLD || neighborData[RIGHT].strength > STRENGTH_THRESHOLD){
        startTime = sec_counter;
        reverb = true;
    }
    else if(startTime + 5 < sec_counter){ // wave lasts 5 seconds after presense is no longer there
        reverb = false;
    }
    
    // my coloumn is active
    if (presenceDetected && !ignorePresence){
        // wiggle
        curAngle = wiggle();
        updated = true;
    }
    
    // presense is detected somewhere in the system
    else if(reverb){
        
        if(wave){
            if(lastStrengthDir == RIGHT) // direction it is moving not where it came from
            {
                curAngle = lastStrength * getDelNeighbor(RIGHT, 600 + offsetVar[0] * 100);
                updated = true;
            }
            else if(lastStrengthDir == LEFT)
            {
                curAngle = lastStrength * getDelNeighbor(LEFT, 600 + offsetVar[0] * 100);
                updated = true;
            }
        }
    }
    
    return updated;
}

// ============================================================================================
// randomly choose a rhythmic update rate
// ============================================================================================
bool randomUpdateRate(){
    bool updated = false;
    if(newPresence){
        updateRate = (int)getRandom(FOUR_HUNDRED, EIGHT_HUNDRED);
        newPresence = false;
    }
    
    if (presenceDetected && !ignorePresence)
        updated = false;
    else
        updateRate = SMOOTH;
    
    return updated;
}

// ============================================================================================
// switches between all the sensor modes
// if  behavior has been altered by sensor interaction returns true (except RATE)
// ============================================================================================
bool sensorBehavior(){
    bool updated = false;
    
    if (presMode != lastPresMode){
        if (presenceDetected && !ignorePresence){
            // smooth transition when changing presence modes
            fprintf_P(&usart_stream, PSTR("changePresMode\r\n"));
            startXFade(0.01);
        }
        
        switch(presMode){
            case POINT:
                presenceTimeOut = 6; // in seconds
                neighborPresenceTimeOut = presenceTimeOut;
                usingNeighborPresence = true;
                break;
        
            case RANDOM:
                presenceTimeOut = 8;
                neighborPresenceTimeOut = presenceTimeOut;
                usingNeighborPresence = false;
                break;
        
            case RATE:
                presenceTimeOut = 20;
                neighborPresenceTimeOut = presenceTimeOut;
                usingNeighborPresence = false;
                break;
        
            case SHAKE:
                presenceTimeOut = 8;
                neighborPresenceTimeOut = presenceTimeOut;
                usingNeighborPresence = false;
                break;
        
            case WAVE:
                presenceTimeOut = 4;
                neighborPresenceTimeOut = presenceTimeOut;
                usingNeighborPresence = true;
                break;
        
            case RHYTHM:
                presenceTimeOut = 40;
                neighborPresenceTimeOut = presenceTimeOut;
                usingNeighborPresence = false;
                break;
        }
    }
    
    // because for rate there is no change of angle just update rate so don't need to transition
    if(newPresence && presMode != RATE && presMode != IGNORE){
        fprintf_P(&usart_stream, PSTR("newPres\r\n"));
        startXFade(0.1);
    }
    
    if (newNeighborPresence && usingNeighborPresence){
        newNeighborPresence = false;
        switch(presMode){
            case POINT:
                fprintf_P(&usart_stream, PSTR("new neighbor\r\n"));

                startXFade(0.05);
                break;
                
        }
    }
    
    switch(presMode)
    {
        case POINT:
            updated = point();
            break;
            
        case SHAKE:
            updated = presenceWave(false);
            break;
            
        case WAVE:
            updated = presenceWave(true);
            break;
            
        case RANDOM:
            updated = randomAngle();
            break;
            
        case RATE:
            updated = randomUpdateRate();
            break;
            
        case RHYTHM:
            updated = toFro();
            
        case IGNORE: // do nothing
            break;
    }
    
    return updated;
}





