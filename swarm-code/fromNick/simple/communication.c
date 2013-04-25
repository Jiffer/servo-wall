

// ============================================================================================
// send message packet
// ============================================================================================
void send_message(uint8_t MessageType, uint8_t direction, int dist, const char str[])
{
	Xgrid::Packet pkt;
	pkt.type = MessageType;
	pkt.flags = 0;	

	switch(MessageType)
	{
		case MESSAGE_COMMAND:

			pkt.data = (uint8_t*)str;
			pkt.radius = dist;
			pkt.data_len = sizeof(str);
			break;

	}
									
	xgrid.send_packet(&pkt, direction);	// send to all neighbors
}


// ============================================================================================
// Send angle packet
// Send Angle data and strength of message to neighbors
// Sends to all ports
// ============================================================================================
void send_neighbor_data(float angle, float strength)
{
	Xgrid::Packet pkt;
	pkt.type = NEIGHBOR_DATA;
	pkt.flags = 0;
	pkt.radius = 1;
	
    sendData.angleValue = angle;
    sendData.sensorValue = sensor_value;
    sendData.strength = strength; //(float)sensor_value / 4096.0;
    sendData.fromDir = strengthDir;
    
	pkt.data = (uint8_t *)&sendData;
    
	pkt.data_len = sizeof(NeighborData);
	
	xgrid.send_packet(&pkt, ALL_DIRECTION);
}

// ============================================================================================
// Receive Packet
// Parse input packet
// ============================================================================================
void rx_pkt(Xgrid::Packet *pkt)
{
    if (pkt->type == _TIME_CALIB)
    {
        if (swarm_id == _MAIN_BOARD)
        {
        }
        
        if (swarm_id != _MAIN_BOARD)
        {
            CalibInfo* lg_ptr= (CalibInfo*)pkt->data;
            if (calib_switch)
            {
                calib_switch = false; // TODO: check if _calib_times is greater than local calib_times?
                calib_times = lg_ptr->_calib_times;
                lg_ptr->_jiffies += _DELAY_CALIB +1;
                jiffies = lg_ptr->_jiffies;
               
                //xgrid.send_packet(pkt,0b00111111);						// for test
                xgrid.send_packet(pkt,(0b00111111 & ~(1<<pkt->rx_node)));	// for real
                LED_PORT.OUTTGL = LED_USR_1_PIN_bm;
            }	
        }
    }

    
    // Neighbors angle and sensor
    if (pkt->type == NEIGHBOR_DATA) {
        uint8_t port = pkt->rx_node;
        
        if(port >= 0 && port < NUM_NEIGHBORS){
            // keep track of who I have recieved messages from
            connected[port] = true;
            
            NeighborData* recvNeighborPtr = (NeighborData*) pkt->data;
            //neighborAngles[port] = recvNeighborPtr->angleValue;
            
            
            if (port == recvNeighborPtr->fromDir || (recvNeighborPtr->fromDir == MOOT && (port == LEFT || port == RIGHT))){
                neighborData[port] = *recvNeighborPtr;

            }
            else{ // don't update strength unless direction matches port
                neighborData[port].angleValue = recvNeighborPtr->angleValue;
                neighborData[port].sensorValue = recvNeighborPtr->sensorValue;
            }
            
            // for columns set sensor value to that of the bottom board in the column
            if (port == BELOW){
                sensor_value = recvNeighborPtr->sensorValue;
            }
        }
    }

        /// parsing broadcast commands from neighbors
           
        // ============================================================================================
    if(pkt->type == MESSAGE_COMMAND)
    {
        char command;
        //fprintf_P(&usart_stream, PSTR("got MESSAGE_COMMAND\r\n"));
        
        uint8_t port = pkt->rx_node;
        
        char* char_ptr = (char*) pkt->data;
        command = *char_ptr;
        
        
        connected[port] = true;
        switch(command)
        {
            case 'Z': reboot_on = true;

                
            // jif
            case 'c': cycleOn = true; break;
            case 'o': cycleOn = false; break;

            case '0':   currentMode = TOGETHER; break;
            case '1':   currentMode = SINY; break;
            case '2':   currentMode = FM; break;
            case '3':   currentMode = AM; break;
            case '4':   currentMode = MESMER; break;
            case '5':   currentMode = SWEEP; break;
            case '6':   currentMode = TWITCH; break;
            case '7':   currentMode = TWITCH_WAVE; break;
            case '9':   currentMode = BREAK; break;
            case '-':   currentMode = ZERO; break;
                
            // sensor modes
            case 'q':   presMode = POINT; break;
            case 'w':   presMode = SHAKE; break;
            case 'e':   presMode = WAVE; break;
            case 'r':   presMode = RANDOM; break;
            case 't':   presMode = RATE; break;
            case 'y':   presMode = RHYTHM; break;
            case 'i':   presMode = IGNORE; break;
                
                
            case 'u':
                offsetVarIndex++;
                offsetVarIndex %= NUMOFFSET;
                break;
                
            case 'a':
                offsetVar[offsetVarIndex] -= 1.0;
                break;
            case 's':
                offsetVar[offsetVarIndex] += 1.0;
                break;
                
            case 'g':
                offsetVar[offsetVarIndex] -= 0.1;
                break;
            case 'h':
                offsetVar[offsetVarIndex] += 0.1;
                break;
                
            case '[':
                amplitudeScaler -= 0.05;
                break;
            case ']':
                amplitudeScaler += 0.05;
                break;
        }
    }
    
}

// ============================================================================================
// KEY INPUT PROCESS
// ============================================================================================
void key_input()
{
	char input_char = 0;
	
	if (usart.available()) input_char = usart.get();
	
	if(input_char == 0x1b) xboot_reset(); //reboot the board

	if(input_char == 'Z')	//Reboot whole system
	{
		send_message(MESSAGE_COMMAND, ALL_DIRECTION, ALL, "Z");
		temp_time = jiffies + 3000;
		while(jiffies < temp_time)
		{
			//LED_PORT.OUTTGL = LED_USR_1_PIN_bm; _delay_ms(100);
		}
		xboot_reset();
	}


    // print version number
	if(input_char == 'v')
		fprintf_P(&usart_stream, PSTR("build number = %ld\r\n"), (unsigned long) &__BUILD_NUMBER);




    
    
    // ============================================================================================
    /// currentMode commands
    // ============================================================================================
    if(input_char == '0'){
        fprintf_P(&usart_stream, PSTR("setting currentMode to TOGETHER\n"));
        currentMode = TOGETHER;
        send_message(MESSAGE_COMMAND, ALL_DIRECTION, ALL, "0");
    }
    if(input_char == '1'){
        fprintf_P(&usart_stream, PSTR("setting currentMode to SINY\n"));
        currentMode = SINY;
        send_message(MESSAGE_COMMAND, ALL_DIRECTION, ALL, "1");
    }
    if(input_char == '2'){
        fprintf_P(&usart_stream, PSTR("setting currentMode to FM\n"));
        currentMode = FM;
        send_message(MESSAGE_COMMAND, ALL_DIRECTION, ALL, "2");
    }
    if(input_char == '3'){
        fprintf_P(&usart_stream, PSTR("setting currentMode to AM\n"));
        currentMode = AM;
        send_message(MESSAGE_COMMAND, ALL_DIRECTION, ALL, "3");
    }
    if(input_char == '4'){
        fprintf_P(&usart_stream, PSTR("setting currentMode to MESMER\n"));
        currentMode = MESMER;
        send_message(MESSAGE_COMMAND, ALL_DIRECTION, ALL, "4");
    }
    if(input_char == '5'){
        fprintf_P(&usart_stream, PSTR("setting currentMode to linear SWEEP\n"));
        currentMode = SWEEP;
        send_message(MESSAGE_COMMAND, ALL_DIRECTION, ALL, "5");
    }
    if(input_char == '6'){
        fprintf_P(&usart_stream, PSTR("setting currentMode to TWITCH\n"));
        currentMode = TWITCH;
        send_message(MESSAGE_COMMAND, ALL_DIRECTION, ALL, "6");
    }
    if(input_char == '7'){
        fprintf_P(&usart_stream, PSTR("setting currentMode to TWITCH_WAVE\n"));
        currentMode = TWITCH_WAVE;
        send_message(MESSAGE_COMMAND, ALL_DIRECTION, ALL, "7");
    }
    if(input_char == '9'){
        fprintf_P(&usart_stream, PSTR("setting currentMode to linear BREAK\n"));
        currentMode = BREAK;
        send_message(MESSAGE_COMMAND, ALL_DIRECTION, ALL, "9");
    }
    if(input_char == '-'){
        fprintf_P(&usart_stream, PSTR("setting currentMode to linear ZERO\n"));
        currentMode = ZERO;
        send_message(MESSAGE_COMMAND, ALL_DIRECTION, ALL, "-");
    }
    
    // cycle all
    if(input_char == 'c'){
        cycleOn = true;
        fprintf_P(&usart_stream, PSTR("'c' - cycle all modes\n"));
        send_message(MESSAGE_COMMAND, ALL_DIRECTION, ALL, "c");

    }
    if(input_char == 'o'){
        cycleOn = false;
        fprintf_P(&usart_stream, PSTR("'o' - cycle all modes off\n"));
        send_message(MESSAGE_COMMAND, ALL_DIRECTION, ALL, "o");
    }
    
    // change presence Mode commands
    if(input_char == 'q'){
        presMode = POINT;
        fprintf_P(&usart_stream, PSTR("presenceMode = %i, POINT\n"), presMode);
        send_message(MESSAGE_COMMAND, ALL_DIRECTION, ALL, "q");
    }
    if(input_char == 'w'){
        presMode = SHAKE;
        fprintf_P(&usart_stream, PSTR("presenceMode = %i, SHAKE\n"), presMode);
        send_message(MESSAGE_COMMAND, ALL_DIRECTION, ALL, "w");
    }
    if(input_char == 'e'){
        presMode = WAVE;
        fprintf_P(&usart_stream, PSTR("presenceMode = %i, WAVE\n"), presMode);
        send_message(MESSAGE_COMMAND, ALL_DIRECTION, ALL, "e");
    }
    if(input_char == 'r'){
        presMode = RANDOM;
        fprintf_P(&usart_stream, PSTR("presenceMode = %i, RANDOM\n"), presMode);
        send_message(MESSAGE_COMMAND, ALL_DIRECTION, ALL, "r");
    }
    if(input_char == 'RATE'){
        presMode = POINT;
        fprintf_P(&usart_stream, PSTR("presenceMode = %i, RATE\n"), presMode);
        send_message(MESSAGE_COMMAND, ALL_DIRECTION, ALL, "t");
    }
    if(input_char == 'y'){
        presMode = RHYTHM;
        fprintf_P(&usart_stream, PSTR("presenceMode = %i, RHYTHM\n"), presMode);
        send_message(MESSAGE_COMMAND, ALL_DIRECTION, ALL, "y");
    }
    
    if(input_char == 'i'){
        presMode = IGNORE;
        fprintf_P(&usart_stream, PSTR("'i' presenceMode = %i, IGNORE\n"), presMode);
        send_message(MESSAGE_COMMAND, ALL_DIRECTION, ALL, "i");
    }

    
    /// offset variable for debug
    if(input_char == 'u'){
        offsetVarIndex++;
        offsetVarIndex %= NUMOFFSET;
        fprintf_P(&usart_stream, PSTR("offsetVarIndex: %i, offset: %f\n"), offsetVarIndex, offsetVar[offsetVarIndex]);
        send_message(MESSAGE_COMMAND, ALL_DIRECTION, ALL, "u");
    }
    
    if(input_char == 'a'){
        offsetVar[offsetVarIndex] -= 1.0;
        fprintf_P(&usart_stream, PSTR("index: %i, offset: %f\n"), offsetVarIndex, offsetVar[offsetVarIndex]);
        send_message(MESSAGE_COMMAND, ALL_DIRECTION, ALL, "a");
    }
    if(input_char == 's'){
        offsetVar[offsetVarIndex] += 1.0;
        fprintf_P(&usart_stream, PSTR("index: %i, offset: %f\n"), offsetVarIndex, offsetVar[offsetVarIndex]);
        send_message(MESSAGE_COMMAND, ALL_DIRECTION, ALL, "s");
    }
    if(input_char == 'g'){
        offsetVar[offsetVarIndex] -= 0.1;
        fprintf_P(&usart_stream, PSTR("index: %i, offset: %f\n"), offsetVarIndex, offsetVar[offsetVarIndex]);
        send_message(MESSAGE_COMMAND, ALL_DIRECTION, ALL, "g");
    }
    if(input_char == 'h'){
        offsetVar[offsetVarIndex] += 0.1;
        fprintf_P(&usart_stream, PSTR("index: %i, offset: %f\n"), offsetVarIndex, offsetVar[offsetVarIndex]);
        send_message(MESSAGE_COMMAND, ALL_DIRECTION, ALL, "h");
    }
    
    if(input_char == '['){
        amplitudeScaler -= 0.05;
        fprintf_P(&usart_stream, PSTR("scaler: %f\n"), amplitudeScaler);
        send_message(MESSAGE_COMMAND, ALL_DIRECTION, ALL, "[");
    }
    if(input_char == ']'){
        amplitudeScaler += 0.05;
        fprintf_P(&usart_stream, PSTR("scaler: %f\n"), amplitudeScaler);
        send_message(MESSAGE_COMMAND, ALL_DIRECTION, ALL, "]");
    }

    
    // ============================================================================================
    /// toggle debug print
    // ============================================================================================
    if(input_char == 'P'){
        fprintf_P(&usart_stream, PSTR("'P' - toggling debug printf on/off \n"));
        debugPrint = !debugPrint;
    }

}


