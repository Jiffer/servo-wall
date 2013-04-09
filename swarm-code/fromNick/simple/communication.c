

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
	
    neighborData.angleValue = angle;
    neighborData.sensorValue = sensor_value;
    neighborData.strength = strength; //(float)sensor_value / 4096.0;
    neighborData.fromDir = strengthDir;
    
	pkt.data = (uint8_t *)&neighborData;
    
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
            neighborAngles[port] = recvNeighborPtr->angleValue;
            neighborSensors[port] = recvNeighborPtr->sensorValue;
            neighborStrength[port] = recvNeighborPtr->strength;
            neighborStrengthDir[port] = recvNeighborPtr->fromDir;
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
            case 'r':
                sec_counter = 0;
                jiffies = 0;    break;
                
            // jif
            case 'c': cycleOn = true; break;
            case 'o': cycleOn = false; break;

            case 'n':   updateRate = NONE; break;
            case '0':   updateRate = SMOOTH; break;
            case '1':   updateRate = ONE_HUNDRED; break;
            case '2':   updateRate = TWO_HUNDRED; break;
            case '3':   updateRate = THREE_HUNDRED; break;
            case '4':   updateRate = FOUR_HUNDRED; break;
            // modes
            case 'b':   currentMode = BREAK; break;
            case 'p':   currentMode = PERIODIC; break;
            case 'V':   currentMode = AVERAGE; break;
            case 'e':   currentMode = SWEEP; break;
            case 't':   currentMode = TOGETHER; break;
            case 'w':   currentMode = TWITCH; break;
            case 'l':   currentMode = LISTEN; break;
            case 'd':   currentMode = DELAYED; break;
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



	if(input_char == 'r')	//set sec_counter as 0
	{
		send_message(MESSAGE_COMMAND, ALL_DIRECTION, ALL, "r");	
		sec_counter = 0;
	}

    
    // ============================================================================================
    /// updateRate commands 
    // ============================================================================================

	// Hold Current Servo Position
    if(input_char == 'n'){
        fprintf_P(&usart_stream, PSTR("setting updateRate to NONE\n"));
        updateRate = NONE;
        send_message(MESSAGE_COMMAND, ALL_DIRECTION, ALL, "n");
    }
    if(input_char == '0'){
        fprintf_P(&usart_stream, PSTR("setting updateRate to 10ms\n"));
        updateRate = SMOOTH;
        send_message(MESSAGE_COMMAND, ALL_DIRECTION, ALL, "0");
    }
    if(input_char == '1'){
        fprintf_P(&usart_stream, PSTR("setting updateRate to 100ms\n"));
        updateRate = ONE_HUNDRED;
        send_message(MESSAGE_COMMAND, ALL_DIRECTION, ALL, "1");
    }
    if(input_char == '2'){
        fprintf_P(&usart_stream, PSTR("setting updateRate to 200ms\n"));
        updateRate = TWO_HUNDRED;
        send_message(MESSAGE_COMMAND, ALL_DIRECTION, ALL, "2");
    }
    if(input_char == '3'){
        fprintf_P(&usart_stream, PSTR("setting updateRate to 300ms\n"));
        updateRate = TWO_HUNDRED;
        send_message(MESSAGE_COMMAND, ALL_DIRECTION, ALL, "3");
    }
    if(input_char == '4'){
        fprintf_P(&usart_stream, PSTR("setting updateRate to 400ms\n"));
        updateRate = FOUR_HUNDRED;
        send_message(MESSAGE_COMMAND, ALL_DIRECTION, ALL, "4");
    }
    
    // ============================================================================================
    /// currentMode commands
    // ============================================================================================
    if(input_char == 'b'){
        fprintf_P(&usart_stream, PSTR("'b' - etting currentMode to BREAK\n"));
        currentMode = BREAK;
        send_message(MESSAGE_COMMAND, ALL_DIRECTION, ALL, "b");
    }
    
    if(input_char == 'p'){
        fprintf_P(&usart_stream, PSTR("'p' - etting currentMode to PERIODIC\n"));
        currentMode = PERIODIC;
        send_message(MESSAGE_COMMAND, ALL_DIRECTION, ALL, "p");
    }
    if(input_char == 'V'){
        fprintf_P(&usart_stream, PSTR("'V' - setting currentMode to AVERAGE\n"));
        currentMode = AVERAGE;
        send_message(MESSAGE_COMMAND, ALL_DIRECTION, ALL, "V");
    }
    if(input_char == 'e'){
        fprintf_P(&usart_stream, PSTR("'e' - setting currentMode to linear SWEEP\n"));
        currentMode = SWEEP;
        send_message(MESSAGE_COMMAND, ALL_DIRECTION, ALL, "e");
    }
    if(input_char == 't'){
        fprintf_P(&usart_stream, PSTR("'t' - setting currentMode to TOGETHER\n"));
        currentMode = TOGETHER;
        send_message(MESSAGE_COMMAND, ALL_DIRECTION, ALL, "t");

    }
    if(input_char == 'w'){
        fprintf_P(&usart_stream, PSTR("'w' - setting currentMode to TWITCH\n"));
        currentMode = TWITCH;
        send_message(MESSAGE_COMMAND, ALL_DIRECTION, ALL, "w");
        
    }
    if(input_char == 'l'){
        fprintf_P(&usart_stream, PSTR("'l' - setting currentMode to linear LISTEN\n"));
        currentMode = LISTEN;
        send_message(MESSAGE_COMMAND, ALL_DIRECTION, ALL, "l");
    }
    if(input_char == 'd'){
        fprintf_P(&usart_stream, PSTR("'d' - setting currentMode to linear DELAYED\n"));
        currentMode = LISTEN;
        send_message(MESSAGE_COMMAND, ALL_DIRECTION, ALL, "d");
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
    
    // ============================================================================================
    /// toggle debug print
    // ============================================================================================
    if(input_char == 'P'){
        fprintf_P(&usart_stream, PSTR("'P' - toggling debug printf on/off \n"));
        debugPrint = !debugPrint;
    }

}


