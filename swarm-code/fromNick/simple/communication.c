// ============================================================================================
// send packet
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

		case MESSAGE_NUMDATA:
			mdata.x0 = agent0.px;	mdata.y0 = agent0.py;
			mdata.x1 = agent1.px;	mdata.y1 = agent1.py;

			pkt.data = (uint8_t *)&mdata;
			pkt.radius = dist;
			pkt.data_len = sizeof(point);
			break;
	}
									
	xgrid.send_packet(&pkt, direction);	// send to all neighbors

	//LED_PORT.OUTTGL = LED_USR_2_PIN_bm;	//green LED
}

struct InforSent {
    int angle_value;
    int times;
} inforSent;

struct AngleData {
    float angleValue;
    float strength; // TODO: use to impart influence on neighbors
} angleData;

void send_new_message(uint8_t MessageType, uint8_t direction, int _radius, int angle)
{
	Xgrid::Packet pkt;
	pkt.type = MessageType;
	pkt.flags = 0;
	pkt.radius = _radius;
	
    inforSent.angle_value = angle;
    inforSent.times = 0;
    
	pkt.data = (uint8_t *)&inforSent;
    
	pkt.data_len = sizeof(InforSent);
	
	xgrid.send_packet(&pkt,direction);
}
void send_angle(float angle)
{
	Xgrid::Packet pkt;
	pkt.type = MOTOR_ANGLE;
	pkt.flags = 0;
	pkt.radius = 1;
	
    angleData.angleValue = angle;
    
	pkt.data = (uint8_t *)&angleData;
    
	pkt.data_len = sizeof(AngleData);
	
	xgrid.send_packet(&pkt, ALL_DIRECTION);
}

// ============================================================================================
// receive packet
// ============================================================================================
void rx_pkt(Xgrid::Packet *pkt)
{
	// where did the packet come from
    uint8_t port = pkt->rx_node;
	char command;

	if(port >= 0 && port < NUM_NEIGHBORS){
        // keep track of who I have recieved messages from
        connected[port] = true;
        
        if (pkt->type == MOTOR_ANGLE) {
            
            AngleData* recvAnglPtr = (AngleData*) pkt->data;
            float angleIn = recvAnglPtr->angleValue;
            neighborAngles[port] = angleIn;
            
            if(debugPrint) {
                fprintf_P(&usart_stream, PSTR("neighbors: %f, %f, %f, %f, %f, %f\r\n"), neighborAngles[0], neighborAngles[1], neighborAngles[2], neighborAngles[3], neighborAngles[4], neighborAngles[5]);
            }
        }

        

        /// parsing broadcast commands from neighbors
        // ============================================================================================
		if(pkt->type == MESSAGE_COMMAND)
		{
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
                case 'd':   debugPrint = !debugPrint; break;
                case 'n':   updateRate = NONE; break;
                case 's':   updateRate = SMOOTH; break;
                case 'h':   updateRate = TWO_HUNDRED; break;
                case 'p':   currentMode = PERIODIC; break;
                case 'V':   currentMode = AVERAGE; break;

                
			}
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
			LED_PORT.OUTTGL = LED_USR_1_PIN_bm; _delay_ms(100);
		}
		xboot_reset();
	}


    // print version number
	if(input_char == 'v')
		fprintf_P(&usart_stream, PSTR("build number = %ld\r\n"), (unsigned long) &__BUILD_NUMBER);

	if(input_char == 'd')
	{
		if(display) display = false;
		else		display = true;
	}

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
    if(input_char == 's'){
        fprintf_P(&usart_stream, PSTR("setting updateRate to 10ms\n"));
        updateRate = SMOOTH;
        send_message(MESSAGE_COMMAND, ALL_DIRECTION, ALL, "s");	
    }
    if(input_char == 'h'){
        fprintf_P(&usart_stream, PSTR("setting updateRate to 200ms\n"));
        updateRate = TWO_HUNDRED;
        send_message(MESSAGE_COMMAND, ALL_DIRECTION, ALL, "h");	
    }
    
    // ============================================================================================
    /// currentMode commands
    // ============================================================================================
    if(input_char == 'p'){
        fprintf_P(&usart_stream, PSTR("setting currentMode to PERIODIC\n"));
        currentMode = PERIODIC;
        send_message(MESSAGE_COMMAND, ALL_DIRECTION, ALL, "p");
    }
    if(input_char == 'V'){
        fprintf_P(&usart_stream, PSTR("setting currentMode to AVERAGE\n"));
        currentMode = AVERAGE;
        send_message(MESSAGE_COMMAND, ALL_DIRECTION, ALL, "V");
    }
    
    // ============================================================================================
    /// toggle debug print
    // ============================================================================================
    if(input_char == 'P'){
        fprintf_P(&usart_stream, PSTR("'P' - toggling debug printf on/off \n"));
        debugPrint = !debugPrint;
    }

}
