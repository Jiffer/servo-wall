/************************************************************************/
/* avr-xgrid                                                            */
/*                                                                      */
/* main.cpp                                                             */
/*                                                                      */
/* Alex Forencich <alex@alexforencich.com>                              */
/*                                                                      */
/* Copyright (c) 2011 Alex Forencich                                    */
/*                                                                      */
/* Permission is hereby granted, free of charge, to any person          */
/* obtaining a copy of this software and associated documentation       */
/* files(the "Software"), to deal in the Software without restriction,  */
/* including without limitation the rights to use, copy, modify, merge, */
/* publish, distribute, sublicense, and/or sell copies of the Software, */
/* and to permit persons to whom the Software is furnished to do so,    */
/* subject to the following conditions:                                 */
/*                                                                      */
/* The above copyright notice and this permission notice shall be       */
/* included in all copies or substantial portions of the Software.      */
/*                                                                      */
/* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,      */
/* EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF   */
/* MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND                */
/* NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS  */
/* BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN   */
/* ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN    */
/* CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE     */
/* SOFTWARE.                                                            */
/*                                                                      */
/************************************************************************/

#include "main.h"

// USART

#define USART_TX_BUF_SIZE 64
#define USART_RX_BUF_SIZE 64
char usart_txbuf[USART_TX_BUF_SIZE];
char usart_rxbuf[USART_RX_BUF_SIZE];
CREATE_USART(usart, UART_DEVICE_PORT);
FILE usart_stream;

#define NODE_TX_BUF_SIZE 32
#define NODE_RX_BUF_SIZE 64
char usart_n0_txbuf[NODE_TX_BUF_SIZE];
char usart_n0_rxbuf[NODE_RX_BUF_SIZE];
CREATE_USART(usart_n0, USART_N0_DEVICE_PORT);
char usart_n1_txbuf[NODE_TX_BUF_SIZE];
char usart_n1_rxbuf[NODE_RX_BUF_SIZE];
CREATE_USART(usart_n1, USART_N1_DEVICE_PORT);
char usart_n2_txbuf[NODE_TX_BUF_SIZE];
char usart_n2_rxbuf[NODE_RX_BUF_SIZE];
CREATE_USART(usart_n2, USART_N2_DEVICE_PORT);
char usart_n3_txbuf[NODE_TX_BUF_SIZE];
char usart_n3_rxbuf[NODE_RX_BUF_SIZE];
CREATE_USART(usart_n3, USART_N3_DEVICE_PORT);
char usart_n4_txbuf[NODE_TX_BUF_SIZE];
char usart_n4_rxbuf[NODE_RX_BUF_SIZE];
CREATE_USART(usart_n4, USART_N4_DEVICE_PORT);
char usart_n5_txbuf[NODE_TX_BUF_SIZE];
char usart_n5_rxbuf[NODE_RX_BUF_SIZE];
CREATE_USART(usart_n5, USART_N5_DEVICE_PORT);

Usart *usart_n[6];

Xgrid xgrid;

// SPI

Spi spi(&SPI_DEV);

// I2C

I2c i2c(&I2C_DEV);

// Timer

volatile unsigned long jiffies = 0;

// Production signature row access
uint8_t SP_ReadCalibrationByte( uint8_t index )
{
        uint8_t result;
        NVM_CMD = NVM_CMD_READ_CALIB_ROW_gc;
        result = pgm_read_byte(index);
        NVM_CMD = NVM_CMD_NO_OPERATION_gc;
        return result;
}

// User signature row access
uint8_t SP_ReadUserSigRow( uint8_t index )
{
        uint8_t result;
        NVM_CMD = NVM_CMD_READ_USER_SIG_ROW_gc;
        result = pgm_read_byte(index);
        NVM_CMD = NVM_CMD_NO_OPERATION_gc;
        return result;
}

// Timer tick ISR (1 kHz)
ISR(TCC0_OVF_vect)
{
        // Timers
        jiffies++;
        
        if (jiffies % 50 == 0)
                LED_PORT.OUTTGL = LED_USR_0_PIN_bm;
        
        xgrid.process();
}

void rx_pkt(Xgrid::Packet *pkt)
{
        usart.write_string("RX: ");
        usart.write(pkt->data, pkt->data_len);
        usart.put('\n');
        LED_PORT.OUTTGL = LED_USR_2_PIN_bm;
}

// Init everything
void init(void)
{
        // clock
        OSC.CTRL |= OSC_RC32MEN_bm; // turn on 32 MHz oscillator
        while (!(OSC.STATUS & OSC_RC32MRDY_bm)) { }; // wait for it to start
        CCP = CCP_IOREG_gc;
        CLK.CTRL = CLK_SCLKSEL_RC32M_gc; // swtich osc
        DFLLRC32M.CTRL = DFLL_ENABLE_bm; // turn on DFLL
        
        // disable JTAG
        CCP = CCP_IOREG_gc;
        MCU.MCUCR = 1;
        
        // Init pins
        LED_PORT.OUTCLR = LED_USR_0_PIN_bm | LED_USR_1_PIN_bm | LED_USR_2_PIN_bm;
        LED_PORT.DIRSET = LED_USR_0_PIN_bm | LED_USR_1_PIN_bm | LED_USR_2_PIN_bm;
        
        // Init buttons
        BTN_PORT.DIRCLR = BTN_PIN_bm;
        
        // UARTs
        usart.set_tx_buffer(usart_txbuf, USART_TX_BUF_SIZE);
        usart.set_rx_buffer(usart_rxbuf, USART_RX_BUF_SIZE);
        usart.begin(UART_BAUD_RATE);
        usart.setup_stream(&usart_stream);
        
        usart_n0.set_tx_buffer(usart_n0_txbuf, NODE_TX_BUF_SIZE);
        usart_n0.set_rx_buffer(usart_n0_rxbuf, NODE_RX_BUF_SIZE);
        usart_n0.begin(NODE_BAUD_RATE);
        usart_n[0] = &usart_n0;
        xgrid.add_node(&usart_n0);
        usart_n1.set_tx_buffer(usart_n1_txbuf, NODE_TX_BUF_SIZE);
        usart_n1.set_rx_buffer(usart_n1_rxbuf, NODE_RX_BUF_SIZE);
        usart_n1.begin(NODE_BAUD_RATE);
        usart_n[1] = &usart_n1;
        xgrid.add_node(&usart_n1);
        usart_n2.set_tx_buffer(usart_n2_txbuf, NODE_TX_BUF_SIZE);
        usart_n2.set_rx_buffer(usart_n2_rxbuf, NODE_RX_BUF_SIZE);
        usart_n2.begin(NODE_BAUD_RATE);
        usart_n[2] = &usart_n2;
        xgrid.add_node(&usart_n2);
        usart_n3.set_tx_buffer(usart_n3_txbuf, NODE_TX_BUF_SIZE);
        usart_n3.set_rx_buffer(usart_n3_rxbuf, NODE_RX_BUF_SIZE);
        usart_n3.begin(NODE_BAUD_RATE);
        usart_n[3] = &usart_n3;
        xgrid.add_node(&usart_n3);
        usart_n4.set_tx_buffer(usart_n4_txbuf, NODE_TX_BUF_SIZE);
        usart_n4.set_rx_buffer(usart_n4_rxbuf, NODE_RX_BUF_SIZE);
        usart_n4.begin(NODE_BAUD_RATE);
        usart_n[4] = &usart_n4;
        xgrid.add_node(&usart_n4);
        usart_n5.set_tx_buffer(usart_n5_txbuf, NODE_TX_BUF_SIZE);
        usart_n5.set_rx_buffer(usart_n5_rxbuf, NODE_RX_BUF_SIZE);
        usart_n5.begin(NODE_BAUD_RATE);
        usart_n[5] = &usart_n5;
        xgrid.add_node(&usart_n5);
        
        // ADC setup
        ADCA.CTRLA = ADC_DMASEL_OFF_gc | ADC_FLUSH_bm;
        ADCA.CTRLB = ADC_CONMODE_bm | ADC_RESOLUTION_12BIT_gc;
        ADCA.REFCTRL = ADC_REFSEL_INT1V_gc | ADC_BANDGAP_bm;
        ADCA.EVCTRL = ADC_SWEEP_0123_gc | ADC_EVSEL_0123_gc | ADC_EVACT_SWEEP_gc;
        ADCA.PRESCALER = ADC_PRESCALER_DIV64_gc;
        ADCA.CALL = SP_ReadCalibrationByte(PROD_SIGNATURES_START + ADCACAL0_offset);
        ADCA.CALH = SP_ReadCalibrationByte(PROD_SIGNATURES_START + ADCACAL1_offset);
        
        ADCA.CH0.CTRL = ADC_CH_GAIN_1X_gc | ADC_CH_INPUTMODE_SINGLEENDED_gc;
        ADCA.CH0.MUXCTRL = ADC_CH_MUXPOS_PIN1_gc;
        ADCA.CH0.INTCTRL = ADC_CH_INTMODE_COMPLETE_gc | ADC_CH_INTLVL_LO_gc;
        ADCA.CH0.RES = 0;
        
        ADCA.CH1.CTRL = ADC_CH_GAIN_1X_gc | ADC_CH_INPUTMODE_SINGLEENDED_gc;
        ADCA.CH1.MUXCTRL = ADC_CH_MUXPOS_PIN2_gc;
        ADCA.CH1.INTCTRL = ADC_CH_INTMODE_COMPLETE_gc | ADC_CH_INTLVL_LO_gc;
        ADCA.CH1.RES = 0;
        
        ADCA.CH2.CTRL = ADC_CH_GAIN_1X_gc | ADC_CH_INPUTMODE_SINGLEENDED_gc;
        ADCA.CH2.MUXCTRL = ADC_CH_MUXPOS_PIN3_gc;
        ADCA.CH2.INTCTRL = ADC_CH_INTMODE_COMPLETE_gc | ADC_CH_INTLVL_LO_gc;
        ADCA.CH2.RES = 0;
        
        ADCA.CH3.CTRL = ADC_CH_GAIN_1X_gc | ADC_CH_INPUTMODE_SINGLEENDED_gc;
        ADCA.CH3.MUXCTRL = ADC_CH_MUXPOS_PIN4_gc;
        ADCA.CH3.INTCTRL = ADC_CH_INTMODE_COMPLETE_gc | ADC_CH_INTLVL_LO_gc;
        ADCA.CH3.RES = 0;
        
        //ADCA.CTRLA |= ADC_ENABLE_bm;
        //ADCA.CTRLB |= ADC_FREERUN_bm;
        
        // TCC
        TCC0.CTRLA = TC_CLKSEL_DIV256_gc;
        TCC0.CTRLB = 0;
        TCC0.CTRLC = 0;
        TCC0.CTRLD = 0;
        TCC0.CTRLE = 0;
        TCC0.INTCTRLA = TC_OVFINTLVL_LO_gc;
        TCC0.INTCTRLB = 0;
        TCC0.CNT = 0;
        TCC0.PER = 125;
        
        // ADC trigger on TCC0 overflow
        //EVSYS.CH0MUX = EVSYS_CHMUX_TCC0_OVF_gc;
        //EVSYS.CH0CTRL = 0;
        
        // I2C
        //i2c.begin(400000L);
        
        // SPI
        //spi.begin(SPI_MODE_2_gc, SPI_PRESCALER_DIV4_gc, 1);
        
        // CS line
        //SPI_CS_PORT.OUTSET = SPI_CS_DEV_PIN_bm;
        //SPI_CS_PORT.DIRSET = SPI_CS_DEV_PIN_bm;
        
        // Interrupts
        PMIC.CTRL = PMIC_LOLVLEN_bm | PMIC_MEDLVLEN_bm;
        
        sei();
}

int main(void)
{
        //char old_btn = 0;
        //char btn;
        uint32_t j = 0;
        
		char input_char, first_char;
		
        _delay_ms(50);
        
        init();
        
        xgrid.rx_pkt = &rx_pkt;
        
        LED_PORT.OUT = LED_USR_0_PIN_bm;
        
        fprintf_P(&usart_stream, PSTR("avr-xgrid build %ld\r\n"), (unsigned long) &__BUILD_NUMBER);
        
		/*
		char str[] = "boot up";
		Xgrid::Packet pkt;
		pkt.type = 0;
		pkt.flags = 0;
		pkt.radius = 1;
		pkt.data = (uint8_t *)str;
		pkt.data_len = 4;
                        
		xgrid.send_packet(&pkt);
		*/
		
		if (usart.available())
		{
			first_char = usart.get();
		}	
		
        while (1)
        {
                if (usart.available())
				{
					input_char = usart.get();
				}	
							
				// main loop
                if (input_char == 0x1b)
                        xboot_reset();
						
				else if(input_char == 0x00)
				{
					//fprintf_P(&usart_stream, PSTR("received: first char (%i)**\r\n"),first_char);
					
				}
				
				else
				{
					fprintf_P(&usart_stream, PSTR("CPU: %c**\r\n"), input_char);
					
					if (input_char == 'a')
					{
						char str[] = "A";
                        Xgrid::Packet pkt;		// Packet is defined onl line 72 of xgrid.h
                        pkt.type = 0;
                        pkt.flags = 0;
                        pkt.radius = 1;
                        pkt.data = (uint8_t *)str;
                        pkt.data_len = 4;
                        
                        xgrid.send_packet(&pkt);
					}
					
					else if(input_char == 'y')
					{
						LED_PORT.OUTTGL = LED_USR_1_PIN_bm;
					}
					
					else if (input_char == '1')
					{
						char str[] = "1";
                        Xgrid::Packet pkt;		// Packet is defined on line 72 of xgrid.h
                        pkt.type = 0;
                        pkt.flags = 0;
                        pkt.radius = 1;
                        pkt.data = (uint8_t *)str;
                        pkt.data_len = 1;
                        
                        xgrid.send_packet(&pkt);
					}
					
					else if (input_char == '2')
					{
						char str[] = "22";
                        Xgrid::Packet pkt;		// Packet is defined on line 72 of xgrid.h
                        pkt.type = 0;
                        pkt.flags = 0;
                        pkt.radius = 1;
                        pkt.data = (uint8_t *)str;
                        pkt.data_len = 2;
                        
                        xgrid.send_packet(&pkt);
					}
					
					else if (input_char == '3')
					{
						char str[] = "333";
                        Xgrid::Packet pkt;		// Packet is defined on line 72 of xgrid.h
                        pkt.type = 0;
                        pkt.flags = 0;
                        pkt.radius = 1;
                        pkt.data = (uint8_t *)str;
                        pkt.data_len = 3;
                        
                        xgrid.send_packet(&pkt);
					}
					
					else if (input_char == '4')
					{
						char str[] = "4444";
                        Xgrid::Packet pkt;		// Packet is defined on line 72 of xgrid.h
                        pkt.type = 0;
                        pkt.flags = 0;
                        pkt.radius = 1;
                        pkt.data = (uint8_t *)str;
                        pkt.data_len = 4;
                        
                        xgrid.send_packet(&pkt);
					}
					
					else if (input_char == '5')
					{
						char str[] = "55555";
                        Xgrid::Packet pkt;		// Packet is defined on line 72 of xgrid.h
                        pkt.type = 0;
                        pkt.flags = 0;
                        pkt.radius = 1;
                        pkt.data = (uint8_t *)str;
                        pkt.data_len = 5;
                        
                        xgrid.send_packet(&pkt);
					}
					
					else if (input_char == '6')
					{
						char str[] = "666666";
                        Xgrid::Packet pkt;		// Packet is defined on line 72 of xgrid.h
                        pkt.type = 0;
                        pkt.flags = 0;
                        pkt.radius = 1;
                        pkt.data = (uint8_t *)str;
                        pkt.data_len = 6;
                        
                        xgrid.send_packet(&pkt);
					}
					
					else if (input_char == '7')
					{
						char str[] = "7777777";
                        Xgrid::Packet pkt;		// Packet is defined on line 72 of xgrid.h
                        pkt.type = 0;
                        pkt.flags = 0;
                        pkt.radius = 1;
                        pkt.data = (uint8_t *)str;
                        pkt.data_len = 7;
                        
                        xgrid.send_packet(&pkt);
					}
					
					else if (input_char == '8')
					{
						char str[] = "88888888";
                        Xgrid::Packet pkt;		// Packet is defined on line 72 of xgrid.h
                        pkt.type = 0;
                        pkt.flags = 0;
                        pkt.radius = 1;
                        pkt.data = (uint8_t *)str;
                        pkt.data_len = 8;
                        
                        xgrid.send_packet(&pkt);
					}
					
					else if (input_char == '9')
					{
						char str[] = "999999999";
                        Xgrid::Packet pkt;		// Packet is defined on line 72 of xgrid.h
                        pkt.type = 0;
                        pkt.flags = 0;
                        pkt.radius = 1;
                        pkt.data = (uint8_t *)str;
                        pkt.data_len = 9;
                        
                        xgrid.send_packet(&pkt);
					}
					
					else if (input_char == '0')
					{
						char str[] = "1010101010";
                        Xgrid::Packet pkt;		// Packet is defined on line 72 of xgrid.h
                        pkt.type = 0;
                        pkt.flags = 0;
                        pkt.radius = 1;
                        pkt.data = (uint8_t *)str;
                        pkt.data_len = 10;
                        
                        xgrid.send_packet(&pkt);
					}
					
					else if (input_char == '-')
					{
						char str[] = "11111111111";
                        Xgrid::Packet pkt;		// Packet is defined on line 72 of xgrid.h
                        pkt.type = 0;
                        pkt.flags = 0;
                        pkt.radius = 1;
                        pkt.data = (uint8_t *)str;
                        pkt.data_len = 11;
                        
                        xgrid.send_packet(&pkt);
					}
					
					else if (input_char == '=')
					{
						char str[] = "121212121212";
                        Xgrid::Packet pkt;		// Packet is defined on line 72 of xgrid.h
                        pkt.type = 0;
                        pkt.flags = 0;
                        pkt.radius = 1;
                        pkt.data = (uint8_t *)str;
                        pkt.data_len = 12;
                        
                        xgrid.send_packet(&pkt);
					}
					
					else if (input_char == '[')
					{
						char str[] = "1313131313131";
                        Xgrid::Packet pkt;		// Packet is defined on line 72 of xgrid.h
                        pkt.type = 0;
                        pkt.flags = 0;
                        pkt.radius = 1;
                        pkt.data = (uint8_t *)str;
                        pkt.data_len = 13;
                        
                        xgrid.send_packet(&pkt);
					}
					
					else if (input_char == ']')
					{
						char str[] = "14141414141414";
                        Xgrid::Packet pkt;		// Packet is defined on line 72 of xgrid.h
                        pkt.type = 0;
                        pkt.flags = 0;
                        pkt.radius = 1;
                        pkt.data = (uint8_t *)str;
                        pkt.data_len = 14;
                        
                        xgrid.send_packet(&pkt);
					}
					
					
					
					input_char = 0x00;	
				}
				
				j = jiffies + 10;
				               
                while (j > jiffies) { };
        }  
}


/*  CODE REMOVED FROM MAIN, 1st level code in while(1)
				//fprintf_P(&usart_stream, PSTR("%c"), 'g');					
                
                // check for button press
				btn = !(BTN_PORT.IN & BTN_PIN_bm);
                
				if (btn && (btn != old_btn))
				{
                        char str[] = "test";
                        Xgrid::Packet pkt;
                        pkt.type = 0;
                        pkt.flags = 0;
                        pkt.radius = 1;
                        pkt.data = (uint8_t *)str;
                        pkt.data_len = 4;
                        
                        xgrid.send_packet(&pkt);
                        
                        LED_PORT.OUTTGL = LED_USR_1_PIN_bm;
                }
                
                old_btn = btn;	
				
				
				
*/
                

