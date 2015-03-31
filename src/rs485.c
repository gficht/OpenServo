#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/crc16.h>
#include <util/delay.h>
#include "rs485.h"
#include "twi.h"
#include "watchdog.h"
#include "registers.h"

// maximum value that can be held
// by unsigned data types (8,16,32bits)
#define MAX_uint8_t 255
#define MAX_uint16_t 65535
#define MAX_U32 4294967295

// maximum values that can be held
// by signed data types (8,16,32bits)
#define MIN_int8_t -128
#define MAX_int8_t 127
#define MIN_int16_t -32768
#define MAX_int16_t 32767
#define MIN_S32 -2147483648
#define MAX_S32 2147483647

#define DATA_BITS_MASK_UCSR0C   0x06
#define DATA_BITS_MASK_UCSR0B   0x04
#define PARITY_BITS_MASK                0x30
#define STOP_BITS_MASK                  0x80



static bool Rs485Used = FALSE;
static volatile uint8_t* Rs485ReDePort;
static volatile uint8_t Rs485ReDePin;

#define FRAMELENGTH     9
#define WAITONSTART     0
#define WAITONADDRESS   1
#define WAITONCMD       2
#define WAITONDATA1     3
#define WAITONDATA2     4
#define WAITONDATA3     5
#define WAITONDATA4     6
#define WAITONCRC1      7
#define WAITONCRC2      8




//volatile bool ByteReceived=FALSE;
volatile uint8_t FrameBytesCount=0;//says which frame byte is received
volatile uint8_t FrameSendBytesCount=0;//says which frame byte will be transmitted
volatile bool FrameReceived=FALSE;//frame waiting in input frame buffer
volatile bool FrameSend=FALSE;//frame waiting in output frame buffer

volatile uint16_t FrameErrors=0;//count of internal frame errors
volatile uint16_t FrameOverflows=0;//how many times there was incoming/outcoming frame and no place in buffer
volatile uint16_t ReceiveErrors=0;//count of uart errors

volatile Frame InFrame;//incoming frame buffer
volatile Frame OutFrame;//outcoming frame buffer

Frame response;
Frame cmd;
Frame erro;

bool UartInit(void)
{
	FrameInit(&erro,0x41,0x44,0x4352,0x4345);
	FrameInit(&response,00,0x53,0x5441,0x5254);
	SendFrame(&response);
    UCSR0A |= _BV(U2X0);  //dual speed                                                  //double speed mode
    UCSR0A &= ~_BV(MPCM0);//multiprocessor mode off                                                  //no multiprocessor
    UCSR0C &= ~(_BV(UMSEL01)|_BV(UMSEL00));//asynchronous USART mode

    UartSetBaud(UART_DEFAULT_BAUD_RATE);
    UartSetDataBits(UART_DEFAULT_DATA_BITS);
    UartSetParity(UART_DEFAULT_PARITY);
    UartSetStopBits(UART_DEFAULT_STOP_BITS);

    UCSR0B |= _BV(RXEN0)|_BV(RXCIE0)  ;    //enable receive and receive interrupt, transmit and transmit interrupt are enabled when data for transmission are present.
    UCSR0B &= ~_BV(TXEN0);                                  //be sure the tx is disabled.
    DDRD &= ~_BV(PD1);                                              //put tx pin in high impendance mode in order to allow others to communicate

    sei();

   return TRUE;
}

bool UartSetBaud(uint32_t baudRate)
{
        uint32_t ubrrReg = 0;

        //configure baud rate
        ubrrReg = (F_CPU/baudRate/8 - 1);
        if( ( ubrrReg > 65535) || ( ubrrReg < 1 ) )
                return FALSE;

        UBRR0H = (uint8_t)((ubrrReg >> 8) & 0x00FF);                //baud rate divisor high byte
        UBRR0L = (uint8_t)(ubrrReg & 0x00FF);                               //baud rate divisor low byte

        return TRUE;
}

inline void UartSetDataBits(UART_DATA_BITS dataBits)
{
        UCSR0C = (UCSR0C & ~DATA_BITS_MASK_UCSR0C) | (dataBits & DATA_BITS_MASK_UCSR0C);
        UCSR0B = (UCSR0B & ~DATA_BITS_MASK_UCSR0B) | ((dataBits>>1) & DATA_BITS_MASK_UCSR0B);
}

inline void UartSetParity(UART_PARITY parity)
{
        UCSR0C = (UCSR0C & ~PARITY_BITS_MASK)|parity;
}

inline void UartSetStopBits(UART_STOP_BITS stopBits)
{
        UCSR0C = (UCSR0C & ~STOP_BITS_MASK)|stopBits;
}

void UartInitRs485(volatile uint8_t *port, uint8_t pinConnectedToReDe)
{
        Rs485Used = TRUE;
        Rs485ReDePort = port;
        Rs485ReDePin = pinConnectedToReDe;

        *(port-1) |= _BV(pinConnectedToReDe);   //configure DDR register
        *port &= ~_BV(pinConnectedToReDe);      //configure PORT register
                                                //reset reDePin -> receive mode
        UCSR0B |= _BV(TXCIE0);                  // Enable Transmit Complete Interrupt
}


void UartStartTx(void)
{
	enterCritical();
    PORTB&=~_BV(PORTB4);//wlacz PB4 - kierunek odbieranie
    *Rs485ReDePort |= _BV(Rs485ReDePin);
	if(FrameSend)                                          // See if this is the first character
    {
		UCSR0B |= _BV(UDRIE0);                                  // Yes, Enable Tx interrupts
		UCSR0B |=_BV(TXCIE0);                                       // Disable trasnmit complete interrupt
   		UCSR0B &=~ _BV(RXCIE0);
	}
	exitCritical();
}

void UartRxFlush(void)
{
	uint8_t  dummy;

    while (bit_is_set(UCSR0A, RXC0))
         dummy = UDR0;

}

uint16_t FrameCRC(volatile const Frame *f)
{
    uint16_t crc = 0xffff;
    crc = _crc16_update(crc,f->address);
    crc = _crc16_update(crc,f->cmd);

    //for(; i<f->length; ++i)
        //crc = _crc16_update(crc, f->data[i]);
	crc = _crc16_update(crc, f->data1>>8);
	crc = _crc16_update(crc, f->data1&0x00FF);
	crc = _crc16_update(crc, f->data2>>8);
	crc = _crc16_update(crc, f->data2&0x00FF);

    return crc;
}

bool FrameCheckCRC(volatile Frame *f)
{
    /*if(f->crc==FrameCRC(f))
    {
        f->valid=FRAMECRCVALID;
        return TRUE;
    }
    else
    {
        f->valid|=FRAMECRCMISMATCH;
        return TRUE;
    }*/
    return (f->crc==FrameCRC(f))?TRUE:FALSE;
}



uint8_t UartGetFrameISR(volatile Frame *f,volatile uint8_t byte);
uint8_t UartPutFrameISR(volatile Frame *f);




uint8_t UartGetFrameISR(volatile Frame *f,volatile uint8_t byte)
{
    if(!FrameReceived)
    {
        switch(FrameBytesCount)
        {
            case WAITONSTART:
                if(byte!='<')
                    return FALSE;
                break;
            case WAITONADDRESS:
                f->address=byte;
                break;
            case WAITONCMD:
                f->cmd=byte;
                break;
            case WAITONDATA1:
                f->data1=(uint16_t)byte<<8;
                break;
            case WAITONDATA2:
                f->data1+=byte& 0xFF;
                break;
            case WAITONDATA3:
                f->data2=(uint16_t)byte<<8;
                break;
            case WAITONDATA4:
                f->data2+=byte& 0xFF;
                break;
            case WAITONCRC1:
                f->crc=(uint16_t)byte<<8;
                break;
            case WAITONCRC2:
			{
                f->crc+=byte& 0xFF;
				FrameReceived=TRUE;
	            FrameBytesCount=0;
	            return TRUE;
			}
            	break;
        }
        ++FrameBytesCount;
    }
    return TRUE;
}

uint8_t UartPutFrameISR(volatile Frame *f)
{
    //if(FrameSend)
    //{
        //++
        switch(FrameSendBytesCount)
        {
            case WAITONSTART:
                return '<';//send frame start sign
            case WAITONADDRESS:
                return f->address;
            case WAITONCMD:
                return f->cmd;
            case WAITONDATA1:
                return (f->data1)>>8;
            case WAITONDATA2:
                return (f->data1)& 0xFF;
            case WAITONDATA3:
                return (f->data2)>>8;
            case WAITONDATA4:
                return (f->data2)& 0xFF;
            case WAITONCRC1:
                return (f->crc)>>8;
            case WAITONCRC2:
			{
				FrameSend=FALSE;//clear "sending frame" flag, ready for new frame to send
	            FrameSendBytesCount=0;
                return (f->crc)& 0xFF;
			}
			//default:
			//	return FrameSend=FALSE;
        }
        ++FrameSendBytesCount;
}

void FrameInit(volatile Frame *f,uint8_t a,uint8_t c,uint16_t d1,int16_t d2)
{
	f->address=a;
	f->cmd=c;
	f->data1=d1;
	f->data2=d2;
	f->crc=FrameCRC(f);
}
void FrameCopy(volatile Frame *f,volatile Frame * from)
{
	f->address=from->address;
	f->cmd=from->cmd;
	f->data1=from->data1;
	f->data2=from->data2;
	f->crc=from->crc;
}

ISR(USART_RX_vect)
{
    uint8_t volatile c;
	uint8_t errors=ReceiveNoError;
    if ((UCSR0A & _BV(FE0)) != 0x00)  //is there a frame error?
            errors|=ReceiveFrameE;

    if ((UCSR0A & _BV(UPE0)) != 0x00) //is there a parity error?
            errors|=ReceiveParityE;

    if ( (UCSR0A & _BV(DOR0)) != 0x00 ) //Is there data overrun?
            errors|=ReceiveOverrunE;

    //Above three bits are cleared automatically when UDR0 is read.                                                                                         //
    c  = UDR0;
	if(errors==ReceiveNoError)
		if(!FrameReceived)
		{
			if(!UartGetFrameISR(&InFrame,c))
				++FrameErrors;
		}
		else
			++FrameOverflows;
    else
    {
        ++ReceiveErrors;
        //if(FrameBytesCount<7)//if only crc left
        FrameBytesCount=0;//drop frame, start receiving new
    }

}

ISR(USART_UDRE_vect)
{

    //PORTB&=~_BV(PORTB3);//|_BV(PORTB5));//wlacz PB3 - kierunek wysylanie
    //PORTB|=_BV(PORTB4);//wlacz PB4 - kierunek wysylanie
    //if(!BufferIsEmpty(&TxBuffer))
	//PORTB&=~_BV(PORTB4);//wlacz PB4 - kierunek odbieranie
	//*Rs485ReDePort |= _BV(Rs485ReDePin);
	if(FrameSend)
    {


			//_delay_us(5);

			UCSR0B |= _BV(TXEN0);                   //enable transmitter we are about to send data on the bus
			//UCSR0B |= _BV(TXCIE0);

			//PORTB&=~_BV(PORTB4);//wlacz PB4 - kierunek odbieranie
			//_delay_us(10);
			volatile uint8_t c=UartPutFrameISR(&OutFrame);
            //if(Rs485Used)
            UDR0=c;
			//_delay_us(5);
            //UDR0 =BufferPopISR(&TxBuffer);
    }
    else
	{
		UCSR0B&=~_BV(UDRIE0);// Buffer empty, Disable Tx interrupts

	}
	//*Rs485ReDePort &= ~_BV(Rs485ReDePin);
	//PORTB|=_BV(PORTB4);//wlacz PB4 - kierunek wysylanie
    //PORTB|=_BV(PORTB3);//|_BV(PORTB5);//wylacz PB3 - kierunek odbieranie
    //PORTB|=_BV(PORTB4);//wylacz PB3 - kierunek odbieranie
}

ISR(USART_TX_vect)
{

        UCSR0B &= ~_BV(TXEN0);                          //disable transmitter, allow other nodes on the uart bus to communicate
        DDRD &= ~_BV(PD1);                                      //put tx pin in high impendance mode in order to allow others to communicate


        *Rs485ReDePort &= ~_BV(Rs485ReDePin);   // Clear RS485 Pin for receive mode
        //_delay_us(10);
		//_delay_us(10);
		PORTB|=_BV(PORTB4);//wlacz PB4 - kierunek wysylanie
		UCSR0B &=~_BV(TXCIE0);                                       // Disable trasnmit complete interrupt
   		UCSR0B |= _BV(RXCIE0);
}


bool SendFrame(Frame *f)
{
	if(FrameSend)//if line busy
	{
		++FrameOverflows;
		return FALSE;//line busy
	}
	//FrameSend=FALSE;

	FrameInit(&OutFrame,f->address,f->cmd,f->data1,f->data2);
	FrameSend=TRUE;
	UartStartTx();
	return TRUE;//frame going out;

}

bool GetFrame(volatile Frame *f)
{
	if(!FrameReceived)//if no frame  received
		return FALSE;//waiting for data , try later
	FrameCopy(f,&InFrame);//InFrame.address,InFrame.cmd,InFrame.data1,InFrame.data2);
    FrameCheckCRC(f);
	if(f->crc!=FrameCRC(f))
        f->valid=FRAMECRCMISMATCH;
	else
		f->valid=FRAMECRCVALID;
	FrameReceived=FALSE;//received frame has been taken care of
						//wait for new data
	return TRUE;
}

void RS485CMD()
{
    if(GetFrame(&cmd))//wait for cmd (without crc errors)
				if(cmd.valid==FRAMECRCVALID&&cmd.address==registers_read_byte(REG_TWI_ADDRESS))
				{
					switch(cmd.cmd)
					{
						case TWI_CMD_RESET:

				            // Reset the servo.
				            watchdog_hard_reset();

				            break;

				        case TWI_CMD_PWM_ENABLE:

				            // Enable PWM to the servo motor.
				            pwm_enable();

				            break;

				        case TWI_CMD_PWM_DISABLE:

				            // Disable PWM to the servo motor.
				            pwm_disable();

				            break;

				        case TWI_CMD_WRITE_ENABLE:

				            // Enable write to read/write protected registers.
				            registers_write_enable();

				            break;

				        case TWI_CMD_WRITE_DISABLE:

				            // Disable write to read/write protected registers.
				            registers_write_disable();

				            break;

				        case TWI_CMD_REGISTERS_SAVE:

				            // Save register values into EEPROM.
				            eeprom_save_registers();

				            break;

				        case TWI_CMD_REGISTERS_RESTORE:

				            // Restore register values into EEPROM.
				            eeprom_restore_registers();

				            break;

				        case TWI_CMD_REGISTERS_DEFAULT:

				            // Restore register values to factory defaults.
				            registers_defaults();
				            break;

				        case TWI_CMD_EEPROM_ERASE:

				            // Erase the EEPROM.
				            eeprom_erase();

				            break;

				        case TWI_CMD_VOLTAGE_READ:

				            // Request a voltage reading.
				            adc_read_voltage();

				            break;

				#if CURVE_MOTION_ENABLED
				        case TWI_CMD_CURVE_MOTION_ENABLE:

				            // Enable curve motion handling.
				            motion_enable();

				            break;

				        case TWI_CMD_CURVE_MOTION_DISABLE:

				            // Disable curve motion handling.
				            motion_disable();

				            break;

				        case TWI_CMD_CURVE_MOTION_RESET:

				            // Reset the motion to the current position.
				            motion_reset(adc_get_position_value());

				            break;

				        case TWI_CMD_CURVE_MOTION_APPEND:

				            // Append motion curve data stored in the registers.
				            motion_append();

				            break;
				#endif
						case CMD_READNORMALREG:
							if( (cmd.data1>>8)<=MAX_WRITE_PROTECT_REGISTER)
							{
								FrameInit(&response,0x00,cmd.cmd,cmd.data1,registers_read_word(cmd.data1>>8, cmd.data1&0x00FF));
								SendFrame(&response);
							}
				        	break;
						case CMD_WRITENORMALREG:
							if((cmd.data1>>8)<=MAX_WRITE_PROTECT_REGISTER)
								registers_write_word(cmd.data1>>8, cmd.data1&0x00FF, cmd.data2);
							break;
						default:
							FrameInit(&response,0x00,CMD_NOTFOUND,cmd.cmd,0);
							SendFrame(&response);
				            // Ignore unknown command.
				            break;
					}
				}
				//else SendFrame(&erro);
}
