/** @mainpage Dokumentacja bibliotek modułu IMU
	*   @par Opis:
    *   Projekt kontrolera serwomechanizmów wyposażonego w interfejs RS485 oparty na projekce OpenServo
	*   @author Jarosław Toliński

	*/
/**
 *  @defgroup tolinski_uart Biblioteka RS485

 *  @code #include <RS485.h> @endcode
 *
 *  @brief Biblioteka RS485
 *
 *  Biblioteka służy wykonywaniu poleceń przesłanych przez interfejs RS485 (do portu UART mikrokontrolera).
 *  Docelowym urządzeniem biblioteki jest atmega168.
 *  Do komunikacji wykorzystywana jest ramka składająca się z 9 bajtów:\n
 *  '<',device_address,cmd,data1MSB,data1LSB,data2MSB,data2LSB,CRCMSB,CRCLSB.\n
 *  Program pominie wszystkie znaki aż do nadejścia bajtu '<'. Po jego odebraniu
 *  do kolejnych bajtów będą zapisywane kolejne odebrane znaki. W przypadku napotkania
 *  błędu odbioru (błąd zwracany przez uart) aktualna ramka zostanie porzucona i
 *  program rozpocznie pobieranie nowej ramki.\n
 *  Po pobraniu ramki przechowywana jest ona w buforze (aktualnie jednoelementowym)
 *  oczekując na wykonie odbranej ramki (lub porzucenie jej). Po wywolaniu \a RS485CMD()
 *  program sprawdza czy zostala odebrana ramka, jesli tak, sprawdzana jest poprawnosc jej
 *  sumy CRC, następnie zgodność adresu z adresem urządzenia oraz poprawność komendy (czy istnieje).\n
 *  Jeśli ramka przeszła weryfikację wykonywane jest zapisane w niej polecenie.\n
 *  W przypadku błędu CRC lub niezgodności adresów ramka zostanie zignorowana.\n
 *  W przypadku niepoprawnej komendy, program odeśle komunikat o błędzie zawierający błędne polecenie.\n
 *  W przypadku braku gotowej ramki funkcja zakończy działanie.\n
 *
 *  @note Oparte na projekcie ZoSuperModiefied

 *  @par Inicjalizacja i użycie
 *      Do inicjalizacji układu wykorzytywana jest funkcja \a UartInit():
 *      @code UartInit(); @endcode
        Wykonanie odebranych komend:
        @code RS485CMD(); @endcode
 *  @author Jaroslaw Toliński
 */

#ifndef RS485_H
#define RS485_H
/**@{*/
#include <stdint.h>
#include "openservo.h"

#include <avr/io.h>
#include <avr/interrupt.h>

typedef enum {
        UART_PARITY_NONE = 0x00,
        UART_PARITY_ODD = 0x30,
        UART_PARITY_EVEN = 0x20
}UART_PARITY;

typedef enum {
        UART_DATA_BITS_5 = 0x00,
        UART_DATA_BITS_6 = 0x02,
        UART_DATA_BITS_7 = 0x04,
        UART_DATA_BITS_8 = 0x06,
        UART_DATA_BITS_9 = 0x0E
}UART_DATA_BITS;

typedef enum {
        UART_STOP_BITS_1=0x00,
        UART_STOP_BITS_2=0x80
}UART_STOP_BITS;



#define UART_DEFAULT_BAUD_RATE                       19200
#define UART_DEFAULT_DATA_BITS                       UART_DATA_BITS_8
#define UART_DEFAULT_PARITY                          UART_PARITY_NONE
#define UART_DEFAULT_STOP_BITS                       UART_STOP_BITS_1
#define UART_DEFAULT_TRANSMIT_TIMEOUT_MILISECONDS    1000
#define UART_DEFAULT_BUFFER_SIZE

typedef struct
{
    uint8_t address;
    uint8_t cmd;
    uint16_t data1;
	uint16_t data2;
    uint16_t crc;
    uint8_t valid;
}Frame;

#define ReceiveNoError  0x00
#define ReceiveParityE  0x01
#define ReceiveFrameE   0x02
#define ReceiveOverrunE 0x04
#define FRAMECRCVALID 	0x00
#define FRAMECRCMISMATCH 0x01

#define CMD_DIAG 		  0x01 //sends errors, 3 frames
#define CMD_RESET 		  0x02
#define CMD_READNORMALREG 0x03
#define CMD_WRITENORMALREG 0x04

#define CMD_NOTFOUND	  0xF0
//initialization functions
/** Inicjuje UART
    @retval FALSE w przypadku błedu
    @retval TRUE w przeciwnym wypadku
*/
bool UartInit(void);
/** Ustawia prędkość transmisji (BaudRate)
    @param baudRate docelowa prędkość transmisji (BaudRate)
    @retval FALSE w przypadku błedu
    @retval TRUE w przeciwnym wypadku
*/
bool UartSetBaud(uint32_t baudRate);
/** Ustawia ilość bitów danych
    @param dataBits ilość bitów danych (wartość musi należeć do UART_DATA_BITS)
    @see UART_DATA_BITS
*/
void UartSetDataBits(UART_DATA_BITS dataBits);
/** Ustawia bity parzytości
    @param parity rodzaj bitu parzystości (wartość musi należeć do UART_PARITY)
    @see UART_PARITY
*/
void UartSetParity(UART_PARITY parity);                    //0 for none, 1 for odd, 2 for even
/** Ustawia ilość bitów stopu
    @param stopBits ilość bitów stopu (wartość musi należeć do UART_STOP_BITS)
    @see UART_STOP_BITS
*/
void UartSetStopBits(UART_STOP_BITS stopBits);

//void UartSetTransmitTimeOut(uint16_t miliseconds);
bool UartSetBuffersSize(uint8_t size);


/** Inicjalizuje pin obsługujący kierunek przeływu danych na potrzeby half-duplexu RS485
    @param port port na którym znajduje się używany pin
    @param pinConnectedToReDe numer używanego pinu (liczony od 0)
*/
void UartInitRs485(volatile uint8_t *port, uint8_t pinConnectedToReDe);



//High level transaction functions
/* Wysyła jeden znak
    @param c wysyłany znak

bool UartPutChar(const uint8_t c);
bool UartGetChar(uint8_t* byte);
bool UartPutString(const uint8_t* buffer);*/

//void UartStartTx(void);
//void UartRxFlush(void);

/** Oblicza sumę kontrolną ramki
    @param f wskaźnik na ramkę
    @return suma kontrolna ramki
*/
uint16_t FrameCRC(volatile const Frame *f);
/** Sprawdza poprawność sumy kontrolnej ramki
    @param f wskaźnik na ramkę
    @retval FALSE gdy ramka ma niepoprawną sumę kontrolną
    @retval TRUE gdy ramka
*/
bool FrameCheckCRC(volatile Frame *f);

//uint8_t UartGetFrameISR(volatile Frame *f,volatile uint8_t byte);
//uint8_t UartPutFrameISR(volatile Frame *f);
/** Inicjuje strukturę ramki podanymi wartościami i oblicza jej sumę kontrolną
    @param f inicjowana ramka
    @param a adres
    @param c polecenie (komenda)
    @param d1 pierwszy bit danych
    @param d2 drugi bit danych
*/
void FrameInit(volatile Frame *f,uint8_t a,uint8_t c,uint16_t d1,int16_t d2);
/** Kopiuje ramkę
    @param to ramka docelowa
    @param from kopiowana ramka
*/
void FrameCopy(volatile Frame *to,volatile Frame *from);

/** Rozpoczyna wysyłanie ramki
    @param f wskaźnik na wysyłaną ramkę
    @retval FALSE jeśli wcześniejsza ramka nie została jeszcze wysłana
    @retval TRUE jeśli rozpoczęto wysyłanie ramki
*/
bool SendFrame(Frame *f);
/** Sprawdza czy pobrano ramkę
    @param[out] f wskaźnik na ramkę do której zostanie zapisana odebrana ramka
    @retval FALSE oczekiwanie na dane (ramka f nie została zmodyfikowana)
    @retval TRUE odebrano ramkę (ramka f zawiera odebrane dane)
*/
bool GetFrame(volatile Frame *f);
/** Sprawdza czy przyszło jakieś polecenie, jeśli tak wykonuje je.
*/
void RS485CMD();
/**@}*/

#endif /* ZO_AVR_UART_0_H */

