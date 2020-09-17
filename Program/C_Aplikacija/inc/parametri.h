#ifndef PARAMETRI_H
#define PARAMETRI_H

	#include <termios.h>

	#define SERVER_ID			1 			  // Slave ID!
	#define COIL_ADDRESS		0   		  // Relejni izlaz!
	#define BCM_PIN_DE			22  		  // Pin za kontrolu RS-485!

   /* Device: "/dev/prosireni_uart", or "/dev/ttyAMA0"(ORIGINAL) */
	#define DEVICE_NAME "/dev/prosireni_uart" /* Karakterski uredjaj koji predstavlja prosirenje UART-a! */

	// ioctl, flegovi za citanje/upis_u pin-a kontrole RS-485!
	#define WR_BCM_PIN_DE _IOW('a','a',int32_t*)
	#define RD_BCM_PIN_DE _IOR('a','b',int32_t*)
	#define WR_SERIAL_PARAM _IOW('a','C',int32_t*)

	// Parametri serijskog prenosa:

    /* Bauds: 9600, 19200, 57600, 115200, etc */
	#define BAUD_RATE  9600

    /* Parity: 'N', 'O', 'E' */
	#define PARITY     'N'

    /* Data bit */
	#define N_BIT       8

    /* Stop bit */
	#define N_STOP_BIT  1

/* Struktura koja reprezentuje prosireni UART driver(Konfiguracija serijskog prenosa)! */
typedef struct struktura_prosireni_uart{
    /* Bauds: 9600, 19200, 57600, 115200, etc */
    int baud;
    /* Data bit */
    uint8_t data_bit;
    /* Stop bit */
    uint8_t stop_bit;
    /* Parity: 'N', 'O', 'E' */
    char parity;
}PARAMETRI_UART_INTERFEJSA;

#endif
