#include <stdio.h>
#include <unistd.h>
#include <errno.h>
#include <stdlib.h>
#include <string.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <modbus.h>			/* Modbus biblioteka! */
#include "parametri.h"		/* Definisanje parametara za izvrsavanje programa! */

#define SPAVANJE_1 5 // Vrijeme spavanja izrazeno u sekundama! 
#define SPAVANJE_2 1 // Vrijeme spavanja izrazeno u sekundama! 

/*********  Metode ***********/
int bcm_pin_de_configuration(int fd);
int uart_serial_param_data_transfera(int fd);

int main()
{

	modbus_t *ctx;
	int ret;

	// Kreiranje modbus konteksta:
	ctx = modbus_new_rtu(DEVICE_NAME, BAUD_RATE, PARITY, N_BIT, N_STOP_BIT);
	if (ctx == NULL) {
		fprintf(stderr, "Unable to create the libmodbus context!\n");
		return -1;
	}

	// Postavljanje slave identifikatora za kom. :
	if(modbus_set_slave(ctx, SERVER_ID)){
		fprintf(stderr, "Unable to set slave!\n");
		modbus_close(ctx);
		modbus_free(ctx);
		return -1;
	}

	// Omogucenje prikazivanja DEBUG informacija:
	modbus_set_debug(ctx, TRUE);

	// Uspostavljanje veze sa slejvom!
	if (modbus_connect(ctx) == -1) {
		fprintf(stderr, "Connection failed.\n");
		modbus_close(ctx);
		modbus_free(ctx);
		return -1;
	}

	// Omogucenje rpi pina!
/**/	//modbus_enable_rpi(ctx, TRUE);
/**/	//modbus_configure_rpi_bcm_pin(ctx, BCM_PIN_DE); 	// Konfiguracija konkretnog pina!
/**/	//modbus_rpi_pin_export_direction(ctx); 			// Eksportovanje pina!


	// Postavljanje pina u odgovarajucem kernel modulu!
	ret = bcm_pin_de_configuration(modbus_get_socket(ctx));
	if(ret < 0){
		fprintf(stderr, "PIN configuration!\n");
		modbus_close(ctx);
		modbus_free(ctx);
		return -1;
	}
		
	// Prenos samo neophodnih parametara za serijski prenos:
	ret = uart_serial_param_data_transfera(modbus_get_socket(ctx));
	if(ret < 0){
		fprintf(stderr, "Serial parametres kernel modul transfer!\n");
		modbus_close(ctx);
		modbus_free(ctx);
		return -1;
	}
	
	// Do some communication over Modbus
	modbus_write_bit(ctx, COIL_ADDRESS, TRUE);  // Ukljuci relejni izlaz!
	sleep(SPAVANJE_1);
	modbus_write_bit(ctx, COIL_ADDRESS, FALSE); // Iskljuci relejni izlaz!
	sleep(SPAVANJE_2);

/**/ //modbus_rpi_pin_unexport_direction(ctx);

	// Closing the connection		
	modbus_close(ctx);
	modbus_free(ctx);
	return 0;
}

/*
* Funkcija 'bcm_pin_de_configuration':
*	Parametri:
*		fd  - file deskriptor kernel modul fajla.
*		ret - uspjesnost izvrsavanja. 
*
*	Opis f-je:
*		Vrsi konfiguraciju pina za upravljanje prenosom podataka preko RS-485 resivera!
*/
int bcm_pin_de_configuration(int fd){

	int PIN = BCM_PIN_DE; // Mozda moze bez int PIN?

    ioctl(fd, WR_BCM_PIN_DE, (int32_t*) &PIN);  	// Postavljanje vrijednosti pina u kernel modulu!

	int RD_PIN = 0;

    ioctl(fd, RD_BCM_PIN_DE, (int32_t*) &RD_PIN); 	// Citanje vrijednosti postavljenog pina u kernel modulu!

	if(RD_PIN != PIN){
		printf("Error: PIN is not configured in kernel module!\n");
    	close(fd);
		return -1;
	}
	return 0;
}

/*
* Funkcija 'uart_serial_param_data_transfera':
*	Parametri:
*		fd  - file deskriptor kernel modul fajla.
*		ret - uspjesnost izvrsavanja. 
*
*	Opis f-je:
*		Vrsi prenos samo neophodnih parametara serijskog prenosa, u kernel modul!
*/
int uart_serial_param_data_transfera(int fd){
	PARAMETRI_UART_INTERFEJSA pui = {.baud = BAUD_RATE, .parity = PARITY, .data_bit = N_BIT, .stop_bit = N_STOP_BIT}; 

    if(ioctl(fd, WR_SERIAL_PARAM, (int32_t*) &pui)){  	// Postavljanje vrijednosti pina u kernel modulu!
		printf("Greska prilikom prenosa parametara! \n");
		return -1;
	}	
	return 0;
}


