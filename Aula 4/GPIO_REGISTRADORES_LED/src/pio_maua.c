
#include <asf.h>

#define PIN_LED_BLUE  19
#define PIN_LED_GREEN 20
#define PIN_LED_RED   20

// Função de Configuração dos Pinos

void pin_config(void){
	
	// 29.17.4 PMC Peripheral Clock Enable Register 0
	// 1: Enables the corresponding peripheral clock.
	// ID_PIOA = 11 - TAB 11-1
	PMC->PMC_PCER0 |= ID_PIOA;
	PMC->PMC_PCER0 |= ID_PIOC;

	//31.6.1 PIO Enable Register
	// 1: Enables the PIO to control the corresponding pin (disables peripheral control of the pin).
	PIOA->PIO_PER |= (1 << PIN_LED_BLUE )|(1 << PIN_LED_GREEN );
	PIOC->PIO_PER |= (1 << PIN_LED_RED );

	// 31.6.46 PIO Write Protection Mode Register
	// 0: Disables the write protection if WPKEY corresponds to 0x50494F (PIO in ASCII).
	PIOA->PIO_WPMR = 0;
	PIOC->PIO_WPMR = 0;
	
	// 31.6.4 PIO Output Enable Register
	// value =
	//	 	1 : Enables the output on the I/O line.
	//	 	0 : do nothing
	PIOA->PIO_OER |=  (1 << PIN_LED_BLUE )|(1 << PIN_LED_GREEN );
	PIOC->PIO_OER |=  (1 << PIN_LED_RED );
	
}

void PIOA_pin_set(void){
	
	PIOA->PIO_SODR = (1 << PIN_LED_BLUE )|(1 << PIN_LED_GREEN );

	
}

void PIOC_pin_set(void){
	
	PIOC->PIO_SODR = (1 << PIN_LED_RED );
	
}

void PIOA_pin_clear(void){
	
	PIOA->PIO_CODR = (1 << PIN_LED_BLUE )|(1 << PIN_LED_GREEN );

	
}

void PIOC_pin_clear(void){
	
	PIOC->PIO_CODR = (1 << PIN_LED_RED );
	
}