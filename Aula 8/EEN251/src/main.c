/**
 * IMT - Rafael Corsi
 * 
 * PIO - 07
 *  Configura o PIO do SAM4S (Banco A, pino 19) para operar em
 *  modo de output. Esse pino está conectado a um LED, que em 
 *  lógica alta apaga e lógica baixa acende.
*/


#include <asf.h>
#include "ASF/sam/user_lib/pio_maua.h"
#include "ASF/sam/user_lib/pmc_maua.h"

int main (void)
{

	/**
	* Inicializando o clock do uP
	*/
	sysclk_init();
	
	_pmc_enable_peripheral_clok(PIOA);
	_pmc_enable_peripheral_clok(PIOB);
	_pmc_enable_peripheral_clok(PIOC);
	
	
	/** 
	*  Desabilitando o WathDog do uP
	*/
	WDT->WDT_MR = WDT_MR_WDDIS;
		


	/**
	*	Loop infinito
	*/
		while(1){

            /*
             * Utilize a função delay_ms para fazer o led piscar na frequência
             * escolhida por você.
             */
            //delay_ms();
		
	}
}