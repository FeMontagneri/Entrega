/**
 * IMT - Rafael Corsi
 * 
 * PIO - 07
 *  Configura o PIO do SAM4S (Banco A, pino 19) para operar em
 *  modo de output. Esse pino está conectado a um LED, que em 
 *  lógica alta apaga e lógica baixa acende.
*/

#include <asf.h>
#include "Lib/pio_maua.h"

/*
 * Prototypes
 */

/** 
 * Definição dos pinos
 * Pinos do uC referente aos LEDS.
 *
 * O número referente ao pino (PIOAxx), refere-se ao
 * bit que deve ser configurado no registrador para alterar
 * o estado desse bit específico.
 *
 * exe : O pino PIOA_19 é configurado nos registradores pelo bit
 * 19. O registrador PIO_SODR configura se os pinos serão nível alto.
 * Nesse caso o bit 19 desse registrador é referente ao pino PIOA_19
 *
 * ----------------------------------
 * | BIT 19  | BIT 18  | ... |BIT 0 |
 * ----------------------------------
 * | PIOA_19 | PIOA_18 | ... |PIOA_0|
 * ----------------------------------
 */


/**
 * Main function
 * 1. configura o clock do sistema
 * 2. desabilita wathdog
 * 3. ativa o clock para o PIOA
 * 4. ativa o controle do pino ao PIO
 * 5. desabilita a proteção contra gravações do registradores
 * 6. ativa a o pino como modo output
 * 7. coloca o HIGH no pino
 */

int main (void)
{

	/**
	* Inicializando o clock do uP
	*/
	sysclk_init();
	
	/** 
	*  Desabilitando o WathDog do uP
	*/
	WDT->WDT_MR = WDT_MR_WDDIS;
	
	pin_config();


	/**
	*	Loop infinito
	*/
		while(1){

            /*
             * Utilize a função delay_ms para fazer o led piscar na frequência
             * escolhida por você.
             */
            //delay_ms();
			
			/*
			delay_ms(500);
			PIOA_pin_clear();
			PIOC_pin_set();
			delay_ms(500);
			PIOA_pin_set();
			PIOC_pin_clear();
			*/
			
			if(PIOB_GetPinValue(PIN_BOT_2)){
				PIOA_pin_set();
			}else{
				PIOA_pin_clear();
			}
			
			if(PIOC_GetPinValue(PIN_BOT_3)){
				PIOC_pin_set();
				}else{
				PIOC_pin_clear();
			}		
	}
}



