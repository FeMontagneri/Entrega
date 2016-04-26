/**
 * @file pio_maua.h
 * @author Rafael Corsi
 * @date 22/3/2016
 * @brief Funções para configurar o PIO do SAM4S
 */

#include <asf.h>
#include "pio_maua.h"

/**
 * \brief Configure one or more pin(s) of a PIO controller as outputs, with
 * the given default value. 
 * 
 * \param p_pio Pointer to a PIO instance.
 * \param ul_mask Bitmask indicating which pin(s) to configure.
 * \param ul_default_level Default level on the pin(s).
 * \param ul_pull_up_enable Indicates if the pin shall have its pull-up
 * \param ul_attribute PIO attribute(s).
 * activated.
 */
void _pio_set_output(Pio *p_pio, const uint32_t ul_mask, const Bool ul_default_level, Bool ul_pull_up_enable, const uint32_t ul_attribute){
	
	switch ((uint32_t)p_pio){
		
		case (uint32_t)PIOA:
		
		PIOA->PIO_PER  |= ul_mask;
		PIOA->PIO_WPMR  = 0;
		PIOA->PIO_OER  |= ul_mask;
		
		if(ul_default_level)  PIOA->PIO_SODR |= ul_mask; else PIOA->PIO_CODR |= ul_mask; 		
		if(ul_pull_up_enable) PIOA->PIO_PUER |= ul_mask; else PIOA->PIO_PUDR |= ul_mask;		
		if(ul_attribute)      PIOA->PIO_MDER |= ul_mask; else PIOA->PIO_MDDR |= ul_mask;
		
		break;
		
		case (uint32_t)PIOB:
		
		PIOB->PIO_PER  |= ul_mask;
		PIOB->PIO_WPMR  = 0;
		PIOB->PIO_OER  |= ul_mask;
		
		if(ul_default_level)  PIOB->PIO_SODR |= ul_mask; else PIOB->PIO_CODR |= ul_mask;
		if(ul_pull_up_enable) PIOB->PIO_PUER |= ul_mask; else PIOB->PIO_PUDR |= ul_mask;
		if(ul_attribute)      PIOB->PIO_MDER |= ul_mask; else PIOB->PIO_MDDR |= ul_mask;
		
		break;
		
		case (uint32_t)PIOC:
		
		PIOC->PIO_PER  |= ul_mask;
		PIOC->PIO_WPMR  = 0;
		PIOC->PIO_OER  |= ul_mask;
		
		if(ul_default_level)  PIOC->PIO_SODR |= ul_mask; else PIOC->PIO_CODR |= ul_mask;
		if(ul_pull_up_enable) PIOC->PIO_PUER |= ul_mask; else PIOC->PIO_PUDR |= ul_mask;
		if(ul_attribute)      PIOC->PIO_MDER |= ul_mask; else PIOC->PIO_MDDR |= ul_mask;
		
		break;		
	}	
}

/**
 * \brief Configure one or more pin(s) or a PIO controller as inputs.
 * Optionally, the corresponding internal pull-up(s) and glitch filter(s) can
 * be enabled.
 *
 * \param p_pio Pointer to a PIO instance.
 * \param ul_mask Bitmask indicating which pin(s) to configure as input(s).
 * \param ul_attribute PIO attribute(s).
 */
void _pio_set_input(Pio *p_pio, const uint32_t ul_mask, const uint32_t ul_attribute){

	switch ((uint32_t)p_pio){
		
		case (uint32_t)PIOA:
		
		PIOA->PIO_PER  |= ul_mask;
		PIOA->PIO_WPMR  = 0;
		PIOA->PIO_ODR  |= ul_mask;
		
		if(ul_attribute == PIO_DEFAULT){
			
			 PIOA->PIO_IFDR |= ul_mask;
			 
		}else if(ul_attribute == PIO_DEGLITCH){
			
			PIOA->PIO_IFER   |=  ul_mask;
			PIOA->PIO_IFSCDR |= ul_mask ;
			
		}else if(ul_attribute == PIO_DEBOUNCE){
			
			PIOA->PIO_IFER   |= ul_mask;
			PIOA->PIO_IFSCER |= ul_mask;
			
		}		
		
		break;
		
		case (uint32_t)PIOB:
		
		PIOB->PIO_PER  |= ul_mask;
		PIOB->PIO_WPMR  = 0;
		PIOB->PIO_ODR  |= ul_mask;
		
		if(ul_attribute == PIO_DEFAULT){
			
			PIOB->PIO_IFDR |= ul_mask;
			
			}else if(ul_attribute == PIO_DEGLITCH){
			
			PIOB->PIO_IFER   |= ul_mask;
			PIOB->PIO_IFSCDR |= ul_mask;
			
			}else if(ul_attribute == PIO_DEBOUNCE){
			
			PIOB->PIO_IFER   |= ul_mask;
			PIOB->PIO_IFSCER |= ul_mask;
			
		}
		
		break;
		
		case (uint32_t)PIOC:
		
		PIOC->PIO_PER  |= ul_mask;
		PIOC->PIO_WPMR  = 0;
		PIOC->PIO_ODR  |= ul_mask;
		
		if(ul_attribute == PIO_DEFAULT){
			
			PIOC->PIO_IFDR |= ul_mask;
			
			}else if(ul_attribute == PIO_DEGLITCH){
			
			PIOC->PIO_IFER   |= ul_mask;
			PIOC->PIO_IFSCDR |= ul_mask;
			
			}else if(ul_attribute == PIO_DEBOUNCE){
			
			PIOC->PIO_IFER   |= ul_mask;
			PIOC->PIO_IFSCER |= ul_mask;
			
		}
		
		break;
	}	
	

			
}

/**
 * \brief Configure PIO internal pull-up.
 *
 * \param p_pio Pointer to a PIO instance.
 * \param ul_mask Bitmask of one or more pin(s) to configure.
 * \param ul_pull_up_enable Indicates if the pin(s) internal pull-up shall be
 * configured.
 */
void _pio_pull_up(Pio *p_pio, const uint32_t ul_mask, const uint32_t ul_pull_up_enable){
		
	switch ((uint32_t)p_pio){
		
		case (uint32_t)PIOA:
		
		if(ul_pull_up_enable) PIOA->PIO_PUER |= ul_mask; else PIOA->PIO_PUDR |= ul_mask;
		
		break;
		
		case (uint32_t)PIOB:
		
		if(ul_pull_up_enable) PIOB->PIO_PUER |= ul_mask; else PIOB->PIO_PUDR |= ul_mask;
		
		break;
		
		case (uint32_t)PIOC:
		
		if(ul_pull_up_enable) PIOC->PIO_PUER |= ul_mask; else PIOC->PIO_PUDR |= ul_mask;
		
		break;
	}
	
}

/**
 * \brief Configure PIO pin internal pull-down.
 *
 * \param p_pio Pointer to a PIO instance.
 * \param ul_mask Bitmask of one or more pin(s) to configure.
 * \param ul_pull_down_enable Indicates if the pin(s) internal pull-down shall
 * be configured.
 */
void _pio_pull_down( Pio *p_pio, const uint32_t ul_mask, const uint32_t ul_pull_down_enable){
	
	switch ((uint32_t)p_pio){
		
		case (uint32_t)PIOA:
		
		if(ul_pull_down_enable) PIOA->PIO_PPDER |= ul_mask; else PIOA->PIO_PPDDR |= ul_mask;
		
		break;
		
		case (uint32_t)PIOB:
		
		if(ul_pull_down_enable) PIOB->PIO_PPDER |= ul_mask; else PIOB->PIO_PPDDR |= ul_mask;
		
		break;
		
		case (uint32_t)PIOC:
		
		if(ul_pull_down_enable) PIOC->PIO_PPDER |= ul_mask; else PIOC->PIO_PPDDR |= ul_mask;
		
		break;
	}
						
						
}

/**
 * \brief Set a high output level on all the PIOs defined in ul_mask.
 * This has no immediate effects on PIOs that are not output, but the PIO
 * controller will save the value if they are changed to outputs.
 *
 * \param p_pio Pointer to a PIO instance.
 * \param ul_mask Bitmask of one or more pin(s) to configure.
 */
void _pio_set(Pio *p_pio, const uint32_t ul_mask){

	switch ((uint32_t)p_pio){
		
		case (uint32_t)PIOA:
		
		PIOA->PIO_SODR |= ul_mask; 
		
		break;
		
		case (uint32_t)PIOB:
		
		PIOB->PIO_SODR |= ul_mask;
		
		break;
		
		case (uint32_t)PIOC:
		
		PIOC->PIO_SODR |= ul_mask;
		
		break;
	}	
}

/**
 * \brief Set a low output level on all the PIOs defined in ul_mask.
 * This has no immediate effects on PIOs that are not output, but the PIO
 * controller will save the value if they are changed to outputs.
 *
 * \param p_pio Pointer to a PIO instance.
 * \param ul_mask Bitmask of one or more pin(s) to configure.
 */
void _pio_clear(Pio *p_pio, const uint32_t ul_mask){
	
	switch ((uint32_t)p_pio){
		
		case (uint32_t)PIOA:
		
		PIOA->PIO_CODR |= ul_mask;
		
		break;
		
		case (uint32_t)PIOB:
		
		PIOB->PIO_CODR |= ul_mask;
		
		break;
		
		case (uint32_t)PIOC:
		
		PIOC->PIO_CODR |= ul_mask;
		
		break;
	}
	
}


/**
 * \brief Return 1 if one or more PIOs of the given Pin are configured to
 * output a high level (even if they are not output).
 * To get the actual value of the pin, use PIO_Get() instead.
 *
 * \param p_pio Pointer to a PIO instance.
 * \param ul_mask Bitmask of one or more pin(s).
 *
 * \retval 1 At least one PIO is configured to output a high level.
 * \retval 0 All PIOs are configured to output a low level.
 */
uint32_t _pio_get_output_data_status(const Pio *p_pio, const uint32_t ul_mask){
	
	if(pio_get(p_pio, PIO_INPUT, ul_mask)) return 1; else return 0;
	
}

