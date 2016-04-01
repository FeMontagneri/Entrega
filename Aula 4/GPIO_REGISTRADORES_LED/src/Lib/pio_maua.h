
#define PIN_LED_BLUE  19
#define PIN_LED_GREEN 20
#define PIN_LED_RED   20
#define PIN_BOT_3     12
#define PIN_BOT_2     3

void pin_config(void);
void PIOA_pin_set(void);
void PIOC_pin_set(void);
void PIOA_pin_clear(void);
void PIOC_pin_clear(void);

Bool PIOB_GetPinValue(void);
Bool PIOC_GetPinValue(uint8_t pin_c);