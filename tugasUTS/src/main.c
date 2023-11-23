/**
 * \file
 *
 * \brief Empty user application template
 *
 */

/**
 * \mainpage User Application template doxygen documentation
 *
 * \par Empty user application template
 *
 * Bare minimum empty user application template
 *
 * \par Content
 *
 * -# Include the ASF header files (through asf.h)
 * -# "Insert system clock initialization code here" comment
 * -# Minimal main function that starts with a call to board_init()
 * -# "Insert application code here" comment
 *
 */

/*
 * Include header files for all drivers that have been imported from
 * Atmel Software Framework (ASF).
 */
 /**
 * Support and FAQ: visit <a href="http://www.atmel.com/design-support/">Atmel Support</a>
 */
#include <asf.h>
#include <stdio.h>
#include <ioport.h>
#include <board.h>
#include <avr/io.h>
#include <util/delay.h>
#include <stdlib.h>

#define LED_IN J1_PIN0
#define BUZZ_IN J4_PIN0

#define MY_ADC    ADCA
#define MY_ADC_CH ADC_CH0
#define PIEZO_ADC_CH ADC_CH2

void setup_timer(void);
void print_message(void);
void print_lcd(uint16_t piezo, int ultra);
static void adc_init(void);
static uint16_t adc_read(void);
static uint16_t piezo_read(void);
void checkLight(uint16_t piezoRead, int ultraRead);

int score = 0;
int phase = 0;
int incremental = 0;
int distance = 0;
static char buffarray[200];

static void adc_init(void)
{
	struct adc_config adc_conf;
	struct adc_channel_config adcch_conf;
	
	adc_read_configuration(&MY_ADC, &adc_conf);
	adcch_read_configuration(&MY_ADC, MY_ADC_CH, &adcch_conf);

	adc_set_conversion_parameters(&adc_conf, ADC_SIGN_OFF, ADC_RES_8, ADC_REF_VCCDIV2);
	adc_set_conversion_trigger(&adc_conf, ADC_TRIG_MANUAL, 1, 0);
	adc_set_clock_rate(&adc_conf, 200000UL);
	adc_write_configuration(&MY_ADC, &adc_conf);
	
	
	//adcch_set_input(&adcch_conf, ADCCH_POS_PIN0, ADCCH_NEG_NONE, 1);
	//adcch_write_configuration(&MY_ADC, MY_ADC_CH, &adcch_conf);
	
	adcch_set_input(&adcch_conf, J2_PIN2, ADCCH_NEG_NONE, 1);
	adcch_write_configuration(&MY_ADC, PIEZO_ADC_CH, &adcch_conf);
}

/*
static uint16_t adc_read(){
	uint16_t result;
	adc_enable(&MY_ADC);
	adc_start_conversion(&MY_ADC, MY_ADC_CH);
	adc_wait_for_interrupt_flag(&MY_ADC, MY_ADC_CH);
	delay_ms(100);
	result = adc_get_result(&MY_ADC, MY_ADC_CH);
	return result;
}
*/

static uint16_t piezo_read(){
	uint16_t result;
	adc_enable(&MY_ADC);
	adc_start_conversion(&MY_ADC, PIEZO_ADC_CH);
	adc_wait_for_interrupt_flag(&MY_ADC, PIEZO_ADC_CH);
	delay_ms(1);
	result = adc_get_result(&MY_ADC, PIEZO_ADC_CH);
	return result;
}

//Fungsi setup timer
void setup_timer(void){
	tc_enable(&TCC0);
	tc_set_overflow_interrupt_callback(&TCC0,print_message);
	tc_set_wgm(&TCC0, TC_WG_NORMAL);
	tc_write_period(&TCC0, 58/8);
	tc_set_overflow_interrupt_level(&TCC0, TC_INT_LVL_HI);
	tc_write_clock_source(&TCC0, TC_CLKSEL_DIV1_gc);
}

//Fungsi ini bukan utk print message, tapi increment nilai variabel "increment" setiap 29us
void print_message(void){
	incremental = incremental + 1;
}

void print_lcd(uint16_t piezo, int ultra) {
	snprintf(buffarray, sizeof(buffarray), "Berat Truk : %6d", piezo);
	gfx_mono_draw_string(buffarray,0, 0, &sysfont);
	snprintf(buffarray, sizeof(buffarray), "Tinggi Truk : %d cm  ", ultra);
	gfx_mono_draw_string(buffarray, 0, 8, &sysfont);
}

void checkLight(uint16_t piezoRead, int ultraRead) {
	if(ultraRead <= 10 && piezoRead <= 250) {
		ioport_set_pin_low(LED_IN);
		ioport_set_pin_low(BUZZ_IN);
	} else if (ultraRead > 10 && piezoRead < 250) {
		ioport_set_pin_high(LED_IN);
		ioport_set_pin_low(BUZZ_IN);
	} else if (ultraRead < 10 && piezoRead > 250) {
		ioport_set_pin_low(LED_IN);
		ioport_set_pin_high(BUZZ_IN);
	} else {
		ioport_set_pin_high(LED_IN);
		ioport_set_pin_high(BUZZ_IN);
	}
}

int main (void)
{
	// Insert system clock initialization code here (sysclk_init()).
	board_init();
	sysclk_init();
	pmic_init();
	gfx_mono_init();
	
	adc_init();
	
	PORTC.DIR |= PIN0_bm;
	PORTE.DIR |= PIN0_bm;
	
	uint16_t result;
	
	gpio_set_pin_high(NHD_C12832A1Z_BACKLIGHT);

	// Workaround for known issue: Enable RTC32 sysclk
	sysclk_enable_module(SYSCLK_PORT_GEN, SYSCLK_RTC);
	while (RTC32.SYNCCTRL & RTC32_SYNCBUSY_bm) {
		// Wait for RTC32 sysclk to become stable
	}
	
	delay_ms(1000);
	setup_timer();
	
	// Insert application code here, after the board has been initialized.
	while(1){
		
		PORTB.DIR = 0b11111111; //Set output
		PORTB.OUT = 0b00000000; //Set low
		PORTB.OUT = 0b11111111; //Set high selama 5us
		delay_us(5);
		PORTB.OUT = 0b00000000; //Kembali menjadi low
		PORTB.DIR = 0b00000000; //Set menjadi input
		delay_us(750); //Delay holdoff selama 750us
		
		int oldinc = incremental;
		delay_us(115); //Delay lagi, kali ini seharusnya pin menjadi high
		
		cpu_irq_enable(); //Mulai interrupt
		while(PORTB.IN & PIN0_bm){
			//Tidak ada apa-apa di sini. Loop ini berfungsi untuk mendeteksi pin 0 PORT B yang berubah menjadi low
		}
		int newinc = incremental; //Catat selisih waktu antara suara dikirim hingga diterima
		cpu_irq_disable(); //Interrupt dimatikan
		
		int inc = newinc - oldinc;
		int newscore = inc/2; //Dibagi 2 seperti rumus sonar
		
		result = piezo_read();
		int ultra_read = 17 - newscore;
		print_lcd(result, ultra_read); 
		checkLight(result, ultra_read);
		
		delay_ms(100);
		incremental = 0; //reset nilai variable incremental
	}
}