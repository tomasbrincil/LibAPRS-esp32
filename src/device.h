#include "constants.h"

#ifndef DEVICE_CONFIGURATION
#define DEVICE_CONFIGURATION

// CPU settings
#ifndef TARGET_CPU
    #define TARGET_CPU m328p
#endif

#ifndef F_CPU
    #define F_CPU 16000000
#endif

#ifndef FREQUENCY_CORRECTION
    #define FREQUENCY_CORRECTION 0
#endif

// Port settings
#if TARGET_CPU == m328p
    #define DAC_PORT PORTD
    #define DAC_DDR  DDRD
    #define LED_PORT PORTB
    #define LED_DDR  DDRB
    #define ADC_PORT PORTC
    #define ADC_DDR  DDRC
#endif

#endif


#define GPIO_AUDIO_TRIGGER GPIO_NUM_37
// Audio input pin is controlled by I2S_ADC_CHANNEL below.
// Audio output pin is ...hardcoded as GPIO 25?
#define GPIO_PTT_OUT GPIO_NUM_33
#define ESP_INTR_FLAG_DEFAULT 0


//i2s number
#define TNC_I2S_NUM           I2S_NUM_0
#define DESIRED_SAMPLE_RATE   (13200)
#define OVERSAMPLING          (8)
//i2s sample rate
#define TNC_I2S_SAMPLE_RATE   (DESIRED_SAMPLE_RATE * OVERSAMPLING)
#define CONFIG_AFSK_DAC_SAMPLERATE (TNC_I2S_SAMPLE_RATE)

//i2s data bits
#define TNC_I2S_SAMPLE_BITS   (I2S_BITS_PER_SAMPLE_16BIT)
// 125ms of audio should be plenty I think
#define TNC_I2S_BUFLEN        (TNC_I2S_SAMPLE_RATE / 8)

//I2S data format
#define TNC_I2S_FORMAT        (I2S_CHANNEL_FMT_ONLY_RIGHT)
#define TNC_I2S_CHANNEL_NUM   (1)

//I2S built-in ADC unit
#define I2S_ADC_UNIT              ADC_UNIT_1
#define I2S_ADC_CHANNEL           ADC1_CHANNEL_0

#define KEEP_RECORDING_THRESH  (5)