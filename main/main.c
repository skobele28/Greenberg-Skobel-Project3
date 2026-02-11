#include "freertos/FreeRTOS.h"
#include <freertos/task.h>
#include <sys/time.h>
#include "../managed_components/esp-idf-lib__hd44780/hd44780.h"
#include "../managed_components/esp-idf-lib__esp_idf_lib_helpers/esp_idf_lib_helpers.h"
#include <inttypes.h>
#include <stdio.h>
#include "driver/gpio.h"
#include "esp_adc/adc_oneshot.h"
#include "driver/ledc.h"

// ignition subsystem
#define PSEAT_PIN       GPIO_NUM_7      // passenger seat button pin 7
#define DSEAT_PIN       GPIO_NUM_5      // driver seat button pin 5
#define PBELT_PIN       GPIO_NUM_15     // passenger belt switch pin 15
#define DBELT_PIN       GPIO_NUM_6      // driver belt switch pin 6
#define IGNITION_BUTTON GPIO_NUM_4      // ignition button pin 4
#define READY_LED       GPIO_NUM_20     // ready LED pin 20
#define SUCCESS_LED     GPIO_NUM_19     // success LED pin 19
#define ALARM_PIN       GPIO_NUM_18     // alarm pin 18

// wiper subsystem
#define WIPER_CONTROL   ADC_CHANNEL_8   // wiper control (potentiometer) ADC1 channel 8
#define INT_WIPER_CONTROL      ADC_CHANNEL_9   // wiper intermittence control (potentiometer) ADC1 channel 9
#define ADC_ATTEN       ADC_ATTEN_DB_12 // set ADC attenuation
#define BITWIDTH        ADC_BITWIDTH_12 // set ADC bitwidth
#define WIPER_POTENT_OFF    (500)       // adcmV level for wipers off
#define WIPER_POTENT_LOW    (1570)      // adcmV level for wipers low
#define WIPER_POTENT_HI     (2650)      // adcmV level for wipers high
#define WIPER_INT_SHORT     (910)       // adcmV level for intermittence short
#define WIPER_INT_LONG      (1960)      // adcmV level for intermittence long

#define LEDC_TIMER      LEDC_TIMER_0
#define LEDC_MODE       LEDC_LOW_SPEED_MODE
#define LEDC_OUTPUT_IO      (16)        // pwm signal to motor pin 16
#define LEDC_CHANNEL    LEDC_CHANNEL_0
#define LEDC_DUTY_RES   LEDC_TIMER_13_BIT // set duty resolution to 13 bits

//Set the PWM signal frequency required by servo motor
#define LEDC_FREQUENCY      (50) // Frequency in Hertz. 

//Calculate the values for the minimum (0.75ms) and center (1.5ms) servo pulse widths
#define LEDC_DUTY_MIN       (210) // Set duty to ~3.75%.
#define LEDC_DUTY_CENTER    (614) // Set duty to ~7.5%.

bool dseat = false;     //Detects when the driver is seated 
bool pseat = false;     //Detects when the passenger is seated
bool dbelt = false;     //Detects when the driver seatbelt is on
bool pbelt = false;     //Detects when the passenger seatbelt is on
bool ignition = false;  //Detects when the ignition is turned on
int executed = 0;       //keeps track of print statements
int ready_led = 0;      //keeps track of whether ready_led should be on or off
int ignition_off = 0;   //keeps track of whether the ignition can be turned off
int wiper = 0;          //keeps track of wiper setting
int wiper_int = 0;      //keeps track of wiper intermittent setting

static void ledc_initialize(void);

// Task to set wipers according to WIPER_CONTROL (potentiometer) and intermittence
void wiper_task(void *pvParameter)
{
    while(executed != 3){
        // if wiper is set to OFF, make motor stationary at minimum angle
        if(wiper == 0){
            ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, LEDC_DUTY_MIN);
            ledc_update_duty(LEDC_MODE, LEDC_CHANNEL);
            vTaskDelay(10/portTICK_PERIOD_MS);
        }
        
        // if wiper is set to INT, rotate to 90 degrees and back at low speed
        else if(wiper == 1){
            int i;
            int delay = 1500/(LEDC_DUTY_CENTER - LEDC_DUTY_MIN);
            for(i = LEDC_DUTY_MIN; i <= LEDC_DUTY_CENTER; i = i + 5){
                ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, i);
                ledc_update_duty(LEDC_MODE, LEDC_CHANNEL);
                vTaskDelay((delay * 5)/portTICK_PERIOD_MS);
            }
            for(i = LEDC_DUTY_CENTER; i >= LEDC_DUTY_MIN; i = i - 5){
                ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, i);
                ledc_update_duty(LEDC_MODE, LEDC_CHANNEL);
                vTaskDelay((delay * 5)/portTICK_PERIOD_MS);
            }
            // if intermittent SHORT, delay 1 second
            if (wiper_int == 1){
                vTaskDelay(1000/portTICK_PERIOD_MS);
            }
            // if intermittence MED, delay 3 seconds
            else if (wiper_int == 2){
                vTaskDelay(3000/portTICK_PERIOD_MS);
            }
            // if intermittence LONG, delay 5 seconds
            else if (wiper_int == 3){
                vTaskDelay(5000/portTICK_PERIOD_MS);
            }
        }
            
        // if wiper set to LOW, rotate to 90 degrees and back to min at low speed (3s period)
        else if(wiper == 2){
            int i;
            int delay = 1500/(LEDC_DUTY_CENTER - LEDC_DUTY_MIN);
            for(i = LEDC_DUTY_MIN; i <= LEDC_DUTY_CENTER; i = i + 5){
                ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, i);
                ledc_update_duty(LEDC_MODE, LEDC_CHANNEL);
                vTaskDelay((delay * 5)/portTICK_PERIOD_MS);
            }
            for(i = LEDC_DUTY_CENTER; i >= LEDC_DUTY_MIN; i = i - 5){
                ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, i);
                ledc_update_duty(LEDC_MODE, LEDC_CHANNEL);
                vTaskDelay((delay * 5)/portTICK_PERIOD_MS);
            }
        }

        // if wiper set to HIGH, rotate to 90 degrees and back to min at high speed (1.2s period)
        else if (wiper == 3){
            int i;
            int delay = 600/(LEDC_DUTY_CENTER - LEDC_DUTY_MIN);
            for(i = LEDC_DUTY_MIN; i <= LEDC_DUTY_CENTER; i = i + 10){
                ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, i);
                ledc_update_duty(LEDC_MODE, LEDC_CHANNEL);
                vTaskDelay((delay * 10)/portTICK_PERIOD_MS);
            }
            for(i = LEDC_DUTY_CENTER; i >= LEDC_DUTY_MIN; i = i - 10){
                ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, i);
                ledc_update_duty(LEDC_MODE, LEDC_CHANNEL);
                vTaskDelay((delay * 10)/portTICK_PERIOD_MS);
            }
        }
        //vTaskDelay(10/portTICK_PERIOD_MS);
    }
}

void app_main(void)
{

    int wiper_adc_bits;                       // wiper potentiometer ADC reading (bits)
    int wiper_adc_mV;                         // wiper potentiometer ADC reading (mV)
    int int_wiper_adc_bits;                   // intermittent potent ADC reading (bits)
    int int_wiper_adc_mV;                     // intermittent potent ADC reading (mV)


    // set driver seat pin config to input and internal pullup
    gpio_reset_pin(DSEAT_PIN);
    gpio_set_direction(DSEAT_PIN, GPIO_MODE_INPUT);
    gpio_pullup_en(DSEAT_PIN);

    // set passenger seat pin config to input and internal pullup
    gpio_reset_pin(PSEAT_PIN);
    gpio_set_direction(PSEAT_PIN, GPIO_MODE_INPUT);
    gpio_pullup_en(PSEAT_PIN);

    // set driver belt pin config to input and internal pullup
    gpio_reset_pin(DBELT_PIN);
    gpio_set_direction(DBELT_PIN, GPIO_MODE_INPUT);
    gpio_pullup_en(DBELT_PIN);

    // set passenger belt pin config to input and internal pullup
    gpio_reset_pin(PBELT_PIN);
    gpio_set_direction(PBELT_PIN, GPIO_MODE_INPUT);
    gpio_pullup_en(PBELT_PIN);

    // set ignition button config to input and internal pullup
    gpio_reset_pin(IGNITION_BUTTON);
    gpio_set_direction(IGNITION_BUTTON, GPIO_MODE_INPUT);
    gpio_pullup_en(IGNITION_BUTTON);

    // set ready led pin config to output, level 0
    gpio_reset_pin(READY_LED);
    gpio_set_direction(READY_LED, GPIO_MODE_OUTPUT);

    // set success led pin config to output, level 0
    gpio_reset_pin(SUCCESS_LED);
    gpio_set_direction(SUCCESS_LED, GPIO_MODE_OUTPUT);

    //set alarm pin config to output, level 0
    gpio_reset_pin(ALARM_PIN);
    gpio_set_direction(ALARM_PIN, GPIO_MODE_OUTPUT);

    // configure adc oneshot for wiper potent and intermittent potent
    adc_oneshot_unit_init_cfg_t init_config1 = {
        .unit_id = ADC_UNIT_1,
    };                                                  // Unit configuration
    adc_oneshot_unit_handle_t adc1_handle;              // Unit handle
    adc_oneshot_new_unit(&init_config1, &adc1_handle);  // Populate unit handle

    adc_oneshot_chan_cfg_t config = {
        .atten = ADC_ATTEN,
        .bitwidth = BITWIDTH
    };                                                  // Channel config
    adc_oneshot_config_channel                          // Configure channel
    (adc1_handle, WIPER_CONTROL, &config);

    adc_oneshot_config_channel
    (adc1_handle, INT_WIPER_CONTROL, &config);

    adc_cali_curve_fitting_config_t cali_config = {
        .unit_id = ADC_UNIT_1,
        .chan = WIPER_CONTROL,
        .atten = ADC_ATTEN,
        .bitwidth = BITWIDTH
    };                                                  // Calibration config
    adc_cali_handle_t adc1_cali_chan_handle;            // Calibration handle
    adc_cali_create_scheme_curve_fitting                // Populate cal handle
    (&cali_config, &adc1_cali_chan_handle);

    // configuration structure for lcd
    hd44780_t lcd =
    {
        .write_cb = NULL,
        .font = HD44780_FONT_5X8,
        .lines = 2,
        .pins = {
            .rs = GPIO_NUM_39,
            .e  = GPIO_NUM_37,
            .d4 = GPIO_NUM_36,
            .d5 = GPIO_NUM_35,
            .d6 = GPIO_NUM_48,
            .d7 = GPIO_NUM_47,
            .bl = HD44780_NOT_USED
        }
    };

    // initialize lcd
    ESP_ERROR_CHECK(hd44780_init(&lcd));

    // Set the LEDC peripheral configuration
    ledc_initialize();
    // Set duty to 3.75% (0 degrees)
    ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, LEDC_DUTY_MIN);
    // Update duty to apply the new value
    ledc_update_duty(LEDC_MODE, LEDC_CHANNEL);

    while (1){
        
        adc_oneshot_read
        (adc1_handle, WIPER_CONTROL, &wiper_adc_bits);              // Read ADC bits (wiper)
        
        adc_cali_raw_to_voltage
        (adc1_cali_chan_handle, wiper_adc_bits, &wiper_adc_mV);         // Convert to mV (wiper)

        adc_oneshot_read
        (adc1_handle, INT_WIPER_CONTROL, &int_wiper_adc_bits);           // Read ADC bits (wiper int)
        
        adc_cali_raw_to_voltage
        (adc1_cali_chan_handle, int_wiper_adc_bits, &int_wiper_adc_mV); // Convert to mV (wiper int)


        // Task Delay to prevent watchdog
        vTaskDelay(10 / portTICK_PERIOD_MS);

        // initialize variables in relation to GPIO pin inputs
        dseat = gpio_get_level(DSEAT_PIN)==0;
        pseat = gpio_get_level(PSEAT_PIN)==0;
        dbelt = gpio_get_level(DBELT_PIN)==0;
        pbelt = gpio_get_level(PBELT_PIN)==0;
        ignition = gpio_get_level(IGNITION_BUTTON)==0;

        // if the driver seat button is pressed, print the welcome message once
        if (dseat){
            if (executed == 0){     // if executed equals 0, print welcome message
                printf("Welcome to enhanced alarm system model 218-W25 \n"); 
                executed = 1;       // set executed = 1 so welcome message only prints once
            }
        }

        // if all of the conditions are met
        if (dseat && pseat && dbelt && pbelt){
            //set ready led to ON
            if (executed == 1 && ready_led == 0){
                gpio_set_level(READY_LED, 1);
                ready_led = 1;
            }
            // if ignition button is pressed while all conditions are met
            if (ignition == true && executed == 1){
                // turn on ignition LED and turn off ready LED
                gpio_set_level(SUCCESS_LED, 1);
                gpio_set_level(READY_LED, 0);
                gpio_set_level(ALARM_PIN, 0);
                // print engine started message once
                printf("Engine started!\n");
                executed = 2;       // set executed = 2 so engine started message only prints once
            }
        }
            
        // otherwise (at least one condition is not satisfied)
        else{
            // set ready LED to OFF and set variable ready_led to 0
            gpio_set_level(READY_LED,0);
            ready_led = 0;
            // if ignition button is pressed while conditions are not satisfied
            if (ignition==true && executed < 2){
                    // turn on alarm buzzer
                    gpio_set_level(ALARM_PIN, 1);
                    printf("Ignition inhibited.\n");
                    // check which conditions are not met, print corresponding message
                    if (!pseat){
                        printf("Passenger seat not occupied.\n");
                    }
                    if (!dseat){
                        printf("Driver seat not occupied.\n");
                    }
                    if (!pbelt){
                        printf("Passenger seatbelt not fastened.\n");
                    }
                    if (!dbelt){
                        printf("Drivers seatbelt not fastened.\n");
                    }
                    executed = 4;   // set executed = 4 so messages print only once
                
            }
        }
        
        // if executed = 4 (failed ignition) and ignition button is released
        if (ignition == false && executed == 4){
            // reset to state after welcome message, testing for conditions
            executed = 1;
        }

        // if iginition successful, set wipers according to potentiometers
        if(executed == 2){
            // print "Wipers: " on LCD screen, line 1
            hd44780_gotoxy(&lcd, 0, 0);
            hd44780_puts(&lcd, "Wipers: ");

            // create wiper task
            xTaskCreate(wiper_task, "Wiper_Task", 2048, NULL, 5, NULL);

            // if potentiometer set to off, write "wipers: off" on LCD, set wiper = 0
            if(wiper_adc_mV < WIPER_POTENT_OFF){
                hd44780_gotoxy(&lcd, 8, 0);
                hd44780_puts(&lcd, "OFF ");
                hd44780_gotoxy(&lcd, 0, 1);
                hd44780_puts(&lcd, "          ");
                wiper = 0;
            }
            
            // if potentiometer set to int, write "wipers: int" on LCD, set wiper = 1
            else if(wiper_adc_mV >= WIPER_POTENT_OFF && wiper_adc_mV < WIPER_POTENT_LOW){
                hd44780_gotoxy(&lcd, 8, 0);
                hd44780_puts(&lcd, "INT  ");
                wiper = 1;
                // if int short, write "int: short" on LCD, set wiper_int = 1
                if (int_wiper_adc_mV < WIPER_INT_SHORT){
                    hd44780_gotoxy(&lcd, 0, 1);
                    hd44780_puts(&lcd, "INT: SHORT");
                    wiper_int = 1;
                    }
                
                // if int medium, write "int: med" on LCD, set wiper_int = 2
                else if (int_wiper_adc_mV >= WIPER_INT_SHORT && int_wiper_adc_mV < WIPER_INT_LONG){
                    hd44780_gotoxy(&lcd, 0, 1);
                    hd44780_puts(&lcd, "INT: MED  ");
                    wiper_int = 2;
                    }
                
                // if int long, write "int: long" on LCD, set wiper_int = 3
                else if (int_wiper_adc_mV >= WIPER_INT_LONG){
                    hd44780_gotoxy(&lcd, 0, 1);
                    hd44780_puts(&lcd, "INT: LONG  ");
                    wiper_int = 3;
                    }
            }
                
            // if wipers set to low, write "wipers: low" on LCD, set wiper = 2
            else if(wiper_adc_mV >= WIPER_POTENT_LOW && wiper_adc_mV < WIPER_POTENT_HI){
                hd44780_gotoxy(&lcd, 8, 0);
                hd44780_puts(&lcd, "LOW ");
                hd44780_gotoxy(&lcd, 0, 1);
                hd44780_puts(&lcd, "          ");
                wiper = 2;
            }

            // if wipers set to high, write "wipers: high" on LCD, set wiper = 3
            else if(wiper_adc_mV >= WIPER_POTENT_HI){
                hd44780_gotoxy(&lcd, 8, 0);
                hd44780_puts(&lcd, "HIGH");
                hd44780_gotoxy(&lcd, 0, 1);
                hd44780_puts(&lcd, "          ");
                wiper = 3;      
            }
        }



        // if ignition is successfully started and then ignition is released, set ignition_off = 1
        if (executed == 2 && ignition == false){
            ignition_off = 1;
        }

        // if ignition_off = 1 and inition is pressed, turn off all LEDs
        if (ignition_off==1 && ignition == true){
            gpio_set_level(SUCCESS_LED,0);          // turn off ignition
            hd44780_clear(&lcd);                    // turn off wiper lcd
            executed = 3;                           // set executed = 3 to keep LEDs off, exit wiper task loop
        }
    }
}

static void ledc_initialize(void)
{
    // Prepare and then apply the LEDC PWM timer configuration
    ledc_timer_config_t ledc_timer = {
        .speed_mode       = LEDC_MODE,
        .duty_resolution  = LEDC_DUTY_RES,
        .timer_num        = LEDC_TIMER,
        .freq_hz          = LEDC_FREQUENCY,  // Set output frequency at 50 Hz
        .clk_cfg          = LEDC_AUTO_CLK
    };
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));

    // Prepare and then apply the LEDC PWM channel configuration
    ledc_channel_config_t ledc_channel = {
        .speed_mode     = LEDC_MODE,
        .channel        = LEDC_CHANNEL,
        .timer_sel      = LEDC_TIMER,
        .intr_type      = LEDC_INTR_DISABLE,
        .gpio_num       = LEDC_OUTPUT_IO,
        .duty           = 0, // Set duty to 0%
        .hpoint         = 0
    };
ledc_channel_config(&ledc_channel);
}