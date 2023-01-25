
#include <stdio.h>
#include "esp_sleep.h"
#include "sdkconfig.h"
#include "driver/rtc_io.h"
#include "esp32/ulp.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/adc.h"
#include "esp_adc_cal.h"



// Our header
#include "ulp_main.h"

// Unlike the esp-idf always use these binary blob names
extern const uint8_t ulp_main_bin_start[] asm("_binary_ulp_main_bin_start");
extern const uint8_t ulp_main_bin_end[]   asm("_binary_ulp_main_bin_end");

void init_run_ulp();
static void start_ulp_program();
static void temperature (float* , int);


void app_main() 
{ 
  esp_adc_cal_characteristics_t adc_chars;
  float temp=0;
  esp_sleep_wakeup_cause_t cause = esp_sleep_get_wakeup_cause();
  esp_sleep_enable_ulp_wakeup();
  if (cause != ESP_SLEEP_WAKEUP_ULP)
    {
      init_run_ulp();
      start_ulp_program();
    }
     esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_DB_11,  ADC_WIDTH_BIT_12,0, &adc_chars);
     esp_err_t err = ulp_run((&ulp_entry - RTC_SLOW_MEM));
     ESP_ERROR_CHECK(err);
     vTaskDelay(100/portTICK_PERIOD_MS);
      while((uint16_t)ulp_adc_value >= 800)  
   // while(1)
      {
        vTaskDelay(1000/portTICK_PERIOD_MS);
        uint32_t res = esp_adc_cal_raw_to_voltage((uint16_t)ulp_adc_value, &adc_chars); //conversion adc-voltage 
        temperature(&temp,res); //conversion temperature-voltage
        ESP_LOGE("[Main]","température = %f°C V= %lumV",temp,res); 
        
      }
     ESP_LOGE("[Main]","Going to sleep...");
     esp_deep_sleep_start();
   
}


// Use this function for all your first time init like setup() used to be
void init_run_ulp() {
    rtc_gpio_init(GPIO_NUM_34);
    //rtc_gpio_init(GPIO_NUM_2);
    // We will be reading and writing on these gpio
    rtc_gpio_set_direction(GPIO_NUM_34,RTC_GPIO_MODE_INPUT_ONLY);
    //rtc_gpio_set_direction(GPIO_NUM_2,RTC_GPIO_MODE_OUTPUT_ONLY);
    adc1_config_channel_atten(ADC1_CHANNEL_6, ADC_ATTEN_DB_11);
    adc1_config_width(ADC_WIDTH_BIT_12);
    adc1_ulp_enable(); 
}

static void start_ulp_program() {
    /* Start the program */
    vTaskDelay(100/portTICK_PERIOD_MS);
    ulp_load_binary(0, ulp_main_bin_start, (ulp_main_bin_end - ulp_main_bin_start) / sizeof(uint32_t));
}


static void temperature (float *temp,int res)
{
    *temp = (res - 500.0)/10;
}
