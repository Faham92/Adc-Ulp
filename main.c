
#include <stdio.h>
#include <stdlib.h>
#include <inttypes.h>
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

void init_run_ulp(esp_adc_cal_characteristics_t*);
static void start_ulp_program();
static void temperature (float* , float);


void app_main() 
{
    
  esp_adc_cal_characteristics_t *adc_chars = calloc(1,sizeof(esp_adc_cal_characteristics_t));
  float temp=0;
  vTaskDelay(2000/portTICK_PERIOD_MS);
  esp_sleep_wakeup_cause_t cause = esp_sleep_get_wakeup_cause();
  esp_sleep_enable_ulp_wakeup();
  if (cause != ESP_SLEEP_WAKEUP_ULP)
  {
    init_run_ulp(adc_chars);
    start_ulp_program();
  }
  float res = (uint16_t)ulp_adc_value*2660/4095; 
  temperature(&temp,res); //conversion temperature-voltage 
   while (temp > 80)  
      {
        float res = (uint16_t)ulp_adc_value*2660/4095;    //conversion adc-voltage 
        temperature(&temp,res); //conversion temperature-voltage
        ESP_LOGE("[MAIN]","temperature = %f\n  Voltage = %fmV", temp,res); 
    }
    esp_err_t err = ulp_run((&ulp_entry - RTC_SLOW_MEM));
    ESP_ERROR_CHECK(err);
    ESP_LOGE("[Main]","Going to sleep...");
    esp_deep_sleep_start();
   
}


// Use this function for all your first time init like setup() used to be
void init_run_ulp(esp_adc_cal_characteristics_t *adc_chars) {
    rtc_gpio_init(GPIO_NUM_34);
    // We will be reading and writing on these gpio
    rtc_gpio_set_direction(GPIO_NUM_34,RTC_GPIO_MODE_INPUT_ONLY);
    adc1_config_channel_atten(ADC1_CHANNEL_6, ADC_ATTEN_DB_11);
    adc1_config_width(ADC_WIDTH_BIT_12);
    esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_DB_11,  ADC_WIDTH_BIT_12,0, adc_chars);
    adc1_ulp_enable(); 
}

static void start_ulp_program() {
    /* Start the program */
    vTaskDelay(100/portTICK_PERIOD_MS);
    ulp_load_binary(0, ulp_main_bin_start, (ulp_main_bin_end - ulp_main_bin_start) / sizeof(uint32_t));
}


static void temperature (float *temp,float res)
{
*temp = (res - 500.0)/10.0;
}
