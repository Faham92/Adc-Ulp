
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
#include "ulp_main.h"  //bibliotheque qui contient les déclarations et défintions des varibles et fonctions initialisées dans le Processeur Ultra Low Power

// Unlike the esp-idf always use these binary blob names
extern const uint8_t ulp_main_bin_start[] asm("_binary_ulp_main_bin_start");
extern const uint8_t ulp_main_bin_end[]   asm("_binary_ulp_main_bin_end");

static void init_run_ulp(esp_adc_cal_characteristics_t*);   //initialisation des pins rtc pour l'adc
static void start_ulp_program();                            // on charge le binaire généré par le code assembleur  
static void temperature (float* , int);
RTC_DATA_ATTR esp_adc_cal_characteristics_t adc_chars;    //initialise les charactéristiques de l'adc dans la mémoire RTC

void app_main() 
{ 
  esp_sleep_wakeup_cause_t cause = esp_sleep_get_wakeup_cause();
  esp_sleep_enable_ulp_wakeup();                       //Prévient le microcontroleur qu'il peut etre reveille par l'ulp
  float temp=0;
  if (cause != ESP_SLEEP_WAKEUP_ULP)
  {
    init_run_ulp(&adc_chars);
    start_ulp_program();
  }
  uint32_t res = esp_adc_cal_raw_to_voltage(ulp_adc_value&0xFFFF, &adc_chars); //conversion adc-voltage 
  temperature(&temp,res); //conversion temperature-voltage
  if(temp >28)
  {
    vTaskDelay(100/portTICK_PERIOD_MS);
    ESP_LOGE("[Main]","Température = %f°C  => Alarme! ",temp);
    while(temp>28)
    {
      res = esp_adc_cal_raw_to_voltage(ulp_adc_value&0xFFFF, &adc_chars); 
      temperature(&temp,res); 
      vTaskDelay(10/portTICK_PERIOD_MS);
    }
    ESP_LOGE("[Main]","Température en dessous du seuil");
  }
  esp_err_t err = ulp_run((&ulp_entry - RTC_SLOW_MEM));
  ESP_ERROR_CHECK(err);
  vTaskDelay(100/portTICK_PERIOD_MS);
  ESP_LOGE("[Main]","Going to sleep");
  esp_deep_sleep_start();                    //passage en mode deep sleep
}


// Use this function for all your first time init like setup() used to be
static void init_run_ulp(esp_adc_cal_characteristics_t* adc_chars) 
{
    rtc_gpio_init(GPIO_NUM_34);
    rtc_gpio_set_direction(GPIO_NUM_34,RTC_GPIO_MODE_INPUT_ONLY);
    adc1_config_channel_atten(ADC1_CHANNEL_6, ADC_ATTEN_DB_11);
    adc1_config_width(ADC_WIDTH_BIT_12);
    esp_adc_cal_characterize(ADC_UNIT_1,ADC_ATTEN_DB_11,ADC_BITWIDTH_12,0,adc_chars);
    adc1_ulp_enable(); 
}

static void start_ulp_program()
{
    /* Start the program */
    vTaskDelay(100/portTICK_PERIOD_MS);
    ulp_load_binary(0, ulp_main_bin_start, (ulp_main_bin_end - ulp_main_bin_start) / sizeof(uint32_t));
}


static void temperature (float *temp,int res)                //fonction qui convertit de la tension d'entree en temperature
{
    *temp = (res - 500.0)/10;
}

