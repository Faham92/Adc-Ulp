

#include "soc/rtc_cntl_reg.h"
#include "soc/rtc_io_reg.h"
#include "soc/soc_ulp.h"
#include "soc/sens_reg.h"


.bss

	.global adc_value
adc_value:
	.long 0


  // ULP code starts here
  
  .text

  .global entry
  .global multisampling
  .global wake_up
  .global valeur
  .global boucle
  
    entry:              
  MOVE r1,0    
  JUMP multisampling      // on saute à la fonction multisampling pour échantillonner la valeur de l'adc 
                                     
wake_up:        
    WAKE                          // Declenche le reveille de l'esp32
    HALT                    // Stop le programme ULP

multisampling: 
    STAGE_RST                //reset le compteur i
    JUMP boucle

boucle:                     //r1+=valeur de l'adc
    STAGE_INC 1
    ADC r0, 0, 7           
    ADD r1,r1,r0
    JUMPS valeur,4,EQ       //Si i==4 on va à la prochaine fonction
    JUMP boucle
 
valeur:
    RSH r1,r1,2            // on divise  r1/4 qui contient la moyenne de nos 4 valeurs
    MOVE r0,adc_value
    ST r1,r0,0           //adc_value contient la valeur échantillonnée de l'adc
    LD r0,r0,0             //charge la valeur de adc_value dans l'adresse de r0 car JUMPR se base sur l'adresse de ce dernier
    JUMPR wake_up,780,ge  // on réveille l'esp32 si la valeur échantillonnée est supérieur à 2000 ce qui équivaut à une température >80°C
    JUMP entry
    








