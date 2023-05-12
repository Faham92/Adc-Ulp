#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"
#include "soc/rtc_io_reg.h"
#include "soc/soc_ulp.h"
#include "soc/rtc_i2c_reg.h"



.bss

	.global adc_value
adc_value:
	.long 0

  .global flag_alarme
flag_alarme:
  .long 0

  // ULP code starts here
  
  .text


  .global entry
  .global entry2

.macro boucle nom_boucle condition
STAGE_INC 1                   //i++
ADC r0, 0, 7           
ADD r1,r1,r0                 //r1+=valeur de l'adc
JUMPS \condition,4,EQ       //Si i==4 on va à la prochaine fonction
JUMP \nom_boucle
.endm

.macro condition entree seuil comparateur sortie
RSH r1,r1,2            // on divise  r1/4 qui contient la moyenne de nos 4 valeurs
MOVE r0,adc_value
ST r1,r0,0           //adc_value contient la valeur échantillonnée de l'adc
LD r0,r0,0           //charge la valeur de adc_value dans l'adresse de r0 car JUMPR se base sur l'adresse de ce dernier
ADD r2,r2,1
JUMPR \sortie,\seuil,\comparateur  // on réveille l'esp32 si la valeur échantillonnée est supérieur ou inférieur à un seuil fixé par l'utilisateur
SUB r1,r2,8  
JUMP exit,EQ
JUMP \entree
.endm0

  
entry: 
 MOVE r2,0   
 MOVE r3,flag_alarme  
 JUMP multisampling

 multisampling: 
 MOVE r1,0 
 STAGE_RST                //reset le compteur i
 JUMP boucleSup 
                                     
wake_up:        
    MOVE r1,1 //r1=1      
    ST r1,r3,0 //[r3]=r1                 
    WAKE       // Declenche le reveille de l'esp32
    HALT
    

entry2:   
    STAGE_RST
    MOVE r3,flag_alarme
    MOVE r2,0  
    JUMP multisampling2

 multisampling2: 
 MOVE r1,0              
 JUMP boucleInf
    
fin:
MOVE r1,0 //r2=0
ST r1,r3,0  //[r3]=r2
WAKE
HALT

exit:
MOVE r2,0
HALT

                    
boucleSup:  
STAGE_INC 1                   //i++
ADC r0, 0, 7           
ADD r1,r1,r0                 //r1+=valeur de l'adc
JUMPS ConditionSup,4,EQ       //Si i==4 on va à la prochaine fonction
JUMP boucleSup                

boucleInf:                     
boucle boucleInf ConditionInf
 

ConditionSup:
condition multisampling 1330 gt wake_up

ConditionInf:
condition multisampling2 1270 lt fin

