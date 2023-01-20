


#include "soc/rtc_cntl_reg.h"
#include "soc/rtc_io_reg.h"
#include "soc/soc_ulp.h"
#include "soc/sens_reg.h"

.bss

  .global wake_up
  
	.global adc_value
adc_value:
	.long 0

  // ULP code starts here
  
  .text

  .global entry

entry:      
  MOVE r1,adc_value          // R1 <- @adc_value
  /* Measure value using ADC1 pad 2 for 100 cycles and move result to R0 */
  ADC r0, 0, 7                // R0 <- ADC1
  ST r0,r1,0                  //charge la valeur de ADC1 dans R
  JUMPR wake_up,2000,ge      //compare la valeur de R0 avec le seuil qui vaut 2000
  JUMP entry   // Jump if last ALU has set overflow flag (ADC > seuil)                            
        
 
	.global wake_up
   
  wake_up:
          WAKE                          // Trigger wake up
          HALT                    // Stop the ULP program
         
