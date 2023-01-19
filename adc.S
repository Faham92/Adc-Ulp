

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
  MOVE r2,adc_value          // R2 <- @adc_value
  /* Measure value using ADC1 pad 2 for 100 cycles and move result to R1 */
  ADC r1, 0, 7                // R1 <- ADC1
  ST r1, r2, 0                // R2 <- [R1]  Store value of R1 into R2(memory)
  LD r0,r2,0                  // R0 <- [R2]  Load value of R2(memory) into R0
  JUMPR wake_up,2230,GT       // if R0>Threshold(2230) wake_up the esp32
  JUMP entry   // Jump if last ALU has set overflow flag (ADC > Threshold)                            
        
 
	.global wake_up
   
  wake_up:
          WAKE                          // Trigger wake up
          HALT                    // Stop the ULP program
         
