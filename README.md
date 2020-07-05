# nucleo_F030R8_tone

A simple app for an STM32 **Nucleo-F030R8** board that produces a variable freqency tone in the audio range based on a potentiometer. Piezo speaker is driven with a differential square wave to get 6.6Vp-p.

Place a piezo speaker across ```PA6``` and ```PB6```, which are mapped to ```TIM16_CH1``` and ```TIM16_CH1N```.

Connect a potentiometer wiper at ```PA0```, which is mapped to ```ADC1_IN0```.  Connect the ends of the potentiometer to ```CN7``` pin 8 (```GND```) and pin 12 (```3.3V```).

The current frequency is output on the STLINK UART (38400,8N1) and udated every 100 ms.

The TIM16 and ADC1 drivers are hand-written.
