
## Version
* Developer: HHH
* Email: harry.hoa.huynh.01@gmail.com
* Date: 4/2021

## Goal
* Collect temperature sensor from STM32F401RE, then pass the data to STM32F429ZI via UART using Polling Method. And then display sensor data via a display. 

## Set up
1. STM32F401RE 
* Temperature MCP9700A: PA0	[ADC 10bits with 13 clock cycles]
* UART: TX = PC6 | RX = PC7 [Baud rate of 9600bs, Word length of 8b]

2. STM32F429ZI 
* UART: TX = PC6 | RX = PC7 [Baud rate of 9600bs, Word length of 8b]
* Display the temperature via LCD:
* + RS	PD1
* + EN	PD7
* + DB4	PD6
* + DB5	PD5
* + DB6	PD4
* + DB7	PD3
		
## Component
1. STM Nucleo STM32F429ZI
2. STM Nucleo SMT32F401RE
3. Temperature MCP9700A
4. LCD HC16102

## Images
Ignore my STM32 and PIR in the circuit.

![STM32CubeMX](https://github.com/HHH-01/STM32-UART/blob/1eaeda5f7e291fc096ddba685d724d1e20f56898/Images/STM32F401RET.PNG)
![STM32CubeMX](https://github.com/HHH-01/STM32-UART/blob/1eaeda5f7e291fc096ddba685d724d1e20f56898/Images/STM32F429ZIT.PNG)

![Circuit Set up1](https://github.com/HHH-01/STM32-UART/blob/1eaeda5f7e291fc096ddba685d724d1e20f56898/Images/Circuit%20Setup1.jpg)
![Circuit Set up2](https://github.com/HHH-01/STM32-UART/blob/1eaeda5f7e291fc096ddba685d724d1e20f56898/Images/CircuitSetup2.jpg)
![Circuit Set up3](https://github.com/HHH-01/STM32-UART/blob/1eaeda5f7e291fc096ddba685d724d1e20f56898/Images/CircuitSetup3.jpg)
