
## Version
* Developer: HHH
* Email: harry.hoa.huynh.01@gmail.com
* Date: 4/4/2021

## Goal
* Collect temperature sensor from STM32F401RE, then pass the data to STM32F429ZI via UART using Polling Method. And then display sensor data via a display. 

## Set up
1. STM32F401RE 
* Temperature MCP9700A: PA0	[ADC 10bits with 13 clock cycles]
* UART: TX = PC6 | RX = PC7 [Baud rate of 9600bs, Word length of 8b]

2. STM32F429ZI 
* UART: TX = PC6 | RX = PC7 [Baud rate of 9600bs, Word length of 8b]
* Display the temperature via LCD:
######  RS	PD1
*  EN	PD7
* DB4	PD6
* DB5	PD5
* DB6	PD4
* DB7	PD3
		
## Component
1. STM Nucleo STM32F429ZI
2. STM Nucleo SMT32F401RE
3. Temperature MCP9700A
4. LCD HC16102

## Picture for reference
![STM32CubeMX](https://github.com/HHH-01/STM32/blob/f028fc426ff353cd08ee4fa8f19110355a3476b8/Images/STM32CubeMx.PNG)

![Circuit Set up](https://github.com/HHH-01/STM32/blob/f028fc426ff353cd08ee4fa8f19110355a3476b8/Images/Circuit.jpg)

![Debugging Window](https://github.com/HHH-01/STM32/blob/63101e5e8e58ca9a7d79cf8a742d6c34a3291a6b/Images/DebuggingWindow.PNG)
