# Simulated Function Generator
A software-defined function generator implemented on the STM32L476 Nucleo board, capable of generating sine, square, triangle, and sawtooth waveforms.

## Description
This project integrates different components such as the STM32L476 Nucleo, 4x4 keypad, and a MCP4921 DAC to create a program that simulates the behavior similar to a function generator. A function generator generates electrical waveforms and outputs corresponding frequency and duty cycle according to its setting. These different components work together to be able to generate a sin, triangle, sawtooth, and square waveforms. The STM32L476 MCU handles processing tasks and reads keypad inputs to control various functions efficiently. A keypad is integrated into this project to be able to press on a specific key and change the displayed waveforms, frequency from 100 Hz - 500 Hz in increments of 100 Hz, increase or decrease duty cycle (minimum 10% and maximum 90%) on a square wave. The MCP4921 SPI DAC converts a voltage input into a digital value, which is then translated back into an analog voltage output ranging from 0 to 3 volts, ultimately measurable on an oscilloscope. LEDs are added into the circuit to verify and keep track of the keypads functionality since the LEDs display the key pressed on the LEDs as a binary number. 


Specifics:
	•	Waveform type: Sine, Square, Triangle, Sawtooth
	•	Frequency: Adjustable in real-time
	•	Duty Cycle: For square wave
	•	Amplitude: 3V
	•	Real-time updates: Through oscilloscope interface

* This project was created as part of an embedded systems course.

⸻

🛠️ Hardware & Tools

Hardware
	•	STM32L476RG Nucleo board
	•	USB-Mini cable (data-capable)
	•	Oscilloscope for waveform visualization

Software
	•	STM32CubeIDE 1.15.0
	•	Git for version control

## Author
Elizabeth Acevedo
