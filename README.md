# STM32-ChromaWave-Synth  
  
**Project Summary**  
Embedded C project implementing real-time Direct Digital Synthesis (DDS) on an STM32F4 Discovery Board. It converts color data (via I2C sensor) into audio frequency and timbre.  

**Key Technical Features**  
- DDS Implementation: Custom logic for wavetable look-up and phase accumulation for sine, ramp, and square waves.	 
- Real-Time Mixing: Algorithm to blend three waveforms based on the RGB color ratios.	
- Hardware Integration: Implemented I2C and ADC drivers to interface with the RGB color sensor and volume potentiometer.	 
- Buffer Management: Utilized ping-pong buffering logic to ensure low-latency, continuous audio output.

**Core Logic Files**  
The primary synthesis logic, mixing algorithms, and buffer management are contained in main.c.
