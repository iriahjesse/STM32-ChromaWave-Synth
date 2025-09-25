# ChromaWave: Real-Time Audio Synthesizer (STM32F4)

Embedded C project implementing real-time **Direct Digital Synthesis (DDS)** on an **STM32F4 Discovery Board**. This system converts visual data (via an I2C color sensor) into synthesized audio frequency and timbre.

---

### Key Technical Features

* **Real-Time DDS:** Custom C logic for wavetable generation (sine, ramp, square) and phase accumulation, ensuring low-latency sound synthesis.
* **Audio Mixing Algorithm:** Implemented a unique formula to blend three base waveforms based on the real-time ratio of RGB color values, dynamically controlling timbre.
* **Hardware Integration:** Utilized **I2C** to read the color sensor and **ADC** to manage the volume potentiometer, demonstrating full hardware-software interfacing.
* **Buffer Management:** Employed a **ping-pong buffering** architecture within the audio driver to maintain continuous, gapless audio output.

**The primary synthesis logic, mixing algorithms, and buffer management are contained within the `main.c` file.**
