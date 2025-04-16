# Demsay_Internship
 
Projects I worked on during my Demsay internship program
https://drive.google.com/drive/folders/1EzfbrS7SoxPwu5g6q6GF4zo4jYcyu8LK?usp=drive_link

DEM_PWM_LED

RGB LED Controller with Demedukit

🇬🇧 English:
This project controls an RGB LED using STM32's PWM signals. It cycles through 50 predefined colors with smooth fading. The LED's red, green, and blue channels are driven by TIM16 (red) and TIM1 (blue/green) at 16-bit resolution. Colors defined in 0-255 range are automatically scaled to 0-65535. The fade effect is achieved by gradually interpolating between colors in 10ms steps.

🇹🇷 Türkçe:
Bu proje, STM32'nin PWM sinyalleriyle bir RGB LED'i kontrol eder. 50 önceden tanımlanmış renk arasında yumuşak geçişler yapar. Kırmızı (TIM16), mavi ve yeşil (TIM1) kanalları 16-bit çözünürlükte sürülür. 0-255 aralığındaki renkler otomatik olarak 0-65535'e ölçeklenir. Renkler arası geçişler, 10ms'lik adımlarla kademeli olarak yapılır.


DEM_MULTI_ADC

STM32 & ESP32 UART Data Logger

🇬🇧 English:
This project establishes UART communication between STM32 (DemeduKit) and ESP32. The STM32 reads analog sensor data (two trimpots, NTC thermistor, LDR) via ADC with DMA, processes the values into resistance (Ω), temperature (°C), and light percentage (%), then sends them via UART (115200 baud) to ESP32. The ESP32 forwards the data to a PC serial monitor.

🇹🇷 Türkçe:
Bu proje, STM32 (DemeduKit) ile ESP32 arasında UART haberleşmesi kurar. STM32, ADC+DMA ile iki trimpot direnci (0-10kΩ), NTC sıcaklık (°C) ve LDR ışık (%) değerlerini okuyup ESP32'ye UART (115200 baud) ile iletir. ESP32 bu verileri bilgisayara aktarır.


