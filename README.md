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


DEM_FREERTOS

STM32 RTOS LED Control Project
(Using DemeduKit with FreeRTOS)

🇬🇧 English
This project implements a multi-task LED control system using FreeRTOS on STM32. The system uses three parallel tasks that interact through a shared counter variable. The default task toggles three LEDs every second while incrementing a counter. When the counter reaches 3, a second task suspends the default task and controls another LED. At count 8, a third task activates an emergency LED and buzzer before terminating all tasks.

🇹🇷 Türkçe
Bu proje, STM32'de FreeRTOS kullanarak çoklu görevli LED kontrol sistemi kurar. Üç paralel görev, paylaşılan bir sayaç değişkeni ile etkileşir. Ana görev her saniye üç LED'i toggle ederken sayaç artar. Sayaç 3'e ulaştığında ikinci görev ana görevi duraklatıp başka bir LED'i kontrol eder. Sayaç 8'de ise üçüncü görev acil durum LED'i ve buzzerı aktifleştirip tüm görevleri sonlandırır.

DEM_ACC_ANGLE

STM32 & ESP32 IoT Sensor System

🇬🇧 English
This project uses DemeduKit's built-in STM32 and ESP32 to create a sensor monitoring system. The STM32 reads accelerometer data (LIS2DW12) and temperature (NTC), calculates device orientation angles, and sends the data via UART to the ESP32. The ESP32 then uploads the data to ThingSpeak cloud via WiFi.

🇹🇷 Türkçe
Bu proje, DemeduKit'in dahili STM32 ve ESP32'sini kullanarak bir sensör izleme sistemi oluşturur. STM32, ivmeölçer (LIS2DW12) ve sıcaklık (NTC) verilerini okuyarak cihaz yönelim açılarını hesaplar ve UART ile ESP32'ye gönderir. ESP32 ise bu verileri WiFi üzerinden ThingSpeak bulutuna yükler.

DEM_PLANE_SIM

STM32 & Unity Flight Control System (with DemeduKit)

🇬🇧 English: 
This project creates a flight control system where an STM32 (DemeduKit) reads sensor/input data and sends it via UART to a Unity-based flight simulator. The STM32 collects:

Joystick/Trimpot inputs (Throttle, Yaw, Camera X/Y)

Accelerometer data (Pitch/Roll angles)

Button states (Flaps, Brake, Fire)
and transmits them to Unity, which controls a 3D aircraft model in real-time.

On the STM32 side, ADC is used to read joystick and potentiometer values for throttle, yaw, and camera angle control. The LIS2DW12 accelerometer calculates pitch and roll angles, while GPIO pins monitor button inputs for additional features like flaps, braking, and firing. All this data is formatted and sent via UART at 115200 baud to the Unity application.

In Unity, the serial data is parsed and mapped to aircraft controls. ADC values determine speed and yaw direction, while pitch and roll values directly influence the aircraft's orientation. Camera X/Y controls offer dynamic viewing angles. Take-off mode is activated when flaps are enabled, and full control engages at a defined altitude. Additionally, a fuel consumption system is implemented for added simulation depth.

🇹🇷 Türkçe: 
Bu proje, STM32'nin (DemeduKit) sensör/giriş verilerini okuyup UART ile Unity tabanlı bir uçuş simülatörüne gönderdiği bir uçuş kontrol sistemi oluşturur. STM32:

Joystick/Trimpot değerlerini (Gaz, Yaw, Kamera X/Y)

İvmeölçer verilerini (Pitch/Roll açıları)

Buton durumlarını (Flap, Fren, Ateşleme)
toplayarak bu verileri gerçek zamanlı olarak 3D uçak modelini kontrol eden Unity uygulamasına iletir.

STM32 tarafında, throttle, yaw ve kamera açıları için joystick ve potansiyometre değerleri ADC ile okunur. Pitch ve roll açıları LIS2DW12 ivmeölçer ile hesaplanırken, flap, fren ve ateşleme gibi ek özellikler GPIO pinleri üzerinden algılanır. Tüm veriler yapılandırılmış formatta, 115200 baud hızında UART üzerinden Unity’ye gönderilir.

Unity tarafında, seri porttan gelen veriler ayrıştırılır ve uçak kontrol sistemine aktarılır. ADC değerlerine göre hız ve yön belirlenirken, pitch ve roll açıları doğrudan uçağın yönelimine etki eder. Kamera X/Y kontrolleri dinamik görüş açıları sunar. Flap aktifken kalkış modu devreye girer ve belirli bir yüksekliğe ulaşıldığında tam kontrol sağlanır. Ayrıca, yakıt tüketimi gibi ekstra simülasyon öğeleri de sisteme entegre edilmiştir.
