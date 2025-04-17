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

🇬🇧 STM32 & Unity Flight Control System
This project establishes a flight control system in which an STM32-based embedded device (DemeduKit) collects sensor and user input data and transmits it in real-time to a Unity-based flight simulator. The STM32 microcontroller reads joystick, potentiometer, and button inputs, and calculates pitch and roll angles using an onboard accelerometer. All data is transmitted via UART at 115200 baud to a PC running Unity.

On the Unity side, a C# script parses the incoming serial data and updates the aircraft's speed, rotation, orientation, and camera movements accordingly. In addition to basic control features, the system also includes takeoff logic, flap management, fuel consumption simulation, camera rotation, and firing controls.

STM32 Side Implementation:
Reads ADC values from joystick and potentiometers (Throttle, Yaw, Cam X/Y).

Calculates pitch and roll angles using a LIS2DW12 accelerometer.

Detects button inputs (Flaps, Brake, Fire1/2) through GPIO.

Transmits all collected data in structured format over UART to the PC.

Unity Side Implementation:
Parses data received via the serial port and maps them to variables.

Controls aircraft speed and yaw rotation based on ADC input ranges.

Updates aircraft orientation using the provided pitch and roll values.

Adjusts the camera angle based on joystick inputs for immersive control.

Enables takeoff mode when conditions (speed + flap) are met; full control is granted once altitude increases.

Simulates fuel consumption over time to add realism to the experience.

🇹🇷 STM32 & Unity Uçuş Kontrol Sistemi
Bu proje, STM32 tabanlı bir gömülü sistemin (DemeduKit) sensör ve giriş verilerini okuyarak Unity oyun motorunda geliştirilmiş bir uçuş simülasyonuna gerçek zamanlı olarak aktardığı bir uçuş kontrol sistemini kapsamaktadır. STM32 mikrodenetleyici, uçuş kontrolöründen gelen joystick, potansiyometre ve buton girişlerini toplar; ayrıca ivmeölçer (accelerometer) üzerinden uçağın eğim verilerini (pitch ve roll açıları) hesaplar. Bu veriler UART üzerinden, 115200 baud hızında seri bağlantı aracılığıyla Unity’ye aktarılır.

Unity tarafında geliştirilen C# betiği, bu seri verileri işler ve 3D uçak modelinin hız, yön, açı ve kamera kontrollerini senkronize şekilde günceller. Sistem, sadece temel uçuş kontrollerini değil; aynı zamanda kalkış modu, flap kullanımı, yakıt tüketimi, kamera açısı kontrolü ve ateşleme gibi ek işlevleri de içerir.

STM32 Tarafında Gerçekleştirilenler:
ADC kullanılarak joystick ve potansiyometre verileri okunur (Throttle, Yaw, Kamera X/Y).

LIS2DW12 ivmeölçer aracılığıyla pitch ve roll açıları hesaplanır.

Butonlar (GPIO) üzerinden flap, fren ve ateşleme gibi girişler algılanır.

Tüm bu veriler yapılandırılmış formatta UART ile bilgisayara iletilir.

Unity Tarafında Gerçekleştirilenler:
Seri port üzerinden gelen veriler ayrıştırılır ve ilgili değişkenlere aktarılır.

ADC değerlerine göre uçağın hızı ve yaw hareketleri belirlenir.

Pitch ve roll açıları doğrudan Unity içindeki uçağın dönüşlerine yansıtılır.

Kamera X/Y kontrolleri ile uçuş sırasında oyuncuya farklı görüş açıları sağlanır.

Flap açıkken kalkış modu etkinleştirilir; belirli yükseklikten sonra tam uçuş kontrolü aktif hale gelir.

Yakıt zamanla azalır; bu da daha gerçekçi bir uçuş simülasyonu oluşturur.
