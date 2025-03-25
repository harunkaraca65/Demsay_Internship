HardwareSerial MySerial(1);  // UART1 kullanımı
void setup() {
    Serial.begin(115200);        // ESP32'nin USB seri portu
    MySerial.begin(115200, SERIAL_8N1, 26, 25); // RX = GPIO16, TX = GPIO17
}

void loop() {
    if (MySerial.available()) {
        String receivedData = "";
        while (MySerial.available()) {
            char c = MySerial.read();
            receivedData += c;
            delay(2); // Veri kaybını önlemek için küçük bir gecikme
        }
        Serial.print(receivedData); // Alınan veriyi seri port ekranında göster
    }
}