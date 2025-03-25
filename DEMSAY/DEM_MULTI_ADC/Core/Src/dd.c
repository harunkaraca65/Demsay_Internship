// DMA ile ADC değerlerini tutmak için global değişkenler
uint16_t ADC_VAL[4];
int isADCFinished = 0;

// DMA callback fonksiyonu
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc) {
    isADCFinished = 1;
}

// Ana döngü
while (1) {
    if (isADCFinished == 1) {
        isADCFinished = 0;
        HAL_ADC_Start_DMA(&hadc1, (uint32_t*)ADC_VAL, 4); // DMA ile ADC kanallarını oku
    }

    // Okunan değerleri işle
    Trimpot1_Resistance_Value = R_map_value(ADC_VAL[0]);
    Trimpot2_Resistance_Value = R_map_value(ADC_VAL[1]);
    Temperature = Termistor(ADC_VAL[2]);
    Light_Percentage = map_to_percentage(ADC_VAL[3]);

    // Verileri UART üzerinden gönder
    sprintf(buffer, "%d,%d,%d,%d\n", Trimpot1_Resistance_Value, Trimpot2_Resistance_Value, Temperature, Light_Percentage);
    HAL_UART_Transmit(&huart2, (uint8_t*)buffer, strlen(buffer), HAL_MAX_DELAY);
}