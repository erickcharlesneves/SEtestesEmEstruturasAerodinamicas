#include "main.h"
#include "servo_motor.h"

// Buffer para média móvel
#define MAX_QUEUE_SIZE 100 // Tamanho máximo permitido para a fila
uint32_t adc_buffer[MAX_QUEUE_SIZE] = {0}; // Buffer circular para armazenar os valores do ADC
uint8_t queue_size = 10; // Tamanho atual da fila (configurável em tempo de execução)
uint8_t queue_index = 0; // �?ndice para o próximo elemento a ser sobrescrito
uint32_t sum_adc_values = 0; // Soma dos valores na fila
uint8_t num_elements_in_queue = 0; // Número atual de elementos na fila
void ServoMotor_Init(TIM_HandleTypeDef *htim) {

    HAL_TIM_PWM_Start(htim, TIM_CHANNEL_1);
}

void ServoMotor_Control(ADC_HandleTypeDef *hadc, TIM_HandleTypeDef *htim) {
	uint32_t adc_value = 0;
	HAL_ADC_Start(hadc);
	if (HAL_ADC_PollForConversion(hadc, HAL_MAX_DELAY) == HAL_OK) {
	    // Obtém o valor convertido (resolução de 10 bits: 0 - 1023)
		adc_value = HAL_ADC_GetValue(hadc);
	}
	HAL_ADC_Stop(hadc);

    // Atualização da fila circular para média móvel
    if (num_elements_in_queue < queue_size) {
        // Adiciona o valor à soma e incrementa o número de elementos
        sum_adc_values += adc_value;
        adc_buffer[queue_index] = adc_value;
        num_elements_in_queue++;
    } else {
        // Remove o valor mais antigo da soma, sobrescreve o valor e atualiza a soma
        sum_adc_values -= adc_buffer[queue_index];
        adc_buffer[queue_index] = adc_value;
        sum_adc_values += adc_value;
    }

    // Atualiza o índice circular
    queue_index = (queue_index + 1) % queue_size;

    // Calcula a média móvel
    uint32_t adc_average = sum_adc_values / num_elements_in_queue;

//    if (contador_conta_tempo_para_coletar_dados_dos_sensores > 40) (na versão 6)

	if (adc_average >= 40 && adc_average <= 231) {
	  TIM1->CCR1 = adc_average;

	  // Ajusta o duty cycle (por exemplo, 50%)
	  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 0);
	}
	else {
	  // Ajusta o duty cycle (por exemplo, 50%)
	  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 10000);
	}
}
