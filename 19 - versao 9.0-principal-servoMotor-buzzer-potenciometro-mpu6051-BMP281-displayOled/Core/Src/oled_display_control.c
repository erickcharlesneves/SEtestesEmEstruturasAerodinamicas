//controle do display oled funções extras
#include "oled_display_control.h"
#include "ssd1306.h"
#include "ssd1306_fonts.h"
#include <stdio.h>

#define SSD1306_COLOR_BLACK 0 // Representa cor preta
#define SSD1306_COLOR_WHITE 1 // Representa cor branca



void funcao_exibir_dados_de_temperatura(float temperatureBMP280) {
    char message[100] = "Temperatura:";
    char message2[100] = "";

    sprintf(message2, "%.1f C", temperatureBMP280);

    ssd1306_Fill(SSD1306_COLOR_BLACK); // Limpe o display

    ssd1306_SetCursor(25, 20); // Defina a posição inicial
    ssd1306_WriteString(message, Font_7x10, SSD1306_COLOR_WHITE); // Escreva a mensagem

    ssd1306_SetCursor(30, 40); // Defina a posição inicial
    ssd1306_WriteString(message2, Font_7x10, SSD1306_COLOR_WHITE); // Escreva a mensagem

    ssd1306_UpdateScreen(); // Atualize o display
}

void funcao_exibir_dados_de_pressao(float pressureBMP280) {
    char message[100] = "Pressao:";
    char message2[100] = "";

    sprintf(message2, "%.1f hPa", pressureBMP280);

    ssd1306_Fill(SSD1306_COLOR_BLACK); // Limpe o display

    ssd1306_SetCursor(25, 20); // Defina a posição inicial
    ssd1306_WriteString(message, Font_7x10, SSD1306_COLOR_WHITE); // Escreva a mensagem

    ssd1306_SetCursor(30, 40); // Defina a posição inicial
    ssd1306_WriteString(message2, Font_7x10, SSD1306_COLOR_WHITE); // Escreva a mensagem

    ssd1306_UpdateScreen(); // Atualize o display
}

void funcao_exibir_dados_de_altitude(float altitudeBMP280) {
    char message[100] = "Altitude:";
    char message2[100] = "";

    sprintf(message2, "%.1f M", altitudeBMP280);

    ssd1306_Fill(SSD1306_COLOR_BLACK); // Limpe o display

    ssd1306_SetCursor(25, 20); // Defina a posição inicial
    ssd1306_WriteString(message, Font_7x10, SSD1306_COLOR_WHITE); // Escreva a mensagem

    ssd1306_SetCursor(30, 40); // Defina a posição inicial
    ssd1306_WriteString(message2, Font_7x10, SSD1306_COLOR_WHITE); // Escreva a mensagem

    ssd1306_UpdateScreen(); // Atualize o display
}


void TrocarInterfaceDisplay(int *contador_interfaces, TIM_HandleTypeDef *htim3) {
    if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_1) == GPIO_PIN_RESET) {
        // Efeito do buzzer
        while (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_1) == GPIO_PIN_RESET) {
            for (int i = 64995; i <= 65000; i++) {
                __HAL_TIM_SET_COMPARE(htim3, TIM_CHANNEL_3, i);
                HAL_Delay(1);
            }
            HAL_Delay(1);
        }
        for (int i = 65000; i >= 64995; i--) {
            __HAL_TIM_SET_COMPARE(htim3, TIM_CHANNEL_3, i);
            HAL_Delay(1);
        }
        __HAL_TIM_SET_COMPARE(htim3, TIM_CHANNEL_3, 0);

        // Incrementa contador
        (*contador_interfaces)++;
        if (*contador_interfaces > 2) {
            *contador_interfaces = 0;
        }
    }
}

void AtualizarDisplay(int interface_ativa, float temperatura, float pressao, float altitude) {
    if (interface_ativa == 0) {
        funcao_exibir_dados_de_temperatura(temperatura);
    } else if (interface_ativa == 1) {
        funcao_exibir_dados_de_pressao(pressao);
    } else if (interface_ativa == 2) {
        funcao_exibir_dados_de_altitude(altitude);
    }
}
