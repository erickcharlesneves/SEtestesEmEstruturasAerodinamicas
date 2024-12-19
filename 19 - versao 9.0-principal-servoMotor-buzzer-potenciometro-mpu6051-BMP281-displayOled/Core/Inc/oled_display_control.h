#ifndef OLED_DISPLAY_CONTROL_H
#define OLED_DISPLAY_CONTROL_H

#include "ssd1306.h" // Biblioteca necessária para controle do display OLED

// Protótipos das funções de controle do display OLED
void funcao_exibir_dados_de_temperatura(float temperature);
void funcao_exibir_dados_de_pressao(float pressure);
void funcao_exibir_dados_de_altitude(float altitude);

#endif // OLED_DISPLAY_CONTROL_H
