# Sistema Embarcado Para Testes em Estruturas Aerodinâmicas
(README EM EDIÇÃO...)

Projeto que apresenta uma solução de engenharia para monitoramento de fenômenos físicos em estruturas aerodinâmicas, utilizando STM32-F446re com sensores de temperatura, pressão, altitude, giroscópio e acelerômetro. Inclui um servo motor, display OLED e software em Python que analisa e exibe dados em tempo real, transmitidos via USART. 


O programa para Windows: [codigo recebe dados do STM32 3.0.exe](https://github.com/erickcharlesneves/SEtestesEmEstruturasAerodinamicas/blob/main/19%20-%20versao%209.0-principal-servoMotor-buzzer-potenciometro-mpu6051-BMP281-displayOled/Core/Src/codigo%20recebe%20dados%20do%20STM32%203.0.exe) armazena os dados coletados, os exibe em gráficos em tempo real e permite exportá-los em formato .xlsx para análises posteriores. 

### Instruções:
- Execute o programa [codigo recebe dados do STM32 3.0.exe](https://github.com/erickcharlesneves/SEtestesEmEstruturasAerodinamicas/blob/main/19%20-%20versao%209.0-principal-servoMotor-buzzer-potenciometro-mpu6051-BMP281-displayOled/Core/Src/codigo%20recebe%20dados%20do%20STM32%203.0.exe) após rodar o código principal na placa STM32-F446re com todos os periféricos necessarios conectados na mesma conforme diagrama.
- Para evitar conflitos da porta serial não deixar outro tipo de terminal aberto nesta versão do programa. 
- Digite a porta COM disponível, o baudrate configurado: 115200, e a taxa de aquisição de dados em segundos desejada.  
- Para encerrar o programa de coleta de dados e armazena-los em .xlsx, pressione *Ctrl + & por 10* segundos, e salve o arquivo com nome desejado na pasta desejada.

  OBS: Se o programa não funcionar verifique se há a instalação do [Microsoft Visual C++](https://learn.microsoft.com/pt-br/cpp/windows/latest-supported-vc-redist?view=msvc-170) no windows. 

 (README EM EDIÇÃO...)

Equipe:  Erick Charles Neves Cassiano
         Robson Alves Vilar 
