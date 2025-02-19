// Atividade do capítulo 8
// Aluna: Maryana Souza Silveira

#include <stdio.h>
#include <stdlib.h>
#include "pico/stdlib.h"
#include "hardware/adc.h"
#include "hardware/i2c.h"
#include "hardware/pwm.h"
#include "auxiliares/ssd1306.h"
#include "auxiliares/font.h"

#define led_green 11 // Pino do led verde
#define led_blue 12  // Pino do led azul
#define led_red 13  // Pino do led vermelho

#define buttonA 5 // Pino do botão A
#define buttonJoy 22 // Pino do botão do joystick

#define eixo_x_joystick 26  // GPIO para eixo X
#define eixo_y_joystick 27  // GPIO para eixo Y

#define I2C_PORT i2c1 // Define o barramento I2C
#define I2C_SDA 14 // Define o pino SDA
#define I2C_SCL 15 // Define o pino SCL
#define endereco 0x3C // Endereço do display

bool ativado = true; // Variável para controle dos leds pwm

// Variáveis para desenhar o retângulo
int v1 = 0; // Posição do retângulo
int v2 = 0; // Posição do retângulo
int v3 = 128; // Largura do retângulo
int v4 = 64; // Altura do retângulo

ssd1306_t ssd; // Inicializa a estrutura do display

// Variáveis para debounce do botão
volatile bool estado_led = false;  // Estado do LED verde
absolute_time_t last_time = {0};  // Último tempo de pressão do botão

void initial_configs(); // Função para inicializar as configurações
void ssd1306_square(ssd1306_t *ssd, uint8_t x, uint8_t y); // Função para desenhar um quadrado
bool debounce(uint gpio, uint32_t tempo_debounce_ms); // Função para implementar o debounce do botão
void gpio_irq_handler(uint gpio, uint32_t events); // Função para tratar a interrupção dos botões

int main()
{
  initial_configs(); // Inicializa as configurações

  gpio_set_irq_enabled_with_callback(buttonA, GPIO_IRQ_EDGE_FALL, true, &gpio_irq_handler); // Habilita a interrupção do botão A
  gpio_set_irq_enabled_with_callback(buttonJoy, GPIO_IRQ_EDGE_FALL, true, &gpio_irq_handler); // Habilita a interrupção do botão do joystick

  uint16_t adc_value_x; // Variável para armazenar o valor do ADC do eixo X
  uint16_t adc_value_y;  // Variável para armazenar o valor do ADC do eixo Y
  
  while (true)
  {
    adc_select_input(0); // Seleciona o ADC para eixo X
    adc_value_x = adc_read(); // Lê o valor do ADC
    //Realiza a calibragem do joystick: Valores lidos em x refletem em um movimento vertical no display, por isso a inversão dos valores
    //Obs: O cálculo utiliza o valor de 4080 pois é o valor máximo que o meu joystick atinge
    int ADY = (adc_value_x * 55) / (4080); // Converte o valor lido para variação na altura do display (tendo cuidado com as margens)
    int duty_cycle_blue = (adc_value_x * 40000) / 4080;  // Calcula o ciclo de trabalho para o led azul

    adc_select_input(1); // Seleciona o ADC para eixo Y
    adc_value_y = adc_read(); // Lê o valor do ADC
    // Realiza a calibragem do joystick: Valores lidos em y refletem em um movimento horizontal no display, por isso a inversão dos valores
    int ADX = (adc_value_y * 119) / (4080);  // Converte o valor lido para variação na largura do display (tendo cuidado com as margens)
    int duty_cycle_red = (adc_value_y * 40000) / 4080;  // Calcula o ciclo de trabalho para o led vermelho

    ssd1306_fill(&ssd, false); // Limpa o display
    ssd1306_rect(&ssd, v1, v2, v3, v4, true, false); // Desenha um retângulo
    ssd1306_square(&ssd, ADX, 56 - ADY); // Desenha o quadrado que acompanha o joystick (calibrado para que o movimento em y acompanhe o joystick)
    ssd1306_send_data(&ssd); // Atualiza o display

    pwm_set_gpio_level(led_blue, duty_cycle_blue);  // Define o ciclo de trabalho para o led azul
    pwm_set_gpio_level(led_red, duty_cycle_red);  // Define o ciclo de trabalho para o led vermelho
    
    sleep_ms(100); // Aguarda 100ms
  }
}

void initial_configs(){ // Função para inicializar as configurações
  gpio_init(led_green); // Inicializa o pino do led verde
  gpio_set_dir(led_green, GPIO_OUT); // Configura o pino como saída

  gpio_init(buttonA); // Inicializa o pino do botão A
  gpio_set_dir(buttonA, GPIO_IN); // Configura o pino como entrada
  gpio_pull_up(buttonA); // Habilita o pull-up do botão A

  gpio_init(buttonJoy); // Inicializa o pino do botão do joystick
  gpio_set_dir(buttonJoy, GPIO_IN); // Configura o pino como entrada
  gpio_pull_up(buttonJoy); // Habilita o pull-up do botão do joystick

  i2c_init(I2C_PORT, 400 * 1000); // Inicializa o barramento I2C a 400kHz

  gpio_set_function(I2C_SDA, GPIO_FUNC_I2C); // Configura a função do pino SDA
  gpio_set_function(I2C_SCL, GPIO_FUNC_I2C); // Configura a função do pino SCL
  gpio_pull_up(I2C_SDA); // Habilita o pull-up do pino SDA
  gpio_pull_up(I2C_SCL); // Habilita o pull-up do pino SCL
  ssd1306_init(&ssd, 128, 64, false, endereco, I2C_PORT); // Inicializa o display
  ssd1306_config(&ssd); // Configura o display
  ssd1306_send_data(&ssd); // Envia os dados para o display

  // Limpa o display. O display inicia com todos os pixels apagados.
  ssd1306_fill(&ssd, false);
  ssd1306_send_data(&ssd);

  adc_init(); // Inicializa o ADC
  adc_gpio_init(eixo_x_joystick); // Inicializa o pino do eixo X do joystick
  adc_gpio_init(eixo_y_joystick); // Inicializa o pino do eixo Y do joystick

  gpio_set_function(led_blue, GPIO_FUNC_PWM);  // Configura o pino do led azul como PWM
  uint slice_blue = pwm_gpio_to_slice_num(led_blue);  // Obtém o canal PWM da GPIO
  pwm_set_clkdiv(slice_blue, 2.0); //define o divisor de clock do PWM
  pwm_set_wrap(slice_blue, 40000); //definir o valor de wrap
  pwm_set_enabled(slice_blue, true); //habilita o pwm no slice correspondente

  gpio_set_function(led_red, GPIO_FUNC_PWM);  // Configura o pino do led vermelho como PWM
  uint slice_red = pwm_gpio_to_slice_num(led_red);  // Obtém o canal PWM da GPIO
  pwm_set_clkdiv(slice_red, 2.0); //define o divisor de clock do PWM
  pwm_set_wrap(slice_red, 40000); //definir o valor de wrap
  pwm_set_enabled(slice_red, true); //habilita o pwm no slice correspondente
}

void ssd1306_square(ssd1306_t *ssd, uint8_t x, uint8_t y){ // Função para desenhar um quadrado no display
  for (uint8_t i = 0; i < 8; ++i){ // Desenha um quadrado de 8x8 pixels
    static uint8_t square[] = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff}; // Define o quadrado
    uint8_t line = square[i]; // Obtém a linha do quadrado
    for (uint8_t j = 0; j < 8; ++j){ // Desenha a coluna do quadrado
      ssd1306_pixel(ssd, x + i, y + j, line & (1 << j)); // Desenha o pixel
    }
  }
}

bool debounce(uint gpio, uint32_t tempo_debounce_ms) { // Função para realizar o debounce do botão
    absolute_time_t current_time = get_absolute_time();  // Obtém o tempo atual
    if (absolute_time_diff_us(last_time, current_time) >= tempo_debounce_ms * 1000) { // Verifica se o tempo de pressão é maior que o tempo de debounce
        last_time = current_time;  // Atualiza o tempo de pressão
        return true;  // Retorna verdadeiro (pressão válida)
    }
    return false;  // Retorna falso (pressão inválida)
}

void gpio_irq_handler(uint gpio, uint32_t events){ // Função para tratar a interrupção dos botões
  if (debounce(gpio, 200)){ // Verifica se a pressão do botão é válida
    if (gpio == buttonA ) { // Verifica se o botão pressionado é o botão A
      uint slice_blue = pwm_gpio_to_slice_num(led_blue);  // Obtém o canal PWM da GPIO
      pwm_set_enabled(slice_blue, !ativado); // Habilita ou desabilita o pwm no slice correspondente

      uint slice_red = pwm_gpio_to_slice_num(led_red);  // Obtém o canal PWM da GPIO
      pwm_set_enabled(slice_red, !ativado); // Habilita ou desabilita o pwm no slice correspondente

      ativado = !ativado; // Atualiza a variável de controle dos leds pwm
    }
    if (gpio == buttonJoy) { // Verifica se o botão pressionado é o botão do joystick
      gpio_put(led_green, !gpio_get(led_green)); // Inverte o estado do led verde
      if (v1 == 0 && v2 == 0 && v3 == 128 && v4 == 64) { // Verifica se o retângulo está na posição inicial, se sim, muda para a posição 2
        v1 = 8; // Atualiza a posição do retângulo
        v2 = 8; // Atualiza a posição do retângulo
        v3 = 112; // Atualiza a largura do retângulo
        v4 = 48; // Atualiza a altura do retângulo
      }
      else if (v1 == 8 && v2 == 8 && v3 == 112 && v4 == 48) { // Verifica se o retângulo está na posição 2, se sim, volta para a posição inicial
        v1 = 0; // Atualiza a posição do retângulo
        v2 = 0; // Atualiza a posição do retângulo
        v3 = 128; // Atualiza a largura do retângulo
        v4 = 64; // Atualiza a altura do retângulo
      }
    }
  }
}

