# STM_NES
Este repositório contem o projeto final da disciplina de Sistemas Embarcados, do curso de engenharia elétrica da UFCG.

O objetivo principal do projeto é desenvolver uma plataforma de hardware baseada na placa Bluepill (STM32F1) para a criação de jogos simples. A placa desenvolvida integra múltiplos periféricos (joystick, botões, buzzer, LEDs RGB e display OLED), permitindo implementar diferentes mecânicas de interação típicas de jogos.

<img width="833" height="487" alt="Captura de Tela (57)" src="https://github.com/user-attachments/assets/64f77ebb-26a1-4e98-9cd2-946314a46c04" />

## Estrutura do Repositório

### Projeto PCB
Contém todos os arquivos referentes à confecção da Placa de Circuito Impresso (PCB) desenvolvida, incluindo esquemático, layout e o arquivo exportado para a fresadora.

### Firmware
Contém o código desenvolvido para testes individuais de todos os componentes da placa, utilizando o STM32CubeIDE.

### Jogo
Pasta destinada ao código final do jogo a ser implementado, utilizando os periféricos disponíveis no hardware projetado.

## Softwares Utilizados

### EasyEDA

O EasyEDA <https://easyeda.com/> foi utilizado para projetar a PCB do projeto. Ele é uma ferramenta online de design eletrônico que integra captura esquemática, simulação de circuitos e design de placas de circuito impresso (PCB).
Com ele, foi possível:

1. Criar o esquemático do circuito.

2. Roteamento manual/automático das trilhas.

3. Gerar os arquivos de fabricação para a fresadora.

4. Visualizar o projeto em 2D e 3D para verificação do encaixe dos componentes.

### STMCubeIDE

O STM32CubeIDE foi utilizado para o desenvolvimento do firmware da Bluepill (STM32F103C8T6).
Principais funcionalidades utilizadas:

Configuração dos periféricos internos do microcontrolador através do CubeMX (GPIOs, I2C, Timers, etc).

Escrita, compilação e depuração do código em C.

Organização de drivers e bibliotecas para comunicação com os módulos externos (como o display OLED).


# Hardware e PCB

A placa foi projetada ao redor da Bluepill (STM32F103C8T6), integrando diversos periféricos para interação em jogos:

# Joystick Analógico (XY + botão)

> Permite movimentação em duas direções (X e Y).

> Possui um botão integrado quando pressionado.

> Conectado às entradas analógicas/digitais do STM32 para leitura dos valores.

- LEDs RGB (x2)

> Dois LEDs RGB endereçáveis, que podem ser usados para indicar status do jogo (vidas, pontuação, efeitos visuais, etc).

> Cada cor (R, G, B) é controlada separadamente via GPIOs.

- Botões (KEY1 e KEY2)

> Botões de uso geral que podem ser atribuídos a ações como "Start", "Pause", "Fire", etc.

> São ligados a pinos digitais configurados com pull-up interno.

- Buzzer (SG1)

> Permite emitir sons simples durante o jogo (como efeitos de colisão ou música básica).

> Conectado a um pino PWM do microcontrolador, para gerar tons em diferentes frequências.

- Display OLED (I2C)

> Display OLED de 128x64 pixels, comunicação via I2C.

> Utilizado para exibir gráficos e informações do jogo.

> Permite criar desde interfaces simples até sprites animados.

## Objetivo

O hardware foi projetado para servir como base no desenvolvimento de jogos embarcados.
Com a integração dos periféricos descritos, é possível criar jogos simples diretamente controlados pela Bluepill, como:

- Snake

- Pong

- Space Invaders

- Jogos de corrida simples

- Mini-games personalizados

O projeto demonstra a capacidade de unir conceitos de hardware, firmware e sistemas embarcados, resultando em uma plataforma completa para experimentação.
