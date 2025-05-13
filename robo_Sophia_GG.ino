/**
 * @file robo_Sophia_GG.ino
 * @brief Implementação do Robô Desviador - Projeto SOPHIA
 *
 * Este arquivo contém todas as definições e implementações das funções utilizadas no robô desviador
 * desenvolvido por Gustavo e Guilherme. O projeto usa a biblioteca AFMotor.h para controle dos motores DC,
 * sensores ultrassônicos para desvio de obstáculos e encoders para alinhamento e controle de velocidade.
 *
 * As macros definidas no início do código permitem fácil ajuste de parâmetros como velocidade, distância mínima,
 * tempo de curva, entre outros. Os valores estão otimizados para uma saída de 7.2V alimentando o robô.
 *
 * Fique à vontade para personalizar as rotinas no setup() e loop(), ou ajustar os parâmetros das macros
 * conforme necessário para adaptar o robô ao seu ambiente ou propósito.
 *
 * @author Gustavo_Bueno_AdB & Guilherme Oliver
 * @date 02/05/2025
 * @version 3.0
 */



#include <AFMotor.h>

// === Configurações das marchas (velocidade) ===
#define N_MARCHAS 5 // numero de marchas
#define MIN_VEL 205 // velocidade minima e velocidade da marcha 1
//==================================

// === Configurações do sensor ultrassônico ===
#define DISTANCIA_MIN 25 // Distancia minima permitida antes de desviar
#define ECHO 39 // porta ligada ao sensor
#define TRIG 38 // porta ligada ao sensor
#define PARAM_DIV_SENSOR 58.2 // parametro do sensor (especificado pelo fabricante)
//==================================

// === Parâmetros para curvas do robô ===
#define TEMPO_CURVA_ANG_M1 3.17 // tempo de delay aproximado para girar um angulo em marcha 1
#define ESQUERDA 0 // marcro para curva
#define DIREITA 1 // marcro para curva
//==================================

// === Portas onde os motores estão conectados ===
#define PORTA_MOT_E 1 // porta do shield do motor esquerdo
#define PORTA_MOT_D 2 // porta do shield do motor direito
//==================================

// === Configurações dos encoders (monitoramento dos motores) ===
#define PINO_ENC_ESQ 18 // ligado na interrupção 5 (monitoramento do motor esquerdo)
#define PINO_ENC_DIR 19 // ligado na interrupção 4 (monitoramento do motor direito)
#define N_ENC_POR_VOLTA 20 // número de pulsos por volta (modo falling)
#define MS_POR_ALINHADA 2000 // tempo (ms) antes de tentar corrigir o alinhamento dos motores
#define VEL_REAL_INC 0.02 // quanto a velocidade real pode ser ajustada (2% por vez)
#define VEL_REAL_MIN 0.6 // menor velocidade permitida para os motores (em %)
//==================================

//=== Definições do comportamento ==
#define SEGURANCA 1 // faz curva só na marcha 1
//==================================


// === Definindo as funções usadas =========================================

/**
 * @brief Inicia os 2 motores em marcha 1.
 *
 * inicia os valores base de velocidade e de marcha da maneira correta.
*/
void da_partida();

/**
 * @brief Desliga os 2 motores.
 *
 * desliga e reinicia os valores bases da maneira correta para a proxima partida.
*/
void desliga();

/**
 * @brief Anda pra frente ou pra trás.
 *
 * Os motores andam na direção passada (pra frente ou pra trás).
 *
 * @param direcao Direção em que deve andar (FORWARD ou BACKWARD).
 */
void anda(int direcao);


/**
 * @brief Vira o carrinho pro lado que mandar.
 *
 * Gira o carrinho num ângulo escolhido e na direção (direita ou esquerda).
 * Se SEGURANCA tiver ligada, sempre faz a curva na marcha 1.
 *
 * @param direcao Lado pra onde vai virar (ESQUERDA ou DIREITA).
 * @param angulo Ângulo da curva, em graus.
 */
void curva(int direcao, double angulo);

/**
 * @brief Escolhe o melhor lado pra fazer a curva.
 *
 * Testa virar pros dois lados e compara as distâncias.
 *
 * @return Retorna ESQUERDA ou DIREITA.
 */
int testa_curva();

/**
 * @brief Incrementa ou decrementa a marcha atual.
 *
 * Vai da marcha atual até a marcha destino de um em um recursivamente
 * cada troca de marcha faz a velocidade variar em ((255 - VEL_MIN) / N_MARCHAS) para
 * cima ou para baixo dependendo se incrementa ou decrementa a marcha.
 *
 * @param destino Valor da marcha que se quer chegar (entre [0 - N_MARCHAS])
*/
void troca_marcha(uint8_t destino);

/**
 * @brief Atualiza a velocidade e a direcao de ambos os motores
 *
 * @warning set_motores é uma função usada para implementar outras funções, usa-la diretamente no codigo de loop() pode gerar problemas.
 *  
 * @param velocidade a nova velocidade dos motores
 * @param direcao_esq a direcao do motor esquerdo (FORWARD, BACKWARD ou RELEASE)
 * @param direcao_dir a direcao do motor direito (FORWARD, BACKWARD ou RELEASE)
*/
void set_motores(uint8_t velocidade, int direcao_esq, int direcao_dir);

/**
 * @brief Contador do encoder esquerdo.
 *
 * Soma 1 toda vez que o sensor do motor esquerdo detecta movimento.
 * 
 * @warning Função chamada automaticamente por interrupção.
 */
void interrupcao_esq();

/**
 * @brief Contador do encoder direito.
 *
 * Soma 1 toda vez que o sensor do motor direito detecta movimento.
 *
 * @warning Função chamada automaticamente por interrupção.
 */
void interrupcao_dir();

/**
 * @brief Retorna a distancia lida pelo sensor.
 *
 * Com o sensor de profundidade calcula e ja divide pelo PARAM_DIV_SENSOR
 * retornando a distancia aproximada em centimetros do objeto mais proximo
 * na linha do sensor e atualizando a variavel global distancia.
 *
 * @return um valor entre [5 - 250] estimativamente ou um valor negativo, caso esteja fora do alcançe.
*/
int confere_distancia();

/**
 * @brief Ajusta os motores antes de começar de verdade.
 *
 * Anda um pouco pra frente e pra trás, calibrando os dois motores.
 */
void alinhando_motores_antes();

/**
 * @brief Corrige a diferença entre os motores.
 *
 * Se um motor estiver andando mais que o outro, ajusta a velocidade real dos motores 
 * para se aproximar da velocidade um do outro.
 */
void alinha_motores();
// =========================================================================

//=== Definindo variaveis de escopo global =================================
AF_DCMotor Motor_esq(PORTA_MOT_E);
AF_DCMotor Motor_dir(PORTA_MOT_D);

//valor alterado a cada troca de marcha
uint8_t incremento_marcha = (uint8_t) (255 - MIN_VEL) / N_MARCHAS;

// distancia captada pelo sensor
int distancia;

//contador de tiks de cada motor
volatile int cont_dir = 0;
volatile int cont_esq = 0;

//velocidade ideal aplicada nos motores
uint8_t velocidade = 0;
uint8_t marcha = 0;

//velocidade real dos motores (em %)
double vel_motor_dir_real = 0.8;
double vel_motor_esq_real = 0.8;

//controle para testes de tempo
unsigned long int ms_ultima_alinhada = 0;
//==========================================================================

void setup() 
{
  //definindo as portas de entrada e saida.
  pinMode(TRIG, OUTPUT);
  pinMode(ECHO, INPUT);
  pinMode(PINO_ENC_ESQ, INPUT);
  pinMode(PINO_ENC_DIR, INPUT);

  //iniciando as interrupcoes
  attachInterrupt(5, interrupcao_esq, FALLING);
  attachInterrupt(4, interrupcao_dir, FALLING);

  //iniciando os motores
  da_partida();
  alinhando_motores_antes();
}

void loop() 
{
  //atualiza a distancia
  confere_distancia();
  
  if(distancia < (DISTANCIA_MIN * marcha) && distancia > 0)
  {
    detachInterrupt(5);
    detachInterrupt(4);
    curva(testa_curva(), 90);
    attachInterrupt(5, interrupcao_esq, FALLING);
    attachInterrupt(4, interrupcao_dir, FALLING);

  }

  anda(FORWARD);

  // alinhando os motores com encoders
  if ((millis() - ms_ultima_alinhada) > MS_POR_ALINHADA)
    alinha_motores();
}

void alinhando_motores_antes()
{
  for (int i = 0 ; i < 3 ; i++)
  {
    anda(FORWARD);
    delay(MS_POR_ALINHADA);
    alinha_motores();

    anda(BACKWARD);
    delay(MS_POR_ALINHADA);
    alinha_motores();
  }
}

int testa_curva()
{
  curva(ESQUERDA, 90);
  int esq = confere_distancia();
  
  curva(DIREITA, 180);
  int dir = confere_distancia();
  
  curva(ESQUERDA, 90);
  
  
  if (esq > dir && esq > DISTANCIA_MIN)
    return ESQUERDA;  
  else if (esq < dir && dir > DISTANCIA_MIN)
    return DIREITA;
  else
  {
    curva(DIREITA, 90);
    return DIREITA;
  }
}

void alinha_motores()
{
  unsigned long int intervalo = millis() - ms_ultima_alinhada;
  if (intervalo == 0) return;
  unsigned long int rpt_esq = cont_esq / intervalo;
  unsigned long int rpt_dir = cont_dir / intervalo;
  cont_esq = 0;
  cont_dir = 0;

  if (rpt_esq > rpt_dir)
  {
    if (vel_motor_dir_real < 1.0)
      vel_motor_dir_real += VEL_REAL_INC;
    else if (vel_motor_esq_real > VEL_REAL_MIN)
      vel_motor_esq_real -= VEL_REAL_INC;
  }
  else if (rpt_esq < rpt_dir) 
  {
    if (vel_motor_esq_real < 1.0)
      vel_motor_esq_real += VEL_REAL_INC;
    else if (vel_motor_dir_real > VEL_REAL_MIN)
      vel_motor_dir_real -= VEL_REAL_INC;
  }
  ms_ultima_alinhada = millis();
}

int confere_distancia()
{
  digitalWrite(TRIG, HIGH);
  delay(1);
  digitalWrite(TRIG, LOW);
  distancia = ( pulseIn(ECHO, HIGH) / PARAM_DIV_SENSOR );
  return distancia; 
}

void da_partida()
{
  marcha = 1;
  velocidade = MIN_VEL;
  set_motores(velocidade, FORWARD, FORWARD);
}

void desliga()
{
  troca_marcha(1);
  marcha = 0;
  velocidade = 0;
  set_motores(velocidade, RELEASE, RELEASE);
}

void troca_marcha(uint8_t destino)
{
  if (marcha < destino && destino <= N_MARCHAS)
  {
    //acelera aos pucos até o valor da proxima marcha
    uint8_t i;
    for (i = velocidade ; i < velocidade + incremento_marcha ; i++)
      set_motores(i, FORWARD, FORWARD);

    //atualiza os parametros de referencia
    velocidade = i;
    marcha++;

    // recursao para saltar mais de uma marcha
    troca_marcha(destino);
  }
  
  else if (marcha > destino && destino >= 1)
  {
    //desacelera aos pucos até o valor da marcha anterior
    uint8_t i;
    for (i = velocidade ; i > velocidade - incremento_marcha ; i--)
      set_motores(i, FORWARD, FORWARD);

    //atualiza os parametros de referencia
    velocidade = i;
    marcha--;

    // recursao para saltar mais de uma marcha
    troca_marcha(destino);
  }
}

void set_motores(uint8_t velocidade, int direcao_esq, int direcao_dir)
{
  Motor_esq.setSpeed((uint8_t) (velocidade * vel_motor_esq_real));
  Motor_dir.setSpeed((uint8_t) (velocidade * vel_motor_dir_real));
  Motor_esq.run(direcao_esq);
  Motor_dir.run(direcao_dir);
}

void curva(int direcao, double angulo)
{
  if (SEGURANCA)
    troca_marcha(1);
   
  if (direcao == ESQUERDA)
    set_motores(velocidade, FORWARD, BACKWARD);
  else if (direcao == DIREITA)
    set_motores(velocidade, BACKWARD, FORWARD);

  delay(TEMPO_CURVA_ANG_M1 * angulo);
}

void anda(int direcao)
{
  set_motores(velocidade, direcao, direcao);
}

void interrupcao_esq()
{
  cont_esq++;
}

void interrupcao_dir()
{
  cont_dir++;
}