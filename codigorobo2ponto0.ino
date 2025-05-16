#include <AFMotor.h>

AF_DCMotor motorE(1);
AF_DCMotor motorD(2);

//A velocidade dos motores é separada para fazer com que o robô ande reto (já que na prática os motores tem uma força levemente diferente)
uint8_t velocidadeE = 150;
uint8_t velocidadeD = 150;
//Define-se uma faixa de velocidade com mínimo e máximo para evitar que a variável velocidade faça o motor ficar lento demais para andar sozinho ou
//sofra com overflow (uint8_t tem 8 bits, então seu valor máximo é de 255)
uint8_t velocidade_min = 105;
uint8_t velocidade_max = 255;

//Variavéis para serem usadas nos delay.

//Tempo que ficará fazendo a curva.
//Um diferente para direita ou esquerda por causa da diferença de força de cada motor, o que gera uma divergência no quanto se vira pra cada direção.
//Precisa ser ajustada para cada robô
unsigned long tempo_curvaE = 320;
unsigned long tempo_curvaD = 300;
//Ajustar conforme a vontade, serve apenas para os tempos em que o robô fica com o motor parado. Pode ser zerado.
unsigned long tempo_espera = 200;

uint8_t TRIG = 40;
uint8_t ECHO = 38;

uint8_t pinoSensor1 = 19;
uint8_t pinoSensor2 = 20;

int duracao = 0;
int distancia = 0;
int distancia_min = 35;

unsigned int pulsos1;
unsigned int pulsos2;
volatile double timeold;

//Funções executadas a cada interrupção
void contador1(){
  pulsos1++;
}

void contador2(){
  pulsos2++;
}

//Iguala a velocidade dos motores
void igualaVel(){
  //Verifica no máximo uma vez por segundo, para evitar leituras constantes demais
  if(millis() - timeold >= 1){

    uint8_t temp_velocidadeE = velocidadeE;
    uint8_t temp_velocidadeD = velocidadeD;
    uint8_t temp_dist = sonar();

    //Caso de motor esquerdo mais lento que o direito
    if(pulsos1 < pulsos2){

      //Pausa temporariamente a observação do motor esquerdo para igualar a velocidade dos motores com base na quantidade antiga de pulsos
      //Isso é necessário pois, caso não o seja feito, o valor do interruptor mudará constantemente, o que arrisca num loop infinito ou comportamento diferente do esperado
      detachInterrupt(3);

      //Iguala os motores, desacelerando o motor direito e acelerando o esquerdo gradativamente até que ambos esjejam na mesma velocidade na prática
      while(pulsos1 < pulsos2 && (temp_dist >= 0 && temp_dist <= distancia_min)){
        temp_velocidadeE = constrain((temp_velocidadeE++), velocidade_min, velocidade_max);
        temp_velocidadeD = constrain((temp_velocidadeD--), velocidade_min, velocidade_max);
        
        motorE.setSpeed(temp_velocidadeE);
        motorD.setSpeed(temp_velocidadeD);

        motorE.run(FORWARD);
        motorD.run(FORWARD);

        delay(30);
        temp_dist = sonar();
      }

      //Retoma a observação
      attachInterrupt(3, contador2, FALLING);
    }

    //Caso de motor esquerdo mais rápido que direito
    else if(pulsos1 > pulsos2){
      detachInterrupt(4);
      

      //Iguala os motores, desacelerando o motor esquerdo e acelerando o direito gradativamente até que ambos esjejam na mesma velocidade na prática
      while(pulsos1 > pulsos2 && (temp_dist >= 0 && temp_dist <= distancia_min)){
        temp_velocidadeE = constrain((temp_velocidadeE--), velocidade_min, velocidade_max);
        temp_velocidadeD = constrain((temp_velocidadeD++), velocidade_min, velocidade_max);

        motorE.setSpeed(temp_velocidadeE);
        motorD.setSpeed(temp_velocidadeD);
        
        motorE.run(FORWARD);
        motorD.run(FORWARD);
        
        delay(30);

        temp_dist = sonar();
      }
      attachInterrupt(4, contador1, FALLING);
    }

    //Zera os pulsos pra próxima iteração
    pulsos1 = 0;
    pulsos2 = 0;
    timeold = millis();
  }  
}

//Função do sensor ultrassonico
int sonar(){
  digitalWrite(TRIG, HIGH);
  delay(1);
  digitalWrite(TRIG, LOW);

  int dur = pulseIn(ECHO, HIGH);
  int dist = dur/58.2;

  return dist;
}

//Funções de movimentação 

void viraEsquerda(){
  //Tira a leitura dos encoders para evitar erro na hora de igualar os motores
  detachInterrupt(4);
  detachInterrupt(3);

  motorE.setSpeed(velocidadeE);
  motorD.setSpeed(velocidadeD);

  motorE.run(BACKWARD);
  motorD.run(FORWARD);
  delay(tempo_curvaE);

  //Retorna a leitura dos encoders
  attachInterrupt(4, contador1, FALLING);
  attachInterrupt(3, contador2, FALLING);

  motorE.run(RELEASE);
  motorD.run(RELEASE);
  delay(tempo_espera);
}

void viraDireita(){
  //Tira a leitura dos encoders para evitar erro na hora de igualar os motores
  detachInterrupt(4);
  detachInterrupt(3);

  motorE.setSpeed(velocidadeE);
  motorD.setSpeed(velocidadeD);

  motorE.run(FORWARD);
  motorD.run(BACKWARD);

  delay(tempo_curvaD);

  //Retorna a leitura dos encoders
  attachInterrupt(4, contador1, FALLING);
  attachInterrupt(3, contador2, FALLING);

  motorE.run(RELEASE);
  motorD.run(RELEASE);
  delay(tempo_espera);

  //O código abaixo é responsável por fazer ele voltar pra direção original, se possível, ou a continuar contornando se necessário

  rotina();
  viraEsquerda();
}

void andaFrente(){

  motorE.run(FORWARD);
  motorD.run(FORWARD);

  igualaVel();
}


//A rotina a ser chamada no loop, está separado ao invés de no loop principal pois também é usada na função de virar à direita
void rotina(){
  //Verifica a distância
  distancia = sonar();

  //Tenta contornar o obstáculo (se não for possível, como é o caso de quando atinge a parede, ele começa a dar a volta) 
  if(distancia >= 0 && distancia <= distancia_min){
    viraDireita();
  }

  else{
    andaFrente();
  }
}

void setup() {
  pinMode(TRIG, OUTPUT);  
  pinMode(ECHO, INPUT);

  pinMode(pinoSensor1, INPUT);
  attachInterrupt(4, contador1, FALLING);
  pulsos1 = 0;

  pinMode(pinoSensor2, INPUT);
  attachInterrupt(3, contador2, FALLING);
  pulsos2 = 0;

  motorE.setSpeed(velocidadeE);
  motorD.setSpeed(velocidadeD);

}

//Essa variação do código original tenta sempre seguir pelo mesmo caminho,
//virando pra direita pra desviar de obstáculos mas voltando pra direção original se possível
void loop() {
  rotina();
}