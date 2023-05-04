
#include <bibliotecamotor.h>

// mapa de hardware *******************************************************************
#define sdd 2
#define sd 3
#define sc 4
#define se 7
#define see 8
//#define D8 A0
uint8_t pinosSensores[5] = { see, se, sc, sd, sdd };
//*************************************************************************************

int sensorMin = 1023;
int sensorMax = 0;


motor md(11, 10);
motor me(5, 6);
void rode(int esq, int dir){
  if(esq > 0) me.frente(esq);
  else me.re(esq * -1);

  if(dir > 0) md.frente(dir);
  else md.re(dir * -1);
}

int leituras[8] = {};
byte leitura = 0x00;
//chamando o objeto motor da biblioteca motor


#define preto 0

void setup() {
  Serial.begin(9600);
  // pinMode(ir, OUTPUT);

  // pin mode analog:  //declaração sensores
  DDRC |= 0b00000000;
  /*
        na faixa > 900;
        no branco < 800;

        contagem do d1 até o d8 é da esquerda para a direitta


        nos valores de calibração funciona da seguinte forma:
        preto valores baixos(< 140)
        branco valores altos(>200)
  */
}


// variaveis pid:
int Kp = 60, Kd = 20, Ki = 0, errorAcumulado = 0, lastError = 0, error = 0;
bool activatePID = true;

void loop() {
  // leitura dos sensores
  //for (int i = 0; i < 8; i++) leituras[i] = analogRead(pinosSensores[i]);
  leitura = 0;
  for (int i = 0; i < 5; i++) leitura |= (digitalRead(pinosSensores[i]) == preto ? 1 : 0) << i;
  // pra esquerda é negativo para a direita é positivo;
  switch (leitura) {
    case 0b00100:
      activatePID = true;
      error = 0;
      break;
    case 0b00010:
      error = 1;
      break;
    case 0b01000:
      error = -1;
      break;
    case 0b00011:
      error = 2;
      break;
    case 0b11000:
      error = -2;
      break;
    case 0b10000:
      error = -2.5;
      break;
    case 0b00001:
      error = 2.5;
      break;
    case 0b00111:
      //activatePID = false;
      error = 3;
      break;
    case 0b11100:
      //activatePID = false;
      error = -3;
      break;
  }
  //Serial.println(leitura, BIN);

  if (activatePID) {
    int limiteIntegrativa = 50;
    errorAcumulado = ((limiteIntegrativa > errorAcumulado) && (errorAcumulado > -1*limiteIntegrativa))?(errorAcumulado+error):0;
    
    float PID = Kp * error + Kd * (error - lastError) + Ki * errorAcumulado;
    lastError = error;
    Serial.println(leitura, BIN);
    Serial.println(error);
    Serial.println(errorAcumulado);
    int v = 70;
        // esquerda   direita
    rode(constrain(v + PID, -255,255), constrain(v - PID, -255,255));
  }
}
