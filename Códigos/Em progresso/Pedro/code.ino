#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>


//#define Esp32
//#define Arduino


class Motor {
  public:
  Motor(byte *pubpins);
  void frente(int v = 4095);
  void esquerda(int v = 4095);
  void direita(int v = 4095);
  void speedctrl(int v = 4095);
  void PIDctrl(float pid);
  void noventagrausesq();
  void noventagrausdir();
  private:
  byte *pins;
};

/*
2 - pwm esquerdo
4 - direita tras
5 - direita frente
18 - esquerda frente
19 - esquerda tras
23 - pwm direito
*/

Motor::Motor(byte *pubpins) {
  pins = pubpins;
  for(int i = 0; i < 4; i++) {

    #ifdef Esp32
    pinMode(pins[i], OUTPUT);
    ledcSetup(1, 5000, 12);
    ledcSetup(2, 5000, 12);
    ledcAttachPin(pins[4], 1);
    ledcAttachPin(pins[5], 2);
    #endif

    #ifdef Arduino
      
    #endif
  }
}

void Motor::frente(int v) {
  #ifdef Esp32
  digitalWrite(pins[0], 0);
  digitalWrite(pins[1], 1);
  digitalWrite(pins[2], 1);
  digitalWrite(pins[3], 0);
  #endif

  #ifdef Arduino
  digitalWrite(pins[0], 0);
  digitalWrite(pins[1], 1);
  digitalWrite(pins[2], 1);
  digitalWrite(pins[3], 0);
  #endif
}

void Motor::direita(int v) {


  #ifdef Esp32
  digitalWrite(pins[0], 1);
  digitalWrite(pins[1], 0);
  digitalWrite(pins[2], 1);
  digitalWrite(pins[3], 0);
   #endif

  #ifdef Arduino
  digitalWrite(pins[0], 1);
  digitalWrite(pins[1], 0);
  digitalWrite(pins[2], 1);
  digitalWrite(pins[3], 0);
  #endif


}

void Motor::esquerda(int v) {
  #ifdef Esp32
  digitalWrite(pins[0], 0);
  digitalWrite(pins[1], 1);
  digitalWrite(pins[2], 0);
  digitalWrite(pins[3], 1);
  #endif

  #ifdef Arduino
  digitalWrite(pins[0], 0);
  digitalWrite(pins[1], 1);
  digitalWrite(pins[2], 0);
  digitalWrite(pins[3], 1);
  #endif
}

void Motor::PIDctrl(float pid) {
  

  int a = 0,b = 0;
  a = v + pid;
  b = v - pid;

  #ifdef Esp32
  int v = 4095*0.6;
  if(a > 4095) a = 4095;
  if(a < -4095) a = -4095;
  if(b < -4095) b = -4095;
  if(b > 4095) b = 4095;
  #endif

  #ifdef Arduino
  int v = 1023*0.6;
  if(a > 1023) a = 1023;
  if(a < -1023) a = -1023;
  if(b < -1023) b = -1023;
  if(b > 1023) b = 1023;
  #endif

  if(pid == 0.1) {
    //90 graus esq
  }
  else if(pid == -0.1) {
    //90 graus dir
  } else {
    if(pid >= 0) { // vira para a direita
      if(b >= 0) {
        #ifdef Esp32
        ledcWrite(1, a);
        ledcWrite(2, b);
        digitalWrite(pins[0], 0);
        digitalWrite(pins[1], 0);
        digitalWrite(pins[2], 1);
        digitalWrite(pins[3], 0);
        #endif 

        #ifdef Arduino
        analogWrite(pins[4], a);
        analogWrite(pins[5], b);
        digitalWrite(pins[0], 0);
        digitalWrite(pins[1], 0);
        digitalWrite(pins[2], 1);
        digitalWrite(pins[3], 0);
        #endif

      } else {

        #ifdef Esp32
        ledcWrite(1, a);
        ledcWrite(2, -b);
        digitalWrite(pins[0], 1);
        digitalWrite(pins[1], 0);
        digitalWrite(pins[2], 1);
        digitalWrite(pins[3], 0);
        #endif

        #ifdef Arduino
        analogWrite(pins[4], a);
        analogWrite(pins[5], -b);
        digitalWrite(pins[0], 1);
        digitalWrite(pins[1], 0);
        digitalWrite(pins[2], 1);
        digitalWrite(pins[3], 0);
        #endif

      }
    } else {  // vira para esquerda
      if(a >= 0) {
        #ifdef Esp32
        ledcWrite(1, b);
        ledcWrite(2, a);
        digitalWrite(pins[0], 0);
        digitalWrite(pins[1], 1);
        digitalWrite(pins[2], 0);
        digitalWrite(pins[3], 0);
        #endif

        #ifdef Arduino
        analogWrite(pins[4], b);
        analogWrite(pins[5], a);
        digitalWrite(pins[0], 0);
        digitalWrite(pins[1], 1);
        digitalWrite(pins[2], 0);
        digitalWrite(pins[3], 0);
        #endif

      } else {
        #ifdef Esp32
        ledcWrite(1, b);
        ledcWrite(2, -a);
        digitalWrite(pins[0], 0);
        digitalWrite(pins[1], 1);
        digitalWrite(pins[2], 0);
        digitalWrite(pins[3], 1);
        #endif

        #ifdef Arduino
        analogWrite(pins[4], b);
        analogWrite(pins[5], -a);
        digitalWrite(pins[0], 0);
        digitalWrite(pins[1], 1);
        digitalWrite(pins[2], 0);
        digitalWrite(pins[3], 0);
        #endif
      }
    }
  }
}

void Motor::noventagrausdir() {
    // motor da esquerda
  while((valsensors & 0b0000010) == 0b00000010) {
      ledcWrite(3, 4095);
  }
}


void Motor::noventagrausesq() {
  while((valsensors & 0b00100010) == 0b00000010) {
      ledcWrite(1, 4095);
  }
}


class IRline {
public:
  IRline(byte *pubpins, byte pubnumIR, byte pubmuxpin = 0, bool pubmode = 1);
  byte updateIR(int debouncetime = 0);
  void calibrateIR(int waittime = 5000);
  void showIR();
  int PID();
private:
  int mid[8];
  byte valsensors = 0;
  byte ci[16][3] = {
    { 0, 0, 0 },
    { 0, 0, 1 },
    { 0, 1, 0 },
    { 0, 1, 1 },
    { 1, 0, 0 },
    { 1, 0, 1 },
    { 1, 1, 0 },
    { 1, 1, 1 },
  };
  byte valsensors;
  float error, lasterror;
  bool mode;
  byte numIR;
  byte muxpin;
  byte *pins;
  unsigned long pastmillis = 0;
};

IRline::IRline(byte *pubpins, byte pubnumIR, byte pubmuxpin, bool pubmode) {
  muxpin = pubmuxpin;
  numIR = pubnumIR;
  mode = pubmode;
  pins = pubpins;

  if (mode == 1) {
    for (int i = 0; i < numIR; i++) {
      pinMode(pins[i], INPUT);  // modo sem multiplexador
    }
  } else {
    for (int i = 0; i < 3; i++) {
      pinMode(pins[i], OUTPUT);
      pinMode(muxpin, INPUT);  // modo com multiplexador
    }
  }
}



void IRline::calibrateIR(int waittime) {
  #ifdef Esp32
  int mins[numIR] = {4095,4095,4095,4095,4095,4095,4095,4095};
  #endif
  
  #ifdef Arduino
  int mins[numIR] = {1023,1023,1023,1023,1023,1023,1023,1023};
  #endif 

  int calibratedvals[numIR]; 
  int maxs[numIR] = {0,0,0,0,0,0,0,0};

  
  while (millis() < waittime) {
    for (int i = 0; i < numIR; i++) {
      calibratedvals[i] = analogRead(pins[i]);
      if (calibratedvals[i] > maxs[i]) maxs[i] = calibratedvals[i];
      if (calibratedvals[i] < mins[i]) mins[i] = calibratedvals[i];
      calibratedvals[i] = constrain(analogRead(pins[i]), mins[i], maxs[i]);
      mid[i] = int((mins[i] + maxs[i]) / 2.0);
    }
  }
}

byte IRline::updateIR(int debouncetime) {
  valsensors = 0;
  if (mode == 1) {  // modo sem multiplexador
    if (millis() - pastmillis >= debouncetime) {
      pastmillis = millis();
      for (int i = 0; i < numIR; i++) {
        valsensors = valsensors | (((analogRead(pins[i]) > (mid[i]*1.1))? 1:0) << i);
      }
    }
  }
  
  /*
  else {
    if (millis() - pastmillis >= debouncetime) {
      pastmillis = millis();
      for (int x = 0; x < numIR; x++) {

        for (int y = 0; y < 3; y++) digitalWrite(pins[y], ci[x][y]);
      
        val[x] = constrain(analogRead(muxpin), Min, Max);
        val[x] = map(analogRead(muxpin), Min, Max, 0, 1023); 
      
      }
    }
  }
*/
  switch (valsensors) {
    if(numIR == 8) {
      default:
        error = lasterror;
        break;

      case 0b10000000:
        error = 7;
        break;

      case 0b11000000:
        error = 6;
        break;

      case 0b01000000:
        error = 5;
        break;

      case 0b01100000:
        error = 4;
        break;

      case 0b00100000:
        error = 3;
        break;

      case 0b00110000:
        error = 2;
        break;

      case 0b00010000:
        error = 1;
        break;

      case 0b00011000:
        error = 0;
        break;

      case 0b00001000:
        error = 1;
        break;

      case 0b00001100:
        error = -2;
        break;

      case 0b00000100:
        error = -3;
        break;

      case 0b00000110:
        error = -4;
        break;

      case 0b00000010:
        error = -5;
        break;

      case 0b00000011:
        error = -6;
        break;

      case 0b00000001:
        error = -7;
        break;

      case (0b11110000 || 0b11111000)
        error = 0.1;
        break;

      case (0b00001111 || 0b00011111)
        error = -0.1;
        break;  
    }
    else if(numIR == 5) {
      default:
        error = lasterror;
        break;
      case: 0b10000
        error = 2;
        break;

      case: 0b01000
        error = 1;
        break;

      case: 0b00100
        error = 0;
        break;

      case: 0b00010
        error = -1;
        break;

      case: 0b00001
        error = -2;
        break;

      case (0b11000 || 0b11100)
        error = 0.1;
        break;

      case (0b00011 || 0b00111)
        error = -0.1;
        break;
    }
  }
  return valsensors;
}

float IRline::PID() {
  float P, I, D;
  int val = 0;
  for(int i = 0; i < 100; i++) {
    val = analogRead(35) + val;
  }
  int Kp = (val/100);
  int Ki = 0;
  int Kd = 0;
  
  P = error;
  I = I + error;
  D = error - lasterror;


  if (error == 0) I = 0;
  if (I > 255) I = 255;
  else if (I < -255) I = -255;

  int PID = int((Kp * P) + (Ki * I) + (Kd * D));
  lasterror = error;
  if(PID > 4095) PID = 4095;
  else if(PID < -4095) PID = -4095;
  if(error == 0.1 || error == -0.1) {
    return error;
  } else {
     return int(PID);
  }
}


byte m[4] = {4,5,18,19,2,23}; // IN1 - IN2 - IN3 - IN4 - PWM1 - PWM2
/*
4 - direita tras
5 - direita frente
18 - esquerda frente
19 - esquerda tras
23 - pwm direita
*/
byte pinos[8] = { 13, 12, 14, 27, 26, 25, 33, 32 };

Adafruit_SSD1306 display(128, 64, &Wire, -1);
IRline ir(pinos, 8);
Motor motor(m);

void setup() {
  Serial.begin(115200);
  if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) { 
    Serial.println(F("falha na alocação do endereço i2c do display"));
    for(;;); // Don't proceed, loop forever
  }
  display.clearDisplay();
  ir.calibrateIR();
  
}




void loop() {
  ir.updateIR();
  motor.PIDctrl(ir.PID(), ir.updateIR());
  int val = 0;
  for(int i = 0; i < 100; i++) {
    val = analogRead(35) + val;
  }
  //Serial.println(val/100);
  display.clearDisplay();
  display.setTextSize(3);
  display.setTextColor(WHITE);
  display.setCursor(0,0);
  display.print(val/100);
  display.display();
}
