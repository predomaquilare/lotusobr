#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define OLED
#define Esp32
//#define Arduino

//#define TestIR

class IRline {
  public:
    IRline(byte *pubpins, byte pubnumIR, byte pubmuxpin = 0, bool pubmode = 1);
    void calibrateIR(int waittime = 5000);
    float getPlusUpdateIR(int debouncetime = 0);
    float getError();
    void showIR();
    float PID();
  private:
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
    int mid[8];
    float error = 0, lasterror = 0;
    bool mode;
    int numIR;
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
  int mins[numIR] = {4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095};
#endif

#ifdef Arduino
  int mins[numIR] = {1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023};
#endif

  int calibratedvals[numIR];
  int maxs[numIR] = {0, 0, 0, 0, 0, 0, 0, 0};


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
float IRline::getPlusUpdateIR(int debouncetime) {
  valsensors = 0;
  if (mode == 1) {  // modo sem multiplexador
    if (millis() - pastmillis >= debouncetime) {
      pastmillis = millis();
      for (int i = 0; i < numIR; i++) {
#ifdef TestIR
        valsensors = valsensors | (((analogRead(pins[i]) > (mid[i] * 1.1)) ? 0 : 1) << i);
#endif
#ifndef TestIR
        valsensors = valsensors | (((analogRead(pins[i]) > (mid[i] * 1.1)) ? 1 : 0) << i);
#endif
      }
    }
  }
  else {
    //if (millis() - pastmillis >= debouncetime) {
    //pastmillis = millis();
    //for (int x = 0; x < numIR; x++) {
    //for (int y = 0; y < 3; y++) digitalWrite(pins[y], ci[x][y]);
    //val[x] = constrain(analogRead(muxpin), Min, Max);
    //val[x] = map(analogRead(muxpin), Min, Max, 0, 1023);
    //}
    //}
  }
  if (numIR == 8) {
    switch (valsensors) {
      case 0b10000000:
        error = 4;
        break;

      case 0b11000000:
        error = 3.5;
        break;

      case 0b01000000:
        error = 3;
        break;

      case 0b01100000:
        error = 2.5;
        break;

      case 0b00100000:
        error = 2;
        break;

      case 0b00110000:
        error = 1.5;
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
        error = -1.5;
        break;

      case 0b00000100:
        error = -2;
        break;

      case 0b00000110:
        error = -2.5;
        break;

      case 0b00000010:
        error = -3;
        break;

      case 0b00000011:
        error = -3.5;
        break;

      case 0b00000001:
        error = -4;
        break;

      default:
        error = lasterror;
        break;

    }
  }
  else if (numIR == 5) {
    switch (valsensors) {
      case 0b10000:
        error = 2;
        break;

      case 0b01000:
        error = 1;
        break;

      case 0b00100:
        error = 0;
        break;

      case 0b00010:
        error = -1;
        break;

      case 0b00001:
        error = -2;
        break;
      default:
        error = lasterror;
        break;
    }
  }
  return valsensors;
}
float IRline::PID() {
  float P, I, D;
  int val = 0;
  for (int i = 0; i < 100; i++) {
    val = analogRead(35) + val;
  }
  int Kp = (val / 100);
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
  //if(PID > 4095) PID = 4095;
  //else if(PID < -4095) PID = -4095;
  if (error == 0.1 || error == -0.1) {
    return error;
  } else {
    return int(PID);
  }
}
float IRline::getError() {
  return error;
}
void IRline::showIR() {
  Serial.println(valsensors, BIN);
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

class Motor {
  public:
    Motor(byte *pubpins);
    void frente(int v = 4095);
    void esquerda(int v = 4095);
    void direita(int v = 4095);
    void PIDctrl(float pid, byte sensors);
  private:
    byte valsensores;
    unsigned long previousMillis = 0;
    void noventagrausesq() {
      unsigned long currentMillis = millis();
      frente(4095 / 2);
      while (millis() - previousMillis < 1000) {}
      while ((valsensores & 0b00000110) == 0b00000110) {
        esquerda();
      }
    }
    void noventagrausdir() {
      unsigned long currentMillis = millis();
      frente(4095 / 2);
      while (millis() - previousMillis < 1000) {}
      while ((valsensores & 0b01100000) == 0b01100000) {
        direita();
      }
    }
    byte *pins;
};

Motor::Motor(byte *pubpins) {
  pins = pubpins;
  for (int i = 0; i < 4; i++) {

#ifdef Esp32
    pinMode(pins[i], OUTPUT);
    ledcSetup(1, 5000, 12);
    ledcSetup(2, 5000, 12);
    ledcAttachPin(pins[4], 1);
    ledcAttachPin(pins[5], 2);
#endif

#ifdef Arduino
    pinMode(pins[i], OUTPUT);
#endif
  }
}



void Motor::frente(int v) {
#ifdef Esp32
  ledcWrite(1, v);
  ledcWrite(2, v);
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
  ledcWrite(1, v);
  ledcWrite(2, v);
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
  ledcWrite(1, v);
  ledcWrite(2, v);
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

void Motor::PIDctrl(float pid, byte sensors) {
  float constant = 0.8;
  valsensores = sensors;

#ifdef Esp32
  int v = 4095 * constant;
#endif

#ifdef Arduino
  int v = 1023 * constant;
#endif


  int a = 0, b = 0;
  a = v + pid;
  b = v - pid;

#ifdef Esp32
  if (a > 4095) a = 4095;
  if (a < -4095) a = -4095;
  if (b < -4095) b = -4095;
  if (b > 4095) b = 4095;
#endif

#ifdef Arduino
  if (a > 1023) a = 1023;
  if (a < -1023) a = -1023;
  if (b < -1023) b = -1023;
  if (b > 1023) b = 1023;
#endif


  if ((sensors != 0b11111100) || (sensors != 0b00111111)) {
    if (pid > 0) { // vira para a direita
      if (b > 0) {
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
    } else if (pid < 0) { // vira para esquerda
      if (a > 0) {
#ifdef Esp32
        ledcWrite(1, a);
        ledcWrite(2, b);
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
        ledcWrite(1, -a);
        ledcWrite(2, b);
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
    } else if (pid == 0) {
#ifdef Esp32
      ledcWrite(1, a);
      ledcWrite(2, b);
      digitalWrite(pins[0], 0);
      digitalWrite(pins[1], 1);
      digitalWrite(pins[2], 1);
      digitalWrite(pins[3], 0);
#endif

#ifdef Arduino

#endif
    }
  } else {
    if (sensors == 0b11111100) {
      noventagrausesq();
    }
    else if (sensors == 0b00111111) {
      noventagrausdir();
    }
  }
  Serial.print(a);
  Serial.print(" ");
  Serial.print(b);
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


/*
  4 - direita tras
  5 - direita frente
  18 - esquerda frente
  19 - esquerda tras
  23 - pwm direita
*/
void pidoled();
void showK();
void oledstart();

byte pinos[8] = { 13, 12, 14, 27, 26, 25, 33, 32 };
byte m[6] = {4, 5, 18, 19, 2, 23};

Adafruit_SSD1306 display(128, 64, &Wire, -1);
IRline ir(pinos, 8);
Motor motor(m);



void setup() {
  Serial.begin(115200);
  oledstart();
  ir.calibrateIR();
}




void loop() {
  ir.getPlusUpdateIR();
  motor.PIDctrl(ir.PID(), ir.getPlusUpdateIR());
  Serial.print("    ");
  Serial.println(ir.PID());
  //ir.showIR();
  pidoled();
}




void oledstart() {
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println(F("falha na alocação do endereço i2c do display"));
    for (;;); // Don't proceed, loop forever
  }
  display.clearDisplay();
}
void pidoled() {
#ifdef OLED
  int val = 0;
  for (int i = 0; i < 100; i++) {
    val = analogRead(35) + val;
  }
  display.clearDisplay();
  display.setTextSize(3);
  display.setTextColor(WHITE);
  display.setCursor(0, 0);
  display.print(val / 100);
  display.display();
#endif
}
