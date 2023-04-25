#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

class Motor
{
public:
    Motor(byte *pubpins);
    void frente(int v = 4095);
    void esquerda(int v = 4095);
    void direita(int v = 4095);
    void speedctrl(int v = 4095);
    void PIDctrl(int pid);

private:
    byte *pins;
};

Motor::Motor(byte *pubpins)
{
    pins = pubpins;
    for (int i = 0; i < 4; i++)
    {
        ledcSetup(i + 1, 5000, 12);
        ledcAttachPin(pins[i], i + 1);
    }
}

void Motor::frente(int v)
{
    ledcWrite(1, v);
    ledcWrite(2, 0);
    ledcWrite(3, v);
    ledcWrite(4, 0);
}

void Motor::direita(int v)
{
    ledcWrite(1, 0);
    ledcWrite(2, v);
    ledcWrite(3, v); // motor da esquerda
    ledcWrite(4, 0);
}

void Motor::esquerda(int v)
{
    ledcWrite(1, v); // motor da direita
    ledcWrite(2, 0);
    ledcWrite(3, 0);
    ledcWrite(4, v);
}

void Motor::PIDctrl(int pid)
{
    int v = 4095 / 2;   
    int a = 0, b = 0;

    a = v + pid;
    b = v - pid;
    if ((v + pid) > 4095)
        a = 4095;
    if ((v - pid) < 0)
        b = 0;

    if (pid > 0)
    {
        a = v + pid;
        b = v - pid;
        if ((v + pid) > 4095)
            a = 4095;
        if ((v - pid) < 0)
            b = 0;
        ledcWrite(1, b); // direita
        ledcWrite(3, a); // esquerda
    }
    else
    {
        a = v - pid;
        b = v + pid;
        if ((v + pid) > 4095)
            a = 4095;
        if ((v - pid) < 0)
            b = 0;
        ledcWrite(1, a); // direita
        ledcWrite(3, b); // esquerda
    }
    Serial.print(a);
    Serial.print(" ");
    Serial.println(b);
}

class IRline
{
public:
    IRline(byte *pubpins, byte pubnumIR, byte pubmuxpin = 0, bool pubmode = 1);
    void updateIR(int debouncetime = 0);
    void calibrateIR(int waittime = 5000);
    void showIR();
    int PID();

private:
    int mid[8];
    byte ci[16][3] = {
        {0, 0, 0},
        {0, 0, 1},
        {0, 1, 0},
        {0, 1, 1},
        {1, 0, 0},
        {1, 0, 1},
        {1, 1, 0},
        {1, 1, 1},
    };
    float error, lasterror;
    int valsensors = 0;
    bool mode;
    byte numIR;
    byte muxpin;
    byte *pins;
    unsigned long pastmillis = 0;
};

IRline::IRline(byte *pubpins, byte pubnumIR, byte pubmuxpin, bool pubmode)
{
    muxpin = pubmuxpin;
    numIR = pubnumIR;
    mode = pubmode;
    pins = pubpins;

    if (mode == 1)
    {
        for (int i = 0; i < numIR; i++)
        {
            pinMode(pins[i], INPUT); // modo sem multiplexador
        }
    }
    else
    {
        for (int i = 0; i < 3; i++)
        {
            pinMode(pins[i], OUTPUT);
            pinMode(muxpin, INPUT); // modo com multiplexador
        }
    }
}

void IRline::calibrateIR(int waittime)
{
    int calibratedvals[numIR];
    int maxs[numIR] = {0, 0, 0, 0, 0, 0, 0, 0};
    int mins[numIR] = {4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095};
    while (millis() < waittime)
    {
        for (int i = 0; i < numIR; i++)
        {
            calibratedvals[i] = analogRead(pins[i]);
            if (calibratedvals[i] > maxs[i])
                maxs[i] = calibratedvals[i];
            if (calibratedvals[i] < mins[i])
                mins[i] = calibratedvals[i];
            calibratedvals[i] = constrain(analogRead(pins[i]), mins[i], maxs[i]);
            mid[i] = int((mins[i] + maxs[i]) / 2.0);
        }
    }
}

void IRline::updateIR(int debouncetime)
{
    valsensors = 0;
    if (mode == 1)
    { // modo sem multiplexador
        if (millis() - pastmillis >= debouncetime)
        {
            pastmillis = millis();
            for (int i = 0; i < numIR; i++)
            {
                valsensors = valsensors | (((analogRead(pins[i]) > (mid[i] * 1.1)) ? 1 : 0) << i);
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
    switch (valsensors)
    {
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
    }
}

int IRline::PID()
{
    float P, I, D;
    int val = 0;
    for (int i = 0; i < 100; i++)
    {
        val = analogRead(35) + val;
    }
    int Kp = (val / 100) / 2;
    int Ki = (val / 100) / 2;
    int Kd = 0;

    P = error;
    I = I + error;
    D = error - lasterror;

    if (error == 0)
        I = 0;
    if (I > 255)
        I = 255;
    else if (I < -255)
        I = -255;

    int PID = int((Kp * P) + (Ki * I) + (Kd * D));
    lasterror = error;
    if (PID > 4095)
        PID = 4095;
    else if (PID < -4095)
        PID = -4095;
    return PID;
}

byte m[4] = {4, 5, 18, 19};
byte pinos[8] = {13, 12, 14, 27, 26, 25, 33, 32};



Adafruit_SSD1306 display(128, 64, &Wire, -1);
IRline ir(pinos, 8);
Motor motor(m);

void setup()
{
    Serial.begin(115200);
    if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C))
    {
        Serial.println(F("falha na alocação do endereço i2c do display"));
        for (;;)
            ; // Don't proceed, loop forever
    }
    display.clearDisplay();
    ir.calibrateIR();
}

void loop()
{
    ir.updateIR();
    motor.PIDctrl(ir.PID());
    int val = 0;
    for (int i = 0; i < 100; i++)
    {
        val = analogRead(35) + val;
    }
    Serial.println(ir.PID());
    display.clearDisplay();
    display.setTextSize(3);
    display.setTextColor(WHITE);
    display.setCursor(0, 0);
    display.print(val / 100);
    display.display();
}