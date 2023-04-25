class Motor
{
public:
    Motor(byte *pubpins);
    /* void frente(int v = 4095);
     void esquerda(int v = 4095);
     void direita(int v = 4095);
     void speedctrl(int v = 4095); */
    void PIDctrl(int pid);
    void run(int va, int vb);

private:
    byte *pins;
    max_pwm = 4095;
};

void Motor::run(int va, int vb)
{

    ledcWrite(pins[0], va > 0 ? va : 0);
    ledcWrite(pins[1], va < 0 ? va : 0);
    ledcWrite(pins[2], vb > 0 ? vb : 0);
    ledcWrite(pins[3], vb < 0 ? vb : 0);
}

void Motor::PIDctrl(int pid)
{
    int v = max_pwm / 2;
    int va = 0, vb = 0;

    va = v + pid;
    vb = v - pid;

    if (va > max_pwm)
        va = max_pwm;
    else if (va < -max_pwm)
        va = -max_pwm;
    if (vb > max_pwm)
        vb = max_pwm;
    else if (vb < -max_pwm)
        vb = -max_pwm;

    run(va, vb);

    Serial.print(a);
    Serial.print(" ");
    Serial.println(b);
}

class IRline
{
public:
    IRline() {}
    IRline(byte *pubpins, byte pubnumIR,
           byte pubmuxpin = 0, bool pubmode = 1);

    void updateIR(int debouncetime = 0);
    void calibrateIR(int waittime = 5000);
    void showIR();
    int PID();

private:
    int mid[8];
    byte ci[8][3] = {
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
                // valsensors = valsensors | (((analogRead(pins[i]) > (mid[i] * 1.1)) ? 1 : 0) << i);
                valsensors |= digitalRead(pins[i]) << i;
            }
        }
    }
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

    int Kp = 0;
    int Ki = 0;
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

class DigitalIRline : public IRline
{
public:
    DigitalIRline() {}
};

class AnalogIRline : public IRline
{
};
class Mux
{
};

class DigitalMuxIRline : public Mux, public DigitalIRline
{
};

IRline &IRfactory(bool MUX, bool ANALOG)
{
    if (MUX == false && ANALOG == false)
    {
        static DigitalMuxIRline result;
        return result;
    }
}

byte m[4] = {4, 5, 18, 19};
byte pinos[8] = {13, 12, 14, 27, 26, 25, 33, 32};

// IRline(pinos,8,4);
IRline ir(pinos, 8);
Motor motor(m);

void setup()
{
    Serial.begin(115200);
    ir.calibrateIR();
}

void loop()
{
    ir.updateIR();
    motor.PIDctrl(ir.PID());
    Serial.println(ir.PID());
}

class IRline
{
public:
    int pid();
    float update_leitura();

protected:
    byte *pinos[8];
};

class mux
{
public:
    mux();
    void select_output(int output);

private:
    byte *pinos_switch[3] = {};
};

class IRline_digital : public IRline
{
public:
    IRline_digital();
    byte update_leitura();

protected:
    byte leitura = 0b0;
};

class IRline_com_MUX : public IRline, public IRline_digital
{
};

IRline &IRline_factory(int type){
    if(type == 1){
        static IRline_digital myIR;
        return myIR;
    }
}

IRline& myIR = IRline_factory(1);