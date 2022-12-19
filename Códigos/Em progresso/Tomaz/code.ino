#include <Arduino.h>


class motor {
    private:
        enum status {
            frente,
            tras,
            inerte,
            break,
        };
        status oldstate = inerte;
        status newstate;

        uint8_t pino_1_motor
        uint8_t pino_2_do_motor
    public:
    //definir com a equipe que pino do motor vai ser o 1 e que pino do motor vai ser o 2
        motor(uint8_t pino_motor1,uint8_t pino_motor2){
            pino_1_do_motor = pino_motor1;
            pino_2_do_motor = pino_2_do_motor;
            
            pinMode(pi)
       
        }
        void gire(status change){
            if(change != oldstate){
                switch(change){
                    case frente:
                        break;
                    case tras:

                        break; 
                }
            }
        }
        
}


