#include <Arduino.h>

struct status {
    int previous_status;
    int current_status;
}
status mystatus;
mystatus.previous_status = 0;
while(true){



    mystatus.previous_status = mystatus.current_status;
}