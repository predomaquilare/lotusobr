/*
============================ /// ===================================================
          
          
          Biblioteca de Motores controlados por Shift Register
          @GRELIFPB 
          By Eduardo,Tomaz,Yan & Pedro.



============================ /// ====================================================
*/
#ifndef shiftmotor_H
#define shiftmotor_H

#include "arduino.h"

class motor{
  public:
  //declaração de objeto
  motor(byte latch, byte shift, byte data);
  //declaração de métodos
  void frente();
  void re();
  void direita();
  void esquerda();

  private:
  //declaração de variavéis privadas
  int bin;
  byte latchpriv;
  byte shiftpriv;
  byte datapriv;
  void shifto(byte dataPin, byte clockPin, byte bitOrder, byte val);
};

#endif
