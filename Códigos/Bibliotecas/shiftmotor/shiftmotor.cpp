/*
============================ /// ===================================================
          
          
          Biblioteca de Motores controlados por Shift Register
          @GRELIFPB 
          By Eduardo,Tomaz,Yan & Pedro.



============================ /// ====================================================
*/
#include <Arduino.h>
#include <shiftmotor.h>

motor::motor(latch,shift,data) {

  //declaração de variavéis encapsulamento equivalentes as variáveis do objeto
  latchpriv = latch;
  shiftpriv = shift;
  datapriv = data;

  //declaração de pinos
  pinMode(latchpriv,OUTPUT);
  pinMode(shiftpriv,OUTPUT);
  pinMode(datapriv,OUTPUT);
}

motor::frente(){
  bin;//binario
  digitalWrite(latchpriv,HIGH);

  for (i = 0; i < 8; i++)  {
    if (bitOrder == LSBFIRST) {
      digitalWrite(datapriv, bin & 1);
      bin >>= 1;
    } else {  
      digitalWrite(datapriv, (bin & 128) != 0);
      bin <<= 1;
    }
    digitalWrite(clockPin, HIGH);
    digitalWrite(clockPin, LOW);    
  }



  
  digitalWrite(latchpriv,LOW);
}


motor::re(){
  digitalWrite(latchpriv,HIGH);
  for (i = 0; i < 8; i++)  {
    if (bitOrder == LSBFIRST) {
      digitalWrite(datapriv, bin & 1);
      bin >>= 1;
    } else {  
      digitalWrite(datapriv, (bin & 128) != 0);
      bin <<= 1;
    }
    digitalWrite(clockPin, HIGH);
    digitalWrite(clockPin, LOW);    
  }
  digitalWrite(latchpriv,LOW);
}

motor::direita(){
  digitalWrite(latchpriv,HIGH);
  for (i = 0; i < 8; i++)  {
    if (bitOrder == LSBFIRST) {
      digitalWrite(datapriv, bin & 1);
      bin >>= 1;
    } else {  
      digitalWrite(datapriv, (bin & 128) != 0);
      bin <<= 1;
    }
    digitalWrite(latchpriv,LOW);
    digitalWrite(clockPin, HIGH);
    digitalWrite(clockPin, LOW);    
  }
  digitalWrite(latchpriv,LOW);
}
motor::esquerda(){
  digitalWrite(latchpriv,HIGH);
  for (i = 0; i < 8; i++)  {
    if (bitOrder == LSBFIRST) {
      digitalWrite(datapriv, bin & 1);
      bin >>= 1;
    } else {  
      digitalWrite(datapriv, (bin & 128) != 0);
      bin <<= 1;
    }
    digitalWrite(latchpriv,LOW);
    digitalWrite(clockPin, HIGH);
    digitalWrite(clockPin, LOW);    
  }
  digitalWrite(latchpriv,LOW);
}
motor::shifto( dataPin,  clockPin,  bitOrder,  val){
  uint8_t i;

  
}
