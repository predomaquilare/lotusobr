
#define see 13
#define se  A0
#define sc 8
#define sd 4
#define sdd 2

#define mef 11
#define mer 10
#define mdf 5
#define mdr 3

#include <bibliotecamotor.h>

motor md (mdf, mdr);
motor me (mef, mer);

void setup() {
  pinMode(sdd, INPUT);
  pinMode(sd, INPUT);
  pinMode(sc, INPUT);
  pinMode(se, INPUT);
  pinMode(see, INPUT);
  pinMode(9, OUTPUT);
  pinMode(A0, INPUT);

  Serial.begin(9600);
}

void loop() {

  // put your main code here, to run repeatedly:
  //analogWrite(A1,1023);
  //analogWrite(A2,1023);
  //Serial.println(digitalRead(13));
  //
  //Serial.println(digitalRead(A0));
  //leitura ();
  segue_faixa();
  //teste();

}

void segue_faixa() {
  /*if (digitalRead(see) == 1 && digitalRead(se) == 1 && digitalRead(sc) == 0 && digitalRead(sd) == 1 && digitalRead(sdd) == 1) {
    frente(140);
    }
  */
  //direita com um só
  if (digitalRead(see) == 1 && digitalRead(se) == 1 && digitalRead(sc) == 1 && digitalRead(sd) == 0 && digitalRead(sdd) == 1) {
    while (digitalRead(sc) == 1) {
      direita(130);
    }

  }
  //esquerda com um só
  else if (digitalRead(see) == 0 && digitalRead(se) == 0 && digitalRead(sc) == 1 && digitalRead(sd) == 1 && digitalRead(sdd) == 1) {
    while (digitalRead(sc) == 1) {
      esquerda(130);
    }
  }
  // direita com o da ponta
  else if (digitalRead(see) == 1 && digitalRead(se) == 1 && digitalRead(sc) == 1 && digitalRead(sd) == 1 && digitalRead(sdd) == 0) {
    direita(130);
    delay(400);

  }
  //esquerda com o da penta
  else if (digitalRead(see) == 0 && digitalRead(se) == 1 && digitalRead(sc) == 1 && digitalRead(sd) == 1 && digitalRead(sdd) == 1) {
    esquerda(130);
    delay(400);
  }
  //direita com os dois
  else if (digitalRead(see) == 1 && digitalRead(se) == 1 && digitalRead(sc) == 1 && digitalRead(sd) == 0 && digitalRead(sdd) == 0 ) {
    frente(130);
    delay(130);
    while (digitalRead(sc) == 1) {
      direita(140);
    }
  }
  //esquerda com os dois
  else if (digitalRead(see) == 0 && digitalRead(se) == 0 && digitalRead(sc) == 1 && digitalRead(sd) == 1 && digitalRead(sdd) == 1 ) {
    frente(130);
    delay(100);
    while (digitalRead(sc) ==  1) {
      esquerda(140);
    }
  }
  //direita com os tres
  else if (digitalRead(see) == 1 && digitalRead(se) == 1 && digitalRead(sc) == 0 && digitalRead(sd) == 0 && digitalRead(sdd) == 0 ) {
    para();
    delay(200);
    frente(130);
    delay(100);
    while (digitalRead(se) == 1) {
      me.frente(180);
      md.re(100);
    }
  }
  //esquerda com os tres
  else if (digitalRead(see) == 0 && digitalRead(se) == 0 && digitalRead(sc) == 0 && digitalRead(sd) == 1 && digitalRead(sdd) == 1) {
    para();
    delay(200);
    frente(130);
    delay(100);
    while (digitalRead(sd) == 1) {
      md.frente(180);
      me.re(100);;
    }
  }

  //direita com os quatro
  else if (digitalRead(see) == 1 && digitalRead(se) == 0 && digitalRead(sc) == 0 && digitalRead(sd) == 0 && digitalRead(sdd) == 0 ) {
    para();
    delay(200);
    frente(120);
    delay(150);
    while (digitalRead(sc) == 1) {
      direita(200);
    }
  }
  //esquerda com os quatro
  else if (digitalRead(see) == 0 && digitalRead(se) == 0 && digitalRead(sc) == 0 && digitalRead(sd) == 0 && digitalRead(sdd) == 1) {
    para();
    delay(200);
    frente(120);
    delay(150);
    while (digitalRead(sc) == 1) {
      esquerda(200);
    }
  }
  else if (digitalRead(see) == 0 && digitalRead(se) == 0 && digitalRead(sc) == 0 && digitalRead(sd) == 0 && digitalRead(sdd) == 0) {
    para();
    delay(500);
    frente(150);
    delay(500);
  }

  else {
    frente(120);
  }


}

void frente(int v) {
  md.frente(v);
  me.frente(v);
}
void re(int v) {
  md.re(v);
  me.re(v);
}
void direita(int v) {
  me.frente(v);
  md.re(v);
}
void esquerda(int v) {
  md.frente(v);
  me.re(v);
}
void esq_so() {
  md.frente(90);
  me.para();
}
void dir_so() {
  me.frente(90);
  md.para();
}
void para() {
  md.para();
  me.para();
}
void teste() {

  for (int i = 0; i <= 255; i++) {
    me.frente(i);
    delay(10);
  }
  for (int i = 0; i <= 255; i++) {
    me.re(i);
    delay(100);
  }
  /*md.para();
    delay(1000);
    md.re(20);
    delay(1000);
    md.para();
    delay(1000);
    me.frente(20);
    delay(1000);
    me.para();
    delay(1000);
    me.re(20);
    delay(1000);
    me.para();
    delay(1000);
  */
}
void leitura() {
  Serial.println("leitura do sdd");
  Serial.print(digitalRead(sdd));
  Serial.print("\n");
  Serial.println("leitura do sd");
  Serial.print(digitalRead(sd));
  Serial.print("\n");
  Serial.println("leitura do sc");
  Serial.println(digitalRead(sc));
  Serial.print("\n");
  Serial.println("leitura do se");
  Serial.print(digitalRead(se));
  Serial.print("\n");
  Serial.println("leitura do see");
  Serial.print(digitalRead(see));
  Serial.print("\n");
  delay(1000);
}
