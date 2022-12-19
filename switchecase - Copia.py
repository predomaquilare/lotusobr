#include <stdio.h>
#include <math.h>

int main(void) {
 int n;
  printf("digite um número entre 0 a 10:\n");
  scanf("%d",&n);
  switch(n){
    case 0:
      printf("preto!");
      break;
    case 1:
      printf("marrom!");
      break;
    case 2:
      printf("vermelho!");
      break;
    case 3:
      printf("laranja!");
      break;
    case 4:
      printf("amarelo!");
      break;
    case 5:
      printf("verde!");
      break;
    case 6:
      printf("azul!");
      break;
    case 7:
      printf("violeta!");
      break;
    case 8:
      printf("cinza!");
      break;
    case 9:
      printf("branca!");
      break;
  }
  return 0;
}