# Motores e seus Drivers
<div style="text-align: justify">
Garantimos mobilidade ao robô através de um conjunto de rodas e motores. Os motores usados provem do kit de robótica educacional Lego MindStorm EV3. Utilizamos o tamanho large de motor. </br>

![](https://github.com/predomaquilare/lotusobr/blob/main/Assets/motor-large-ev3.jpg)

</br>	

## Controlando os motores com MCU
É de conhecimento geral que motores são cargas que demandam correntes elevadas do circuito (se comparado aos valores disponibilizados pela saída de um microcontrolador). Com isso se faz necessário o uso de uma interface entre a parte lógica do robô (como o próprio microcontrolador) e a parte que demanda mais **potência** (os motores). </br> </br>
Para tal fim usamos uma ***ponte H*** .  </br>
Mais especificamente, o CI l293d, de fácil operação e facilmente adquirível.
![l293d](https://github.com/predomaquilare/lotusobr/blob/main/Assets/l293d.gif) 







</div>