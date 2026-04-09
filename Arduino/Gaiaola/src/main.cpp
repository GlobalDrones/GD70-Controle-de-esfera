#include <Arduino.h> 
#include <RotaryEncoder.h>
#include <Servo.h>

// Encoder
RotaryEncoder encoder(A2, A3);

// PWM estilo servo
Servo pwmEnc;
Servo pwmPot;

// Pinos
const int pinoPWM_Encoder = 6;
const int pinoPWM_Pot = 5;
const int pinoPotenciometro = A0;
const int botaoPin = 7;

// Variáveis
int valor = 0;
int ultimaPorcentagem = -1;
int ultimaPosicaoEncoder = -999;

int grausAtual = 0;
int pwmEncoder_us = 1500; // neutro
int pwmPot_us = 1500;

void setup()
{
  pinMode(botaoPin, INPUT);

  // Inicializa PWM tipo servo
  pwmEnc.attach(pinoPWM_Encoder);
  pwmPot.attach(pinoPWM_Pot);

  Serial.begin(9600);
  Serial.println("Gire o encoder ou mova o potenciometro...");

  // Começa em neutro
  pwmEnc.writeMicroseconds(1500);
  pwmPot.writeMicroseconds(1500);
}

void loop()
{
  bool precisaImprimir = false;

  // -------- POTENCIÔMETRO --------
  int leituraRaw = analogRead(pinoPotenciometro);
  int porcentagem = map(leituraRaw, 0, 1023, 0, 100);

  if (porcentagem != ultimaPorcentagem) {

    pwmPot_us = map(leituraRaw, 0, 1023, 1000, 2000);
    pwmPot.writeMicroseconds(pwmPot_us);

    ultimaPorcentagem = porcentagem;
    precisaImprimir = true;
  }

  // -------- BOTÃO --------
  valor = digitalRead(botaoPin);
  if (valor == 0)
  {
    Serial.println("Botao pressionado");
    while (digitalRead(botaoPin) == 0)
      delay(10);
  }

  // -------- ENCODER --------
  encoder.tick();
  int newPos = encoder.getPosition();

  if (ultimaPosicaoEncoder != newPos) {

    int anguloBruto = newPos * 2;
    grausAtual = (anguloBruto % 360 + 360) % 360;

    pwmEncoder_us = map(grausAtual, 0, 359, 1000, 2000);
    pwmEnc.writeMicroseconds(pwmEncoder_us);

    ultimaPosicaoEncoder = newPos;
    precisaImprimir = true;
  }

  // -------- SERIAL --------
  if (precisaImprimir) {
    Serial.print("Angulo: ");
    Serial.print(grausAtual);

    Serial.print(" | PWM(us): ");
    Serial.print(pwmEncoder_us);

    Serial.print(" | Intensidade: ");
    Serial.print(porcentagem);
    Serial.println("%");
  }
}