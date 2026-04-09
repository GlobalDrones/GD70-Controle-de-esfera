#include <Arduino.h> 
#include <Servo.h> 

// Definição dos Pinos
#define BF_PIN 6
#define AS_PIN 5
#define TR_PIN 4
#define ENCODER_PIN 9 

Servo esc_bf; 
Servo esc_as; 
Servo esc_tr; 

void setup() { 
  Serial.begin(115200); 
  pinMode(ENCODER_PIN, INPUT); 

  // Anexa os ESCs aos pinos
  esc_bf.attach(BF_PIN); 
  esc_as.attach(AS_PIN); 
  esc_tr.attach(TR_PIN); 

  // 1. PASSO: Envia o sinal MÁXIMO (2000us) imediatamente ao ligar o Arduino
  Serial.println("=== MODO DE CALIBRAÇÃO DE ESC ===");
  Serial.println("Enviando sinal MAX (2000us).");
  Serial.println("-> LIGUE A BATERIA DOS ESCs AGORA e aguarde os bips musicais!");
  
  esc_bf.writeMicroseconds(2000); 
  esc_as.writeMicroseconds(2000); 
  esc_tr.writeMicroseconds(2000); 

  // 2. PASSO: Trava o código aqui até que o sinal do encoder passe de 10
  Serial.println("Aguardando sinal do encoder (> 10) para baixar para 1000us...");
  
  int pwm_encoder = 0;
  while (pwm_encoder <= 10) {
    // Lê o pulso. Se não houver sinal, pulseIn retorna 0.
    pwm_encoder = pulseIn(ENCODER_PIN, HIGH, 25000); 
    delay(10); // Pequeno delay para estabilidade do loop
  }

  // 3. PASSO: Sinal do encoder recebido! Baixa para o MÍNIMO (1000us)
  Serial.print("Sinal recebido: ");
  Serial.println(pwm_encoder);
  Serial.println("Enviando sinal MIN (1000us)... Aguarde os bips de confirmacao longos!");
  
  esc_bf.writeMicroseconds(1000); 
  esc_as.writeMicroseconds(1000); 
  esc_tr.writeMicroseconds(1000); 
  
  Serial.println("Calibracao finalizada com sucesso!");
} 

void loop() { 
  // Mantém o sinal em 1000us (neutro/desligado) para segurança
  // Não precisamos fazer nada no loop além de manter os motores desarmados
  esc_bf.writeMicroseconds(1000); 
  esc_as.writeMicroseconds(1000); 
  esc_tr.writeMicroseconds(1000); 
  delay(100);
}