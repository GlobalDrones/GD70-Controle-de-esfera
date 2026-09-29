//* 
#include <Arduino.h>
#include <Wire.h>
#include <Servo.h>
#include <AS5600.h> // Biblioteca dedicada do AS5600

#define DEADZONE 3
#define ESC_MIN_POWER 80

// --- LIMITES ABSOLUTOS DE PWM DOS MOTORES ---
#define LIMIT_PWM_MAX 2000  
#define LIMIT_PWM_MIN 1000  

// --- PINOS MAPEADOS PARA A STM32F401/F411 (BLACK PILL) ---
#define BF_PIN PA0   // Frente
#define AS_PIN PA1   // Lados
#define TR_PIN PA6   // Traseira

// ============================================================
// --- PINOS DA PONTE H BTS7960 ---
#define EN_R_PIN  PB7 
#define EN_L_PIN  PB10 
#define RPWM_PIN  PB6  
#define LPWM_PIN  PB5 

// --- DEFINIÇÕES DOS PINOS ---
// Ponte H L298N (Canal B - ligado em OUT3/OUT4)
#define IN3_PIN PB12 
#define IN4_PIN PB13 

// Fins de Curso
#define FIM_CURSO_CIMA  PB14 
#define FIM_CURSO_BAIXO PB15 

// Novos pinos para o Sensor Óptico (Encoder / LM393)
#define SENSOR_OLHAL_D0 PA4 // Conectado ao pino D0 do sensor
#define SENSOR_OLHAL_A0 PA5 // Conectado ao pino A0 do sensor

// Pinos I2C para o Encoder AS5600 (STM32 - Hardware I2C1)
#define AS5600_SDA PB9
#define AS5600_SCL PB8

// LED Onboard (Black Pill - Active LOW)
#define LED_PIN PC13


#define ESC_NEUTRAL 1500
#define MPU6050_ADDR 0x68
#define SAMPLES_GYRO 500


// INICIALIZAÇÃO DO ENCODER AS5600
// Instanciando o objeto do Encoder
AS5600 encoder;

// Estado atual do movimento: 1 = Subindo, -1 = Descendo, 0 = Parado
int estadoMotor = 0;

// ============================================================
// --- VARIÁVEIS PARA O FILTRO (DEBOUNCE) DO OLHAL ---
// ============================================================
int estadoEstavelOlhal = HIGH; // Iniciamos assumindo que o feixe está bloqueado (sem olhal)     
int ultimoEstadoFisicoOlhal = HIGH;
unsigned long ultimoTempoFiltro = 0;
unsigned long tempoFiltro = 100; // Reduzido para 100ms pois sensor óptico tem menos ruído que mecânico
// ============================================================

// --- FUNÇÃO DO ENCODER AS5600 (USANDO A BIBLIOTECA) ---
void lerEncoderAS5600() {
    if (encoder.isConnected()) {
        int anguloBruto = encoder.readAngle();
        float graus = (anguloBruto / 4095.0) * 360.0;

        Serial.print(">> AS5600 - Angulo Bruto: ");
        Serial.print(anguloBruto);
        Serial.print(" | Graus: ");
        Serial.println(graus, 2);
    } else {
        Serial.println("[ERRO] AS5600 não encontrado! Verifique a fiação I2C.");
    }
}

// --- FUNÇÕES DE CONTROLE DO MOTOR (L298N) ---
void pararMotor() {
    digitalWrite(IN3_PIN, LOW);
    digitalWrite(IN4_PIN, LOW);
    estadoMotor = 0;
}

void moverSubir() {
    digitalWrite(IN3_PIN, HIGH);
    digitalWrite(IN4_PIN, LOW);
    estadoMotor = 1;
    Serial.println(">> Motor L298N: SUBINDO");
}

void moverDescer() {
    digitalWrite(IN3_PIN, LOW);
    digitalWrite(IN4_PIN, HIGH);
    estadoMotor = -1;
    Serial.println(">> Motor L298N: DESCENDO");
}

// --- FUNÇÕES DE CONTROLE DA BTS7960 ---
void desativarDriver() {
    digitalWrite(EN_R_PIN, LOW);
    digitalWrite(EN_L_PIN, LOW);
}

void ativarDriver() {
    digitalWrite(EN_R_PIN, HIGH);
    digitalWrite(EN_L_PIN, HIGH);
}

void pararBTS() {
    digitalWrite(RPWM_PIN, LOW);
    digitalWrite(LPWM_PIN, LOW);
    digitalWrite(LED_PIN, HIGH); 
    Serial.println(">> BTS7960: Motor PARADO");
}

void girarSentidoA() {
    ativarDriver();
    digitalWrite(RPWM_PIN, HIGH);
    digitalWrite(LPWM_PIN, LOW);
    digitalWrite(LED_PIN, LOW); 
    Serial.println(">> BTS7960: Girando no SENTIDO A (Direita/Horario)");
}

void girarSentidoD() {
    ativarDriver();
    digitalWrite(RPWM_PIN, LOW);
    digitalWrite(LPWM_PIN, HIGH);
    digitalWrite(LED_PIN, LOW); 
    Serial.println(">> BTS7960: Girando no SENTIDO D (Esquerda/Anti-horario)");
}

//INICIALIZAÇÃO DO MPU6050
Servo esc_bf;
Servo esc_as;
Servo esc_tr;

HardwareSerial Serial2(PA3, PA2);

int16_t gyro_z_raw;
float gyro_offset_z = 0;
float filtered_gyro_rate = 0;
float alpha = 0.2;

float yaw_gyro = 0;
float desired_yaw = 0;
float intensidade_mult = 1.0;

float pos_error = 0;
float yaw_output = 0;

unsigned long last_time;
unsigned long last_telemetry_time = 0;
const unsigned long TELEMETRY_INTERVAL = 100; // 10Hz

float angle_diff(float a, float b) {
  float d = a - b;
  while (d > 180.0) d -= 360.0;
  while (d < -180.0) d += 360.0;
  return d;
}

// --- FUNÇÃO DE DESTRAVAMENTO FÍSICO DO BARRAMENTO I2C ---
void destravar_barramento_I2C() {
  pinMode(AS5600_SCL, OUTPUT);       
  pinMode(AS5600_SDA, INPUT_PULLUP); 

  for (int i = 0; i < 9; i++) {
    digitalWrite(AS5600_SCL, LOW);
    delayMicroseconds(5);
    digitalWrite(AS5600_SCL, HIGH);
    delayMicroseconds(5);
  }

  pinMode(AS5600_SDA, OUTPUT);
  digitalWrite(AS5600_SDA, LOW);
  delayMicroseconds(5);
  digitalWrite(AS5600_SCL, HIGH);
  delayMicroseconds(5);
  digitalWrite(AS5600_SDA, HIGH);
}

void init_mpu() {
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(0x6B);
  Wire.write(0x00); 
  Wire.endTransmission();
 
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(0x37);
  Wire.write(0x02);
  Wire.endTransmission();
}

void calibrate_gyro() {
  Serial.println("=== CALIBRANDO GYRO ===");
  for(int j = 0; j < 20; j++) {
    digitalWrite(LED_PIN, HIGH); delay(100);
    digitalWrite(LED_PIN, LOW); delay(100);
  }

  long sum_z = 0;
  bool calib_led_state = false;

  for (int i = 0; i < SAMPLES_GYRO; i++) {
    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(0x43);
    if (Wire.endTransmission(false) != 0) { i--; delay(3); continue; } 

    Wire.requestFrom(MPU6050_ADDR, 6);
    if (Wire.available() >= 6) {
      Wire.read(); Wire.read();
      Wire.read(); Wire.read();
      int16_t raw_z = (int16_t)(Wire.read() << 8 | Wire.read());
      sum_z += raw_z;
    }

    if (i % 25 == 0) {
      calib_led_state = !calib_led_state;
      digitalWrite(LED_PIN, calib_led_state ? HIGH : LOW);
    }
    delay(3);
  }
  digitalWrite(LED_PIN, LOW);
  gyro_offset_z = (sum_z / (float)SAMPLES_GYRO) / 131.0;
}

void read_gyro(float dt) {
    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(0x43);
    if (Wire.endTransmission(false) != 0) return; 

    Wire.requestFrom(MPU6050_ADDR, 6);
    if (Wire.available() >= 6) {
        Wire.read(); Wire.read();
        Wire.read(); Wire.read();
        gyro_z_raw = (int16_t)(Wire.read() << 8 | Wire.read());

        float raw_rate = (gyro_z_raw / 131.0) - gyro_offset_z;
        filtered_gyro_rate = (alpha * raw_rate) + ((1.0 - alpha) * filtered_gyro_rate);

        if (abs(filtered_gyro_rate) < 0.5) {
            filtered_gyro_rate = 0.0;
        }

        yaw_gyro += filtered_gyro_rate * dt;

        if (yaw_gyro >= 360.0) yaw_gyro -= 360.0;
        if (yaw_gyro < 0.0) yaw_gyro += 360.0;
    }
}

String inputBuffer = "";
void checar_serial_nao_bloqueante() {
    while (Serial2.available() > 0) {
        char c = Serial2.read();
        if (c == '\n') {
            inputBuffer.trim();
            if (inputBuffer.length() > 0) {
                float camera_angle = inputBuffer.toFloat();
                yaw_gyro = 0;
                desired_yaw = camera_angle;
            }
            inputBuffer = "";
        } else {
            inputBuffer += c;
            if (inputBuffer.length() > 50) inputBuffer = "";
        }
    }
}

void init_L298() {
    pinMode(IN3_PIN, OUTPUT);
    pinMode(IN4_PIN, OUTPUT);
    pararMotor();

    pinMode(EN_R_PIN, OUTPUT);
    pinMode(EN_L_PIN, OUTPUT);
    pinMode(RPWM_PIN, OUTPUT);
    pinMode(LPWM_PIN, OUTPUT);
   
    ativarDriver();
    pararBTS();

    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, HIGH);

    pinMode(FIM_CURSO_CIMA, INPUT_PULLDOWN);
    pinMode(FIM_CURSO_BAIXO, INPUT_PULLDOWN);
    
    // Configura os pinos do Sensor Óptico
    pinMode(SENSOR_OLHAL_D0, INPUT);
    pinMode(SENSOR_OLHAL_A0, INPUT); 

    Serial.begin(115200);
    while (!Serial && millis() < 4000);

    Wire.setSDA(AS5600_SDA);
    Wire.setSCL(AS5600_SCL);
    Wire.begin();
    encoder.begin();

    Serial.println("==============================================");
    Serial.println(" SISTEMA L298N + BTS7960 + AS5600 INICIADO    ");
    Serial.println("  L298N:   'w'=Subir | 's'=Descer | 'x'=Parar ");
    Serial.println("  BTS7960: 'a'=Giro A| 'd'=Giro D | 'c'=Parar ");
    Serial.println("  AS5600:  'e'=Ler Angulo Atual               ");
    Serial.println("==============================================");
}

void setup() {
  pinMode(LED_PIN, OUTPUT);

  esc_bf.attach(BF_PIN);
  esc_as.attach(AS_PIN);
  esc_tr.attach(TR_PIN);

  esc_bf.writeMicroseconds(ESC_NEUTRAL);
  esc_as.writeMicroseconds(ESC_NEUTRAL);
  esc_tr.writeMicroseconds(ESC_NEUTRAL);

  Serial.begin(115200);
  Serial2.begin(115200);

  destravar_barramento_I2C();

  Wire.setSCL(AS5600_SCL);
  Wire.setSDA(AS5600_SDA);
  Wire.begin();
  Wire.setTimeout(3); 
  delay(100);
 
  init_mpu();
  calibrate_gyro();
 
  yaw_gyro = 0;
  last_time = micros();
  last_telemetry_time = millis();

  init_L298();

  delay(1000);
}

unsigned long last_blink = 0;
bool led_state = false;
bool adjust_needed = false;

void loop() {
    unsigned long now = micros();
    float dt = (now - last_time) / 1000000.0;
    last_time = now;
   
    if(dt <= 0.000001) dt = 0.005;
    if(dt > 0.5) dt = 0.01;

    checar_serial_nao_bloqueante();

    intensidade_mult = 1.0;
    read_gyro(dt);

    // -------- CONTROLE PID --------
    pos_error = angle_diff(yaw_gyro, desired_yaw);

    static float integral_error = 0;
    static float last_error = 0;

    float Kp = 5;  
    float Ki = 1.0;  
    float Kd = 10; 
    
    //float Kp = 6;  
    //float Ki = 0.5;  
    //float Kd = 18;  

    float P = pos_error * Kp;

    /*if (abs(pos_error) > (DEADZONE / 3.0)) {
        integral_error += pos_error * dt;
    } else {
        integral_error = 0;
    }*/

    integral_error += pos_error * dt;

    if ((pos_error > 0 && last_error < 0) || (pos_error < 0 && last_error > 0)) {
        integral_error = 0;
    }

    integral_error = constrain(integral_error, -400, 400);
    float I = integral_error * Ki;

    float derivative_error = (pos_error - last_error) / dt;
    float D = derivative_error * Kd;

    last_error = pos_error;

    yaw_output = (P + I + D) * intensidade_mult;

// -------- TRATAMENTO DA ZONA MORTA --------
    if (abs(pos_error) <= DEADZONE) {
        // Chegou no alvo (dentro da tolerância): desliga os motores
        yaw_output = 0;
        integral_error = 0;          // Zera a integral para não acumular erro parado
        digitalWrite(LED_PIN, HIGH); // Apaga o LED indicando estabilidade
    } else {
        // Fora do alvo: O valor de yaw_output calculado pelo PID será mantido
        digitalWrite(LED_PIN, LOW);  // Acende o LED indicando que está em movimento
    }



    if (yaw_output > 0 && yaw_output < ESC_MIN_POWER) yaw_output = ESC_MIN_POWER;   
    if (yaw_output < 0 && yaw_output > -ESC_MIN_POWER) yaw_output = -ESC_MIN_POWER;

    // -------- ACIONAMENTO DOS MOTORES --------
    int bf = ESC_NEUTRAL;
    int as = ESC_NEUTRAL;
    int tr = ESC_NEUTRAL;

    if (yaw_output != 0) {
        bf = ESC_NEUTRAL - yaw_output;
        as = ESC_NEUTRAL + yaw_output;
    }

    bf = constrain(bf, LIMIT_PWM_MIN, LIMIT_PWM_MAX);
    as = constrain(as, LIMIT_PWM_MIN, LIMIT_PWM_MAX);

    esc_bf.writeMicroseconds(bf);
    esc_as.writeMicroseconds(as);
    esc_tr.writeMicroseconds(tr);

    // -------- ENVIO DA TELEMETRIA PARA A RASPBERRY PI --------
    if (millis() - last_telemetry_time >= TELEMETRY_INTERVAL) {
        last_telemetry_time = millis();

        String string_telemetria = String(yaw_gyro, 2) + "," +
                                   String(filtered_gyro_rate, 2) + "," +
                                   String(bf) + "," +
                                   String(as) + "," +
                                   String(pos_error, 2) + "," +
                                   String(yaw_output, 2);

        Serial.println(string_telemetria);  
        Serial2.println(string_telemetria);  
    }

    // 1. Leitura contínua dos fins de curso mecânicos do L298N
    bool bateuCima = (digitalRead(FIM_CURSO_CIMA) == HIGH);
    bool bateuBaixo = (digitalRead(FIM_CURSO_BAIXO) == HIGH);
   
    // ============================================================
    // LÓGICA DO FILTRO (DEBOUNCE) PARA O NOVO SENSOR ÓPTICO (OLHAL)
    // ============================================================
    int leituraCruaOlhal = digitalRead(SENSOR_OLHAL_D0);

    if (leituraCruaOlhal != ultimoEstadoFisicoOlhal) {
        ultimoTempoFiltro = millis();
    }

    if ((millis() - ultimoTempoFiltro) > tempoFiltro) {
        if (leituraCruaOlhal != estadoEstavelOlhal) {
            estadoEstavelOlhal = leituraCruaOlhal;

            if (estadoEstavelOlhal == LOW) {
                // Luz passando = LOW = Olhal detectado
                Serial.println("olhal chegou ao mecanismo (Luz detectada)");
                
                // Opcional: printar o valor analógico do pino A0 para checar possíveis ruídos na lente do encoder
                // int valorA0 = analogRead(SENSOR_OLHAL_A0); 
                // Serial.print("Valor analogico atual: "); 
                // Serial.println(valorA0);
            } else {
                // Luz não passando = HIGH = Sem olhal
                Serial.println("olhal removido do mecanismo (Feixe bloqueado)");
            }
        }
    }
   
    ultimoEstadoFisicoOlhal = leituraCruaOlhal;
    // ============================================================

    // 2. SEGURANÇA EM TEMPO REAL:
    if (bateuCima && estadoMotor == 1) {
        pararMotor();
        Serial.println("[PARADA] Atingiu o topo! Bloqueado para subir, liberado para descer ('s').");
    }
   
    if (bateuBaixo && estadoMotor == -1) {
        pararMotor();
        Serial.println("[PARADA] Atingiu a base! Bloqueado para descer, liberado para subir ('w').");
    }

    if (bateuCima || bateuBaixo) {
        digitalWrite(LED_PIN, LOW);
    } else {
        digitalWrite(LED_PIN, HIGH);
    }

    // 3. PROCESSAMENTO DE COMANDOS SERIAL (LEITURA ÚNICA)
    if (Serial.available() > 0) {
        char cmd = Serial.read();

        // --- COMANDOS DO MOTOR L298N ---
        if (cmd == 'w' || cmd == 'W') {
            if (!bateuCima) {
                moverSubir();
            } else {
                Serial.println("[BLOQUEADO] Fim de curso superior ativo! So pode descer ('s').");
            }
        }
        else if (cmd == 's' || cmd == 'S') {
            if (!bateuBaixo) {
                moverDescer();
            } else {
                Serial.println("[BLOQUEADO] Fim de curso inferior ativo! So pode subir ('w').");
            }
        }
        else if (cmd == ' ' || cmd == 'x' || cmd == 'X') {
            pararMotor();
            Serial.println(">> Motor L298N: PARADO");
        }

        // --- COMANDOS DO MOTOR BTS7960 ---
        else if (cmd == 'a' || cmd == 'A') {
            girarSentidoA();
        }
        else if (cmd == 'd' || cmd == 'D') {
            girarSentidoD();
        }
        else if (cmd == 'c' || cmd == 'C') {
            pararBTS();
        }

        // --- COMANDO DO ENCODER AS5600 ---
        else if (cmd == 'e' || cmd == 'E') {
            lerEncoderAS5600();
        }
    }
}

//*/
// #define BF_PIN PA0   // Frente
// #define AS_PIN PA1   // Lados
// #define TR_PIN PA6   // Traseira


/*
#include <Arduino.h> 
#include <Servo.h> 
// Definição dos Pinos
#define BF_PIN PA0
#define AS_PIN PA1
#define TR_PIN PA6
#define LED_PIN PC13 // Novo pino de sinalização visual

Servo esc_bf; 
Servo esc_as; 
Servo esc_tr; 

void setup() { 
  Serial.begin(115200); 
  
  // Configura o pino do LED como saída
  pinMode(LED_PIN, OUTPUT); 

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

  // Acende o LED e aguarda 5 segundos
  digitalWrite(LED_PIN, HIGH); 
  Serial.println("Aguardando 5 segundos (LED ACESO)...");
  delay(5000); 

  // 2. PASSO: Sinaliza a transição piscando o LED por 2 segundos
  Serial.println("Sinalizando transição: Piscando LED por 2 segundos...");
  // Um loop de 4 iterações com 500ms totais cada (250ms apagado + 250ms aceso) = 2 segundos totais
  for(int i = 0; i < 4; i++) {
    digitalWrite(LED_PIN, LOW);
    delay(250);
    digitalWrite(LED_PIN, HIGH);
    delay(250);
  }

  // 3. PASSO: Baixa para o MÍNIMO (1000us) e apaga o LED
  Serial.println("Enviando sinal MIN (1000us)... Aguarde os bips de confirmacao longos!");
  
  // Apaga o LED em definitivo sinalizando que o sinal baixou
  digitalWrite(LED_PIN, LOW); 
  
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
*/
