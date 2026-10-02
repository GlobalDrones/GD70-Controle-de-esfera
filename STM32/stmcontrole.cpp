#include <Arduino.h>
#include <Wire.h>
#include <Servo.h>
#include <AS5600.h> // Biblioteca dedicada do AS5600

#define DEADZONE 3
#define ESC_MIN_POWER 80

// --- LIMITES ABSOLUTOS DE PWM DOS MOTORES ---
#define LIMIT_PWM_MAX 1750  
#define LIMIT_PWM_MIN 1250  

// --- PINOS MAPEADOS PARA A STM32F401/F411 (BLACK PILL) ---
#define BF_PIN PA0   // Frente
#define AS_PIN PA1   // Lados
#define TR_PIN PA6   // Traseira

// ============================================================
// --- PINOS DA PONTE H1 - Polia BTS7960 ---
#define POLIA_EN_R_PIN  PB7
#define POLIA_EN_L_PIN PB10
#define POLIA_RPWM_PIN  PB6  
#define POLIA_LPWM_PIN  PB5

// ============================================================
// --- PONTE H2 - Pinça BTS7960 ---
// ============================================================
#define PINCA_EN_R_PIN   PB12
#define PINCA_EN_L_PIN   PB13
#define PINCA_RPWM_PIN   PA7
#define PINCA_LPWM_PIN   PB0

// Fins de Curso
#define FIM_CURSO_CIMA  PB14
#define FIM_CURSO_BAIXO PB15

// Novos pinos para o Sensor Óptico (Encoder / LM393)
#define SENSOR_OLHAL_D0 PA5 // Conectado ao pino D0 do sensor
#define SENSOR_OLHAL_A0 PA6 // Conectado ao pino A0 do sensor

// Pinos I2C para o Encoder AS5600 (STM32 - Hardware I2C1)
#define AS5600_SDA PB9
#define AS5600_SCL PB8

// LED Onboard (Black Pill - Active LOW)
#define LED_PIN PC13

#define ESC_NEUTRAL 1500
#define MPU6050_ADDR 0x68
#define SAMPLES_GYRO 500

// Instanciando o objeto do Encoder
AS5600 encoder;

// Estado atual do movimento: 1 = Subindo, -1 = Descendo, 0 = Parado
int estadoMotor = 0;
bool bateuCima = false;
bool bateuBaixo = false;

// --- VARIÁVEIS PARA O FILTRO (DEBOUNCE) DO OLHAL ---
int estadoEstavelOlhal = HIGH;    
int ultimoEstadoFisicoOlhal = HIGH;
unsigned long ultimoTempoFiltro = 0;
unsigned long tempoFiltro = 100;

// --- FUNÇÃO DO ENCODER AS5600 ---
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

// --- FUNÇÕES DE CONTROLE DA PINÇA ---
void pararMotor() {    
    digitalWrite(PINCA_RPWM_PIN, LOW);
    digitalWrite(PINCA_LPWM_PIN, LOW);
    estadoMotor = 0;
    Serial.println(">> PINCA: Motor PARADO");
}

void moverSubir() {    
    digitalWrite(PINCA_EN_R_PIN, HIGH);
    digitalWrite(PINCA_EN_L_PIN, HIGH);
    digitalWrite(PINCA_RPWM_PIN, HIGH);    
    digitalWrite(PINCA_LPWM_PIN, LOW);
    estadoMotor = 1;
    Serial.println(">> PINCA: SUBINDO");
}

void moverDescer() {    
    digitalWrite(PINCA_EN_R_PIN, HIGH);
    digitalWrite(PINCA_EN_L_PIN, HIGH);
    digitalWrite(PINCA_RPWM_PIN, LOW);    
    digitalWrite(PINCA_LPWM_PIN, HIGH);
    estadoMotor = -1;
    Serial.println(">> PINCA: DESCENDO");
}

// --- FUNÇÕES DE CONTROLE DA POLIA (BTS7960) ---
void desativarDriver() {
    digitalWrite(POLIA_EN_R_PIN, LOW);
    digitalWrite(POLIA_EN_L_PIN, LOW);
}

void ativarDriver() {
    digitalWrite(POLIA_EN_R_PIN, HIGH);
    digitalWrite(POLIA_EN_L_PIN, HIGH);
}

void pararBTS() {
    digitalWrite(POLIA_RPWM_PIN, LOW);
    digitalWrite(POLIA_LPWM_PIN, LOW);
    digitalWrite(LED_PIN, HIGH);
    Serial.println(">> POLIA: Motor PARADO");
}

void girarSentidoA() {
    ativarDriver();
    digitalWrite(POLIA_RPWM_PIN, HIGH);
    digitalWrite(POLIA_LPWM_PIN, LOW);
    digitalWrite(LED_PIN, LOW);
    Serial.println(">> POLIA: Girando SENTIDO A (Horario)");
}

void girarSentidoD() {
    ativarDriver();
    digitalWrite(POLIA_RPWM_PIN, LOW);
    digitalWrite(POLIA_LPWM_PIN, HIGH);
    digitalWrite(LED_PIN, LOW);
    Serial.println(">> POLIA: Girando SENTIDO D (Anti-horario)");
}

// --- INICIALIZAÇÃO DO MPU6050 E CONTROLE ---
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
    digitalWrite(LED_PIN, HIGH); delay(200);
    digitalWrite(LED_PIN, LOW); delay(200);
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

// --- PROCESSAMENTO UNIFICADO DA SERIAL (USB E SERIAL2) COM ECO ---
String inputBufferUSB = "";

void processarComando(String cmdStr) {
    cmdStr.trim();
    if (cmdStr.length() == 0) return;

    // ECO DE CONFIRMAÇÃO PARA A RASPBERRY
    Serial.print("[STM_RX]: ");
    Serial.println(cmdStr);
    Serial2.print("[STM_RX]: ");
    Serial2.println(cmdStr);

    bateuCima = (digitalRead(FIM_CURSO_CIMA) == HIGH);
    bateuBaixo = (digitalRead(FIM_CURSO_BAIXO) == HIGH);

    // 1. Comando de letra única (Pinça, Polia, AS5600)
    if (cmdStr.length() == 1 && !isDigit(cmdStr.charAt(0)) && cmdStr.charAt(0) != '-') {
        char cmd = cmdStr.charAt(0);
        if (cmd == 'w' || cmd == 'W') {
            if (!bateuCima) moverSubir();
            else { pararMotor(); Serial.println("[BLOQUEADO] Topo atingido!"); }
        }
        else if (cmd == 's' || cmd == 'S') {
            if (!bateuBaixo) moverDescer();
            else { pararMotor(); Serial.println("[BLOQUEADO] Base atingida!"); }
        }
        else if (cmd == ' ' || cmd == 'x' || cmd == 'X') {
            pararMotor();
        }
        else if (cmd == 'a' || cmd == 'A') {
            girarSentidoA();
        }
        else if (cmd == 'd' || cmd == 'D') {
            girarSentidoD();
        }
        else if (cmd == 'c' || cmd == 'C') {
            pararBTS();
        }
        else if (cmd == 'e' || cmd == 'E') {
            lerEncoderAS5600();
        }
    }
    // 2. Comando Numérico (Ângulo da Câmera vindo da Rasp)
    else {
        float camera_angle = cmdStr.toFloat();
        yaw_gyro = 0;
        desired_yaw = camera_angle;

        String confirmacao = ">> NOVO ANGULO APLICADO: " + String(desired_yaw, 2);
        Serial.println(confirmacao);
        Serial2.println(confirmacao);
    }
}

void checar_serial_nao_bloqueante() {
    // Escuta USB (Serial)
    while (Serial.available() > 0) {
        char c = Serial.read();
        if (c == '\n' || c == '\r') {
            processarComando(inputBufferUSB);
            inputBufferUSB = "";
        } else {
            inputBufferUSB += c;
            if (inputBufferUSB.length() > 50) inputBufferUSB = "";
        }
    }

    // Escuta Pinos PA2/PA3 (Serial2)
    while (Serial2.available() > 0) {
        char c = Serial2.read();
        if (c == '\n' || c == '\r') {
            processarComando(inputBufferUSB);
            inputBufferUSB = "";
        } else {
            inputBufferUSB += c;
            if (inputBufferUSB.length() > 50) inputBufferUSB = "";
        }
    }
}

void init_Pinca() {
    pinMode(PINCA_EN_R_PIN, OUTPUT);
    pinMode(PINCA_EN_L_PIN, OUTPUT);
    pinMode(PINCA_RPWM_PIN, OUTPUT);
    pinMode(PINCA_LPWM_PIN, OUTPUT);
    pararMotor();

    pinMode(POLIA_EN_R_PIN, OUTPUT);
    pinMode(POLIA_EN_L_PIN, OUTPUT);
    pinMode(POLIA_RPWM_PIN, OUTPUT);
    pinMode(POLIA_LPWM_PIN, OUTPUT);
    ativarDriver();
    pararBTS();

    pinMode(FIM_CURSO_CIMA, INPUT_PULLDOWN);
    pinMode(FIM_CURSO_BAIXO, INPUT_PULLDOWN);

    pinMode(SENSOR_OLHAL_D0, INPUT);
    pinMode(SENSOR_OLHAL_A0, INPUT);

    Wire.setSDA(AS5600_SDA);
    Wire.setSCL(AS5600_SCL);
    Wire.begin();
    encoder.begin();

    Serial.println("==============================================");
    Serial.println(" SISTEMA DUPLO BTS7960 + AS5600 INICIADO      ");
    Serial.println("  PINÇA:   'w'=Subir | 's'=Descer | 'x'=Parar ");
    Serial.println("  POLIA:   'a'=Giro A| 'd'=Giro D | 'c'=Parar ");
    Serial.println("  AS5600:  'e'=Ler Angulo Atual               ");
    Serial.println("==============================================");
}

void setup() {
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);

  esc_bf.attach(BF_PIN);
  esc_as.attach(AS_PIN);
  esc_tr.attach(TR_PIN);
  esc_bf.writeMicroseconds(ESC_NEUTRAL);
  esc_as.writeMicroseconds(ESC_NEUTRAL);
  esc_tr.writeMicroseconds(ESC_NEUTRAL);

  Serial.begin(115200);
  Serial2.begin(115200);
  while (!Serial && millis() < 3000); // Aguarda conexão USB

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

  init_Pinca();
  delay(1000);
}

void loop() {
    unsigned long now = micros();
    float dt = (now - last_time) / 1000000.0;
    last_time = now;

    if(dt <= 0.000001) dt = 0.005;
    if(dt > 0.5) dt = 0.01;

    // Atualiza status físico dos fins de curso
    bateuCima = (digitalRead(FIM_CURSO_CIMA) == HIGH);
    bateuBaixo = (digitalRead(FIM_CURSO_BAIXO) == HIGH);

    checar_serial_nao_bloqueante();
    intensidade_mult = 1.0;
    read_gyro(dt);

    // -------- CONTROLE PID --------
    pos_error = angle_diff(yaw_gyro, desired_yaw);
    static float integral_error = 0;
    static float last_error = 0;

    float Kp = 5.0;  
    float Ki = 1.0;  
    float Kd = 10.0;
    float P = pos_error * Kp;

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
        yaw_output = 0;
        integral_error = 0;
        digitalWrite(LED_PIN, HIGH);
    } else {
        digitalWrite(LED_PIN, LOW);
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

        String string_telemetria = "YAW_GYRO: " + String(yaw_gyro, 2) + "," +
                                   String(filtered_gyro_rate, 2) + "," +
                                   String(bf) + "," +
                                   String(as) + ", POS_ERROR: " +
                                   String(pos_error, 2) + "," +
                                   String(yaw_output, 2) + "," +
                                   String(bateuCima ? "1" : "0") + "," +
                                   String(bateuBaixo ? "1" : "0");

        Serial.println(string_telemetria);  
        Serial2.println(string_telemetria);  
    }

    // --- SENSOR ÓPTICO (OLHAL) ---
    int leituraCruaOlhal = digitalRead(SENSOR_OLHAL_D0);
    if (leituraCruaOlhal != ultimoEstadoFisicoOlhal) {
        ultimoTempoFiltro = millis();
    }
    if ((millis() - ultimoTempoFiltro) > tempoFiltro) {
        if (leituraCruaOlhal != estadoEstavelOlhal) {
            estadoEstavelOlhal = leituraCruaOlhal;
            if (estadoEstavelOlhal == LOW) {
                Serial.println("olhal chegou ao mecanismo (Luz detectada)");
            } else {
                Serial.println("olhal removido do mecanismo (Feixe bloqueado)");
            }
        }
    }
    ultimoEstadoFisicoOlhal = leituraCruaOlhal;

    // --- SEGURANÇA EM TEMPO REAL ---
    if (bateuCima && estadoMotor == 1) {
        pararMotor();
        Serial.println("[PARADA] Atingiu o topo! So pode descer ('s').");
    }
    if (bateuBaixo && estadoMotor == -1) {
        pararMotor();
        Serial.println("[PARADA] Atingiu a base! So pode subir ('w').");
    }
}
