#include <Arduino.h>
#include <Wire.h>
#include <Servo.h>

#define DEADZONE 0
#define ESC_MIN_POWER 80

// --- LIMITES ABSOLUTOS DE PWM DOS MOTORES ---
#define LIMIT_PWM_MAX 1650  
#define LIMIT_PWM_MIN 1350  

// --- PINOS MAPEADOS PARA A STM32F401 (BLACK PILL) ---
#define BF_PIN PA0   // Frente
#define AS_PIN PA1   // Lados
#define TR_PIN PA6   // Traseira

#define LED_PIN PC13 // LED embutido da STM32

// --- PINOS DE LEITURA DO RÁDIO SIYI ---
#define CH1_PIN PB12
#define CH2_PIN PB13
#define CH3_PIN PB14
#define CH4_PIN PB15
#define CH5_PIN PA8

#define ESC_NEUTRAL 1500
#define MPU6050_ADDR 0x68
#define SAMPLES_GYRO 500

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

// --- VARIÁVEIS VOLÁTEIS PARA AS INTERRUPÇÕES DO RÁDIO ---
volatile int pwm_ch1 = 1500;
volatile int pwm_ch2 = 1500;
volatile int pwm_ch3 = 1500;
volatile int pwm_ch4 = 1500;
volatile int pwm_ch5 = 1500;

volatile unsigned long timer_ch1, timer_ch2, timer_ch3, timer_ch4, timer_ch5;

// --- FUNÇÕES DE INTERRUPÇÃO DO RÁDIO ---
void calc_ch1() { if(digitalRead(CH1_PIN)) timer_ch1 = micros(); else pwm_ch1 = micros() - timer_ch1; }
void calc_ch2() { if(digitalRead(CH2_PIN)) timer_ch2 = micros(); else pwm_ch2 = micros() - timer_ch2; }
void calc_ch3() { if(digitalRead(CH3_PIN)) timer_ch3 = micros(); else pwm_ch3 = micros() - timer_ch3; }
void calc_ch4() { if(digitalRead(CH4_PIN)) timer_ch4 = micros(); else pwm_ch4 = micros() - timer_ch4; }
void calc_ch5() { if(digitalRead(CH5_PIN)) timer_ch5 = micros(); else pwm_ch5 = micros() - timer_ch5; }

float angle_diff(float a, float b) {
  float d = a - b;
  while (d > 180.0) d -= 360.0;
  while (d < -180.0) d += 360.0;
  return d;
}

// --- FUNÇÃO DE DESTRAVAMENTO FÍSICO DO BARRAMENTO I2C ---
void destravar_barramento_I2C() {
  pinMode(PB8, OUTPUT);       // SCL como saída
  pinMode(PB9, INPUT_PULLUP); // SDA como entrada com resistor interno

  // Envia 9 pulsos de clock manuais para forçar o MPU a liberar a linha SDA se estiver travada
  for (int i = 0; i < 9; i++) {
    digitalWrite(PB8, LOW);
    delayMicroseconds(5);
    digitalWrite(PB8, HIGH);
    delayMicroseconds(5);
  }

  // Força uma condição de STOP manual na linha
  pinMode(PB9, OUTPUT);
  digitalWrite(PB9, LOW);
  delayMicroseconds(5);
  digitalWrite(PB8, HIGH);
  delayMicroseconds(5);
  digitalWrite(PB9, HIGH);
}

void init_mpu() {
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(0x6B);
  Wire.write(0x00); // Acorda o MPU6050
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
    if (Wire.endTransmission(false) != 0) { i--; delay(3); continue; } // Ignora falha de pacote no boot

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
    if (Wire.endTransmission(false) != 0) return; // Sai sem travar o loop se houver erro elétrico

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

// --- LEITURA NÃO-BLOQUEANTE DA SERIAL2 ---
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

void setup() {
  pinMode(LED_PIN, OUTPUT);

  esc_bf.attach(BF_PIN);
  esc_as.attach(AS_PIN);
  esc_tr.attach(TR_PIN);

  esc_bf.writeMicroseconds(ESC_NEUTRAL);
  esc_as.writeMicroseconds(ESC_NEUTRAL);
  esc_tr.writeMicroseconds(ESC_NEUTRAL);

  pinMode(CH1_PIN, INPUT); pinMode(CH2_PIN, INPUT);
  pinMode(CH3_PIN, INPUT); pinMode(CH4_PIN, INPUT);
  pinMode(CH5_PIN, INPUT);

  attachInterrupt(digitalPinToInterrupt(CH1_PIN), calc_ch1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(CH2_PIN), calc_ch2, CHANGE);
  attachInterrupt(digitalPinToInterrupt(CH3_PIN), calc_ch3, CHANGE);
  attachInterrupt(digitalPinToInterrupt(CH4_PIN), calc_ch4, CHANGE);
  attachInterrupt(digitalPinToInterrupt(CH5_PIN), calc_ch5, CHANGE);

  Serial.begin(115200);
  Serial2.begin(115200);

  // 1. Limpa o curto elétrico do I2C antes de inicializar o periférico Wire
  destravar_barramento_I2C();

  // 2. Inicializa o hardware I2C
  Wire.setSCL(PB8);
  Wire.setSDA(PB9);
  Wire.begin();
  Wire.setTimeout(3); // Destrava em 3ms caso ocorram erros
  delay(100);
  
  // 3. Configura e calibra o sensor
  init_mpu();
  calibrate_gyro(); 
  
  yaw_gyro = 0;
  last_time = micros();
  last_telemetry_time = millis();
  delay(1000);
}

unsigned long last_blink = 0;
bool led_state = false;
bool adjust_needed = false;

void loop() {
    unsigned long now = micros();
    float dt = (now - last_time) / 1000000.0;
    last_time = now;
    
    // Evita crash matemático por divisão por zero
    if(dt <= 0.000001) dt = 0.005;
    if(dt > 0.5) dt = 0.01;

    checar_serial_nao_bloqueante();

    intensidade_mult = 1.0; 

    // Lê o MPU de forma não bloqueante
    read_gyro(dt);

    // -------- CONTROLE PID --------
    pos_error = angle_diff(yaw_gyro, desired_yaw);

    static float integral_error = 0;
    static float last_error = 0;

    float Kp = 8.6244 * 0.85;  
    float Ki = 0.541;  
    float Kd = 14.0426 * 1.25;  

    float P = pos_error * Kp;

    if (abs(pos_error) > (DEADZONE / 3.0)) {
        integral_error += pos_error * dt;
    } else {
        integral_error = 0; 
    }

    if ((pos_error > 0 && last_error < 0) || (pos_error < 0 && last_error > 0)) {
        integral_error = 0;
    }

    integral_error = constrain(integral_error, -400, 400);
    float I = integral_error * Ki;

    float derivative_error = (pos_error - last_error) / dt;
    float D = derivative_error * Kd;

    last_error = pos_error;

    yaw_output = (P + I + D) * intensidade_mult;

    if (abs(pos_error) <= DEADZONE) {
        if (abs(pos_error) <= (DEADZONE / 3.0)) {
            yaw_output = 0;
            integral_error = 0;
            adjust_needed = false; 
            digitalWrite(LED_PIN, HIGH);
        } else if (adjust_needed) { 
            yaw_output = (pos_error > 0 ? 1 : -1) * ESC_MIN_POWER / 2;

            if (millis() - last_blink >= 300) {
                led_state = !led_state;
                digitalWrite(LED_PIN, led_state ? HIGH : LOW);
                last_blink = millis();
            }
        }
    } else {
        adjust_needed = true;
        digitalWrite(LED_PIN, LOW);
    }

    if (yaw_output > 0 && yaw_output < ESC_MIN_POWER) yaw_output = ESC_MIN_POWER;
    if (yaw_output < 0 && yaw_output > -ESC_MIN_POWER) yaw_output = -ESC_MIN_POWER;

    // -------- ACIONAMENTO DOS MOTORES --------
    int bf = ESC_NEUTRAL;
    int as = ESC_NEUTRAL;
    int tr = ESC_NEUTRAL;

    if (yaw_output != 0) {
        bf = ESC_NEUTRAL + yaw_output;
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
}
