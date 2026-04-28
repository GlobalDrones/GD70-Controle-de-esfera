#include <Arduino.h>
#include <Wire.h>
#include <Servo.h>
#include <U8glib.h>
 
#define DEADZONE 0
#define ESC_MIN_POWER 80
 
// --- LIMITES ABSOLUTOS DE PWM DOS MOTORES ---
// Padrão máximo é 2000 (frente) e 1000 (ré).
// Reduza o MAX e aumente o MIN para deixar o robô mais lento.
#define LIMIT_PWM_MAX 1600  //
#define LIMIT_PWM_MIN 1400  //
 
#define BF_PIN 6
#define AS_PIN 5
#define TR_PIN 4
 
#define LED_PIN 12
 
// O ENCODER_PIN (9) não será mais usado para o ângulo,
// pois agora virá via TX/RX (Serial), mas mantivemos a definição caso precise depois.
#define ENCODER_PIN 9
#define INTENSITY_PIN 10
 
#define ESC_NEUTRAL 1500
#define MPU6050_ADDR 0x68
 
#define SAMPLES_GYRO 500
 
// Inicialização do OLED usando I2C rápido para minimizar atrasos no PID
U8GLIB_SSD1306_128X64 u8g(U8G_I2C_OPT_FAST);
 
Servo esc_bf;
Servo esc_as;
Servo esc_tr;
 
int16_t gyro_z_raw;
float gyro_rate_z = 0;
float gyro_offset_z = 0;
float filtered_gyro_rate = 0;
 
float alpha = 0.2;
 
float yaw_gyro = 0;
float desired_yaw = 0;
float intensidade_mult = 1.0;
 
// Variáveis que o OLED precisa ler, movidas para global
float pos_error = 0;
float yaw_output = 0;
 
unsigned long last_time;
unsigned long last_display_update = 0; // Timer do OLED
 
float angle_diff(float a, float b) {
  float d = a - b;
  while (d > 180.0) d -= 360.0;
  while (d < -180.0) d += 360.0;
  return d;
}
 
int apply_esc_output(float yaw_output) {
    int esc_value = ESC_NEUTRAL;
    if (yaw_output > 0) {
        esc_value += yaw_output;
    } else if (yaw_output < 0) {
        esc_value += yaw_output;
    }
    return constrain(esc_value, 1100, 1900);
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
  Serial.println(">>> NAO MEXA NA PLACA! <<<");
 
  for(int j = 0; j < 20; j++) {
    digitalWrite(LED_PIN, HIGH);
    delay(100);
    digitalWrite(LED_PIN, LOW);
    delay(100);
  }
 
  long sum_z = 0;
  bool calib_led_state = false;
 
  for (int i = 0; i < SAMPLES_GYRO; i++) {
    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(0x43);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU6050_ADDR, 6);
 
    Wire.read(); Wire.read();
    Wire.read(); Wire.read();
   
    int16_t raw_z = (int16_t)(Wire.read() << 8 | Wire.read());
    sum_z += raw_z;
 
    if (i % 25 == 0) {
      calib_led_state = !calib_led_state;
      digitalWrite(LED_PIN, calib_led_state ? HIGH : LOW);
    }
 
    delay(3);
  }
 
  digitalWrite(LED_PIN, LOW);
 
  gyro_offset_z = (sum_z / (float)SAMPLES_GYRO) / 131.0;
  Serial.print("Offset Z calculado: ");
  Serial.println(gyro_offset_z);
}
 
void read_gyro(float dt) {
    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(0x43);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU6050_ADDR, 6);
 
    Wire.read(); Wire.read(); // Pula X
    Wire.read(); Wire.read(); // Pula Y
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
 
void draw() {
  u8g.setFont(u8g_font_7x13);
  char buf[20];
 
  sprintf(buf, "Alvo: %d", (int)desired_yaw);
  u8g.drawStr(0, 15, buf);
 
  sprintf(buf, "Gyro: %d", (int)yaw_gyro);
  u8g.drawStr(64, 15, buf);
 
  sprintf(buf, "Erro: %d", (int)pos_error);
  u8g.drawStr(0, 35, buf);
 
  sprintf(buf, "Out:  %d", (int)yaw_output);
  u8g.drawStr(64, 35, buf);
 
  sprintf(buf, "Pwr:  %d%%", (int)(intensidade_mult * 100));
  u8g.drawStr(0, 55, buf);
}
 
void setup() {
  pinMode(LED_PIN, OUTPUT);
  pinMode(ENCODER_PIN, INPUT);
  pinMode(INTENSITY_PIN, INPUT);
 
  esc_bf.attach(BF_PIN);
  esc_as.attach(AS_PIN);
  esc_tr.attach(TR_PIN);
 
  esc_bf.writeMicroseconds(ESC_NEUTRAL);
  esc_as.writeMicroseconds(ESC_NEUTRAL);
  esc_tr.writeMicroseconds(ESC_NEUTRAL);
 
  Wire.begin();
  Wire.setWireTimeout(3000, true); // Espera no máx 3ms. Se travar, o 'true' reseta o barramento e salva o loop!
  init_mpu();
 
  Serial.begin(115200);
  Serial.setTimeout(5);
 
  calibrate_gyro();
  yaw_gyro = 0;
 
  u8g.setColorIndex(1);
 
  last_time = micros();
  delay(3000);
}
 
unsigned long last_blink = 0;
bool led_state = false;
bool adjust_needed = false;
 
void loop() {
    unsigned long now = micros();
    float dt = (now - last_time) / 1000000.0;
    last_time = now;
    if(dt > 0.5) dt=0.01;
 
   if (Serial.available() > 0) {
       String input = Serial.readStringUntil('\n');
       input.trim();
       
       if (input.length() > 0) {
           float camera_angle = input.toFloat();
           yaw_gyro = 0;
           desired_yaw = camera_angle;
       }
    }
 
    // -------- PWM INTENSIDADE (Sinal do Rádio) --------
    int pwm_intensidade = pulseIn(INTENSITY_PIN, HIGH, 25000);
    if (pwm_intensidade > 900 && pwm_intensidade < 2100) {
        // NOTA: Se quiser limitar a velocidade máxima PELO RÁDIO, altere o "100" final.
        // Exemplo: map(pwm_intensidade, 1000, 2000, 0, 50) faria o rádio ir até no máximo 50%
        int map_int = map(pwm_intensidade, 1000, 2000, 0, 100);
        intensidade_mult = constrain(map_int, 0, 100) / 100.0;
    }
 
    // -------- CONTROLE PID --------
    read_gyro(dt);
 
    pos_error = angle_diff(yaw_gyro, desired_yaw);
 
    static float integral_error = 0;
    static float last_error = 0;
 
    float Kp = 8.6244*0.85;  
    float Ki = 0.541;  
    float Kd = 14.0426*1.25;  
 
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
 
    // --- TRAVA DE SINAL PWM ABSOLUTA ---
    // Nenhuma variável ou PID pode ultrapassar esses valores. É uma parede física no código.
    bf = constrain(bf, LIMIT_PWM_MIN, LIMIT_PWM_MAX);
    as = constrain(as, LIMIT_PWM_MIN, LIMIT_PWM_MAX);
 
    esc_bf.writeMicroseconds(bf);
    esc_as.writeMicroseconds(as);
    esc_tr.writeMicroseconds(tr);
 
    // Atualização OLED
    unsigned long currentMillis = millis();
    if (currentMillis - last_display_update >= 250) {
      last_display_update = currentMillis;
      u8g.firstPage();
      do {
        draw();
      } while (u8g.nextPage());
    }
 
    Serial.print("Alvo:");
    Serial.print(desired_yaw);
    Serial.print("\t Gyro:");
    Serial.print(yaw_gyro);
    Serial.print("\t Erro:");
    Serial.print(pos_error);
    Serial.print("\t Out:");
    Serial.println(yaw_output);
}
 
