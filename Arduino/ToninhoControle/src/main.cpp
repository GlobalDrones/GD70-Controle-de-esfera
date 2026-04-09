#include <Arduino.h> 
#include <Wire.h> 
#include <Servo.h> 
#include <U8glib.h> // Adicionado U8glib

#define DEADZONE 0
#define ESC_MIN_POWER 80

#define BF_PIN 6
#define AS_PIN 5
#define TR_PIN 4

#define LED_PIN 12 

#define ENCODER_PIN 9 
#define INTENSITY_PIN 10 

#define ESC_NEUTRAL 1500 
#define MPU6050_ADDR 0x68 

// Inicialização do OLED usando I2C rápido para minimizar atrasos no PID
U8GLIB_SSD1306_128X64 u8g(U8G_I2C_OPT_FAST);

Servo esc_bf; 
Servo esc_as; 
Servo esc_tr; 

int16_t gyro_z_raw; 
float gyro_rate_z = 0; 
float gyro_offset_z = 0; 

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
  
  // Pisca o LED durante os 2 segundos de estabilização inicial (Aviso para tirar a mão)
  for(int j = 0; j < 20; j++) {
    digitalWrite(LED_PIN, HIGH);
    delay(100);
    digitalWrite(LED_PIN, LOW);
    delay(100);
  }

  long sum_z = 0;
  bool calib_led_state = false;
  
  for (int i = 0; i < 500; i++) {
    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(0x43);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU6050_ADDR, 6);

    Wire.read(); Wire.read(); 
    Wire.read(); Wire.read(); 
    
    // Cast (int16_t) para corrigir erro matemático de drift
    int16_t raw_z = (int16_t)(Wire.read() << 8 | Wire.read()); 
    sum_z += raw_z; 

    // Pisca o LED bem rápido enquanto faz as 500 leituras do sensor
    if (i % 25 == 0) { // Alterna estado a cada ~75ms
      calib_led_state = !calib_led_state;
      digitalWrite(LED_PIN, calib_led_state ? HIGH : LOW);
    }

    delay(3);
  }
  
  // Apaga o LED ao finalizar a calibração
  digitalWrite(LED_PIN, LOW); 

  gyro_offset_z = (sum_z / 500.0) / 131.0;
  Serial.print("Offset Z calculado: ");
  Serial.println(gyro_offset_z);
}

void read_gyro(float dt) { 
  Wire.beginTransmission(MPU6050_ADDR); 
  Wire.write(0x43); 
  Wire.endTransmission(false); 
  Wire.requestFrom(MPU6050_ADDR, 6); 

  Wire.read(); Wire.read(); 
  Wire.read(); Wire.read(); 
  
  // Cast (int16_t) para corrigir erro matemático de drift
  gyro_z_raw = (int16_t)(Wire.read() << 8 | Wire.read()); 

  gyro_rate_z = (gyro_z_raw / 131.0) - gyro_offset_z; 

  // Filtro Anti-Drift (Zona Morta do Sensor): ignora ruído abaixo de 1 grau/segundo
  if (abs(gyro_rate_z) < 1.0) {
      gyro_rate_z = 0.0;
  }

  yaw_gyro += gyro_rate_z * dt; 

  if (yaw_gyro >= 360.0) yaw_gyro -= 360.0; 
  if (yaw_gyro < 0.0) yaw_gyro += 360.0; 
} 

// Função de desenho do OLED
void draw() {
  u8g.setFont(u8g_font_7x13);
  char buf[20];

  // Cast para int usado para display limpo e processamento rápido
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
  init_mpu(); 
  
  Serial.begin(115200); 
  
  calibrate_gyro(); // Agora possui o pisca-pisca de aviso integrado
  yaw_gyro = 0; 

  // Ajuste de cor do U8g se necessário (padrão é branco)
  u8g.setColorIndex(1);

  last_time = micros(); 
  delay(3000); 
} 

// Variável global para controle do pisca-pisca do LED
unsigned long last_blink = 0;
bool led_state = false;
bool adjust_needed = false;

void loop() { 
    unsigned long now = micros();
    float dt = (now - last_time) / 1000000.0;
    last_time = now;
    if(dt > 0.5) dt=0.01;

    // -------- PWM -------- 
    int pwm_angulo = pulseIn(ENCODER_PIN, HIGH, 25000); 
    int pwm_intensidade = pulseIn(INTENSITY_PIN, HIGH, 25000); 

    if (pwm_angulo > 900 && pwm_angulo < 2100) { 
        desired_yaw = map(pwm_angulo, 1000, 2000, 0, 359); 
    }

    if (pwm_intensidade > 900 && pwm_intensidade < 2100) { 
        int map_int = map(pwm_intensidade, 1000, 2000, 0, 100); 
        intensidade_mult = constrain(map_int, 0, 100) / 100.0; 
    }

    // -------- CONTROLE -------- 
    read_gyro(dt); 

    pos_error = angle_diff(yaw_gyro, desired_yaw); 

    static float integral_error = 0;
    static float last_error = 0;

    float Kp = 4.7*0.8;  
    float Ki = 2.0*1.3;  
    float Kd = 6.0*0.6;  

    float P = pos_error * Kp;

    if (abs(pos_error) > (DEADZONE / 3.0)) {
        integral_error += pos_error * dt;
    } else {
        integral_error = 0; // Alvo atingido perfeitamente, limpa a memória
    }

    integral_error = constrain(integral_error, -400, 400); 
    float I = integral_error * Ki;

    float derivative_error = (pos_error - last_error) / dt;
    float D = derivative_error * Kd;

    last_error = pos_error;

    yaw_output = (P + I + D) * intensidade_mult;

    // --- Zona morta com ajuste fino ---
    if (abs(pos_error) <= DEADZONE) {
        if (abs(pos_error) <= (DEADZONE / 3.0)) {
            // Centro da deadzone: para
            yaw_output = 0;
            integral_error = 0;
            adjust_needed = false; // Reseta flag de ajuste fino
            digitalWrite(LED_PIN, HIGH);  // LED aceso constante
        } else if (adjust_needed) { //só entra em ajuste fino se já tiver saido da zona
            // Ajuste fino: pisca LED
            yaw_output = (pos_error > 0 ? 1 : -1) * ESC_MIN_POWER / 2;

            // Pisca LED a cada 300 ms
            if (millis() - last_blink >= 300) {
                led_state = !led_state;
                digitalWrite(LED_PIN, led_state ? HIGH : LOW);
                last_blink = millis();
            }
        }
    } else {
        // Fora da deadzone: PID normal
        adjust_needed = true;
        digitalWrite(LED_PIN, LOW); 
    }

    // --- Limite mínimo de potência
    if (yaw_output > 0 && yaw_output < ESC_MIN_POWER) yaw_output = ESC_MIN_POWER;
    if (yaw_output < 0 && yaw_output > -ESC_MIN_POWER) yaw_output = -ESC_MIN_POWER;
    yaw_output = constrain(yaw_output, -400, 400); 

    int bf = ESC_NEUTRAL; 
    int as = ESC_NEUTRAL; 
    int tr = ESC_NEUTRAL; 

    if (yaw_output != 0) {
        bf = ESC_NEUTRAL + yaw_output; 
        as = ESC_NEUTRAL + yaw_output; 
    }

    bf = constrain(bf, 1100, 1900); 
    as = constrain(as, 1100, 1900); 

    esc_bf.writeMicroseconds(bf); 
    esc_as.writeMicroseconds(as); 
    esc_tr.writeMicroseconds(tr); 

    // Atualização Não-Bloqueante do OLED (a cada 250ms)
    unsigned long currentMillis = millis();
    if (currentMillis - last_display_update >= 250) {
      last_display_update = currentMillis;
      u8g.firstPage();
      do {
        draw();
      } while (u8g.nextPage());
    }

    // Serial mantido para debug, mas pode comentar no futuro para poupar ainda mais CPU
    Serial.print("Alvo:");
    Serial.print(desired_yaw);
    Serial.print("\t Gyro:");
    Serial.print(yaw_gyro);
    Serial.print("\t Erro:");
    Serial.print(pos_error);
    Serial.print("\t Out:");
    Serial.println(yaw_output); 
}