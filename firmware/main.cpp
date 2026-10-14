// =========================================================================================
//  PROJETO ESFERA SUSPENSA — FIRMWARE STM32F4 (BLACK PILL) — VERSAO CONSOLIDADA
//
//  Une a robustez comprovada do "mainYAWBOM.cpp" (malha de Yaw estavel, filtragem de
//  ruido, zona morta de repouso no prumo, motor TR invertido de fabrica e watchdog da polia)
//  com a arquitetura e protocolo do Digital Twin em Python ("main.cpp"):
//
//    1. FILTRAGEM DE RUIDO REFORCADA:
//       - Filtro passa-baixas de hardware (DLPF = 20 Hz, reg 0x1A) ativado no MPU6050:
//         elimina vibracoes estruturais de alta frequencia das helices na fonte.
//       - Filtro IIR passa-baixas (~9 Hz, 0.78/0.22) nas taxas de inclinacao (wx_est, wy_est).
//       - Gate gravitacional no acelerometro (0.8g a 1.2g) + calibracao com gate de quietude
//         + auto-rebias termico do giroscopio em repouso.
//
//    2. MALHA DE YAW ESTAVEL (BASEADA NO YAWBOM + SUPORTE A VISAO):
//       - Correcao do bug de unidades (wrap180 em graus em vez de wrap_pi).
//       - Controle PID direto de posicao angular (Kp=5.0, Ki=1.0, Kd=10.0) com derivada
//         sobre a taxa filtrada (-rate_z_filt, sem chute derivativo em mudancas de setpoint).
//       - Zona morta de estabilizacao (YAW_DEADZONE_DEG = 2.5°) + reset imediato do termo
//         integral ao cruzar o ponto de equilibrio (elimina o ciclo-limite / oscilacao).
//       - Transicao suave de potencia minima (sem o salto cego de 75 us que gerava trancos).
//
//    3. ANTI-BALANCO POR VELOCIDADE NO REFERENCIAL DO MUNDO (ABORDAGEM 2):
//       - Atua como freio viscoso ativo (v_hat = L * w_mundo), drenando a energia de
//         oscilacao enquanto a gravidade restaura a posicao de prumo naturalmente.
//       - Rotacao da forca de controle do referencial do MUNDO para o CORPO por Rz(-yaw).
//       - Zona morta de repouso no prumo com histerese (< 0.60° e < 1.50 °/s): desliga os
//         motores em 1500 us quando a esfera atinge o equilibrio.
//
//    4. MODELO PADRAO IDENTICO PARA OS 3 MOTORES + MOTOR C (TR) INVERTIDO DE FABRICA:
//       - Mesmos valores padrao de kf_fwd e kf_rev para os 3 propulsores.
//       - Motor c (TR, PA1) configurado de fabrica com yaw_sign = -1.0 e vetor tangencial
//         invertido (+0.5, +0.866), garantindo desacoplamento entre Yaw e Anti-Balanco
//         desde o boot sem depender do envio de calibracao pela serial.
// =========================================================================================

#include <Arduino.h>
#include <Wire.h>
#include <Servo.h>
#include <AS5600.h>

// =========================================================================================
// --- MAPEAMENTO DE PINOS (BLACK PILL STM32F401 / F411) ---
// =========================================================================================
#define BF_PIN PA0   // Motor a (0): BF — Braco Superior (Frente). Empurra +X do corpo.
#define AS_PIN PA15  // Motor b (1): AS — Inferior Esquerdo (+120°)
#define TR_PIN PA1   // Motor c (2): TR — Inferior Direito (-120°, helice/ESC invertido)

// Ponte H1: Polia (BTS7960)
#define POLIA_EN_R_PIN  PB7
#define POLIA_EN_L_PIN  PB10
#define POLIA_RPWM_PIN  PB6
#define POLIA_LPWM_PIN  PB5

// Ponte H2: Pinca (BTS7960)
#define PINCA_EN_R_PIN   PB12
#define PINCA_EN_L_PIN   PB13
#define PINCA_RPWM_PIN   PA7
#define PINCA_LPWM_PIN   PB0

// Fins de curso da Pinca
#define FIM_CURSO_CIMA  PB14
#define FIM_CURSO_BAIXO PB15

// Sensor Optico de Olhal (LM393)
#define SENSOR_OLHAL_D0 PA5
#define SENSOR_OLHAL_A0 PA6

// Barramento I2C1 (AS5600 + MPU6050)
#define AS5600_SDA PB9
#define AS5600_SCL PB8

// LED Onboard (Black Pill - Active LOW)
#define LED_PIN PC13

#define MPU6050_ADDR 0x68

// =========================================================================================
// --- PARAMETROS DO HARDWARE E DOS MOTORES (IDENTICOS PARA OS 3 MOTORES) ---
// =========================================================================================
const float M_KG  = 12.0f;    // Massa total da esfera [kg]
const float R_P   = 0.70f;    // Raio do eixo central aos propulsores [m]
const float G_ACC = 9.80665f;

// Limites de PWM dos ESCs bidirecionais
const int   ESC_NEUTRAL      = 1500;
const int   PWM_LIMIT_US     = 450;   // 1500 +- 450 us (1050 a 1950 us - margem segura)
const int   SWING_RESERVE_US = 400;   // Reserva maxima para o anti-balanco [us]
const int   YAW_RESERVE_US   = 300;   // Reserva maxima para o controle de yaw [us]
const int   PWM_TEST_DELTA   = 200;   // Delta de PWM para o teste 'm' [us]

// Curva padrao unica para os 3 motores identicos:
//   F(u) = kf * (|u| - dz)^2 * sinal(u)
const float F_MAX_500   = 18.0f;  // Forca nominal de referencia a 500 us [N]
const float REV_RATIO   = 0.60f;  // Eficiencia do sentido reverso (60%)
const float DB_US       = 30.0f;  // Zona morta fisica do ESC [us]
const float F_DEADZONE  = 0.01f;  // Forca minima para sair do neutro [N]
const float F_MIN_TOTAL = 0.02f;  // Limiar de histerese da forca total XY [N]

// Parametros da malha de Yaw (baseados no mainYAWBOM)
const float YAW_DEADZONE_DEG = 2.5f;  // Zona morta angular para estabilizar sem oscilar [graus]
const float YAW_RESUME_DEG   = 3.2f;  // Histerese para reativar o Yaw [graus]
const float ESC_MIN_YAW_US   = 45.0f; // Pulso minimo efetivo de Yaw fora da deadzone [us]

// Zona morta de repouso no prumo (Anti-Balanco)
const float THETA_DEADZONE_DEG = 0.60f; // Entra em repouso se inclinacao < 0.60°
const float THETA_RESUME_DEG   = 0.85f; // Reativa controle se inclinacao > 0.85°
const float OMEGA_DEADZONE_DPS = 1.50f; // Limiar de velocidade angular p/ repouso (< 1.50 °/s)

// Estrutura de calibracao por motor (mantem compatibilidade com comando M)
struct MotorCalib {
  float tx, ty;     // Vetor unitario de empuxo no corpo quando pulso u > 0 (+PWM)
  float yaw_sign;   // +1: pulso u > 0 gera torque +Yaw; -1: invertido (Motor c / TR)
  float kf_fwd;     // Ganho quadratico frente [N/us^2]
  float kf_rev;     // Ganho quadratico re [N/us^2]
  float dz_us;      // Zona morta [us]
};

// Geometria de 120° com o Motor c (TR, PA1) invertido de fabrica:
// - Motor a (BF, PA0):  angulo 0°    -> t_rot = (+1.0,  0.0),    yaw_sign = +1 -> t_cmd = (+1.0,  0.0)
// - Motor b (AS, PA15): angulo +120° -> t_rot = (-0.5, +0.866),  yaw_sign = +1 -> t_cmd = (-0.5, +0.866)
// - Motor c (TR, PA1):  angulo -120° -> t_rot = (-0.5, -0.866),  yaw_sign = -1 -> t_cmd = (+0.5, +0.866)
MotorCalib motores[3] = {
  {  1.0f,  0.0f,       1.0f, 0.0f, 0.0f, DB_US },
  { -0.5f,  0.8660254f, 1.0f, 0.0f, 0.0f, DB_US },
  {  0.5f,  0.8660254f, -1.0f, 0.0f, 0.0f, DB_US },
};

void init_motores_padrao() {
  float kf_fwd = F_MAX_500 / ((500.0f - DB_US) * (500.0f - DB_US));
  float kf_rev = kf_fwd * REV_RATIO;
  for (int i = 0; i < 3; i++) {
    motores[i].kf_fwd = kf_fwd;
    motores[i].kf_rev = kf_rev;
    motores[i].dz_us  = DB_US;
  }
}

// =========================================================================================
// --- GANHOS DO CONTROLADOR (Ajustaveis via comando G<idx> <val> e lidos via R) ---
// =========================================================================================
// Mantem os 8 indices do protocolo do Digital Twin:
//   0: vel_kp          -> Ganho Proporcional de velocidade XY [N / (m/s)] (amortecimento principal)
//   1: vel_ki          -> Ganho Integral de velocidade XY (0.0 por padrao: gravidade corrige posicao)
//   2: vel_kd          -> Ganho Derivativo de velocidade XY
//   3: head_angle_kp   -> Kp do PID de Yaw (padrao YAWBOM: 5.0)
//   4: head_decel      -> Ki do PID de Yaw (padrao YAWBOM: 1.0)
//   5: head_rate_limit -> Kd do PID de Yaw (padrao YAWBOM: 10.0)
//   6: head_rate_kp_us -> Escala de saida do Yaw (padrao YAWBOM: 0.6667 = 2/3)
//   7: head_rate_ki_us -> Limite do integrador de Yaw (padrao YAWBOM: 400.0)
struct ControllerGains {
  float vel_kp;
  float vel_ki;
  float vel_kd;
  float head_angle_kp;   // Usado como Kp_yaw
  float head_decel;      // Usado como Ki_yaw
  float head_rate_limit; // Usado como Kd_yaw
  float head_rate_kp_us; // Fator de escala de pulso do Yaw (2/3 = 0.6667)
  float head_rate_ki_us; // Limite de anti-windup da integral de Yaw (400.0)
};

ControllerGains ganhos = {
  18.0f,  // G0: vel_kp [N/(m/s)] - amortecimento viscoso suave e firme sem saturar em ruido
  0.0f,   // G1: vel_ki - zero por padrao (deixa a gravidade centralizar no prumo)
  0.0f,   // G2: vel_kd - zero por padrao
  5.0f,   // G3: Kp do Yaw (original YAWBOM)
  1.0f,   // G4: Ki do Yaw (original YAWBOM)
  10.0f,  // G5: Kd do Yaw (original YAWBOM)
  0.6667f,// G6: Escala 2/3 de alocacao do Yaw nos 3 motores (original YAWBOM)
  400.0f  // G7: Limite da integral do Yaw (original YAWBOM)
};

// Parametros do cabo (L em metros)
const float L_MIN = 0.5f, L_MAX = 6.0f;
float L_M = 5.0f; // Comprimento nominal ajustado para 5 metros (ajustavel via L<val> ou 't')

// Sinal da compensacao de Yaw na fusao (teorema do transporte: w x s)
const float COMP_YAW_SIGN = -1.0f;

// Constantes da Fusao Complementar e Filtros Passa-Baixas
// Para L = 5.0 m, o periodo do pendulo e ~4.5 s (f ~ 0.22 Hz).
// ALPHA_FUSAO = 0.9995f define tau ~ 10 s @ 200 Hz: o balanco e 99.7% integrado do gyro
// sem ruido, eliminando cruzamentos falsos por zero, e o acc ancora o bias lentamente.
const float ALPHA_FUSAO     = 0.9995f; // Filtro complementar de inclinacao (tau ~ 10 s @ 200 Hz)
const float ALPHA_RATE_XY   = 0.22f;   // Filtro IIR (~9 Hz) p/ velocidades angulares wx e wy
const float ALPHA_RATE_Z    = 0.20f;   // Filtro IIR p/ taxa de Yaw
const float GYRO_Z_DEADZONE = 0.35f;   // Deadzone anti-deriva na integracao do Yaw parado [°/s]
const float ACC_GATE_LO     = 0.80f; // Limite inferior de confianca do acelerometro [g]
const float ACC_GATE_HI     = 1.20f; // Limite superior de confianca do acelerometro [g]

const unsigned long SWING_INTERVAL_US  = 5000; // Malha de controle e fusao: 200 Hz (5 ms)
const unsigned long TELEMETRY_INTERVAL = 40;   // Telemetria: 25 Hz (40 ms)

// =========================================================================================
// --- ESTADO GLOBAL ---
// =========================================================================================
AS5600 encoder;
Servo esc_a, esc_b, esc_c;

// Estado do Sensor MPU6050
float gyro_bias[3] = {0, 0, 0};     // Offsets do giroscopio [°/s]
float theta_offset_x = 0.0f;        // Zero gravimetrico em X [rad]
float theta_offset_y = 0.0f;        // Zero gravimetrico em Y [rad]
bool  calib_ok = false;

float sx_est = 0.0f, sy_est = 0.0f; // Inclinacao estimada no MUNDO [rad]
float wx_est = 0.0f, wy_est = 0.0f; // Taxa de inclinacao filtrada no MUNDO [rad/s]
float theta_total_deg = 0.0f;       // Inclinacao escalar total [graus]
float yaw_gyro = 0.0f;              // Angulo de Yaw integrado [0 a 360 graus]
float rate_z_filt = 0.0f;           // Taxa de rotacao em Z filtrada [°/s]
float acc_mag_g = 1.0f;             // Modulo da aceleracao medida [g]

// Estado das Malhas de Controle
bool  yaw_ativo = false;            // Inicia DESLIGADO no boot
bool  anti_balanco_ativo = false;   // Inicia DESLIGADO no boot
bool  sw_hysteresis_on = false;
bool  repouso_prumo_ativo = false;  // Histerese de repouso do pendulo no prumo
bool  yaw_em_repouso = false;       // Histerese de repouso do Yaw no alvo

float F_cmd_x = 0.0f, F_cmd_y = 0.0f; // Forca de anti-balanco no MUNDO [N]
float vhat_x = 0.0f, vhat_y = 0.0f;   // Velocidade linear estimada no MUNDO [m/s]
float desired_yaw = 0.0f;             // Setpoint de Yaw [graus]
float pos_error_yaw = 0.0f;           // Erro atual de Yaw [graus]
float integral_error_yaw = 0.0f;      // Acumulador integral do Yaw
float last_error_yaw = 0.0f;          // Ultimo erro de Yaw (para deteccao de cruzamento de zero)
float yaw_us_cmd = 0.0f;              // Comando final de Yaw [us por motor]

// Visao Computacional (recebida da Raspberry Pi via V<bearing> <D>)
float vision_bearing = 0.0f, vision_D = -1.0f;
unsigned long vision_last_ms = 0;
bool vision_fresh() { return (vision_last_ms > 0) && (millis() - vision_last_ms < 500); }

// Wizard (pulso de identificacao W<motor> <us> <ms>)
int  wizard_motor = -1;
int  wizard_us = 0;
long wizard_remaining_ms = 0;

// Mecanismos (Pinca, Polia com Watchdog, Sensor de Olhal)
int  estadoMotor = 0;
bool bateuCima = false, bateuBaixo = false;

unsigned long ultimo_comando_polia_ms = 0;
const unsigned long TIMEOUT_POLIA_MS  = 250; // Watchdog de seguranca da polia [ms]
bool polia_em_movimento = false;

int estadoEstavelOlhal = HIGH, ultimoEstadoFisicoOlhal = HIGH;
unsigned long ultimoTempoFiltro = 0;
const unsigned long tempoFiltro = 100;

// Medicao de periodo natural para calibracao de L (comando 't')
float periodo_medido = 0.0f;
unsigned long ultimo_cruzamento_us = 0;
bool cruzamento_armado = false;

unsigned long last_time_us = 0, last_swing_us = 0, last_telemetry_ms = 0;
int esc_us_atual[3] = {ESC_NEUTRAL, ESC_NEUTRAL, ESC_NEUTRAL};

// =========================================================================================
// --- FUNCOES UTILITARIAS ---
// =========================================================================================
float wrap180(float d) {
  while (d > 180.0f)  d -= 360.0f;
  while (d < -180.0f) d += 360.0f;
  return d;
}

int16_t ler16() {
  uint8_t hi = Wire.read();
  uint8_t lo = Wire.read();
  return (int16_t)((hi << 8) | lo);
}

// Formatacao de float imune a newlib-nano sem %f
void fmt_f2(char *out, float v) {
  bool neg = v < 0;
  if (neg) v = -v;
  long iv = (long)lroundf(v * 100.0f);
  snprintf(out, 16, "%s%ld.%02ld", neg ? "-" : "", iv / 100, iv % 100);
}

void fmt_f3(char *out, float v) {
  bool neg = v < 0;
  if (neg) v = -v;
  long iv = (long)lroundf(v * 1000.0f);
  snprintf(out, 16, "%s%ld.%03ld", neg ? "-" : "", iv / 1000, iv % 1000);
}

// =========================================================================================
// --- ESTRUTURA PID PARA VELOCIDADE XY (COM ANTI-WINDUP E RESET NO CRUZAMENTO) ---
// =========================================================================================
struct PID {
  float kp = 0, ki = 0, kd = 0, out_limit = 0;
  float integral = 0;
  bool  has_last = false;
  float last_meas = 0;
  float last_err = 0;

  void reset() {
    integral = 0;
    has_last = false;
    last_meas = 0;
    last_err = 0;
  }

  float update(float meas, float setpoint, float dt) {
    float error = setpoint - meas;
    float deriv = has_last ? -(meas - last_meas) / dt : 0.0f;

    // Zera a integral se o erro inverter de sinal (evita memoria contraria ao movimento)
    if (has_last && ((error > 0.0f && last_err < 0.0f) || (error < 0.0f && last_err > 0.0f))) {
      integral = 0.0f;
    }

    float p = kp * error;
    float d = kd * deriv;
    float inc = ki * error * dt;

    if (out_limit > 0.0f) {
      float out_before = p + d + ki * integral;
      bool saturating = fabsf(out_before + inc) > out_limit;
      bool reduces = (out_before * inc) < 0.0f;
      if (!saturating || reduces) integral += error * dt;
    } else {
      integral += error * dt;
    }

    float out = p + d + ki * integral;
    if (out_limit > 0.0f) out = constrain(out, -out_limit, out_limit);

    last_meas = meas;
    last_err = error;
    has_last = true;
    return out;
  }
};

PID pid_vx, pid_vy;

void setup_pids() {
  float kf_padrao = F_MAX_500 / ((500.0f - DB_US) * (500.0f - DB_US));
  float f_max_swing = 1.5f * kf_padrao * ((SWING_RESERVE_US - DB_US) * (SWING_RESERVE_US - DB_US));
  pid_vx.kp = ganhos.vel_kp;
  pid_vx.ki = ganhos.vel_ki;
  pid_vx.kd = ganhos.vel_kd;
  pid_vx.out_limit = f_max_swing;
  pid_vy = pid_vx;
}

// =========================================================================================
// --- MPU6050: INICIALIZACAO COM DLPF 20 HZ, CALIBRACAO E FUSAO DE SENSORES ---
// =========================================================================================
struct MpuSample {
  float gx, gy, gz; // [°/s]
  float ax, ay, az; // [LSB @ +-2g]
  bool  ok;
};

MpuSample ler_mpu() {
  MpuSample s = {0, 0, 0, 0, 0, 0, false};
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(0x3B);
  if (Wire.endTransmission(false) != 0) return s;
  Wire.requestFrom(MPU6050_ADDR, 14);
  if (Wire.available() < 14) return s;
  int16_t ax = ler16(), ay = ler16(), az = ler16();
  ler16(); // Temperatura
  int16_t gx = ler16(), gy = ler16(), gz = ler16();
  s.ax = (float)ax; s.ay = (float)ay; s.az = (float)az;
  s.gx = gx / 131.0f; s.gy = gy / 131.0f; s.gz = gz / 131.0f;
  s.ok = true;
  return s;
}

void init_mpu() {
  // 1. Acorda o MPU6050
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(0x6B); Wire.write(0x00); Wire.endTransmission();

  // 2. [NOVO] Ativa o Filtro Passa-Baixas Digital de Hardware (DLPF = 0x04 -> Banda de 20 Hz)
  // Corta vibracoes de alta frequencia dos 3 motores diretamente no chip antes da leitura I2C!
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(0x1A); Wire.write(0x04); Wire.endTransmission();

  // 3. Giroscopio em +-250 °/s (131 LSB/°/s)
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(0x1B); Wire.write(0x00); Wire.endTransmission();

  // 4. Acelerometro em +-2g (16384 LSB/g)
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(0x1C); Wire.write(0x00); Wire.endTransmission();

  // 5. Bypass I2C
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(0x37); Wire.write(0x02); Wire.endTransmission();
}

bool calibrar_mpu(bool verboso) {
  if (verboso) Serial.println("=== CALIBRANDO MPU6050 (mantenha a esfera parada) ===");
  const int N = 600;
  float sx = 0, sy = 0, sz = 0, sax = 0, say = 0, saz = 0;
  float max_dev = 0;

  MpuSample s0 = ler_mpu();
  if (!s0.ok) return false;
  float ref_gx = s0.gx, ref_gy = s0.gy, ref_gz = s0.gz;

  for (int i = 0; i < N; i++) {
    MpuSample s = ler_mpu();
    if (!s.ok) { i--; delay(2); continue; }
    sx += s.gx; sy += s.gy; sz += s.gz;
    sax += s.ax; say += s.ay; saz += s.az;
    float dev = fabsf(s.gx - ref_gx) + fabsf(s.gy - ref_gy) + fabsf(s.gz - ref_gz);
    if (dev > max_dev) max_dev = dev;
    if (i % 50 == 0) digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    delay(2);
  }
  digitalWrite(LED_PIN, HIGH);

  // Gate de quietude (mais tolerante com o DLPF ativo, mas barra movimentos reais)
  if (max_dev > 4.5f) {
    if (verboso) Serial.println("[AVISO] Movimento detectado na calibracao — mantendo ajuste com cautela.");
  }

  gyro_bias[0] = sx / N;
  gyro_bias[1] = sy / N;
  gyro_bias[2] = sz / N;

  float ax0 = sax / N, ay0 = say / N, az0 = saz / N;
  theta_offset_x = atan2f(-ax0, az0);
  theta_offset_y = atan2f(ay0, az0);

  sx_est = 0; sy_est = 0; wx_est = 0; wy_est = 0;
  rate_z_filt = 0;
  yaw_gyro = desired_yaw;
  integral_error_yaw = 0;
  calib_ok = true;
  if (verboso) Serial.println(">> Calibracao concluida! Zero gravimetrico e gyro ajustados.");
  return true;
}

// Auto-rebias termico lento quando motores estao em neutro e esfera parada por > 3 s
void talvez_rebias(const MpuSample &s) {
  static unsigned long still_since = 0;
  bool motores_parados = (esc_us_atual[0] == ESC_NEUTRAL &&
                          esc_us_atual[1] == ESC_NEUTRAL &&
                          esc_us_atual[2] == ESC_NEUTRAL);
  float vel_ang = sqrtf(wx_est * wx_est + wy_est * wy_est);
  bool quieto = motores_parados && (vel_ang < 0.015f) && (fabsf(rate_z_filt) < 0.4f);
  unsigned long now = millis();
  if (quieto) {
    if (still_since == 0) still_since = now;
    if (now - still_since > 3000) {
      gyro_bias[0] += 0.0002f * (s.gx - gyro_bias[0]);
      gyro_bias[1] += 0.0002f * (s.gy - gyro_bias[1]);
      gyro_bias[2] += 0.0002f * (s.gz - gyro_bias[2]);
    }
  } else {
    still_since = 0;
  }
}

void passo_fusao(const MpuSample &s, float dt) {
  // 1. Taxas angulares do giroscopio no corpo [rad/s]
  float rate_x = (s.gy - gyro_bias[1]) * (PI / 180.0f);
  float rate_y = (s.gx - gyro_bias[0]) * (PI / 180.0f);
  float raw_yaw_rate_dps = s.gz - gyro_bias[2];
  float wz = raw_yaw_rate_dps * (PI / 180.0f);

  // 2. Filtro passa-baixas na taxa de Yaw (como no YAWBOM: alpha = 0.20)
  rate_z_filt = ALPHA_RATE_Z * raw_yaw_rate_dps + (1.0f - ALPHA_RATE_Z) * rate_z_filt;

  // Integracao do Yaw com deadzone de ruido parado (evita deriva lenta na bancada)
  float rate_z_int = (fabsf(rate_z_filt) < GYRO_Z_DEADZONE) ? 0.0f : rate_z_filt;
  yaw_gyro += rate_z_int * dt;
  if (yaw_gyro >= 360.0f) yaw_gyro -= 360.0f;
  if (yaw_gyro < 0.0f)    yaw_gyro += 360.0f;

  // 3. Taxas de inclinacao no referencial do MUNDO (compensadas pela rotacao de Yaw)
  float wx_world = rate_x + COMP_YAW_SIGN * wz * sy_est;
  float wy_world = rate_y - COMP_YAW_SIGN * wz * sx_est;

  // 4. [TRAZIDO DO YAWBOM] Filtro passa-baixas IIR (~9 Hz) nas velocidades angulares
  // Corta o ruido residual antes de entrar na integral de atitude e no controlador de velocidade!
  wx_est = (1.0f - ALPHA_RATE_XY) * wx_est + ALPHA_RATE_XY * wx_world;
  wy_est = (1.0f - ALPHA_RATE_XY) * wy_est + ALPHA_RATE_XY * wy_world;

  // 5. Inclinacao gravimetrica pelo acelerometro com Gate Gravitacional (rejeita vibracoes)
  acc_mag_g = sqrtf(s.ax * s.ax + s.ay * s.ay + s.az * s.az) / 16384.0f;
  bool acc_ok = (acc_mag_g > ACC_GATE_LO && acc_mag_g < ACC_GATE_HI);

  float th_acc_x = atan2f(-s.ax, s.az) - theta_offset_x;
  float th_acc_y = atan2f(s.ay, s.az) - theta_offset_y;

  // 6. Fusao complementar com constante longa (tau ~ 10 s):
  // Durante o balanco pendular, o angulo e 99.7% integrado da taxa filtrada (sem ruido),
  // cruzando o zero estritamente no meio do ciclo real. O acc corrige a deriva DC lentamente.
  if (acc_ok) {
    sx_est = ALPHA_FUSAO * (sx_est + wx_est * dt) + (1.0f - ALPHA_FUSAO) * th_acc_x;
    sy_est = ALPHA_FUSAO * (sy_est + wy_est * dt) + (1.0f - ALPHA_FUSAO) * th_acc_y;
  } else {
    sx_est += wx_est * dt;
    sy_est += wy_est * dt;
  }

  theta_total_deg = sqrtf(sx_est * sx_est + sy_est * sy_est) * (180.0f / PI);
}

// =========================================================================================
// --- CONTROLE ANTI-BALANCO POR VELOCIDADE (ABORDAGEM 2) ---
// =========================================================================================
void passo_controle_xy(float dt) {
  if (!anti_balanco_ativo) {
    F_cmd_x = 0.0f;
    F_cmd_y = 0.0f;
    pid_vx.reset();
    pid_vy.reset();
    sw_hysteresis_on = false;
    repouso_prumo_ativo = false;
    return;
  }

  // [TRAZIDO DO YAWBOM] Zona morta de equilibrio no prumo com histerese:
  // Quando a esfera ja esta centralizada (< 0.60°) e parada (< 1.50 °/s), desliga os motores.
  float vel_ang_total_dps = sqrtf(wx_est * wx_est + wy_est * wy_est) * (180.0f / PI);
  if (repouso_prumo_ativo) {
    if (theta_total_deg > THETA_RESUME_DEG || vel_ang_total_dps > (1.8f * OMEGA_DEADZONE_DPS)) {
      repouso_prumo_ativo = false;
    }
  } else {
    if (theta_total_deg < THETA_DEADZONE_DEG && vel_ang_total_dps < OMEGA_DEADZONE_DPS) {
      repouso_prumo_ativo = true;
    }
  }

  if (repouso_prumo_ativo) {
    F_cmd_x = 0.0f;
    F_cmd_y = 0.0f;
    pid_vx.reset();
    pid_vy.reset();
    return;
  }

  // Velocidade tangencial estimada no MUNDO: v_hat = L * w_mundo [m/s]
  vhat_x = L_M * wx_est;
  vhat_y = L_M * wy_est;

  // Freio viscoso ativo (setpoint de velocidade = 0.0 m/s; gravidade restaura a posicao)
  F_cmd_x = pid_vx.update(vhat_x, 0.0f, dt);
  F_cmd_y = pid_vy.update(vhat_y, 0.0f, dt);

  // Histerese anti-chattering para forcas muito pequenas
  float norm_F = sqrtf(F_cmd_x * F_cmd_x + F_cmd_y * F_cmd_y);
  float limiar = sw_hysteresis_on ? F_MIN_TOTAL : (2.0f * F_MIN_TOTAL);
  sw_hysteresis_on = (norm_F > limiar);
  if (!sw_hysteresis_on) {
    F_cmd_x = 0.0f;
    F_cmd_y = 0.0f;
  }
}

// =========================================================================================
// --- CONTROLE DE YAW (ESTRUTURA ESTAVEL DO YAWBOM + SUPORTE A VISAO) ---
// =========================================================================================
void passo_controle_yaw(float dt) {
  // Calcula o erro em GRAUS com wrap180 (corrige o bug do wrap_pi em graus!)
  if (vision_fresh()) {
    pos_error_yaw = vision_bearing; // Erro direto da camera [graus]
    // Sincroniza continuamente o setpoint com o heading absoluto do cabo:
    // O cabo está na direção (yaw_gyro + vision_bearing).
    // Conforme a visão centraliza o cabo (vision_bearing -> 0), desired_yaw converge
    // para o heading real onde o cabo está. Se a visão perder o cabo (timeout 500 ms),
    // desired_yaw já guardará este último heading, e o controle por gyro manterá
    // exatamente essa direção sem salto angular nem giro de retorno!
    desired_yaw = wrap180(yaw_gyro + vision_bearing);
  } else {
    pos_error_yaw = wrap180(desired_yaw - yaw_gyro);
  }

  if (!yaw_ativo) {
    yaw_us_cmd = 0.0f;
    integral_error_yaw = 0.0f;
    last_error_yaw = pos_error_yaw;
    yaw_em_repouso = false;
    digitalWrite(LED_PIN, HIGH);
    return;
  }

  // Histerese de zona morta angular no ponto de estabilizacao (como no YAWBOM)
  float abs_err = fabsf(pos_error_yaw);
  if (yaw_em_repouso) {
    if (abs_err > YAW_RESUME_DEG) yaw_em_repouso = false;
  } else {
    if (abs_err <= YAW_DEADZONE_DEG && fabsf(rate_z_filt) < 1.5f) {
      yaw_em_repouso = true;
    }
  }

  if (yaw_em_repouso) {
    yaw_us_cmd = 0.0f;
    integral_error_yaw = 0.0f;
    last_error_yaw = pos_error_yaw;
    digitalWrite(LED_PIN, HIGH);
    return;
  }

  digitalWrite(LED_PIN, LOW);

  // Ganhos do PID de Yaw (padrao YAWBOM: Kp=5.0, Ki=1.0, Kd=10.0, escala=2/3)
  float Kp = ganhos.head_angle_kp;
  float Ki = ganhos.head_decel;
  float Kd = ganhos.head_rate_limit;
  float escala_aloc = ganhos.head_rate_kp_us; // 0.6667 (2/3)
  float max_int = ganhos.head_rate_ki_us;     // 400.0

  // Termo Proporcional
  float P = Kp * pos_error_yaw;

  // Termo Integral com reset imediato no cruzamento de zero (impede overshoot e ciclo-limite!)
  integral_error_yaw += pos_error_yaw * dt;
  if ((pos_error_yaw > 0.0f && last_error_yaw < 0.0f) ||
      (pos_error_yaw < 0.0f && last_error_yaw > 0.0f)) {
    integral_error_yaw = 0.0f;
  }
  integral_error_yaw = constrain(integral_error_yaw, -max_int, max_int);
  float I = Ki * integral_error_yaw;

  // Termo Derivativo sobre a velocidade angular filtrada (-rate_z_filt evita chute em degrau de setpoint)
  float D = -Kd * rate_z_filt;
  last_error_yaw = pos_error_yaw;

  float raw_yaw_out = P + I + D;

  // Comando em us por motor (equivalente ao roundf((2/3) * u_yaw_cmd) do YAWBOM)
  float u_cmd = escala_aloc * raw_yaw_out;

  // Piso minimo suave para vencer a zona morta dos ESCs quando fora da deadzone de angulo
  if (u_cmd > 0.0f && u_cmd < ESC_MIN_YAW_US)  u_cmd = ESC_MIN_YAW_US;
  if (u_cmd < 0.0f && u_cmd > -ESC_MIN_YAW_US) u_cmd = -ESC_MIN_YAW_US;

  yaw_us_cmd = constrain(u_cmd, -(float)YAW_RESERVE_US, (float)YAW_RESERVE_US);
}

// =========================================================================================
// --- ALOCACAO DE FORCAS NOS 3 PROPULSORES A 120° E ESCRITA NOS ESCS ---
// =========================================================================================
// Conversao de forca [N] para pulso [us] com rampa de transicao suave na zona morta
// (identica ao mainYAWBOM: evita saltos bruscos de 75 us e usa o mesmo kf para os 3 motores)
float us_from_force(int i, float f_n) {
  float f_abs = fabsf(f_n);
  if (f_abs < F_DEADZONE) return 0.0f;
  float kf = (f_n > 0.0f) ? motores[i].kf_fwd : motores[i].kf_rev;
  float dz = motores[i].dz_us;
  float fator_transicao = fminf(1.0f, (f_abs - F_DEADZONE) / (0.04f - F_DEADZONE));
  float us = fator_transicao * dz + sqrtf(f_abs / kf);
  return (f_n > 0.0f ? 1.0f : -1.0f) * us;
}

void escrever_escs(int us_a, int us_b, int us_c) {
  esc_us_atual[0] = us_a;
  esc_us_atual[1] = us_b;
  esc_us_atual[2] = us_c;
  esc_a.writeMicroseconds(us_a);
  esc_b.writeMicroseconds(us_b);
  esc_c.writeMicroseconds(us_c);
}

void realizar_controle(float Fwx, float Fwy, float yaw_us, float psi_rad) {
  // 1. Rotaciona a forca do referencial do MUNDO para o CORPO: F_corpo = Rz(-psi) * F_mundo
  float c = cosf(psi_rad), s = sinf(psi_rad);
  float Fbx =  c * Fwx + s * Fwy;
  float Fby = -s * Fwx + c * Fwy;

  // 2. Projeta nos 3 motores a 120° (motores[2] ja contem o sinal invertido do motor TR)
  float f_des[3] = {
    (2.0f / 3.0f) * (motores[0].tx * Fbx + motores[0].ty * Fby),
    (2.0f / 3.0f) * (motores[1].tx * Fbx + motores[1].ty * Fby),
    (2.0f / 3.0f) * (motores[2].tx * Fbx + motores[2].ty * Fby),
  };

  // 3. Reescala proporcional caso algum motor ultrapasse a reserva SWING_RESERVE_US
  float lam = 1.0f;
  for (int i = 0; i < 3; i++) {
    float kf = (f_des[i] >= 0.0f) ? motores[i].kf_fwd : motores[i].kf_rev;
    float du_max = (float)SWING_RESERVE_US - motores[i].dz_us;
    float cap = kf * du_max * du_max;
    if (fabsf(f_des[i]) > 1e-6f) {
      lam = fminf(lam, cap / fabsf(f_des[i]));
    }
  }

  // 4. Combina pulso de Anti-Balanco + pulso de Yaw (respeitando yaw_sign de cada motor)
  int us[3];
  for (int i = 0; i < 3; i++) {
    float u_swing = us_from_force(i, f_des[i] * lam);
    float u_total = u_swing + motores[i].yaw_sign * yaw_us;
    us[i] = ESC_NEUTRAL + (int)constrain(lroundf(u_total), -PWM_LIMIT_US, PWM_LIMIT_US);
  }
  escrever_escs(us[0], us[1], us[2]);
}

// =========================================================================================
// --- MECANISMOS: PINCA, POLIA (COM WATCHDOG), OLHAL E AS5600 ---
// =========================================================================================
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
  polia_em_movimento = false;
  Serial.println(">> POLIA: Motor PARADO");
}

void girarSentidoA() {
  ativarDriver();
  digitalWrite(POLIA_RPWM_PIN, HIGH);
  digitalWrite(POLIA_LPWM_PIN, LOW);
  digitalWrite(LED_PIN, LOW);
  ultimo_comando_polia_ms = millis();
  polia_em_movimento = true;
  Serial.println(">> POLIA: Girando SENTIDO A (Horario)");
}

void girarSentidoD() {
  ativarDriver();
  digitalWrite(POLIA_RPWM_PIN, LOW);
  digitalWrite(POLIA_LPWM_PIN, HIGH);
  digitalWrite(LED_PIN, LOW);
  ultimo_comando_polia_ms = millis();
  polia_em_movimento = true;
  Serial.println(">> POLIA: Girando SENTIDO D (Anti-horario)");
}

void lerEncoderAS5600() {
  if (encoder.isConnected()) {
    int anguloBruto = encoder.readAngle();
    float graus = (anguloBruto / 4095.0f) * 360.0f;
    Serial.print(">> AS5600 - Angulo Bruto: ");
    Serial.print(anguloBruto);
    Serial.print(" | Graus: ");
    Serial.println(graus, 2);
  } else {
    Serial.println("[ERRO] AS5600 nao encontrado! Verifique a fiacao I2C.");
  }
}

void destravar_barramento_I2C() {
  pinMode(AS5600_SCL, OUTPUT);
  pinMode(AS5600_SDA, INPUT_PULLUP);
  for (int i = 0; i < 9; i++) {
    digitalWrite(AS5600_SCL, LOW);  delayMicroseconds(5);
    digitalWrite(AS5600_SCL, HIGH); delayMicroseconds(5);
  }
  pinMode(AS5600_SDA, OUTPUT);
  digitalWrite(AS5600_SDA, LOW);    delayMicroseconds(5);
  digitalWrite(AS5600_SCL, HIGH);   delayMicroseconds(5);
  digitalWrite(AS5600_SDA, HIGH);
}

void testarIdentificacaoMotores() {
  Serial.println("\n==========================================================");
  Serial.println("  TESTE E IDENTIFICACAO DOS 3 MOTORES                      ");
  Serial.println("==========================================================");
  Serial2.println("[TESTE_MOTORES]: Iniciando sequencia...");
  escrever_escs(ESC_NEUTRAL, ESC_NEUTRAL, ESC_NEUTRAL);
  delay(1000);

  struct { const char *nome; const char *pino; Servo *esc; } lista[3] = {
    {"MOTOR a (BF)", "PA0",  &esc_a},
    {"MOTOR b (AS)", "PA15", &esc_b},
    {"MOTOR c (TR)", "PA1",  &esc_c},
  };

  bool yaw_antes = yaw_ativo, swing_antes = anti_balanco_ativo;
  yaw_ativo = anti_balanco_ativo = false;

  for (int i = 0; i < 3; i++) {
    Serial.print(">>> TESTANDO "); Serial.print(lista[i].nome);
    Serial.print(" no pino ");     Serial.println(lista[i].pino);
    digitalWrite(LED_PIN, LOW);
    lista[i].esc->writeMicroseconds(ESC_NEUTRAL + PWM_TEST_DELTA);
    delay(2000);
    lista[i].esc->writeMicroseconds(ESC_NEUTRAL);
    digitalWrite(LED_PIN, HIGH);
    delay(1000);
    digitalWrite(LED_PIN, LOW);
    lista[i].esc->writeMicroseconds(ESC_NEUTRAL - PWM_TEST_DELTA);
    delay(2000);
    lista[i].esc->writeMicroseconds(ESC_NEUTRAL);
    digitalWrite(LED_PIN, HIGH);
    delay(1000);
  }

  yaw_ativo = yaw_antes;
  anti_balanco_ativo = swing_antes;
  last_time_us = micros();
  last_swing_us = micros();
  Serial.println("  TESTE CONCLUIDO!\n");
  Serial2.println("[TESTE_MOTORES]: Concluido!");
}

// =========================================================================================
// --- PARSER DE COMANDOS SERIAL (USB E SERIAL2 UNIFICADOS) ---
// =========================================================================================
const char *GAIN_NAMES[8] = {
  "vel_kp", "vel_ki", "vel_kd", "head_angle_kp",
  "head_decel", "head_rate_limit", "head_rate_kp_us", "head_rate_ki_us"
};

float *gain_ptr(int idx) {
  switch (idx) {
    case 0: return &ganhos.vel_kp;          case 1: return &ganhos.vel_ki;
    case 2: return &ganhos.vel_kd;          case 3: return &ganhos.head_angle_kp;
    case 4: return &ganhos.head_decel;      case 5: return &ganhos.head_rate_limit;
    case 6: return &ganhos.head_rate_kp_us; case 7: return &ganhos.head_rate_ki_us;
  }
  return nullptr;
}

void set_gains_from_struct() {
  pid_vx.kp = ganhos.vel_kp;
  pid_vx.ki = ganhos.vel_ki;
  pid_vx.kd = ganhos.vel_kd;
  pid_vy.kp = ganhos.vel_kp;
  pid_vy.ki = ganhos.vel_ki;
  pid_vy.kd = ganhos.vel_kd;
}

void parada_geral() {
  yaw_ativo = false;
  anti_balanco_ativo = false;
  wizard_motor = -1;
  wizard_remaining_ms = 0;
  integral_error_yaw = 0.0f;
  yaw_us_cmd = 0.0f;
  F_cmd_x = F_cmd_y = 0.0f;
  pid_vx.reset();
  pid_vy.reset();
  escrever_escs(ESC_NEUTRAL, ESC_NEUTRAL, ESC_NEUTRAL);
  pararBTS();
  pararMotor();
  Serial.println(">> [STM]: PARADA GERAL (loops off + ESCs neutro + mecanismos)");
  Serial2.println(">> [STM]: PARADA GERAL");
}

void comando_char(char c) {
  bateuCima  = (digitalRead(FIM_CURSO_CIMA) == HIGH);
  bateuBaixo = (digitalRead(FIM_CURSO_BAIXO) == HIGH);
  switch (c) {
    case 'w': case 'W':
      if (!bateuCima) moverSubir();
      else { pararMotor(); Serial.println("[BLOQUEADO] Topo atingido!"); }
      break;
    case 's': case 'S':
      if (!bateuBaixo) moverDescer();
      else { pararMotor(); Serial.println("[BLOQUEADO] Base atingida!"); }
      break;
    case ' ': case 'x': case 'X': pararMotor(); break;
    case 'a': case 'A': girarSentidoA(); break;
    case 'd': case 'D': girarSentidoD(); break;
    case 'c': case 'C': pararBTS(); break;
    case 'e': case 'E': lerEncoderAS5600(); break;
    case 'm': testarIdentificacaoMotores(); break;
    case 'y': case 'Y':
      yaw_ativo = !yaw_ativo;
      Serial.println(yaw_ativo ? ">> [YAW]: ATIVADO" : ">> [YAW]: DESATIVADO");
      Serial2.println(yaw_ativo ? ">> [YAW]: ATIVADO" : ">> [YAW]: DESATIVADO");
      if (!yaw_ativo) {
        integral_error_yaw = 0.0f;
        yaw_us_cmd = 0.0f;
        yaw_em_repouso = false;
      }
      break;
    case 'b': case 'B': case 'p': case 'P':
      anti_balanco_ativo = !anti_balanco_ativo;
      Serial.println(anti_balanco_ativo ? ">> [PENDULO]: ANTI-BALANCO ATIVADO"
                                        : ">> [PENDULO]: ANTI-BALANCO DESATIVADO");
      Serial2.println(anti_balanco_ativo ? ">> [PENDULO]: ATIVADO" : ">> [PENDULO]: DESATIVADO");
      if (!anti_balanco_ativo) {
        F_cmd_x = F_cmd_y = 0.0f;
        pid_vx.reset();
        pid_vy.reset();
        repouso_prumo_ativo = false;
      }
      break;
    case 'k': case 'K': parada_geral(); break;
    case 'z': case 'Z':
      calibrar_mpu(true);
      Serial.println(">> [PENDULO]: Zero gravimetrico recalibrado.");
      break;
    case 't': case 'T':
      if (periodo_medido > 1.0f) {
        float L_calc = G_ACC * powf(periodo_medido / (2.0f * PI), 2.0f);
        L_M = constrain(L_calc, L_MIN, L_MAX);
        Serial.print(">> [PENDULO]: L calibrado pelo periodo = ");
        Serial.print(L_M, 2); Serial.println(" m");
      } else {
        Serial.println(">> [PENDULO]: Balance a esfera livre para medir o periodo.");
      }
      break;
    case '0':
      desired_yaw = 0.0f;
      integral_error_yaw = 0.0f;
      Serial.println(">> [YAW]: desired_yaw = 0");
      Serial2.println(">> [YAW]: desired_yaw = 0");
      break;
  }
}

void comando_linha(const String &cmd) {
  if (cmd.length() == 0) return;

  // L<val>: define comprimento do cabo [m]
  if ((cmd[0] == 'L' || cmd[0] == 'l') && cmd.length() > 1) {
    float novo = cmd.substring(1).toFloat();
    if (novo >= L_MIN && novo <= L_MAX) {
      L_M = novo;
      Serial.print(">> [PENDULO]: L = "); Serial.print(L_M, 2); Serial.println(" m");
    } else {
      Serial.println("[ERRO]: L fora da faixa (0.5 a 6.0 m).");
    }
    return;
  }

  // G<idx> <val>: ajusta ganho em runtime
  if (cmd[0] == 'G' && cmd.length() > 1) {
    int space = cmd.indexOf(' ');
    if (space > 1) {
      int idx = cmd.substring(1, space).toInt();
      float val = cmd.substring(space + 1).toFloat();
      float *p = gain_ptr(idx);
      if (p && (idx >= 0 && idx < 8)) {
        *p = val;
        set_gains_from_struct();
        Serial.print(">> [GANHO] "); Serial.print(GAIN_NAMES[idx]);
        Serial.print(" = "); Serial.println(val, 4);
      }
    }
    return;
  }

  // R: retorna todos os ganhos atuais
  if (cmd[0] == 'R') {
    for (int i = 0; i < 8; i++) {
      String s = "GAIN," + String(i) + "," + GAIN_NAMES[i] + "," + String(*gain_ptr(i), 4);
      Serial.println(s);
      Serial2.println(s);
    }
    return;
  }

  // V<bearing> <D>: entrada da visao computacional
  if (cmd[0] == 'V' && cmd.length() > 1) {
    int space = cmd.indexOf(' ');
    if (space > 1) {
      vision_bearing = cmd.substring(1, space).toFloat();
      vision_D = cmd.substring(space + 1).toFloat();
      vision_last_ms = millis();
    }
    return;
  }

  // W<motor> <us> <ms>: pulso de identificacao do wizard
  if (cmd[0] == 'W' && cmd.length() > 1) {
    int s1 = cmd.indexOf(' ');
    int s2 = cmd.indexOf(' ', s1 + 1);
    if (s1 > 1 && s2 > s1) {
      int m = cmd.substring(1, s1).toInt();
      int us = cmd.substring(s1 + 1, s2).toInt();
      long ms = cmd.substring(s2 + 1).toInt();
      if (m >= 0 && m < 3 && fabsf(us) <= PWM_LIMIT_US && ms > 0 && ms <= 3000) {
        yaw_ativo = false;
        anti_balanco_ativo = false;
        wizard_motor = m;
        wizard_us = us;
        wizard_remaining_ms = ms;
        Serial.print(">> [WIZARD] motor "); Serial.print(m);
        Serial.print(" pulso "); Serial.print(us);
        Serial.print(" us por "); Serial.print(ms); Serial.println(" ms");
      }
    }
    return;
  }

  // M<motor> <tx> <ty> <yaw_sign> <kf_fwd> <kf_rev> <dz>:
  // Aceita atualizacao de yaw_sign e direcao, mas mantem kf_fwd/kf_rev padrao identicos
  // caso os valores enviados sejam discrepantes, evitando assimetria entre motores iguais.
  if (cmd[0] == 'M' && cmd.length() > 1) {
    int fields[6];
    int fi = 0;
    for (unsigned int i = 0; i < cmd.length() && fi < 6; i++) {
      if (cmd[i] == ' ') fields[fi++] = (int)i;
    }
    if (fi == 6) {
      int m = cmd.substring(1, fields[0]).toInt();
      if (m >= 0 && m < 3) {
        float ys = cmd.substring(fields[2] + 1, fields[3]).toFloat();
        motores[m].yaw_sign = (ys >= 0.0f) ? 1.0f : -1.0f;
        // Reaplica os kf padrao identicos para garantir simetria nos 3 motores
        init_motores_padrao();
        Serial.print(">> [MOTOR "); Serial.print(m);
        Serial.print("] yaw_sign ajustado para ");
        Serial.print(motores[m].yaw_sign, 0);
        Serial.println(" (kf padrao simetrico mantido).");
      }
    }
    return;
  }

  // <numero>: novo setpoint de angulo de Yaw [graus]
  char c0 = cmd[0];
  if (isdigit(c0) || c0 == '-' || c0 == '+') {
    float ang = cmd.toFloat();
    yaw_gyro = 0.0f;
    desired_yaw = ang;
    integral_error_yaw = 0.0f;
    yaw_em_repouso = false;
    String msg = ">> NOVO ANGULO APLICADO: " + String(desired_yaw, 2);
    Serial.println(msg);
    Serial2.println(msg);
    return;
  }

  if (cmd.length() == 1) comando_char(cmd[0]);
}

const char WHITELIST[] = "wsSxX aAdDcCeEmYyBbPpKkZzTt0";
bool eh_comando_imediato(char c) {
  for (const char *p = WHITELIST; *p; p++) {
    if (*p == ' ') continue;
    if (*p == c) return true;
  }
  return false;
}

String bufUSB = "", bufS2 = "";
void processar_serial(Stream &porta, String &buf) {
  while (porta.available() > 0) {
    char c = porta.read();
    if (c == '\n' || c == '\r') {
      if (buf.length() > 0) {
        comando_linha(buf);
        buf = "";
      }
    } else if (buf.length() == 0 && eh_comando_imediato(c)) {
      comando_char(c);
    } else {
      buf += c;
      if (buf.length() > 60) buf = "";
    }
  }
}

// =========================================================================================
// --- TELEMETRIA 25 HZ (COMPATIVEL COM O DIGITAL TWIN / RASPBERRY PI) ---
// =========================================================================================
void enviar_telemetria() {
  unsigned flags = (yaw_ativo ? 1 : 0) |
                   (anti_balanco_ativo ? 2 : 0) |
                   (calib_ok ? 4 : 0) |
                   (vision_fresh() ? 8 : 0) |
                   (wizard_motor >= 0 ? 16 : 0);

  char line[192], num[16];
  int n = snprintf(line, sizeof(line), "TEL,%lu,", millis());
  fmt_f2(num, yaw_gyro);                               n += snprintf(line + n, sizeof(line) - n, "%s,", num);
  fmt_f2(num, vision_fresh() ? vision_bearing : 0.0f); n += snprintf(line + n, sizeof(line) - n, "%s,", num);
  fmt_f2(num, desired_yaw);                            n += snprintf(line + n, sizeof(line) - n, "%s,", num);
  fmt_f2(num, rate_z_filt);                            n += snprintf(line + n, sizeof(line) - n, "%s,", num);
  fmt_f3(num, sx_est * 57.2958f);                      n += snprintf(line + n, sizeof(line) - n, "%s,", num);
  fmt_f3(num, sy_est * 57.2958f);                      n += snprintf(line + n, sizeof(line) - n, "%s,", num);
  fmt_f3(num, wx_est * 57.2958f);                      n += snprintf(line + n, sizeof(line) - n, "%s,", num);
  fmt_f3(num, wy_est * 57.2958f);                      n += snprintf(line + n, sizeof(line) - n, "%s,", num);
  n += snprintf(line + n, sizeof(line) - n, "%d,%d,%d,",
                esc_us_atual[0], esc_us_atual[1], esc_us_atual[2]);
  fmt_f2(num, L_M);                                    n += snprintf(line + n, sizeof(line) - n, "%s,", num);
  fmt_f2(num, vision_fresh() ? vision_D : -1.0f);      n += snprintf(line + n, sizeof(line) - n, "%s,", num);
  snprintf(line + n, sizeof(line) - n, "%u", flags);

  Serial.println(line);
  Serial2.println(line);
}

// =========================================================================================
// --- SETUP E LOOP PRINCIPAL ---
// =========================================================================================
void setup() {
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);

  esc_a.attach(BF_PIN);
  esc_b.attach(AS_PIN);
  esc_c.attach(TR_PIN);
  escrever_escs(ESC_NEUTRAL, ESC_NEUTRAL, ESC_NEUTRAL);

  init_motores_padrao();
  setup_pids();

  Serial.begin(115200);
  Serial2.setRx(PA3);
  Serial2.setTx(PA2);
  Serial2.begin(115200);

  destravar_barramento_I2C();
  Wire.setSCL(AS5600_SCL);
  Wire.setSDA(AS5600_SDA);
  Wire.begin();
  Wire.setClock(400000); // I2C Fast Mode 400 kHz
  Wire.setTimeout(3);
  delay(100);

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
  encoder.begin();

  init_mpu();
  calibrar_mpu(true);

  yaw_gyro = 0.0f;
  last_time_us = micros();
  last_swing_us = micros();
  last_telemetry_ms = millis();

  Serial.println("==========================================================");
  Serial.println(" ESFERA CONSOLIDADA — YAWBOM + ANTI-BALANCO DE VELOCIDADE ");
  Serial.println("  PINCA:   'w'=Subir | 's'=Descer | 'x'=Parar             ");
  Serial.println("  POLIA:   'a'=Giro A| 'd'=Giro D | 'c'=Parar (WD 250ms)  ");
  Serial.println("  AS5600:  'e'=Ler Angulo                                 ");
  Serial.println("  YAW:     'y'=on/off | <graus>=setpoint | '0'=zero       ");
  Serial.println("  PENDULO: 'b' ou 'p'=anti-balanco | 'z'=calibra MPU      ");
  Serial.println("           't'=L pelo periodo | 'L2.0'=L manual | 'm'=teste");
  Serial.println("  GANHOS:  'G<idx> <val>' | 'R' ler | VISAO: 'V<b> <D>'   ");
  Serial.println("  PARADA GERAL: 'k'   (Motor c/TR invertido de fabrica)   ");
  Serial.println("==========================================================");
}

void loop() {
  unsigned long now_us = micros();
  float dt = (now_us - last_time_us) * 1e-6f;
  last_time_us = now_us;
  if (dt <= 1e-6f || dt > 0.5f) dt = 0.005f;

  processar_serial(Serial, bufUSB);
  processar_serial(Serial2, bufS2);

  // Watchdog da Polia (Deadman Switch de 250 ms trazido do YAWBOM)
  if (polia_em_movimento && (millis() - ultimo_comando_polia_ms > TIMEOUT_POLIA_MS)) {
    pararBTS();
  }

  // --- Malha fixa de 200 Hz (5 ms): Fusao + Controle XY + Controle Yaw ---
  if (now_us - last_swing_us >= SWING_INTERVAL_US) {
    float dt_swing = (now_us - last_swing_us) * 1e-6f;
    last_swing_us = now_us;

    MpuSample s = ler_mpu();
    if (s.ok) {
      passo_fusao(s, dt_swing);
      talvez_rebias(s);
    }

    if (wizard_motor >= 0) {
      if (wizard_remaining_ms <= 0) {
        escrever_escs(ESC_NEUTRAL, ESC_NEUTRAL, ESC_NEUTRAL);
        wizard_motor = -1;
        Serial.println(">> [WIZARD] pulso concluido");
      } else {
        int us[3] = {ESC_NEUTRAL, ESC_NEUTRAL, ESC_NEUTRAL};
        us[wizard_motor] = ESC_NEUTRAL + wizard_us;
        escrever_escs(us[0], us[1], us[2]);
        wizard_remaining_ms -= (long)(dt_swing * 1000.0f);
      }
    } else {
      passo_controle_xy(dt_swing);
      passo_controle_yaw(dt_swing);
      // F_cmd_x e F_cmd_y sao derivados de wx_est e wy_est (giroscopio no corpo).
      // Portanto, F_cmd ja esta no referencial do CORPO da esfera!
      // Passar psi_rad = 0.0f impede que a forca seja rotacionada por -yaw (FW-03),
      // garantindo amortecimento viscoso perfeitamente alinhado em qualquer orientacao de Yaw.
      realizar_controle(F_cmd_x, F_cmd_y, yaw_us_cmd, 0.0f);
    }

    // Medicao de periodo em oscilacao livre (quando anti-balanco desligado)
    if (!anti_balanco_ativo && wizard_motor < 0) {
      float vel_ang = sqrtf(wx_est * wx_est + wy_est * wy_est);
      if (vel_ang > (2.0f * PI / 180.0f)) cruzamento_armado = true;
      if (cruzamento_armado && vel_ang < (0.5f * PI / 180.0f)) {
        unsigned long t_cruz = micros();
        if (t_cruz - ultimo_cruzamento_us > 800000UL) {
          cruzamento_armado = false;
          float T_amostra = (t_cruz - ultimo_cruzamento_us) * 2e-6f;
          ultimo_cruzamento_us = t_cruz;
          if (T_amostra > 1.2f && T_amostra < 7.5f) {
            periodo_medido = (periodo_medido == 0.0f)
                             ? T_amostra
                             : 0.7f * periodo_medido + 0.3f * T_amostra;
          }
        }
      }
    }
  }

  // --- Telemetria a 25 Hz ---
  if (millis() - last_telemetry_ms >= TELEMETRY_INTERVAL) {
    last_telemetry_ms = millis();
    enviar_telemetria();
  }

  // --- Debounce do Sensor Optico do Olhal ---
  int leituraCrua = digitalRead(SENSOR_OLHAL_D0);
  if (leituraCrua != ultimoEstadoFisicoOlhal) ultimoTempoFiltro = millis();
  if ((millis() - ultimoTempoFiltro) > tempoFiltro) {
    if (leituraCrua != estadoEstavelOlhal) {
      estadoEstavelOlhal = leituraCrua;
      Serial.println(estadoEstavelOlhal == LOW
        ? "olhal chegou ao mecanismo (Luz detectada)"
        : "olhal removido do mecanismo (Feixe bloqueado)");
    }
  }
  ultimoEstadoFisicoOlhal = leituraCrua;

  // --- Seguranca dos Fins de Curso da Pinca ---
  bateuCima  = (digitalRead(FIM_CURSO_CIMA) == HIGH);
  bateuBaixo = (digitalRead(FIM_CURSO_BAIXO) == HIGH);
  if (bateuCima && estadoMotor == 1) {
    pararMotor();
    Serial.println("[PARADA] Atingiu o topo! So pode descer ('s').");
  }
  if (bateuBaixo && estadoMotor == -1) {
    pararMotor();
    Serial.println("[PARADA] Atingiu a base! So pode subir ('w').");
  }
}
