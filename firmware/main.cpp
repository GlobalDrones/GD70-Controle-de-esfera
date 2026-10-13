// =========================================================================================
//  PROJETO ESFERA SUSPENSA — FIRMWARE STM32F4 (BLACK PILL)
//
//  Esfera de 12 kg suspensa por cabo (L variável ENTRE sessões, 0,5–6 m) sobre linhas de
//  transmissão. 3 motores bidirecionais (ESCs F45A, PWM 1500 µs neutro, faixa completa
//  ±500 µs) com hélices tangentes a 120° controlam 3 GDL:
//    - ANTI-BALANÇO (X/Y): PID de VELOCIDADE estimada v̂ = L·θ̇_mundo (amortecimento)
//    - HEADING (yaw): cascata apontamento → perfil de taxa com frenagem → PI de taxa em µs
//      O erro de apontamento vem da VISÃO (V,<bearing>,<D> pela Serial2: câmeras fixas
//      ao sensor enxergam o cabo da linha para baixo) quando fresco; senão do setpoint
//      interno vs. yaw integrado do gyro (deriva — re-zero periódico).
//
//  PARIDADE COM O SIMULADOR/TWIN (Python): mesmo modelo por motor, mesma alocação, mesmos
//  ganhos (domínio de µs no heading). Calibração por motor ajustável em runtime (comando
//  M) — o wizard do twin mede direções/curvas de cada motor e envia; nada assume que os
//  3 motores foram soldados/montados iguais.
//
//  REFERENCIAIS (convenção fixa deste firmware):
//    - CORPO = referencial do sensor (MPU6050 + câmeras), gira com o heading.
//    - Motor a (BF, PA0) empurra no +X do corpo [verificado fisicamente]; b (AS, PA15) e
//      c (TR, PA1) a ±120°: t_a=(1,0), t_b=(−1/2,+√3/2), t_c=(−1/2,−√3/2).
//    - MUNDO = referencial fixo vertical. A força de controle é computada no MUNDO e
//      girada para o corpo antes da alocação (o firmware antigo aplicava direto no corpo
//      — direção errada quando yaw ≠ 0).
//    - COMP_YAW_SIGN: sinal da compensação de yaw na fusão (teorema do transporte).
//      Se o teste de bancada (girar a esfera à mão inclinada e ver sx/sy não mudarem)
//      falhar, inverta para −1.
//
//  PRINCIPAIS CORREÇÕES vs. FIRMWARE ANTERIOR:
//    1. pos_error nunca era calculado → PID de yaw morto (agora cascata completa).
//    2. Força anti-balanço sem rotação mundo→corpo → direção errada com yaw ≠ 0.
//    3. Taxas de inclinação no referencial do CORPO vs. ângulos em MUNDO (inconsistência)
//       → taxas compensadas para o mundo.
//    4. Deadzone de 0,5°/s no caminho de integração do yaw → deriva sistemática (removida).
//    5. Sem gate gravitacional no acelerômetro → vibração dos motores corrompia a fusão
//       (agora: correção do acc só com |acc| ≈ 1 g).
//    6. Calibração sem gate de quietude → offsets errados silenciosos (agora: verificação
//       de variância + auto-rebias do gyro em repouso).
//    7. Atuador modelado globalmente (1 curva, DIRs só no yaw) → modelo por motor
//       calibrável (ESC invertido/hélice espelhada detectáveis pelo wizard).
//    8. I2C a 100 kHz → 400 kHz (MPU6050 suporta; 4× mais leituras).
//    9. Whitelist de comandos duplicada → parser unificado nas duas portas.
//
//  PROTOCOLO (USB e Serial2/Rasp aceitam os mesmos comandos):
//    chars:  w/s/x=pinça sobe/desce/para  a/d/c=polia A/D/para  e=lê AS5600
//            m=teste motores  y=yaw on/off  b|p=anti-balanço on/off  k=PARADA GERAL
//            z=calibra MPU (com gate de quietude)  t=calibra L pelo período  0=yaw desejado 0
//    linhas: L<val> comprimento do cabo (m)     <num> yaw desejado (graus)
//            G<idx> <val> ajusta ganho          R devolve ganhos
//            V<bearing> <D> visão (graus no corpo, m até o cabo da linha)
//            W<motor> <us> <ms> pulso de identificação (wizard; loops off)
//            M<motor> <tx> <ty> <yaw_sign> <kf_fwd> <kf_rev> <dz> calibração por motor
//    telemetria 25 Hz (ambas as portas):
//      TEL,t_ms,yaw,bearing,yaw_des,rate_z,sx,sy,wx,wy,pwm_a,pwm_b,pwm_c,L,D,flags
//      (ângulos em graus, taxas em °/s, pwm em µs; D=-1 sem visão;
//       flags: 1=yaw_on 2=swing_on 4=calib_ok 8=visão_fresca 16=wizard)
// =========================================================================================

#include <Arduino.h>
#include <Wire.h>
#include <Servo.h>
#include <AS5600.h>

// =========================================================================================
// --- PINOS (inalterados do firmware original) ---
// =========================================================================================
#define BF_PIN PA0   // Motor a: BF — Braço Superior (Frente). Empurra +X do sensor.
#define AS_PIN PA15  // Motor b: AS — Inferior Esquerdo (+120°)
#define TR_PIN PA1   // Motor c: TR — Inferior Direito (−120°)

#define POLIA_EN_R_PIN  PB7
#define POLIA_EN_L_PIN  PB10
#define POLIA_RPWM_PIN  PB6
#define POLIA_LPWM_PIN  PB5

#define PINCA_EN_R_PIN   PB12
#define PINCA_EN_L_PIN   PB13
#define PINCA_RPWM_PIN   PA7
#define PINCA_LPWM_PIN   PB0

#define FIM_CURSO_CIMA  PB14
#define FIM_CURSO_BAIXO PB15

#define SENSOR_OLHAL_D0 PA5
#define SENSOR_OLHAL_A0 PA6

#define AS5600_SDA PB9
#define AS5600_SCL PB8

#define LED_PIN PC13  // Active LOW

#define MPU6050_ADDR 0x68

// =========================================================================================
// --- PARÂMETROS DO HARDWARE (espelham actuators.py / controller.py do Python) ---
// =========================================================================================
const float M_KG         = 12.0f;   // massa total [kg]
const float R_P          = 0.70f;   // braço eixo central → propulsores [m]
const float G_ACC        = 9.80665f;

// PWM dos ESCs: neutro e limites. Faixa COMPLETA do ESC (±500 µs, calibração
// de faixa feita) — decisão de operação da equipe; verificar corrente/térmica
// dos F45A na primeira sessão com autoridade alta.
const int   ESC_NEUTRAL      = 1500;
const int   PWM_LIMIT_US     = 500;   // 1500 ± 500 (faixa completa)
const int   SWING_RESERVE_US = 490;   // reserva do anti-balanço
const int   YAW_RESERVE_US   = 300;   // reserva do heading

// Curva de empuxo PROVISÓRIA até o wizard calibrar por motor:
//   F(u) = kf·(u−dz)²·sinal(u);  kf_fwd = F_MAX_500/(500−dz)²
const float F_MAX_500 = 18.0f;   // N @ ±500 µs (chute; medir por motor)
const float REV_RATIO = 0.60f;   // kf_rev/kf_fwd (chute)
const float DB_US     = 30.0f;   // zona morta (chute)
const float F_MIN_N   = 0.01f;   // abaixo disso, µs = neutro (anti-chatter)
// Travamento direcional do ESC observado na bancada: comando pequeno num
// sentido (~±60 µs, PWM ~1440–1560) trava AQUELE sentido; só comando válido
// no sentido OPOSTO destrava. PROVISÓRIO — o wizard medirá por ESC. Para
// nunca habitar a faixa: comandos sub-piso viram neutro (0 ou ≥ latch+15).
const float ESC_LATCH_US    = 60.0f;
const float LATCH_MARGIN_US = 15.0f;
const int   PWM_TEST_DELTA = 120; // teste de identificação 'm'

// --- Modelo POR MOTOR (defaults geométricos; comando M ajusta em runtime) ---
struct MotorCalib {
  float tx, ty;       // vetor de empuxo unitário no corpo
  float yaw_sign;     // +1: empuxo positivo gera torque de heading +
  float kf_fwd;       // N/µs²
  float kf_rev;       // N/µs²
  float dz_us;        // zona morta [µs]
};

MotorCalib motores[3] = {
  // a=BF: +X do sensor (verificado); b/c a ±120°. yaw_sign default geométrico.
  { 1.0f,  0.0f,             1.0f, 0.0f, 0.0f, DB_US },
  { -0.5f,  0.8660254f,      1.0f, 0.0f, 0.0f, DB_US },
  { -0.5f, -0.8660254f,      1.0f, 0.0f, 0.0f, DB_US },
};

void init_motores_calib() {
  float kf_fwd = F_MAX_500 / ((500.0f - DB_US) * (500.0f - DB_US));
  for (int i = 0; i < 3; i++) {
    if (motores[i].kf_fwd == 0.0f) {
      motores[i].kf_fwd = kf_fwd;
      motores[i].kf_rev = kf_fwd * REV_RATIO;
    }
  }
}

// --- Ganhos do controlador (mesmos valores/default do Python; comando G ajusta) ---
struct ControllerGains {
  float vel_kp;         // N por (m/s)
  float vel_ki;
  float vel_kd;
  float head_angle_kp;  // (rad/s) por rad
  float head_decel;     // rad/s² do perfil de frenagem
  float head_rate_limit;// rad/s
  float head_rate_kp_us;// µs por (rad/s)
  float head_rate_ki_us;// µs por rad
};

ControllerGains ganhos = {
  45.0f, 2.2f, 0.0f,
  2.5f, 1.5f, 2.5f,
  900.0f, 2000.0f
};

// Parâmetros do cabo e limites
const float L_MIN = 0.5f, L_MAX = 6.0f;
float L_M = 1.0f;  // comprimento nominal da sessão [m]

// Compensação de yaw na fusão (ver checklist: girar a esfera inclinada à mão;
// sx/sy NÃO podem girar junto. Se girarem, troque o sinal).
const float COMP_YAW_SIGN = 1.0f;

// Fusão complementar e filtros
const float ALPHA_FUSAO   = 0.88f;
const float ACC_GATE_LO   = 0.80f;  // |acc| ∈ [0,8, 1,2] g → acc confiável
const float ACC_GATE_HI   = 1.20f;
const unsigned long SWING_INTERVAL_US = 5000;   // malha de controle: 200 Hz
const unsigned long TELEMETRY_INTERVAL = 40;    // telemetria: 25 Hz

// =========================================================================================
// --- ESTADO GLOBAL ---
// =========================================================================================
AS5600 encoder;
Servo esc_a, esc_b, esc_c;

// Sensor
float gyro_bias[3] = {0, 0, 0};      // °/s
float theta_offset_x = 0, theta_offset_y = 0;  // rad (zero gravimétrico)
bool  calib_ok = false;

float sx_est = 0, sy_est = 0;        // inclinação no MUNDO [rad]
float wx_est = 0, wy_est = 0;        // taxa de inclinação no MUNDO [rad/s]
float yaw_gyro = 0;                  // [graus] (deriva; visão corrige o apontamento)
float rate_z_filt = 0;               // °/s filtrada (controle/telemetria)
float acc_mag_g = 1.0f;              // |acc| em g (gate)

// Controle
bool yaw_ativo = false;              // DESLIGADO no boot (segurança)
bool anti_balanco_ativo = false;     // DESLIGADO no boot
bool sw_hysteresis_on = false;
float F_cmd_x = 0, F_cmd_y = 0;      // N, no MUNDO
float yaw_us_cmd = 0;                // µs por motor
float vhat_x = 0, vhat_y = 0;        // v̂ = L·θ̇_mundo [m/s]
float desired_yaw = 0;               // graus (setpoint interno)

// Visão (vinda da Rasp): bearing do cabo da linha no corpo [graus] + distância D [m]
float vision_bearing = 0, vision_D = -1;
unsigned long vision_last_ms = 0;
bool vision_fresh() { return millis() - vision_last_ms < 500 && vision_last_ms > 0; }

// Wizard (pulso de identificação)
int  wizard_motor = -1;              // −1 = inativo
int  wizard_us = 0;
long wizard_remaining_ms = 0;

// Mecanismos (inalterados)
int estadoMotor = 0;
bool bateuCima = false, bateuBaixo = false;
int estadoEstavelOlhal = HIGH, ultimoEstadoFisicoOlhal = HIGH;
unsigned long ultimoTempoFiltro = 0;
const unsigned long tempoFiltro = 100;

// Medição de período natural (calibração de L)
float periodo_medido = 0;
unsigned long ultimo_cruzamento_us = 0;
bool cruzamento_armado = false;

unsigned long last_time_us = 0, last_swing_us = 0, last_telemetry_ms = 0;

// Últimos µs escritos nos ESCs (telemetria/auto-rebias/parada)
int esc_us_atual[3] = {ESC_NEUTRAL, ESC_NEUTRAL, ESC_NEUTRAL};

// =========================================================================================
// --- UTILIDADES ---
// =========================================================================================
float wrap180(float d) {
  while (d > 180.0f) d -= 360.0f;
  while (d < -180.0f) d += 360.0f;
  return d;
}
float wrap_pi(float a) {
  while (a > PI) a -= 2.0f * PI;
  while (a < -PI) a += 2.0f * PI;
  return a;
}
int16_t ler16() {
  uint8_t hi = Wire.read();
  uint8_t lo = Wire.read();
  return (int16_t)((hi << 8) | lo);
}

// =========================================================================================
// --- PID COM ANTI-WINDUP (espelha pid_controller.py) ---
// =========================================================================================
struct PID {
  float kp = 0, ki = 0, kd = 0, out_limit = 0;
  float integral = 0;
  bool  has_last = false;
  float last_meas = 0;

  void reset() { integral = 0; has_last = false; }

  float update(float meas, float setpoint, float dt) {
    float error = setpoint - meas;
    float deriv = has_last ? -(meas - last_meas) / dt : 0.0f;
    float p = kp * error;
    float d = kd * deriv;
    float inc = ki * error * dt;
    if (out_limit > 0) {
      float out_before = p + d + ki * integral;
      bool saturating = fabsf(out_before + inc) > out_limit;
      bool reduces = out_before * inc < 0;
      if (!saturating || reduces) integral += error * dt;
    } else {
      integral += error * dt;
    }
    float out = p + d + ki * integral;
    if (out_limit > 0) out = constrain(out, -out_limit, out_limit);
    last_meas = meas;
    has_last = true;
    return out;
  }
};

PID pid_vx, pid_vy, pid_rate;

void setup_pids() {
  float envelope = 0.8f * 1.5f * 3.0f *
      (F_MAX_500 / ((500.0f - DB_US) * (500.0f - DB_US))) *
      ((SWING_RESERVE_US - DB_US) * (SWING_RESERVE_US - DB_US));  // ~4,3 N
  pid_vx.kp = ganhos.vel_kp;  pid_vx.ki = ganhos.vel_ki;  pid_vx.kd = ganhos.vel_kd;
  pid_vx.out_limit = envelope;
  pid_vy = pid_vx;
  pid_rate.kp = ganhos.head_rate_kp_us;
  pid_rate.ki = ganhos.head_rate_ki_us;
  pid_rate.kd = 0;
  pid_rate.out_limit = (float)YAW_RESERVE_US;
}

// =========================================================================================
// --- MPU6050: LEITURA, CALIBRAÇÃO COM GATE DE QUIETUDE, FUSÃO COM GATE GRAVIMÉTRICO ---
// =========================================================================================
struct MpuSample {
  float gx, gy, gz;   // °/s (bias ainda não removido)
  float ax, ay, az;   // LSB (escala ±2g)
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
  ler16();  // temperatura
  int16_t gx = ler16(), gy = ler16(), gz = ler16();
  s.ax = ax; s.ay = ay; s.az = az;
  s.gx = gx / 131.0f; s.gy = gy / 131.0f; s.gz = gz / 131.0f;
  s.ok = true;
  return s;
}

void init_mpu() {
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(0x6B); Wire.write(0x00); Wire.endTransmission();  // acordar
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(0x1B); Wire.write(0x00); Wire.endTransmission();  // gyro ±250 °/s
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(0x1C); Wire.write(0x00); Wire.endTransmission();  // acc ±2g
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(0x37); Wire.write(0x02); Wire.endTransmission();  // bypass
}

// Calibração: média de 800 amostras COM GATE DE QUIETUDE — se a esfera estiver
// se movendo (variância do gyro alta), aborta em vez de calibrar errado.
bool calibrar_mpu(bool verboso) {
  if (verboso) Serial.println("=== CALIBRANDO MPU (esfera deve estar parada) ===");
  const int N = 800;
  float sx = 0, sy = 0, sz = 0, sax = 0, say = 0, saz = 0;
  float max_dev = 0;
  // pré-amostra para referência de desvio
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
    if (i % 100 == 0) { digitalWrite(LED_PIN, !digitalRead(LED_PIN)); }
    delay(2);
  }
  digitalWrite(LED_PIN, HIGH);
  if (max_dev > 3.0f) {  // > ~1 °/s de desvio por eixo: estava se movendo
    if (verboso) Serial.println("[ERRO] MPU em movimento — calibracao abortada!");
    return false;
  }
  gyro_bias[0] = sx / N; gyro_bias[1] = sy / N; gyro_bias[2] = sz / N;
  float ax0 = sax / N, ay0 = say / N, az0 = saz / N;
  theta_offset_x = atan2f(-ax0, az0);
  theta_offset_y = atan2f(ay0, az0);
  sx_est = 0; sy_est = 0; wx_est = 0; wy_est = 0;
  yaw_gyro = desired_yaw;  // recalibra = re-zero do heading
  calib_ok = true;
  if (verboso) Serial.println(">> Calibracao concluida (quietude verificada).");
  return true;
}

// Auto-rebias do gyro em repouso: se tudo parado (motores neutros, taxas ~0,
// inclinação pequena) por alguns segundos, adapta o bias lentamente. Não
// precisa de prumo — só corrige deriva térmica do bias de taxa.
void talvez_rebias(const MpuSample &s) {
  static unsigned long still_since = 0;
  bool motores_parados = (esc_us_atual[0] == ESC_NEUTRAL && esc_us_atual[1] == ESC_NEUTRAL
                          && esc_us_atual[2] == ESC_NEUTRAL);
  float vel_ang = sqrtf(wx_est * wx_est + wy_est * wy_est);
  bool quieto = motores_parados && vel_ang < 0.01f && fabsf(rate_z_filt) < 0.3f;
  unsigned long now = millis();
  if (quieto) {
    if (still_since == 0) still_since = now;
    if (now - still_since > 3000) {
      // adaptação lenta (constante de tempo ~50 s)
      gyro_bias[0] += 0.0002f * ((s.gx - gyro_bias[0]));
      gyro_bias[1] += 0.0002f * ((s.gy - gyro_bias[1]));
      gyro_bias[2] += 0.0002f * ((s.gz - gyro_bias[2]));
    }
  } else {
    still_since = 0;
  }
}

// =========================================================================================
// --- FUSÃO DE ATITUDE (complementar, gate gravimétrico, taxas em MUNDO) ---
// =========================================================================================
// Entradas: amostra do MPU (bias já removido nas taxas), dt [s].
// Saídas (globais): sx_est/sy_est [rad, MUNDO], wx_est/wy_est [rad/s, MUNDO],
//                   yaw_gyro [graus], rate_z_filt [°/s], acc_mag_g.
void passo_fusao(const MpuSample &s, float dt) {
  // Taxas do corpo [rad/s]. Convenção: giro em torno de Y inclina o eixo X;
  // giro em torno de X inclina o eixo Y. (Verificar sinais no checklist.)
  float rate_x = (s.gy - gyro_bias[1]) * (PI / 180.0f);
  float rate_y = (s.gx - gyro_bias[0]) * (PI / 180.0f);
  float wz     = (s.gz - gyro_bias[2]) * (PI / 180.0f);

  // Integração do yaw SEM deadzone (deadzone no integrador gera deriva
  // sistemática; a taxa é filtrada apenas para o controle/telemetria).
  yaw_gyro += (s.gz - gyro_bias[2]) * dt;
  if (yaw_gyro >= 360.0f) yaw_gyro -= 360.0f;
  if (yaw_gyro < 0.0f) yaw_gyro += 360.0f;
  rate_z_filt += 0.3f * ((s.gz - gyro_bias[2]) - rate_z_filt);

  // Taxas de inclinação no MUNDO (teorema do transporte — mesma correção da
  // propagação do ângulo abaixo; COMP_YAW_SIGN invertível na bancada).
  float wx_world = rate_x + COMP_YAW_SIGN * wz * sy_est;
  float wy_world = rate_y - COMP_YAW_SIGN * wz * sx_est;

  // Inclinação gravimétrica absoluta (atan2) — só confiável se |acc| ≈ 1 g
  // (vibração dos motores/acelerações rejeitadas pelo gate).
  acc_mag_g = sqrtf(s.ax * s.ax + s.ay * s.ay + s.az * s.az) / 16384.0f;
  bool acc_ok = (acc_mag_g > ACC_GATE_LO && acc_mag_g < ACC_GATE_HI);

  float th_acc_x = atan2f(-s.ax, s.az) - theta_offset_x;
  float th_acc_y = atan2f(s.ay, s.az) - theta_offset_y;

  if (acc_ok) {
    sx_est = ALPHA_FUSAO * (sx_est + wx_world * dt) + (1.0f - ALPHA_FUSAO) * th_acc_x;
    sy_est = ALPHA_FUSAO * (sy_est + wy_world * dt) + (1.0f - ALPHA_FUSAO) * th_acc_y;
  } else {
    // só gyro (a correção absoluta volta quando o gate abrir)
    sx_est += wx_world * dt;
    sy_est += wy_world * dt;
  }
  wx_est = wx_world;
  wy_est = wy_world;
}

// =========================================================================================
// --- ATUADORES: CURVA POR MOTOR + ALOCAÇÃO (espelha actuators.py, sem lag —
//     o atraso real do ESC/hélice já está no hardware) ---
// =========================================================================================
float us_from_force(int i, float f) {
  // Gate anti-travamento: o menor µs não-nulo já nasce FORA da faixa do ESC
  // (latch + margem) — comandos sub-gate viram neutro, como no simulador.
  float floor_us = ESC_LATCH_US + LATCH_MARGIN_US;
  float kf_gate = (f >= 0) ? motores[i].kf_fwd : motores[i].kf_rev;
  float du = floor_us - motores[i].dz_us;
  float gate = (du > 0) ? kf_gate * du * du : 0.0f;
  if (fabsf(f) < fmaxf(F_MIN_N, gate)) return 0.0f;
  float kf = (f > 0) ? motores[i].kf_fwd : motores[i].kf_rev;
  return (f > 0 ? 1.0f : -1.0f) *
         (motores[i].dz_us + sqrtf(fabsf(f) / kf));
}
float force_from_us(int i, float u) {
  float dz = motores[i].dz_us;
  if (fabsf(u) <= dz) return 0.0f;
  float kf = (u > 0) ? motores[i].kf_fwd : motores[i].kf_rev;
  return (u > 0 ? 1.0f : -1.0f) * kf * (fabsf(u) - dz) * (fabsf(u) - dz);
}

void escrever_escs(int us_a, int us_b, int us_c) {
  esc_us_atual[0] = us_a; esc_us_atual[1] = us_b; esc_us_atual[2] = us_c;
  esc_a.writeMicroseconds(us_a);
  esc_b.writeMicroseconds(us_b);
  esc_c.writeMicroseconds(us_c);
}

// Realização: força desejada no MUNDO + yaw_us por motor → µs nos ESCs.
//   1. F_mundo → F_corpo (Rz(−ψ))  [correção vs firmware antigo]
//   2. f_i = (2/3)(t_i·F_corpo), reescala λ p/ caber na reserva
//   3. u_i = us_from_force(f_i) + yaw_sign_i·yaw_us; clamp ±250
void realizar_controle(float Fwx, float Fwy, float yaw_us, float psi) {
  float c = cosf(psi), s = sinf(psi);
  float Fbx =  c * Fwx + s * Fwy;   // Rz(−ψ)·F_mundo
  float Fby = -s * Fwx + c * Fwy;

  float f_des[3] = {
    (2.0f / 3.0f) * (motores[0].tx * Fbx + motores[0].ty * Fby),
    (2.0f / 3.0f) * (motores[1].tx * Fbx + motores[1].ty * Fby),
    (2.0f / 3.0f) * (motores[2].tx * Fbx + motores[2].ty * Fby),
  };

  // Reescala proporcional dentro da reserva (prioriza direção)
  float lam = 1.0f;
  for (int i = 0; i < 3; i++) {
    float kf = (f_des[i] >= 0) ? motores[i].kf_fwd : motores[i].kf_rev;
    float cap = kf * (SWING_RESERVE_US - motores[i].dz_us) * (SWING_RESERVE_US - motores[i].dz_us);
    if (fabsf(f_des[i]) > 1e-6f) lam = fminf(lam, cap / fabsf(f_des[i]));
  }

  int us[3];
  float floor_us = ESC_LATCH_US + LATCH_MARGIN_US;
  for (int i = 0; i < 3; i++) {
    float u = us_from_force(i, f_des[i] * lam) + motores[i].yaw_sign * yaw_us;
    // Quantização anti-banda no µs FINAL (swing+yaw): a soma pode cair dentro
    // da faixa de travamento — 0 ou ≥ piso, nunca dentro (paridade com o
    // _actuate do Python; o caminho do wizard NÃO quantiza, de propósito).
    if (u > 0.0f && u < floor_us) u = 0.0f;
    else if (u < 0.0f && u > -floor_us) u = 0.0f;
    us[i] = ESC_NEUTRAL + (int)constrain(lroundf(u), -PWM_LIMIT_US, PWM_LIMIT_US);
  }
  escrever_escs(us[0], us[1], us[2]);
}

// =========================================================================================
// --- CONTROLE (espelha controller.py) ---
// =========================================================================================
// Anti-balanço: PID de VELOCIDADE estimada v̂ = L·θ̇_mundo (mesmo significado
// de ganho do simulador; θ̇_mundo vem da fusão). Sem lead predictor: o
// simulador validou o PID de velocidade com o atraso de 120 ms modelado —
// a malha não precisa de antecipação (o termo D do PID cumpre o papel).
void passo_controle_xy(float dt) {
  if (!anti_balanco_ativo) {
    F_cmd_x = F_cmd_y = 0;
    pid_vx.reset(); pid_vy.reset();
    sw_hysteresis_on = false;
    return;
  }

  vhat_x = L_M * wx_est;   // v̂ ≈ L·θ̇ (pequenos ângulos)
  vhat_y = L_M * wy_est;

  F_cmd_x = pid_vx.update(vhat_x, 0.0f, dt);
  F_cmd_y = pid_vy.update(vhat_y, 0.0f, dt);

  // Histerese anti-chatter (comando ínfimo → neutro)
  float norm_F = sqrtf(F_cmd_x * F_cmd_x + F_cmd_y * F_cmd_y);
  float limiar = sw_hysteresis_on ? 0.5f * F_MIN_N : 2.0f * F_MIN_N;
  sw_hysteresis_on = (norm_F > limiar);
  if (!sw_hysteresis_on) { F_cmd_x = 0; F_cmd_y = 0; }
}

// Heading: cascata apontamento → perfil de taxa com frenagem → PI de taxa (µs).
void passo_controle_yaw(float dt) {
  if (!yaw_ativo) {
    yaw_us_cmd = 0;
    pid_rate.reset();
    return;
  }
  float angle_error;
  if (vision_fresh()) {
    // Câmeras fixas ao sensor: o bearing do cabo da linha no corpo É o erro
    // de apontamento — absoluto, imune à deriva do gyro.
    angle_error = vision_bearing * (PI / 180.0f);
  } else {
    angle_error = wrap_pi(desired_yaw - yaw_gyro) * (PI / 180.0f);
  }

  // Perfil de taxa: min(kp·|e|, √(2·a·|e|), limite) — a distância de frenagem
  // nunca excede o erro restante (elimina sobressinal por construção).
  float abs_err = fabsf(angle_error);
  float rate_sp = fminf(fminf(
      ganhos.head_angle_kp * abs_err,
      sqrtf(2.0f * ganhos.head_decel * abs_err)),
      ganhos.head_rate_limit);
  if (angle_error < 0) rate_sp = -rate_sp;

  float rate_meas = rate_z_filt * (PI / 180.0f);
  yaw_us_cmd = pid_rate.update(rate_meas, rate_sp, dt);
}

// =========================================================================================
// --- MECANISMOS: POLIA / PINÇA / OLHAL / AS5600 (inalterados do firmware original) ---
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
  Serial.println("  Observe qual helice gira e qual o sentido do fluxo!      ");
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
  yaw_ativo = anti_balanco_ativo = false;  // segurança durante o teste
  for (int i = 0; i < 3; i++) {
    Serial.print(">>> TESTANDO "); Serial.print(lista[i].nome);
    Serial.print(" no pino ");     Serial.println(lista[i].pino);
    Serial.println("  [1/2] +120 us (horario) por 2 s...");
    digitalWrite(LED_PIN, LOW);
    lista[i].esc->writeMicroseconds(ESC_NEUTRAL + PWM_TEST_DELTA);
    delay(2000);
    lista[i].esc->writeMicroseconds(ESC_NEUTRAL);
    digitalWrite(LED_PIN, HIGH);
    delay(1000);
    Serial.println("  [2/2] -120 us (anti-horario) por 2 s...");
    digitalWrite(LED_PIN, LOW);
    lista[i].esc->writeMicroseconds(ESC_NEUTRAL - PWM_TEST_DELTA);
    delay(2000);
    lista[i].esc->writeMicroseconds(ESC_NEUTRAL);
    digitalWrite(LED_PIN, HIGH);
    delay(1000);
  }
  yaw_ativo = yaw_antes; anti_balanco_ativo = swing_antes;
  Serial.println("  TESTE CONCLUIDO!\n");
  Serial2.println("[TESTE_MOTORES]: Concluido!");
}

// =========================================================================================
// --- COMANDOS (parser unificado, mesmas regras nas duas portas) ---
// =========================================================================================
// Ganhos endereçáveis por índice (comando G/R) — mesma ordem do twin.
const char *GAIN_NAMES[8] = {
  "vel_kp", "vel_ki", "vel_kd", "head_angle_kp",
  "head_decel", "head_rate_limit", "head_rate_kp_us", "head_rate_ki_us"
};
float *gain_ptr(int idx) {
  switch (idx) {
    case 0: return &ganhos.vel_kp;         case 1: return &ganhos.vel_ki;
    case 2: return &ganhos.vel_kd;         case 3: return &ganhos.head_angle_kp;
    case 4: return &ganhos.head_decel;     case 5: return &ganhos.head_rate_limit;
    case 6: return &ganhos.head_rate_kp_us; case 7: return &ganhos.head_rate_ki_us;
  }
  return nullptr;
}
void set_gains_from_struct() {
  pid_vx.kp = ganhos.vel_kp;  pid_vx.ki = ganhos.vel_ki;  pid_vx.kd = ganhos.vel_kd;
  pid_vy = pid_vx;
  pid_rate.kp = ganhos.head_rate_kp_us;
  pid_rate.ki = ganhos.head_rate_ki_us;
}

void parada_geral() {
  yaw_ativo = false;
  anti_balanco_ativo = false;
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
    case 'm': testarIdentificacaoMotores(); break;  // 'M' maiúsculo = calibração de motor (linha)
    case 'y': case 'Y':
      yaw_ativo = !yaw_ativo;
      Serial.println(yaw_ativo ? ">> [YAW]: ATIVADO" : ">> [YAW]: DESATIVADO");
      Serial2.println(yaw_ativo ? ">> [YAW]: ATIVADO" : ">> [YAW]: DESATIVADO");
      if (!yaw_ativo) { pid_rate.reset(); yaw_us_cmd = 0; }
      break;
    case 'b': case 'B': case 'p': case 'P':
      anti_balanco_ativo = !anti_balanco_ativo;
      Serial.println(anti_balanco_ativo ? ">> [PENDULO]: ANTI-BALANCO ATIVADO"
                                        : ">> [PENDULO]: ANTI-BALANCO DESATIVADO");
      Serial2.println(anti_balanco_ativo ? ">> [PENDULO]: ATIVADO" : ">> [PENDULO]: DESATIVADO");
      if (!anti_balanco_ativo) { F_cmd_x = F_cmd_y = 0; pid_vx.reset(); pid_vy.reset(); }
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
    case '0': desired_yaw = 0; pid_rate.reset(); Serial.println(">> [YAW]: desejado = 0"); break;
  }
}

// Comandos em linha (terminados por \n). Retorna mensagem curta p/ as duas portas.
void comando_linha(const String &cmd) {
  if (cmd.length() == 0) return;

  // L<val>: comprimento do cabo
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
  // G<idx> <val>: ajusta ganho
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
  // R: devolve todos os ganhos
  if (cmd[0] == 'R') {
    for (int i = 0; i < 8; i++) {
      String s = "GAIN," + String(i) + "," + GAIN_NAMES[i] + "," +
                 String(*gain_ptr(i), 4);
      Serial.println(s); Serial2.println(s);
    }
    return;
  }
  // V<bearing> <D>: visão (câmeras na Rasp, olhando o cabo da linha p/ baixo)
  if (cmd[0] == 'V' && cmd.length() > 1) {
    int space = cmd.indexOf(' ');
    if (space > 1) {
      vision_bearing = cmd.substring(1, space).toFloat();
      vision_D = cmd.substring(space + 1).toFloat();
      vision_last_ms = millis();
    }
    return;
  }
  // W<motor> <us> <ms>: pulso de identificação (wizard) — loops off durante
  if (cmd[0] == 'W' && cmd.length() > 1) {
    int s1 = cmd.indexOf(' ');
    int s2 = cmd.indexOf(' ', s1 + 1);
    if (s1 > 1 && s2 > s1) {
      int m = cmd.substring(1, s1).toInt();
      int us = cmd.substring(s1 + 1, s2).toInt();
      long ms = cmd.substring(s2 + 1).toInt();
      if (m >= 0 && m < 3 && fabsf(us) <= PWM_LIMIT_US && ms > 0 && ms <= 3000) {
        yaw_ativo = false; anti_balanco_ativo = false;
        wizard_motor = m; wizard_us = us; wizard_remaining_ms = ms;
        Serial.print(">> [WIZARD] motor "); Serial.print(m);
        Serial.print(" pulso "); Serial.print(us);
        Serial.print(" us por "); Serial.print(ms); Serial.println(" ms");
      }
    }
    return;
  }
  // M<motor> <tx> <ty> <yaw_sign> <kf_fwd> <kf_rev> <dz>: calibração por motor
  if (cmd[0] == 'M' && cmd.length() > 1) {
    int fields[6];  // posições dos 6 espaços separadores
    int fi = 0;
    for (unsigned int i = 0; i < cmd.length() && fi < 6; i++) {
      if (cmd[i] == ' ') fields[fi++] = (int)i;
    }
    if (fi == 6) {
      int m = cmd.substring(1, fields[0]).toInt();
      if (m >= 0 && m < 3) {
        float tx = cmd.substring(fields[0] + 1, fields[1]).toFloat();
        float ty = cmd.substring(fields[1] + 1, fields[2]).toFloat();
        float ys = cmd.substring(fields[2] + 1, fields[3]).toFloat();
        float kf = cmd.substring(fields[3] + 1, fields[4]).toFloat();
        float kr = cmd.substring(fields[4] + 1, fields[5]).toFloat();
        float dz = cmd.substring(fields[5] + 1).toFloat();
        float norm = sqrtf(tx * tx + ty * ty);
        if (norm > 1e-6f && kf > 0 && kr > 0 && dz >= 0) {
          motores[m].tx = tx / norm; motores[m].ty = ty / norm;
          motores[m].yaw_sign = (ys >= 0) ? 1.0f : -1.0f;
          motores[m].kf_fwd = kf; motores[m].kf_rev = kr;
          motores[m].dz_us = dz;
          Serial.print(">> [MOTOR "); Serial.print(m);
          Serial.print("] calibrado: t=(");
          Serial.print(motores[m].tx, 3); Serial.print(", ");
          Serial.print(motores[m].ty, 3); Serial.print(") yaw_sign=");
          Serial.print(motores[m].yaw_sign, 0); Serial.println(")");
        }
      }
    }
    return;
  }
  // <número>: yaw desejado (setpoint interno, graus)
  char c0 = cmd[0];
  if (isdigit(c0) || c0 == '-' || c0 == '+') {
    float ang = cmd.toFloat();
    yaw_gyro = 0;  // novo setpoint absoluto = re-zero (comportamento original)
    desired_yaw = ang;
    pid_rate.reset();
    String msg = ">> NOVO ANGULO APLICADO: " + String(desired_yaw, 2);
    Serial.println(msg); Serial2.println(msg);
    return;
  }
  // caractere único conhecido (compatibilidade: sem newline)
  if (cmd.length() == 1) comando_char(cmd[0]);
}

// Parser não bloqueante: chars da whitelist executam imediatamente (comportamento
// original); o resto acumula em linha até \n. MESMA lógica nas duas portas.
const char WHITELIST[] = "wWsSxX aAdDcCeEmYyBbPpKkZzTt0";
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
      if (buf.length() > 0) { comando_linha(buf); buf = ""; }
    } else if (buf.length() == 0 && eh_comando_imediato(c)) {
      comando_char(c);
    } else {
      buf += c;
      if (buf.length() > 60) buf = "";  // proteção
    }
  }
}

// =========================================================================================
// --- TELEMETRIA (25 Hz, ambas as portas) ---
// =========================================================================================
void enviar_telemetria() {
  unsigned flags = (yaw_ativo ? 1 : 0) | (anti_balanco_ativo ? 2 : 0) |
                   (calib_ok ? 4 : 0) | (vision_fresh() ? 8 : 0) |
                   (wizard_motor >= 0 ? 16 : 0);
  char line[160];
  snprintf(line, sizeof(line),
    "TEL,%lu,%.2f,%.2f,%.2f,%.2f,%.3f,%.3f,%.3f,%.3f,%d,%d,%d,%.2f,%.2f,%u",
    millis(),
    yaw_gyro,
    vision_fresh() ? vision_bearing : 0.0f,
    desired_yaw,
    rate_z_filt,
    sx_est * 57.2958f, sy_est * 57.2958f,
    wx_est * 57.2958f, wy_est * 57.2958f,
    esc_us_atual[0], esc_us_atual[1], esc_us_atual[2],
    L_M, vision_fresh() ? vision_D : -1.0f,
    flags);
  Serial.println(line);
  Serial2.println(line);
}

// =========================================================================================
// --- SETUP / LOOP ---
// =========================================================================================
void setup() {
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);

  esc_a.attach(BF_PIN);
  esc_b.attach(AS_PIN);
  esc_c.attach(TR_PIN);
  escrever_escs(ESC_NEUTRAL, ESC_NEUTRAL, ESC_NEUTRAL);
  init_motores_calib();
  setup_pids();

  Serial.begin(115200);
  Serial2.setRx(PA3);
  Serial2.setTx(PA2);
  Serial2.begin(115200);

  destravar_barramento_I2C();
  Wire.setSCL(AS5600_SCL);
  Wire.setSDA(AS5600_SDA);
  Wire.begin();
  Wire.setClock(400000);   // correção #8: MPU6050 suporta 400 kHz
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

  yaw_gyro = 0;
  last_time_us = micros();
  last_swing_us = micros();
  last_telemetry_ms = millis();

  Serial.println("==========================================================");
  Serial.println(" ESFERA v2 — controle em paridade com o twin (Python)");
  Serial.println("  PINCA:   'w'=Subir | 's'=Descer | 'x'=Parar            ");
  Serial.println("  POLIA:   'a'=Giro A| 'd'=Giro D | 'c'=Parar            ");
  Serial.println("  AS5600:  'e'=Ler Angulo                                 ");
  Serial.println("  YAW:     'y'=on/off | <graus>=setpoint | '0'=zero       ");
  Serial.println("  PENDULO: 'p'=anti-balanco on/off | 'z'=calibra MPU      ");
  Serial.println("           't'=L pelo periodo | 'L1.5'=L manual | 'm'=teste");
  Serial.println("  GANHOS:  'G<idx> <val>' | 'R' ler | VISAO: 'V<b> <D>'   ");
  Serial.println("  PARADA GERAL: 'k'   (loops OFF no boot por seguranca)   ");
  Serial.println("==========================================================");
}

void loop() {
  unsigned long now_us = micros();
  float dt = (now_us - last_time_us) * 1e-6f;
  last_time_us = now_us;
  if (dt <= 1e-6f || dt > 0.5f) dt = 0.005f;

  processar_serial(Serial, bufUSB);
  processar_serial(Serial2, bufS2);

  MpuSample s = ler_mpu();

  // --- malha de 200 Hz: fusão + (wizard OU controle), sempre na mesma cadência
  // (o ALPHA_FUSAO é sintonizado para dt = 5 ms — rodar a fusão em outra taxa
  // mudaria o filtro) ---
  if (now_us - last_swing_us >= SWING_INTERVAL_US) {
    float dt_swing = (now_us - last_swing_us) * 1e-6f;
    last_swing_us = now_us;

    if (s.ok) {
      passo_fusao(s, dt_swing);
      talvez_rebias(s);
    }

    if (wizard_motor >= 0) {
      // Pulso de identificação tem prioridade; loops já foram desligados.
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
      realizar_controle(F_cmd_x, F_cmd_y, yaw_us_cmd, yaw_gyro * (PI / 180.0f));
    }

    // Medição de período em oscilação livre (calibração de L)
    if (!anti_balanco_ativo && wizard_motor < 0) {
      float vel_ang = sqrtf(wx_est * wx_est + wy_est * wy_est);
      if (vel_ang > (2.0f * PI / 180.0f)) cruzamento_armado = true;
      if (cruzamento_armado && vel_ang < (0.5f * PI / 180.0f)) {
        cruzamento_armado = false;
        unsigned long t_cruz = micros();
        float T_amostra = (t_cruz - ultimo_cruzamento_us) * 2e-6f;
        ultimo_cruzamento_us = t_cruz;
        if (T_amostra > 1.2f && T_amostra < 6.0f) {
          periodo_medido = (periodo_medido == 0) ? T_amostra
                          : 0.7f * periodo_medido + 0.3f * T_amostra;
        }
      }
    }
  }

  // --- telemetria 25 Hz ---
  if (millis() - last_telemetry_ms >= TELEMETRY_INTERVAL) {
    last_telemetry_ms = millis();
    enviar_telemetria();
  }

  // --- debounce do sensor óptico do olhal (inalterado) ---
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

  // --- segurança fins de curso da pinça (inalterado) ---
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
