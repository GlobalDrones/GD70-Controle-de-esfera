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
//
//  REVISAO 14/10/2026 (AUDITORIA_CONTROLE.md — itens SC-xx citados no codigo):
//    - SC-01: IMU -> corpo por ROTACAO (IMU_MOUNT_DEG), nao mais por troca de eixos.
//             A troca antiga (rate_x=+gy, rate_y=+gx) era um espelhamento: um eixo do
//             anti-balanco tinha realimentacao POSITIVA. Validar com o teste de eixos.
//    - SC-07: velocidade no corpo direto do gyro (sem termo de transporte w x s).
//    - SC-02: estimador opcional da velocidade do CG (acelerometro no pino + gyro), G8.
//    - SC-03: alocacao de forca XY + torque de yaw JUNTOS no dominio de forca.
//    - SC-09: heading controlado em TORQUE [N.m] (ganhos G3..G7 mudaram de unidade).
//    - SC-04: calibracao do gyro de ~9 s com gate real (z recusa movimento; Z forca).
//    - SC-05: zona morta de integracao do yaw 0,35 -> 0,05 deg/s; rebias mais estrito.
//    - SC-06: visao por evento, com compensacao de latencia.
//    - SC-10/14/15/19: '0' fora da whitelist, dt limitado, falha da MPU, entradas validadas.
//    - TEL ganhou 5 campos (F_cmd_x, F_cmd_y, Tz, vhat_x, vhat_y) e novas flags.
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

// Parametros da malha de Yaw (SC-09: controle em torque; a alocacao lineariza o atuador)
const float YAW_DEADZONE_DEG = 1.0f;  // Zona morta angular [graus] (era 2.5 com o PID em us)
const float YAW_RESUME_DEG   = 1.5f;  // Histerese para reativar o Yaw [graus]
// Travamento direcional do ESC: |u| abaixo disto nao gira (README ~60 us). Comandos
// por motor menores que isso vao a neutro em vez de ficarem "presos" na faixa morta.
const float ESC_LATCH_US     = 60.0f; // [us] — medir na bancada e ajustar

// Visao (SC-06): atraso tipico captura->processamento->serial [s]. O V pode trazer a
// idade do quadro como 3o campo (ms); sem ele, usa esta constante. Medir na bancada.
const float VISION_LATENCY_S = 0.10f;

// Montagem da MPU (SC-01). Convencao do sistema: +X do corpo = direcao em que o motor a
// EMPURRA a esfera; +Y = 90 graus anti-horario visto de cima. IMU_MOUNT_DEG = angulo do
// eixo X da MPU medido a partir do +X do corpo (anti-horario visto de cima), com o Z da
// MPU para CIMA. Determinar pelo teste de eixos do docs/BRINGUP.md (Etapa 3).
// Com a MPU de Z para cima nao existe montagem que exija espelhar eixos: se o teste so
// fechar com um espelhamento, a MPU esta de cabeca para baixo (IMU_Z_UP = false).
const float IMU_MOUNT_DEG = 0.0f;
const bool  IMU_Z_UP      = true;

// Zona morta de repouso no prumo (Anti-Balanco)
// Com 0.60/0.85/1.50 sobrava um balanco residual de ~6 cm (L = 5 m) sem controle; apertado
// para ~3 cm. Se aparecer liga/desliga dos motores perto do prumo, voltar a abrir.
const float THETA_DEADZONE_DEG = 0.35f; // Entra em repouso se inclinacao < 0.35°
const float THETA_RESUME_DEG   = 0.55f; // Reativa controle se inclinacao > 0.55°
const float OMEGA_DEADZONE_DPS = 1.00f; // Limiar de velocidade angular p/ repouso (< 1.00 °/s)

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
// Indices do comando G (o supervisorio usa os NOMES; o twin Python tem semantica propria
// para 3..7 e os valores dele sao recusados pela validacao de faixa abaixo):
//   0: vel_kp       -> Ganho de velocidade XY [N / (m/s)] (amortecimento principal)
//   1: vel_ki       -> Integral de velocidade XY (0 por padrao; zera a cada inversao de sinal)
//   2: vel_kd       -> Derivativo de velocidade XY (0 por padrao)
//   3: yaw_kp       -> Kp do heading [N.m / rad]           (SC-09: antes era us/grau)
//   4: yaw_ki       -> Ki do heading [N.m / (rad.s)]
//   5: yaw_kd       -> Kd do heading [N.m / (rad/s)]
//   6: yaw_tz_max   -> Torque maximo de heading [N.m]
//   7: yaw_int_max  -> Limite da integral do heading [rad.s]
//   8: vel_mode     -> Fonte da velocidade XY: 0 = L*gyro, 1 = estimador do CG (SC-02)
//   9: d_pino_cg    -> Distancia pino -> CG [m] (usada pelo estimador)
// Ponto de partida do heading para Iz ~ 1,73 kg.m2: wn ~ 2,4 rad/s, zeta ~ 0,85.
struct ControllerGains {
  float vel_kp;
  float vel_ki;
  float vel_kd;
  float yaw_kp;
  float yaw_ki;
  float yaw_kd;
  float yaw_tz_max;
  float yaw_int_max;
  float vel_mode;
  float d_pino_cg;
};

ControllerGains ganhos = {
  18.0f,  // G0: vel_kp [N/(m/s)]
  0.0f,   // G1: vel_ki
  0.0f,   // G2: vel_kd
  10.0f,  // G3: yaw_kp [N.m/rad]
  0.0f,   // G4: yaw_ki [N.m/(rad.s)] (cabo sem torcao: integral desnecessaria por ora)
  7.0f,   // G5: yaw_kd [N.m.s/rad]
  8.0f,   // G6: yaw_tz_max [N.m]
  0.5f,   // G7: yaw_int_max [rad.s]
  0.0f,   // G8: vel_mode (0 = gyro; 1 = estimador do CG — testar na bancada)
  0.10f   // G9: d_pino_cg [m] (CG logo abaixo do pino; medir)
};
const int N_GAINS = 10;

// Parametros do cabo (L em metros)
const float L_MIN = 0.5f, L_MAX = 6.0f;
float L_M = 5.0f; // Comprimento nominal ajustado para 5 metros (ajustavel via L<val> ou 't')

// (SC-07) O termo de transporte w x s saiu da VELOCIDADE: a forca e aplicada no CORPO e a
// velocidade no corpo sai direto do gyro. Ele continua (com o sinal certo) so na integracao
// da atitude, que precisa girar junto com o corpo.

// Constantes da Fusao Complementar e Filtros Passa-Baixas
// Para L = 5.0 m, o periodo do pendulo e ~4.5 s (f ~ 0.22 Hz).
// ALPHA_FUSAO = 0.9995f define tau ~ 10 s @ 200 Hz: o balanco e 99.7% integrado do gyro
// sem ruido, eliminando cruzamentos falsos por zero, e o acc ancora o bias lentamente.
const float ALPHA_FUSAO     = 0.9995f; // Filtro complementar de inclinacao (tau ~ 10 s @ 200 Hz)
const float ALPHA_RATE_XY   = 0.22f;   // Filtro IIR (~9 Hz) p/ velocidades angulares wx e wy
const float ALPHA_RATE_Z    = 0.20f;   // Filtro IIR p/ taxa de Yaw
const float GYRO_Z_DEADZONE = 0.05f;   // (SC-05) era 0.35: escondia ate 21 graus/min de giro real [°/s]
const float ACC_GATE_LO     = 0.80f; // Limite inferior de confianca do acelerometro [g]
const float ACC_GATE_HI     = 1.20f; // Limite superior de confianca do acelerometro [g]

// Calibracao do gyro (SC-04): janela >= 2 periodos do pendulo (T ~ 4,5 s a L = 5 m) e
// gate por desvio-padrao de cada eixo. 'z' recusa se houver movimento; 'Z' forca.
const unsigned long CALIB_MS      = 9000;
const float CALIB_MAX_STD_DPS     = 0.30f;
// Estimador da velocidade do CG (SC-02): integrador com vazamento da aceleracao do pino.
const float TAU_VPIN_S            = 8.0f;
// Falha da MPU (SC-15): apos N leituras seguidas sem resposta, malhas off + ESCs neutro.
const int   MPU_FAIL_LIMIT        = 10;

const unsigned long SWING_INTERVAL_US  = 5000; // Malha de controle e fusao: 200 Hz (5 ms)
const unsigned long TELEMETRY_INTERVAL = 40;   // Telemetria: 25 Hz (40 ms)

// =========================================================================================
// --- ESTADO GLOBAL ---
// =========================================================================================
AS5600 encoder;
Servo esc_a, esc_b, esc_c;

// Estado do Sensor MPU6050
float gyro_bias[3] = {0, 0, 0};     // Offsets do giroscopio [°/s] (eixos da MPU)
float theta_offset_x = 0.0f;        // Zero gravimetrico em X [rad]
float theta_offset_y = 0.0f;        // Zero gravimetrico em Y [rad]
bool  calib_ok = false;

float sx_est = 0.0f, sy_est = 0.0f; // Atitude da esfera no CORPO [rad] (sx>0: deslocada p/ +X)
float wx_est = 0.0f, wy_est = 0.0f; // d(sx)/dt, d(sy)/dt filtradas, no CORPO [rad/s]
float vpin_x = 0.0f, vpin_y = 0.0f; // Velocidade do pino (estimador SC-02) [m/s]
float vcg_x = 0.0f, vcg_y = 0.0f;   // Velocidade do CG estimada (SC-02) [m/s]
int   mpu_falhas = 0;               // Leituras seguidas sem resposta (SC-15)
bool  mpu_falhou = false;           // Latch de falha (limpa em 'z'/'Z' bem-sucedido)
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

float F_cmd_x = 0.0f, F_cmd_y = 0.0f; // Forca de anti-balanco no CORPO [N]
float vhat_x = 0.0f, vhat_y = 0.0f;   // Velocidade realimentada, no CORPO [m/s]
float tz_cmd = 0.0f;                  // Torque de heading comandado [N.m]
float desired_yaw = 0.0f;             // Setpoint de Yaw [graus]
float pos_error_yaw = 0.0f;           // Erro atual de Yaw [graus]
float integral_error_yaw = 0.0f;      // Acumulador integral do Yaw
float last_error_yaw = 0.0f;          // Ultimo erro de Yaw (para deteccao de cruzamento de zero)

// Visao Computacional (recebida da Raspberry Pi via V<bearing> <D>)
float vision_bearing = 0.0f, vision_D = -1.0f;
bool  vision_novo = false;          // Chegou V desde a ultima iteracao do yaw (SC-06)
float vision_idade_s = VISION_LATENCY_S;
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
// (SC-19) fmodf: sem laco infinito para entradas enormes; NaN/inf viram 0.
float wrap180(float d) {
  if (!isfinite(d)) return 0.0f;
  d = fmodf(d + 180.0f, 360.0f);
  if (d < 0.0f) d += 360.0f;
  return d - 180.0f;
}

// MPU -> corpo (SC-01): rotacao pelo angulo de montagem (e inversao de Y se Z p/ baixo).
const float IMU_C = cosf(IMU_MOUNT_DEG * (PI / 180.0f));
const float IMU_S = sinf(IMU_MOUNT_DEG * (PI / 180.0f));
inline void imu_para_corpo(float ix, float iy, float &bx, float &by) {
  if (!IMU_Z_UP) iy = -iy;   // Z invertido: (x, -y, -z) e o mesmo triedro girado 180 em X
  bx = IMU_C * ix - IMU_S * iy;
  by = IMU_S * ix + IMU_C * iy;
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
// Leitura bruta nos eixos da MPU. A conversao para o corpo acontece em passo_fusao()
// (depois de tirar o bias, que e medido nos eixos da propria MPU).
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

// (SC-04) Calibracao do gyro e do zero gravimetrico.
// - Janela de CALIB_MS (~2 periodos do pendulo): um balanco residual se cancela na media.
//   Com a janela antiga (~1,4 s) 1 grau de balanco deixava ~1 grau/s de bias, que virava
//   ~1,6 N de empurrao constante (vel_kp = 18, L = 5 m) e deriva de heading.
// - Gate real: desvio-padrao de cada eixo do gyro < CALIB_MAX_STD_DPS. Se reprovar e
//   forcar == false, mantem a calibracao anterior e retorna false.
// - Nao bloqueia para sempre se a MPU sumir (limite de falhas).
// Chamador deve desligar as malhas antes (comando 'z' faz parada_geral()).
bool calibrar_mpu(bool verboso, bool forcar) {
  if (verboso) {
    Serial.println("=== CALIBRANDO MPU6050 (~9 s, mantenha a esfera PARADA, sem balancar) ===");
  }
  double sgx = 0, sgy = 0, sgz = 0, sgx2 = 0, sgy2 = 0, sgz2 = 0;
  double sax = 0, say = 0, saz = 0;
  long n = 0;
  int falhas = 0;
  unsigned long t0 = millis(), t_led = t0;
  while (millis() - t0 < CALIB_MS) {
    MpuSample s = ler_mpu();
    if (!s.ok) {
      if (++falhas > 200) {
        if (verboso) Serial.println("[ERRO] MPU nao responde — calibracao abortada.");
        digitalWrite(LED_PIN, HIGH);
        return false;
      }
      delay(2);
      continue;
    }
    sgx += s.gx; sgy += s.gy; sgz += s.gz;
    sgx2 += (double)s.gx * s.gx; sgy2 += (double)s.gy * s.gy; sgz2 += (double)s.gz * s.gz;
    sax += s.ax; say += s.ay; saz += s.az;
    n++;
    if (millis() - t_led > 150) { digitalWrite(LED_PIN, !digitalRead(LED_PIN)); t_led = millis(); }
    delay(2);
  }
  digitalWrite(LED_PIN, HIGH);
  if (n < 100) {
    if (verboso) Serial.println("[ERRO] Amostras insuficientes — calibracao abortada.");
    return false;
  }

  float mgx = sgx / n, mgy = sgy / n, mgz = sgz / n;
  float sdx = sqrtf(fmaxf(0.0f, (float)(sgx2 / n) - mgx * mgx));
  float sdy = sqrtf(fmaxf(0.0f, (float)(sgy2 / n) - mgy * mgy));
  float sdz = sqrtf(fmaxf(0.0f, (float)(sgz2 / n) - mgz * mgz));
  float sd_max = fmaxf(sdx, fmaxf(sdy, sdz));
  if (verboso) {
    char a[16], b[16], c[16];
    fmt_f3(a, sdx); fmt_f3(b, sdy); fmt_f3(c, sdz);
    Serial.print(">> desvio-padrao gyro [deg/s]: "); Serial.print(a); Serial.print(" ");
    Serial.print(b); Serial.print(" "); Serial.println(c);
  }
  if (sd_max > CALIB_MAX_STD_DPS && !forcar) {
    if (verboso) {
      Serial.println("[RECUSADO] Esfera em movimento durante a calibracao. Pare o balanco e repita 'z'");
      Serial.println("           (ou 'Z' para forcar, sabendo que o bias pode sair errado).");
    }
    return false;
  }
  if (sd_max > CALIB_MAX_STD_DPS && verboso) {
    Serial.println("[AVISO] Calibracao FORCADA com movimento: bias pode estar errado.");
  }

  gyro_bias[0] = mgx;
  gyro_bias[1] = mgy;
  gyro_bias[2] = mgz;

  // Zero gravimetrico no CORPO (mesma convencao de passo_fusao)
  float abx, aby;
  imu_para_corpo((float)(sax / n), (float)(say / n), abx, aby);
  float abz = IMU_Z_UP ? (float)(saz / n) : -(float)(saz / n);
  theta_offset_x = atan2f(abx, abz);
  theta_offset_y = atan2f(aby, abz);

  sx_est = 0; sy_est = 0; wx_est = 0; wy_est = 0;
  vpin_x = vpin_y = vcg_x = vcg_y = 0.0f;
  rate_z_filt = 0;
  yaw_gyro = desired_yaw;
  integral_error_yaw = 0;
  mpu_falhas = 0;
  mpu_falhou = false;
  calib_ok = true;
  last_swing_us = micros();   // (SC-14) a proxima iteracao nao integra os ~9 s parados
  last_time_us = last_swing_us;
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
  // (SC-05) limiar de yaw 0,4 -> 0,25 deg/s: o rebias absorvia rotacao lenta real no bias
  bool quieto = calib_ok && motores_parados && (vel_ang < 0.015f) && (fabsf(rate_z_filt) < 0.25f);
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
  // 1. Gyro sem bias, nos eixos da MPU -> corpo (SC-01)
  float gbx, gby;
  imu_para_corpo(s.gx - gyro_bias[0], s.gy - gyro_bias[1], gbx, gby);
  float raw_yaw_rate_dps = IMU_Z_UP ? (s.gz - gyro_bias[2]) : -(s.gz - gyro_bias[2]);

  // Cinematica do pendulo (Z para cima): rotacao +Y do corpo leva o CG para -X; rotacao +X
  // leva o CG para +Y. Logo, com a convencao sx>0 = deslocado para +X:
  //   d(sx)/dt = -w_y      d(sy)/dt = +w_x      v_corpo = L * (d(sx)/dt, d(sy)/dt)
  // (O firmware anterior usava (+w_y, +w_x): espelhamento, com um eixo invertido.)
  // (SC-07) Sem termo de transporte w x s: forca e velocidade sao ambas do CORPO.
  float rate_x = -gby * (PI / 180.0f);
  float rate_y =  gbx * (PI / 180.0f);

  // 2. Filtro passa-baixas na taxa de Yaw (alpha = 0.20, ~7 Hz) — usado no D e na integracao
  rate_z_filt = ALPHA_RATE_Z * raw_yaw_rate_dps + (1.0f - ALPHA_RATE_Z) * rate_z_filt;

  // Integracao do Yaw (zona morta minima: SC-05)
  float rate_z_int = (fabsf(rate_z_filt) < GYRO_Z_DEADZONE) ? 0.0f : rate_z_filt;
  yaw_gyro += rate_z_int * dt;
  if (yaw_gyro >= 360.0f) yaw_gyro -= 360.0f;
  if (yaw_gyro < 0.0f)    yaw_gyro += 360.0f;

  // 3. Passa-baixas IIR (~8 Hz) nas taxas de inclinacao (ganho 1 em regime)
  wx_est = (1.0f - ALPHA_RATE_XY) * wx_est + ALPHA_RATE_XY * rate_x;
  wy_est = (1.0f - ALPHA_RATE_XY) * wy_est + ALPHA_RATE_XY * rate_y;

  // 4. Acelerometro no corpo
  float abx, aby;
  imu_para_corpo(s.ax, s.ay, abx, aby);
  float abz = IMU_Z_UP ? s.az : -s.az;
  acc_mag_g = sqrtf(abx * abx + aby * aby + abz * abz) / 16384.0f;
  bool acc_ok = (acc_mag_g > ACC_GATE_LO && acc_mag_g < ACC_GATE_HI);

  // ATENCAO (SC-08): numa esfera pendurada o acelerometro NAO e inclinometro: ele mede a
  // forca do cabo + empuxo, isto e, o angulo do cabo RELATIVO a esfera (~0 no balanco) mais
  // F/(m.g) (15 N ~ 7 graus falsos). O gate por modulo nao rejeita isso. A constante longa
  // (tau ~ 10 s) limita o efeito a um passa-altas suave na atitude; a velocidade do
  // anti-balanco nao depende deste angulo.
  float th_acc_x = atan2f(abx, abz) - theta_offset_x;
  float th_acc_y = atan2f(aby, abz) - theta_offset_y;

  // 5. Fusao complementar da ATITUDE da esfera no corpo (tau ~ 10 s).
  //    Aqui SIM entra o transporte: uma inclinacao fixa no mundo, vista do corpo que gira,
  //    roda com -wz (s' = gyro - w x s). Na VELOCIDADE ele nao entra (SC-07).
  float wz_r = raw_yaw_rate_dps * (PI / 180.0f);
  float dsx = (wx_est + wz_r * sy_est) * dt;
  float dsy = (wy_est - wz_r * sx_est) * dt;
  if (acc_ok) {
    sx_est = ALPHA_FUSAO * (sx_est + dsx) + (1.0f - ALPHA_FUSAO) * th_acc_x;
    sy_est = ALPHA_FUSAO * (sy_est + dsy) + (1.0f - ALPHA_FUSAO) * th_acc_y;
  } else {
    sx_est += dsx;
    sy_est += dsy;
  }
  theta_total_deg = sqrtf(sx_est * sx_est + sy_est * sy_est) * (180.0f / PI);

  // 6. (SC-02) Estimador da velocidade do CG. A MPU esta junto ao pino (ponta do cabo):
  //    forca especifica no corpo a_b = x''_pino + g.sin(atitude + offset). Tirando a
  //    gravidade projetada pela atitude (gyro integrado), sobra a aceleracao horizontal real
  //    do pino; integrada com vazamento da v_pino. v_CG = v_pino + d.(d atitude/dt).
  //    Nao depende de L nem de a esfera acompanhar o cabo. Usado quando vel_mode = 1.
  const float g_lsb = G_ACC / 16384.0f;
  float apin_x = abx * g_lsb - G_ACC * sinf(sx_est + theta_offset_x);
  float apin_y = aby * g_lsb - G_ACC * sinf(sy_est + theta_offset_y);
  //    O vetor velocidade e do mundo mas guardado no corpo: gira com -wz (b' = -w x b).
  float vx0 = vpin_x, vy0 = vpin_y;
  vpin_x += (apin_x + wz_r * vy0 - vx0 / TAU_VPIN_S) * dt;
  vpin_y += (apin_y - wz_r * vx0 - vy0 / TAU_VPIN_S) * dt;
  vcg_x = vpin_x + ganhos.d_pino_cg * wx_est;
  vcg_y = vpin_y + ganhos.d_pino_cg * wy_est;
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

  // Velocidade no CORPO: L * taxa de atitude (vel_mode 0) ou estimador do CG (vel_mode 1)
  if (ganhos.vel_mode >= 0.5f) {
    vhat_x = vcg_x;
    vhat_y = vcg_y;
  } else {
    vhat_x = L_M * wx_est;
    vhat_y = L_M * wy_est;
  }

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
  // (SC-06) Visao por EVENTO: o setpoint so muda quando chega um V novo, referido ao
  // instante da foto (yaw de agora menos o giro durante a latencia). Entre quadros a malha
  // fecha pelo gyro (200 Hz), em vez de segurar o bearing velho como erro.
  if (vision_novo) {
    vision_novo = false;
    float yaw_no_quadro = yaw_gyro - rate_z_filt * vision_idade_s;
    desired_yaw = wrap180(yaw_no_quadro + vision_bearing);
  }
  pos_error_yaw = wrap180(desired_yaw - yaw_gyro);

  if (!yaw_ativo) {
    tz_cmd = 0.0f;
    integral_error_yaw = 0.0f;
    last_error_yaw = pos_error_yaw;
    yaw_em_repouso = false;
    digitalWrite(LED_PIN, HIGH);
    return;
  }

  // Histerese de zona morta angular (1,0 / 1,5 graus)
  float abs_err = fabsf(pos_error_yaw);
  if (yaw_em_repouso) {
    if (abs_err > YAW_RESUME_DEG) yaw_em_repouso = false;
  } else {
    if (abs_err <= YAW_DEADZONE_DEG && fabsf(rate_z_filt) < 1.5f) {
      yaw_em_repouso = true;
    }
  }

  if (yaw_em_repouso) {
    tz_cmd = 0.0f;
    integral_error_yaw = 0.0f;
    last_error_yaw = pos_error_yaw;
    digitalWrite(LED_PIN, HIGH);
    return;
  }

  digitalWrite(LED_PIN, LOW);

  // (SC-09) PID em TORQUE [N.m]. A alocacao (realizar_controle) converte torque -> forca
  // por motor -> us pela inversa da curva quadratica, entao o ganho da malha nao depende
  // mais da amplitude (o PID em us tinha ganho ~0 perto do alvo e ~15x maior longe).
  float err_rad  = pos_error_yaw * (PI / 180.0f);
  float rate_rad = rate_z_filt * (PI / 180.0f);

  integral_error_yaw += err_rad * dt;
  if ((pos_error_yaw > 0.0f && last_error_yaw < 0.0f) ||
      (pos_error_yaw < 0.0f && last_error_yaw > 0.0f)) {
    integral_error_yaw = 0.0f;
  }
  integral_error_yaw = constrain(integral_error_yaw, -ganhos.yaw_int_max, ganhos.yaw_int_max);
  last_error_yaw = pos_error_yaw;

  float tz = ganhos.yaw_kp * err_rad + ganhos.yaw_ki * integral_error_yaw - ganhos.yaw_kd * rate_rad;
  tz_cmd = constrain(tz, -ganhos.yaw_tz_max, ganhos.yaw_tz_max);
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

// Forca maxima de um motor no sentido de f com orcamento de 'us_lim' microssegundos
float cap_motor(int i, float f, float us_lim) {
  float kf = (f >= 0.0f) ? motores[i].kf_fwd : motores[i].kf_rev;
  float du = fmaxf(us_lim - motores[i].dz_us, 0.0f);
  return kf * du * du;
}

// (SC-03) Alocacao de forca XY + torque de yaw JUNTOS, no dominio de FORCA.
// Antes o yaw (em us) era somado aos us do anti-balanco: como F ~ (u - dz)^2 e a re rende
// 60%, um yaw puro gerava ~0,4.F de empurrao lateral (~2,4 N a 300 us) e o balanco mudava
// o torque. Agora cada motor recebe f_i = (2/3).t_i.F + s_i.Tz/(3R) e so no fim vira us.
// Prioridade: o torque de yaw cabe primeiro (limitado por YAW_RESERVE_US); o anti-balanco
// usa o que sobra ate PWM_LIMIT_US, com UM fator de escala comum (preserva a direcao).
void realizar_controle(float Fbx, float Fby, float Tz) {
  float f_sw[3], f_yw[3];
  for (int i = 0; i < 3; i++) {
    f_sw[i] = (2.0f / 3.0f) * (motores[i].tx * Fbx + motores[i].ty * Fby);
    f_yw[i] = motores[i].yaw_sign * Tz / (3.0f * R_P);
  }

  // 1. Escala do yaw dentro da reserva de yaw
  float lam_y = 1.0f;
  for (int i = 0; i < 3; i++) {
    if (fabsf(f_yw[i]) > 1e-6f) {
      lam_y = fminf(lam_y, cap_motor(i, f_yw[i], (float)YAW_RESERVE_US) / fabsf(f_yw[i]));
    }
  }

  // 2. Maior escala do swing (0..1) que cabe no limite total de cada motor (bissecao)
  auto cabe = [&](float lam_s) {
    for (int i = 0; i < 3; i++) {
      float f = lam_y * f_yw[i] + lam_s * f_sw[i];
      if (fabsf(f) > cap_motor(i, f, (float)PWM_LIMIT_US)) return false;
    }
    return true;
  };
  float lam_s = 1.0f;
  if (!cabe(1.0f)) {
    float lo = 0.0f, hi = 1.0f;
    for (int k = 0; k < 14; k++) {
      float mid = 0.5f * (lo + hi);
      if (cabe(mid)) lo = mid; else hi = mid;
    }
    lam_s = lo;
  }

  // 3. Forca -> us por motor (curva propria de frente/re); abaixo do travamento -> neutro
  int us[3];
  for (int i = 0; i < 3; i++) {
    float u = us_from_force(i, lam_y * f_yw[i] + lam_s * f_sw[i]);
    if (fabsf(u) < ESC_LATCH_US) u = 0.0f;
    us[i] = ESC_NEUTRAL + (int)constrain(lroundf(u), -PWM_LIMIT_US, PWM_LIMIT_US);
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
const char *GAIN_NAMES[N_GAINS] = {
  "vel_kp", "vel_ki", "vel_kd", "yaw_kp", "yaw_ki",
  "yaw_kd", "yaw_tz_max", "yaw_int_max", "vel_mode", "d_pino_cg"
};
// (SC-19) Faixas aceitas por ganho: valores fora (inclusive NaN/inf) sao RECUSADOS.
// Isso tambem barra o twin Python, que manda G3..G7 com a semantica antiga (ex.: 900).
const float GAIN_MIN[N_GAINS] = { 0.0f,  0.0f,  0.0f,  0.0f,  0.0f,  0.0f,  0.0f, 0.0f, 0.0f, 0.0f };
const float GAIN_MAX[N_GAINS] = { 80.0f, 20.0f, 20.0f, 60.0f, 20.0f, 40.0f, 20.0f, 5.0f, 1.0f, 1.0f };

float *gain_ptr(int idx) {
  switch (idx) {
    case 0: return &ganhos.vel_kp;      case 1: return &ganhos.vel_ki;
    case 2: return &ganhos.vel_kd;      case 3: return &ganhos.yaw_kp;
    case 4: return &ganhos.yaw_ki;      case 5: return &ganhos.yaw_kd;
    case 6: return &ganhos.yaw_tz_max;  case 7: return &ganhos.yaw_int_max;
    case 8: return &ganhos.vel_mode;    case 9: return &ganhos.d_pino_cg;
  }
  return nullptr;
}

// Le um float exigindo valor finito (String::toFloat aceita "nan"/"inf")
bool ler_float(const String &txt, float &out) {
  float v = txt.toFloat();
  if (!isfinite(v)) return false;
  out = v;
  return true;
}

void eco(const String &msg) {
  Serial.println(msg);
  Serial2.println(msg);
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
  F_cmd_x = F_cmd_y = 0.0f;
  tz_cmd = 0.0f;
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
    case 'w':
      if (!bateuCima) moverSubir();
      else { pararMotor(); Serial.println("[BLOQUEADO] Topo atingido!"); }
      break;
    case 's':
      if (!bateuBaixo) moverDescer();
      else { pararMotor(); Serial.println("[BLOQUEADO] Base atingida!"); }
      break;
    case 'x': pararMotor(); break;
    case 'a': girarSentidoA(); break;   // watchdog 250 ms: repetir para manter girando
    case 'd': girarSentidoD(); break;
    case 'c': pararBTS(); break;
    case 'e': lerEncoderAS5600(); break;
    case 'm': testarIdentificacaoMotores(); break;
    case 'y':
      if (!yaw_ativo && (!calib_ok || mpu_falhou)) {   // (FW-18)
        eco("[RECUSADO] Yaw: MPU sem calibracao valida ou em falha. Rode 'z' com a esfera parada.");
        break;
      }
      yaw_ativo = !yaw_ativo;
      Serial.println(yaw_ativo ? ">> [YAW]: ATIVADO" : ">> [YAW]: DESATIVADO");
      Serial2.println(yaw_ativo ? ">> [YAW]: ATIVADO" : ">> [YAW]: DESATIVADO");
      if (!yaw_ativo) {
        integral_error_yaw = 0.0f;
        tz_cmd = 0.0f;
        yaw_em_repouso = false;
      }
      break;
    case 'p':   // (era b/B/p/P: um comando, uma letra — docs/COMANDOS.md)
      if (!anti_balanco_ativo && (!calib_ok || mpu_falhou)) {   // (FW-18)
        eco("[RECUSADO] Anti-balanco: MPU sem calibracao valida ou em falha. Rode 'z' parado.");
        break;
      }
      anti_balanco_ativo = !anti_balanco_ativo;
      Serial.println(anti_balanco_ativo ? ">> [PENDULO]: ANTI-BALANCO ATIVADO"
                                        : ">> [PENDULO]: ANTI-BALANCO DESATIVADO");
      Serial2.println(anti_balanco_ativo ? ">> [PENDULO]: ATIVADO" : ">> [PENDULO]: DESATIVADO");
      if (!anti_balanco_ativo) {
        F_cmd_x = F_cmd_y = 0.0f;
        pid_vx.reset();
        pid_vy.reset();
        repouso_prumo_ativo = false;
      } else {
        vpin_x = vpin_y = 0.0f;   // estimador parte do repouso
      }
      break;
    case 'k': case 'K': parada_geral(); break;
    case 'z': case 'Z': {
      // (SC-04) 'z' recusa se houver movimento; 'Z' forca. Malhas sao desligadas antes
      // (FW-09): a calibracao bloqueia ~9 s e os ESCs nao podem ficar no ultimo PWM.
      parada_geral();
      bool ok = calibrar_mpu(true, c == 'Z');
      eco(ok ? ">> [MPU]: calibracao OK" : ">> [MPU]: calibracao RECUSADA (malhas bloqueadas)");
      if (!ok) calib_ok = false;
      break;
    }
    case 't':
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

  // '0' sozinho na linha: zera o setpoint (era comando imediato e corrompia "0.5" — SC-10)
  if (cmd == "0") { comando_char('0'); return; }

  // L<val>: define comprimento do cabo [m]
  if ((cmd[0] == 'L' || cmd[0] == 'l') && cmd.length() > 1) {
    float novo = NAN;
    ler_float(cmd.substring(1), novo);
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
      float val = NAN;
      bool fin = ler_float(cmd.substring(space + 1), val);
      float *p = gain_ptr(idx);
      if (p && idx >= 0 && idx < N_GAINS) {
        if (fin && val >= GAIN_MIN[idx] && val <= GAIN_MAX[idx]) {
          *p = val;
          set_gains_from_struct();
          Serial.print(">> [GANHO] "); Serial.print(GAIN_NAMES[idx]);
          Serial.print(" = "); Serial.println(val, 4);
        } else {
          eco(String("[RECUSADO] G") + idx + " fora da faixa [" + String(GAIN_MIN[idx], 2) +
              ", " + String(GAIN_MAX[idx], 2) + "]");
        }
      }
    }
    return;
  }

  // R: retorna todos os ganhos atuais
  if (cmd[0] == 'R') {
    for (int i = 0; i < N_GAINS; i++) {
      String s = "GAIN," + String(i) + "," + GAIN_NAMES[i] + "," + String(*gain_ptr(i), 4);
      Serial.println(s);
      Serial2.println(s);
    }
    return;
  }

  // V<bearing> <D> [idade_ms]: entrada da visao computacional (SC-06)
  // idade_ms opcional = tempo desde a captura do quadro; sem ele usa VISION_LATENCY_S.
  if (cmd[0] == 'V' && cmd.length() > 1) {
    int space = cmd.indexOf(' ');
    if (space > 1) {
      int space2 = cmd.indexOf(' ', space + 1);
      float b = NAN, d = -1.0f, idade = NAN;
      bool okb = ler_float(cmd.substring(1, space), b);
      ler_float(space2 > 0 ? cmd.substring(space + 1, space2) : cmd.substring(space + 1), d);
      if (space2 > 0) ler_float(cmd.substring(space2 + 1), idade);
      if (okb && fabsf(b) <= 180.0f) {
        vision_bearing = b;
        vision_D = d;
        vision_idade_s = (isfinite(idade) && idade >= 0.0f && idade < 1000.0f)
                         ? idade * 0.001f : VISION_LATENCY_S;
        vision_last_ms = millis();
        vision_novo = true;
      }
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

  // M<motor> <tx> <ty> <yaw_sign> <kf_fwd> <kf_rev> <dz>  (WZ-01)
  // - tx, ty: IGNORADOS — direcao de empuxo e geometria (fixa no codigo; validar com 'm').
  // - yaw_sign: aplicado.
  // - kf_fwd, kf_rev, dz: aplicados se validos (kf entre 0,2x e 5x o default, dz 0..150);
  //   kf <= 0 ou dz < 0 significa "manter o atual" (o wizard usa isso sem --push-kf).
  // Responde com os valores efetivamente aplicados.
  if (cmd[0] == 'M' && cmd.length() > 1) {
    int fields[6];
    int fi = 0;
    for (unsigned int i = 0; i < cmd.length() && fi < 6; i++) {
      if (cmd[i] == ' ') fields[fi++] = (int)i;
    }
    if (fi == 6) {
      int m = cmd.substring(1, fields[0]).toInt();
      if (m >= 0 && m < 3) {
        float ys = 1.0f, kfw = -1.0f, kre = -1.0f, dz = -1.0f;
        bool ok = ler_float(cmd.substring(fields[2] + 1, fields[3]), ys);
        ler_float(cmd.substring(fields[3] + 1, fields[4]), kfw);
        ler_float(cmd.substring(fields[4] + 1, fields[5]), kre);
        ler_float(cmd.substring(fields[5] + 1), dz);
        if (ok) motores[m].yaw_sign = (ys >= 0.0f) ? 1.0f : -1.0f;
        const float kf_def = F_MAX_500 / ((500.0f - DB_US) * (500.0f - DB_US));
        if (kfw > 0.0f) {
          if (kfw >= 0.2f * kf_def && kfw <= 5.0f * kf_def) motores[m].kf_fwd = kfw;
          else eco("[RECUSADO] kf_fwd fora de 0,2x..5x o default");
        }
        if (kre > 0.0f) {
          if (kre >= 0.1f * kf_def && kre <= 5.0f * kf_def) motores[m].kf_rev = kre;
          else eco("[RECUSADO] kf_rev fora de 0,1x..5x o default");
        }
        if (dz >= 0.0f) {
          if (dz <= 150.0f) motores[m].dz_us = dz;
          else eco("[RECUSADO] dz fora de 0..150 us");
        }
        char a[16], b[16], c[16];
        fmt_f2(a, motores[m].kf_fwd * 1e5f); fmt_f2(b, motores[m].kf_rev * 1e5f);
        fmt_f2(c, motores[m].dz_us);
        eco(String(">> [MOTOR ") + m + "] yaw_sign=" + String(motores[m].yaw_sign, 0) +
            " kf_fwd=" + a + "e-5 kf_rev=" + b + "e-5 dz=" + c + " us (direcao = geometria)");
      }
    }
    return;
  }

  // <numero>: novo setpoint de angulo de Yaw [graus]
  char c0 = cmd[0];
  if (isdigit(c0) || c0 == '-' || c0 == '+' || c0 == '.') {
    float ang = NAN;
    if (!ler_float(cmd, ang) || fabsf(ang) > 360.0f) {   // (SC-19)
      eco("[RECUSADO] setpoint de yaw invalido (|graus| <= 360)");
      return;
    }
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

// Comandos IMEDIATOS (executam no 1o byte, sem esperar o fim da linha): so mecanismos e a
// parada geral, para quem digita num monitor serial sem newline. Todo o resto exige linha
// completa (todos os programas do projeto mandam '\n'). Tabela unica: docs/COMANDOS.md.
// (SC-10) Sem digitos: "0.5" chegava como '0' imediato + ".5" descartado. Sem maiusculas
// nem 'm'/'y'/'p'/'z': um byte perdido/ruido nao dispara teste de motor nem liga malha.
const char WHITELIST[] = "wsxadcekK";
bool eh_comando_imediato(char c) {
  for (const char *p = WHITELIST; *p; p++) {
    if (*p == c) return true;
  }
  return false;
}

String bufUSB = "", bufS2 = "";
bool descartUSB = false, descartS2 = false;
void processar_serial(Stream &porta, String &buf, bool &descartando) {
  while (porta.available() > 0) {
    char c = porta.read();
    if (c == '\n' || c == '\r') {
      if (!descartando && buf.length() > 0) comando_linha(buf);
      buf = "";
      descartando = false;
    } else if (descartando) {
      // (FW-13) linha longa demais: ignora ate o fim da linha (o resto nao vira comando)
    } else if (buf.length() == 0 && eh_comando_imediato(c)) {
      comando_char(c);
    } else {
      buf += c;
      if (buf.length() > 60) { buf = ""; descartando = true; }
    }
  }
}

// =========================================================================================
// --- TELEMETRIA 25 HZ (COMPATIVEL COM O DIGITAL TWIN / RASPBERRY PI) ---
// =========================================================================================
void enviar_telemetria() {
  // flags: 1 yaw_on | 2 swing_on | 4 calib_ok | 8 visao | 16 wizard | 32 MPU em falha |
  //        64 repouso no prumo | 128 yaw em repouso | 256 vel_mode = estimador
  unsigned flags = (yaw_ativo ? 1 : 0) |
                   (anti_balanco_ativo ? 2 : 0) |
                   (calib_ok ? 4 : 0) |
                   (vision_fresh() ? 8 : 0) |
                   (wizard_motor >= 0 ? 16 : 0) |
                   (mpu_falhou ? 32 : 0) |
                   (repouso_prumo_ativo ? 64 : 0) |
                   (yaw_em_repouso ? 128 : 0) |
                   (ganhos.vel_mode >= 0.5f ? 256 : 0);

  char line[256], num[16];
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
  n += snprintf(line + n, sizeof(line) - n, "%u", flags);
  // Campos 17..21 (TEL v3): forca XY no corpo [N], torque de heading [N.m],
  // velocidade realimentada no corpo [m/s]. Parsers antigos que exigem 16 campos
  // precisam aceitar >= 16 (twin/protocol.py e supervisorio ja aceitam).
  fmt_f2(num, F_cmd_x); n += snprintf(line + n, sizeof(line) - n, ",%s", num);
  fmt_f2(num, F_cmd_y); n += snprintf(line + n, sizeof(line) - n, ",%s", num);
  fmt_f2(num, tz_cmd);  n += snprintf(line + n, sizeof(line) - n, ",%s", num);
  fmt_f3(num, vhat_x);  n += snprintf(line + n, sizeof(line) - n, ",%s", num);
  fmt_f3(num, vhat_y);  snprintf(line + n, sizeof(line) - n, ",%s", num);

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
  if (!calibrar_mpu(true, false)) {
    calib_ok = false;
    Serial.println("[AVISO] Calibracao de boot recusada/falhou: malhas bloqueadas ate um 'z' valido.");
  }

  yaw_gyro = 0.0f;
  last_time_us = micros();
  last_swing_us = micros();
  last_telemetry_ms = millis();

  Serial.println("==========================================================");
  Serial.println(" ESFERA CONSOLIDADA — YAWBOM + ANTI-BALANCO DE VELOCIDADE ");
  Serial.println("  PINCA:   w=sobe | s=desce | x=para                      ");
  Serial.println("  POLIA:   a=giro A | d=giro D | c=para (repetir a/d<250ms)");
  Serial.println("  MALHAS:  y=heading on/off | p=anti-balanco on/off       ");
  Serial.println("  YAW:     <graus>=degrau relativo | 0=zera setpoint      ");
  Serial.println("  MPU:     z=calibra (recusa movimento) | Z=forca         ");
  Serial.println("  OUTROS:  e=AS5600 | t=L pelo periodo | L2.0 | m=teste   ");
  Serial.println("  LINHAS:  G<i> <v> | R | V<b> <D> [ms] | W.. | M..       ");
  Serial.println("  PARADA GERAL: k  (tabela completa: docs/COMANDOS.md)    ");
  Serial.println("==========================================================");
}

void loop() {
  unsigned long now_us = micros();
  float dt = (now_us - last_time_us) * 1e-6f;
  last_time_us = now_us;
  if (dt <= 1e-6f || dt > 0.5f) dt = 0.005f;

  processar_serial(Serial, bufUSB, descartUSB);
  processar_serial(Serial2, bufS2, descartS2);

  // Watchdog da Polia (Deadman Switch de 250 ms trazido do YAWBOM)
  if (polia_em_movimento && (millis() - ultimo_comando_polia_ms > TIMEOUT_POLIA_MS)) {
    pararBTS();
  }

  // --- Malha fixa de 200 Hz (5 ms): Fusao + Controle XY + Controle Yaw ---
  if (now_us - last_swing_us >= SWING_INTERVAL_US) {
    float dt_swing = (now_us - last_swing_us) * 1e-6f;
    last_swing_us = now_us;
    // (SC-14) depois de rotinas bloqueantes ('m', I2C lento) nao integra dt de segundos
    if (dt_swing > 0.02f) dt_swing = 0.02f;

    MpuSample s = ler_mpu();
    if (s.ok) {
      mpu_falhas = 0;
      passo_fusao(s, dt_swing);
      talvez_rebias(s);
    } else if (++mpu_falhas >= MPU_FAIL_LIMIT && !mpu_falhou) {
      // (SC-15) sem MPU o controle seguiria com dado velho: desliga tudo e sinaliza
      mpu_falhou = true;
      parada_geral();
      eco("[FALHA] MPU sem resposta: malhas desligadas. Verifique I2C e rode 'z'.");
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
      // F_cmd e calculada a partir da velocidade no CORPO, logo ja esta no corpo (FW-03):
      // nenhuma rotacao por yaw. Forca e torque vao juntos para a alocacao (SC-03).
      realizar_controle(F_cmd_x, F_cmd_y, tz_cmd);
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
