// Simulacao em malha fechada do firmware REAL (firmware/main.cpp) no PC.
//
// Compila o main.cpp contra stubs minimos do Arduino (pasta atual) e liga a MPU/ESCs
// simulados a uma planta de pendulo (esfera rigida ao cabo, L = 5 m, 12 kg, Iz = 1,73,
// MPU com Z para cima e X = +X do corpo, empuxo com atraso de 0,12 s).
//
// Uso (g++ ou clang):  g++ -std=gnu++17 -O1 -w -I. harness.cpp -o harness && ./harness
//   saida: uma linha por caso "SWING psi dir modo pico_cm..." e "YAW t erro_graus"
// tests/test_firmware_host.py compila, roda e confere os limites.
// Para testar o estimador do CG: ./harness est
#include "Arduino.h"
#include "Wire.h"
#include <vector>
unsigned long g_us = 0;
Stream Serial, Serial2;
TwoWire Wire;
struct Plant { double x=0,y=0,vx=0,vy=0,psi=0,r=0; double F[3]={0,0,0}; double ex=0,ey=0; } P;
const double M=12, G=9.80665, LL=5.0, IZ=1.73, R=0.7, TAU=0.12;
static std::vector<uint8_t> buf;
#include "../main.cpp"

static inline void put16(int16_t v){buf.push_back((uint8_t)(v>>8));buf.push_back((uint8_t)(v&0xff));}
int TwoWire::requestFrom(int,int){
  buf.clear();
  double c=cos(P.psi), s=sin(P.psi);
  double Fbx=0,Fby=0;
  for(int i=0;i<3;i++){ Fbx+=motores[i].tx*P.F[i]; Fby+=motores[i].ty*P.F[i]; }
  double sxd=P.vx/LL, syd=P.vy/LL;
  double bxd= c*sxd+s*syd, byd=-s*sxd+c*syd;     // d(sx)/dt no corpo
  double gyro_y=-bxd, gyro_x=byd;                // cinematica: rot +Y leva o CG p/ -X
  // MPU no pino, esfera rigida ao cabo: a_b = x''_pino + g.sin(att) = F_total/m
  double ax=(Fbx + c*P.ex+s*P.ey)/M, ay=(Fby - s*P.ex+c*P.ey)/M;
  double k=16384/G;
  put16((int16_t)lround(ax*k)); put16((int16_t)lround(ay*k)); put16((int16_t)16384); put16(0);
  put16((int16_t)lround(gyro_x*180/M_PI*131)); put16((int16_t)lround(gyro_y*180/M_PI*131));
  put16((int16_t)lround(P.r*180/M_PI*131));
  return 14;
}
int TwoWire::available(){return (int)buf.size();}
int TwoWire::read(){int v=buf.front();buf.erase(buf.begin());return v;}

static double force_us(int i,int pwm){ double u=pwm-1500; if(fabs(u)<=motores[i].dz_us) return 0;
  double kf=u>0?motores[i].kf_fwd:motores[i].kf_rev; double a=fabs(u)-motores[i].dz_us; return (u>0?1:-1)*kf*a*a; }
static void step(double dt){
  int pw[3]={esc_a.us,esc_b.us,esc_c.us};
  for(int i=0;i<3;i++) P.F[i]+=(force_us(i,pw[i])-P.F[i])*dt/TAU;
  double Fbx=0,Fby=0,tz=0;
  for(int i=0;i<3;i++){ Fbx+=motores[i].tx*P.F[i]; Fby+=motores[i].ty*P.F[i]; tz+=R*motores[i].yaw_sign*P.F[i]; }
  double c=cos(P.psi), s=sin(P.psi);
  double Fwx=c*Fbx-s*Fby+P.ex, Fwy=s*Fbx+c*Fby+P.ey;
  P.vx+=(-G/LL*P.x+Fwx/M)*dt; P.vy+=(-G/LL*P.y+Fwy/M)*dt; P.x+=P.vx*dt; P.y+=P.vy*dt;
  P.r+=tz/IZ*dt; P.psi+=P.r*dt;
}
static void run(double secs){ long n=(long)(secs/0.00025); for(long k=0;k<n;k++){ g_us+=250; step(0.00025); loop(); } }
static double amp(){ return hypot(P.x,P.y); }
static void cmd(const char*c){ Serial.in+=c; }
static void reset_all(double psi){
  P=Plant(); P.psi=psi*M_PI/180; g_us=0; Serial.in.clear(); Serial.out.clear();
  yaw_ativo=false; anti_balanco_ativo=false; desired_yaw=0; yaw_gyro=0;
  setup(); Serial.out.clear();
}

int main(int argc,char**argv){
  bool est = argc>1 && std::string(argv[1])=="est";
  double dirs[][2]={{1,0},{0,1},{0.7071,0.7071}};
  for(double psi: {0.0, 90.0, 180.0}) for(auto&d: dirs){
    reset_all(psi);
    if(est){ ganhos.vel_mode=1.0f; }
    cmd("p\n"); run(0.1);
    P.ex=36*d[0]; P.ey=36*d[1]; run(0.1); P.ex=P.ey=0;   // empurrao: 0,3 m/s
    printf("SWING %.0f %.2f %.2f %s", psi, d[0], d[1], est?"est":"gyro");
    for(int w=0;w<6;w++){ double pk=0; for(int k=0;k<9;k++){run(0.25); pk=fmax(pk,amp());} printf(" %.2f",pk*100); }
    printf("\n");
  }
  // degrau de heading de 30 graus durante um balanco
  reset_all(0);
  if(est) ganhos.vel_mode=1.0f;
  cmd("p\n"); run(0.1); cmd("y\n"); run(0.1);
  P.ex=36; run(0.1); P.ex=0; cmd("30\n");
  for(int k=1;k<=8;k++){ run(0.5); printf("YAW %.1f %.2f %.2f\n", k*0.5, 30.0 - P.psi*180/M_PI, amp()*100); }
  return 0;
}
