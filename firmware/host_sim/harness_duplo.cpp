// Pendulo DUPLO: cabo (theta1) + esfera presa por pino (theta2), por eixo, pequenos angulos.
// - MPU no pino (topo), giroscopio mede a atitude da esfera (theta2'), acelerometro a forca
//   especifica do pino; empuxo aplicado a 'h' do CG (h<0 = abaixo do CG); pino com rigidez
//   rotacional opcional kp (cabo preso/guia rigida -> bamboleio mais rapido).
// - Ancora (drone) pode acelerar: cenario DRONE.
// Uso: ./harness_duplo <vel_mode> <d> <h> <I> <kp_pino> [kf_erro]
//   imprime SWING (empurrao 0,3 m/s) e DRONE (ancora anda 1 m em 2 s): pico da posicao
//   RELATIVA a ancora (cm) em janelas de 2,25 s, e a frequencia do bamboleio livre.
#include "Arduino.h"
#include "Wire.h"
#include <vector>
unsigned long g_us = 0;
Stream Serial, Serial2;
TwoWire Wire;
static std::vector<uint8_t> buf;
#include "../main.cpp"

const double M=12, G=9.80665, IZ=1.73, R=0.7, TAU=0.12;
double LL=5.0, DD=0.15, HH=-0.3, II=2.0, KP=0.0, KF_ERR=1.0;
struct Axis { double t1=0,t2=0,w1=0,w2=0; };
struct Plant { Axis X, Y; double psi=0, r=0; double F[3]={0,0,0};
  double ax_anchor=0, ay_anchor=0, ex=0, ey=0; double accx=0, accy=0; } P;  // acc*: x''_pino (mundo)

static void axis_step(Axis &a, double Fth, double Fext, double a_anchor, double dt, double &acc_pin){
  // M q'' + K q = Q_F*Fth + Q_cg*(Fext - m a_anchor)
  double m11=M*LL*LL, m12=M*LL*DD, m22=M*DD*DD+II;
  double k11=M*G*LL+KP, k12=-KP, k22=M*G*DD+KP;
  double q1 = LL*Fth + LL*(Fext - M*a_anchor) - k11*a.t1 - k12*a.t2;
  double q2 = (DD-HH)*Fth + DD*(Fext - M*a_anchor) - k12*a.t1 - k22*a.t2;
  double det=m11*m22-m12*m12;
  double a1=( m22*q1 - m12*q2)/det, a2=(-m12*q1 + m11*q2)/det;
  a.w1+=a1*dt; a.w2+=a2*dt; a.t1+=a.w1*dt; a.t2+=a.w2*dt;
  acc_pin = a_anchor + LL*a1;            // aceleracao horizontal do pino no mundo
}
static inline void put16(int16_t v){buf.push_back((uint8_t)(v>>8));buf.push_back((uint8_t)(v&0xff));}
int TwoWire::requestFrom(int,int){
  buf.clear();
  double c=cos(P.psi), s=sin(P.psi);
  // mundo -> corpo
  double w2x= c*P.X.w2+s*P.Y.w2, w2y=-s*P.X.w2+c*P.Y.w2;      // d(atitude)/dt no corpo
  double t2x= c*P.X.t2+s*P.Y.t2, t2y=-s*P.X.t2+c*P.Y.t2;
  double apx= c*P.accx+s*P.accy, apy=-s*P.accx+c*P.accy;
  double gyro_y=-w2x, gyro_x=w2y;
  double ax=apx + G*sin(t2x), ay=apy + G*sin(t2y);             // forca especifica no pino
  double am=IMU_MOUNT_DEG*M_PI/180, ca=cos(am), sa=sin(am);
  double iax= ca*ax+sa*ay, iay=-sa*ax+ca*ay, igx= ca*gyro_x+sa*gyro_y, igy=-sa*gyro_x+ca*gyro_y;
  double k=16384/G;
  double az=G*cos(t2x)*cos(t2y);                             // Z realista (grandes angulos)
  put16((int16_t)lround(iax*k)); put16((int16_t)lround(iay*k)); put16((int16_t)lround(az*k)); put16(0);
  put16((int16_t)lround(igx*180/M_PI*131)); put16((int16_t)lround(igy*180/M_PI*131));
  put16((int16_t)lround(P.r*180/M_PI*131));
  return 14;
}
int TwoWire::available(){return (int)buf.size();}
int TwoWire::read(){int v=buf.front();buf.erase(buf.begin());return v;}

static double force_us(int i,int pwm){ double u=pwm-1500; if(fabs(u)<=motores[i].dz_us) return 0;
  double kf=(u>0?motores[i].kf_fwd:motores[i].kf_rev)*KF_ERR; double a=fabs(u)-motores[i].dz_us; return (u>0?1:-1)*kf*a*a; }
static void step(double dt){
  int pw[3]={esc_a.us,esc_b.us,esc_c.us};
  for(int i=0;i<3;i++) P.F[i]+=(force_us(i,pw[i])-P.F[i])*dt/TAU;
  double Fbx=0,Fby=0,tz=0;
  for(int i=0;i<3;i++){ Fbx+=motores[i].tx*P.F[i]; Fby+=motores[i].ty*P.F[i]; tz+=R*motores[i].yaw_sign*P.F[i]; }
  double c=cos(P.psi), s=sin(P.psi);
  double Fwx=c*Fbx-s*Fby, Fwy=s*Fbx+c*Fby;
  axis_step(P.X, Fwx, P.ex, P.ax_anchor, dt, P.accx);
  axis_step(P.Y, Fwy, P.ey, P.ay_anchor, dt, P.accy);
  P.r+=tz/IZ*dt; P.psi+=P.r*dt;
}
static void run(double secs){ long n=(long)(secs/0.00025); for(long k=0;k<n;k++){ g_us+=250; step(0.00025); loop(); } }
// posicao do CG RELATIVA a ancora
static double amp(){ double x=LL*P.X.t1+DD*P.X.t2, y=LL*P.Y.t1+DD*P.Y.t2; return hypot(x,y); }
static void cmd(const char*c){ Serial.in+=c; }
static void reset_all(float mode){
  P=Plant(); g_us=0; Serial.in.clear(); Serial.out.clear();
  yaw_ativo=false; anti_balanco_ativo=false; desired_yaw=0; yaw_gyro=0;
  setup(); Serial.out.clear(); ganhos.vel_mode=mode; L_M=(float)LL;
  if(getenv("D_EST")) ganhos.d_pino_cg=atof(getenv("D_EST")); else ganhos.d_pino_cg=(float)DD;
  if(getenv("KP_VEL")) ganhos.vel_kp=atof(getenv("KP_VEL"));
  if(getenv("K_BAMB")) ganhos.k_bamb=atof(getenv("K_BAMB"));
  set_gains_from_struct();
}
static void windows(const char* tag){
  printf("%s", tag);
  for(int w=0;w<6;w++){ double pk=0; for(int k=0;k<9;k++){run(0.25); pk=fmax(pk,amp());} printf(" %6.1f",pk*100); }
  printf("\n");
}
int main(int argc,char**argv){
  float mode = argc>1?atof(argv[1]):0; if(argc>2) DD=atof(argv[2]); if(argc>3) HH=atof(argv[3]);
  if(argc>4) II=atof(argv[4]); if(argc>5) KP=atof(argv[5]); if(argc>6) KF_ERR=atof(argv[6]);
  if(argc>7) LL=atof(argv[7]);
  // frequencia do bamboleio livre (modelo)
  { double m11=M*LL*LL,m12=M*LL*DD,m22=M*DD*DD+II,k11=M*G*LL+KP,k12=-KP,k22=M*G*DD+KP;
    double a=m11*m22-m12*m12, b=-(m11*k22+m22*k11-2*m12*k12), cc=k11*k22-k12*k12;
    double w2=(-b+sqrt(b*b-4*a*cc))/(2*a); printf("BAMBOLEIO %.2f Hz  PENDULO %.2f Hz\n", sqrt(w2)/(2*M_PI), sqrt((-b-sqrt(b*b-4*a*cc))/(2*a))/(2*M_PI)); }
  for(int d=0; d<2; d++){
    reset_all(mode); cmd("p\n"); run(0.1);
    double dx=d?0.7071:1, dy=d?0.7071:0;
    P.ex=36*dx; P.ey=36*dy; run(0.1); P.ex=P.ey=0;
    windows(d?"SWING diag":"SWING x   ");
  }
  reset_all(mode); cmd("p\n"); run(0.1);
  P.ax_anchor=1.0; run(1.0); P.ax_anchor=-1.0; run(1.0); P.ax_anchor=0;   // drone anda 1 m
  windows("DRONE 1m  ");
  return 0;
}
