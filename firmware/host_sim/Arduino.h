#pragma once
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstdint>
#include <cstring>
#include <string>
#include <cctype>
using std::isfinite; using std::isdigit;
#define PI 3.14159265358979f
#define HIGH 1
#define LOW 0
#define OUTPUT 1
#define INPUT 0
#define INPUT_PULLUP 2
#define INPUT_PULLDOWN 3
enum { PA0,PA1,PA2,PA3,PA5,PA6,PA7,PA15,PB0,PB5,PB6,PB7,PB8,PB9,PB10,PB12,PB13,PB14,PB15,PC13 };
template<class T,class A,class B> T constrain(T x,A a,B b){return x<a?a:(x>b?b:x);}
extern unsigned long g_us;
inline unsigned long micros(){return g_us;}
inline unsigned long millis(){return g_us/1000;}
inline void delay(unsigned long ms){g_us+=ms*1000;}
inline void delayMicroseconds(unsigned u){g_us+=u;}
inline void pinMode(int,int){}
inline void digitalWrite(int,int){}
inline int digitalRead(int){return 0;}
class String {
public: std::string s;
  String(){} String(const char*c):s(c){} String(const std::string&x):s(x){}
  String(int v):s(std::to_string(v)){} String(unsigned v):s(std::to_string(v)){} String(long v):s(std::to_string(v)){}
  String(float v,int d=2){char b[32];snprintf(b,32,"%.*f",d,v);s=b;}
  String(double v,int d=2){char b[32];snprintf(b,32,"%.*f",d,v);s=b;}
  unsigned length()const{return s.size();}
  char operator[](unsigned i)const{return s[i];}
  String substring(unsigned a)const{return a>=s.size()?String():String(s.substr(a));}
  String substring(unsigned a,unsigned b)const{return a>=s.size()?String():String(s.substr(a,b-a));}
  int indexOf(char c,unsigned from=0)const{auto p=s.find(c,from);return p==std::string::npos?-1:(int)p;}
  float toFloat()const{return strtof(s.c_str(),nullptr);}
  long toInt()const{return strtol(s.c_str(),nullptr,10);}
  String& operator+=(char c){s+=c;return *this;}
  bool operator==(const char*c)const{return s==c;}
  const char* c_str()const{return s.c_str();}
};
inline String operator+(const String&a,const String&b){return String(a.s+b.s);}
inline String operator+(const String&a,const char*b){return String(a.s+b);}
inline String operator+(const char*a,const String&b){return String(std::string(a)+b.s);}
inline String operator+(const String&a,int b){return String(a.s+std::to_string(b));}
class Stream { public:
  std::string in; std::string out;
  int available(){return in.size();}
  int read(){int c=in[0];in.erase(0,1);return c;}
  void begin(long){} void setRx(int){} void setTx(int){}
  template<class T> void print(T v){out+=String(v).s;}
  template<class T> void print(T v,int d){out+=String(v,d).s;}
  void print(const char*c){out+=c;} void print(const String&x){out+=x.s;}
  template<class T> void println(T v){print(v);out+="\n";}
  template<class T> void println(T v,int d){print(v,d);out+="\n";}
  void println(const char*c){out+=c;out+="\n";} void println(const String&x){out+=x.s+"\n";}
  void println(){out+="\n";}
};
extern Stream Serial, Serial2;
