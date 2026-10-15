#pragma once
#include "Arduino.h"
struct TwoWire{ void beginTransmission(int){} int endTransmission(bool=true){return 0;} void write(int){}
 int requestFrom(int,int); int available(); int read(); void setSCL(int){} void setSDA(int){} void begin(){} void setClock(long){} void setTimeout(int){} };
extern TwoWire Wire;
