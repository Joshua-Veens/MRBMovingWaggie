#include "waggie.hpp"

waggie rijdendeWaggie;

void setup(){
    Serial.begin(9600);
    rijdendeWaggie.setupWaggie();
}


void loop(){
  rijdendeWaggie.pidLoop();
}