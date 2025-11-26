#include "Encoder.hpp"

void Encoder::init(){
    pinMode(pinA_l, INPUT_PULLUP);
    pinMode(pinB_l, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(pinA_l), isrLeft, CHANGE);
    pinMode(pinA_r, INPUT_PULLUP);
    pinMode(pinB_r, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(pinA_r), isrRight, CHANGE);
}

void IRAM_ATTR Encoder::isrRight(){
    if(digitalRead(pinB_r)==1) encoderCountRight++;
    else encoderCountRight--;
}

// digitalRead() bloquant ? --> gpio_get_level()
void IRAM_ATTR Encoder::isrLeft(){ 
    if(digitalRead(pinB_l)==1) encoderCountLeft++;
    else encoderCountLeft--;
}

long Encoder::getRightVal(){
    return encoderCountRight;
}

long Encoder::getLeftVal(){
    return encoderCountLeft;
}






