#pragma once
#include <string>
#include <Arduino.h>


extern volatile long encoderCountLeft;
extern volatile long encoderCountRight;

class Encoder {
private:
public:
    static void init();
    static void IRAM_ATTR isrLeft();
    static void IRAM_ATTR isrRight();
    static long getLeftVal();
    static long getRightVal();
};