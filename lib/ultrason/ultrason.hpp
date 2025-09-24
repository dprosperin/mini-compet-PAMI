#ifndef ULTRASON_HEADER
#define ULTRASON_HEADER

const int trigPin = 5;
const int echoPin = 18;

//define sound speed in cm/uS
#define SOUND_SPEED 0.034
#define CM_TO_INCH 0.39370

void setPinsTrigEcho();
float getDistanceUltrason();

#endif