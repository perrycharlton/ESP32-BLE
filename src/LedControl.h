#ifndef LedControl_h
#define LedControl_h

#include "Arduino.h"
// #include <LedControl.h>
// #include <ArduinoJson.h>

class Ticker;

class LedControl
{
private:
    int* _pinsLed;
    int _freq;
    int _resolution;
    int _sizeArray;


    static void fader();

public:
    LedControl();
    void Setup(int freq, int resolution, int* pinsLed, int sizeArray);
    int getLed(int pin, char[]);
    int setLed(JsonDocument& rcvdMsg, char[]);
    void getLeds(JsonArray& arr);
};
#endif


