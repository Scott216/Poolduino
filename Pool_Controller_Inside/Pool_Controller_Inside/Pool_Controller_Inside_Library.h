#ifndef Pool_Controller_Inside_RELEASE
#define Pool_Controller_Inside_RELEASE 100

#include "Arduino.h"


class Pool {
    
public:
    Pool();
    ~Pool();
    bool isPumpOn();
    bool isWaterFillOn();
    bool isWaterFillHoseOn();
    bool isHeaterOn();
    bool isWaterLevelLow();
    
    
    void  setPumpAmps(float amps);
    float getPumpAmps();
    void  setWaterFillPress(float pressure);
    float getWaterFillPress();
    void  setPreFltrPress(float pressure);
    float getPreFltrPress();
    void  setPostFltrPress(float pressure);
    float getPostFltrPress();
    void  setPreHtrTemp(float temperature);
    float getPreHtrTemp();
    void  setPostHtrTemp(float temperature);
    float getPostHtrTemp();
    void  setPumpTemp(float temperature);
    float getPumpTemp();
    int   getWaterFillMinutes();
    uint8_t getStatus();              // indicates status of pool: pump on, off, alarms etc.
    uint8_t getLowPressureCounter();  // Counts pressure fluctuations when pump is starved for water
    
    
    
protected:
    float tempPreHeat;
    float tempPostHeat;
    float tempPump;
    float pressPreFilter;
    float pressPostFilter;
    float pressWaterFill;
    float pumpAmps;
};


#endif
