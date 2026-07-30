#pragma once
#include <cstdint>
#include <driver/pcnt.h>
#include <atomic>
#include <vector>
using ui8 = uint8_t;
using ui32 = uint32_t;

using i16 = int16_t;
using i32 = int32_t;

struct Motor{
    ui8 outA;
    ui8 outB;
    
    i32 ticks;
    i32 speed;
    ui32 lastTime;
    i32 lastTicks;
    ui8 encA;
    ui8 encB;
    pcnt_unit_t encoderUnit;
    
    ui8 pwm;
    ui8 pwmChannel;
    ui32 pwmFrequency;
    ui8 pwmResolution;
    
    ui32 maxDuty;
    
    Motor(ui8 outA_, ui8 outB_, ui8 pwm_, ui8 encA_, ui8 encB_, ui8 pwmChannel_, pcnt_unit_t encoderUnit_, ui32 pwmFrequency_ = 20000, ui8 pwmResolution_ = 10);

    void begin();
    i32 getTicks();
    void resetTicks();
    void setPWM(ui32 duty);
    void movePWM(i32 duty);
    void updateTicks();
    void updateData();
    void forward();
    void backward();
    void forward(ui32 duty);
    void backward(ui32 duty);
    void updateSpeed();
    i32 getSpeed();

    /// PID
    void speedPID();
    static void SpeedPIDWrapper(void *pv);

    struct FeedForwardPoint{
        i32 tps;
        i32 actualDuty;
    };

    std::vector<FeedForwardPoint> feedForwardMap;
    i32 getFeedForwardDuty(i32 tps);


    const i32 integralLimit = 1000;
    double kP = 0.15, kI = 0.01, kD = 0.01;
    volatile i32 targetTps;

    void setTargetTPS(i32 tps);

};
