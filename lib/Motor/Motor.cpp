#include <Motor.h>
#include <Arduino.h>
#include <driver/pcnt.h>
#define print Serial.print
#define println Serial.println

Motor::Motor(ui8 outA_, ui8 outB_, ui8 pwm_, ui8 encA_, ui8 encB_, ui8 pwmChannel_, pcnt_unit_t encoderUnit_, ui32 pwmFrequency_, ui8 pwmResolution_): 
    feedForwardMap(11)
{
    outA = outA_;
    outB = outB_;

    encA = encA_;
    encB = encB_;
    encoderUnit = encoderUnit_;
    
    pwm = pwm_;
    pwmChannel = pwmChannel_;
    pwmFrequency = pwmFrequency_;
    pwmResolution = pwmResolution_;

    maxDuty = (1 << pwmResolution) - 1; // maxDuty = 2^(resolution) - 1
    
    
}

void Motor::begin(){
    /// Pin + PWM Init
    pinMode(outA, OUTPUT);
    pinMode(outB, OUTPUT);
    ledcSetup(pwmChannel, pwmFrequency, pwmResolution);
    ledcAttachPin(pwm,pwmChannel);

    /// Encoder Init
    // ==========================
    // Channel 0 : A -> Pulse, B -> Ctrl
    // ==========================
    pcnt_config_t cfg0 = {};

    ticks = 0;

    cfg0.pulse_gpio_num = encA;
    cfg0.ctrl_gpio_num  = encB;

    cfg0.unit = encoderUnit;
    cfg0.channel = PCNT_CHANNEL_0;

    cfg0.pos_mode = PCNT_COUNT_INC;
    cfg0.neg_mode = PCNT_COUNT_DEC;

    cfg0.lctrl_mode = PCNT_MODE_KEEP;
    cfg0.hctrl_mode = PCNT_MODE_REVERSE;

    cfg0.counter_h_lim = 32767;
    cfg0.counter_l_lim = -32768;

    pcnt_unit_config(&cfg0);

    // ==========================
    // Channel 1 : B -> Pulse, A -> Ctrl
    // ==========================
    pcnt_config_t cfg1 = {};

    cfg1.pulse_gpio_num = encB;
    cfg1.ctrl_gpio_num  = encA;

    cfg1.unit = encoderUnit;
    cfg1.channel = PCNT_CHANNEL_1;

    cfg1.pos_mode = PCNT_COUNT_DEC;
    cfg1.neg_mode = PCNT_COUNT_INC;

    cfg1.lctrl_mode = PCNT_MODE_KEEP;
    cfg1.hctrl_mode = PCNT_MODE_REVERSE;

    cfg1.counter_h_lim = 32767;
    cfg1.counter_l_lim = -32768;

    pcnt_unit_config(&cfg1);

    // ==========================

    pcnt_counter_pause(encoderUnit);

    pcnt_counter_clear(encoderUnit);

    pcnt_set_filter_value(encoderUnit, 80);
    pcnt_filter_enable(encoderUnit);

    pcnt_counter_resume(encoderUnit);
    lastTime = millis();
    updateSpeed();
}

void Motor::updateTicks(){
    int16_t count = 0;
    
    pcnt_counter_pause(encoderUnit);
    pcnt_get_counter_value(encoderUnit, &count);
    pcnt_counter_clear(encoderUnit);
    pcnt_counter_resume(encoderUnit);
    
    ticks += count;
}

i32 Motor::getTicks(){
    return ticks;
}

void Motor::resetTicks(){
    ticks = 0;
    // pcnt_counter_pause(encoderUnit);
    pcnt_counter_clear(encoderUnit);
    // pcnt_counter_resume(encoderUnit);
}

void Motor::setPWM(ui32 duty){
    if(duty > maxDuty) duty = maxDuty;
    ledcWrite(pwmChannel, duty);
}

void Motor::forward(){
    digitalWrite(outA, HIGH);
    digitalWrite(outB, LOW);
}

void Motor::backward(){
    digitalWrite(outA, LOW);
    digitalWrite(outB, HIGH);
}

void Motor::forward(ui32 duty){
    setPWM(duty);
    forward();
}
void Motor::backward(ui32 duty){
    setPWM(duty);
    backward();
}

void Motor::movePWM(i32 duty){
    if(duty < 0){
        backward(-duty);
    }else{
        forward(duty);
    }
}

void Motor::updateSpeed(){
    ui32 currentTime = millis();
    i32 currentTicks = getTicks();

    if(currentTime == lastTime) return;
    i32 deltaTime = currentTime - lastTime;
    i32 deltaTicks = currentTicks - lastTicks;

    speed = ((currentTicks - lastTicks)*1000) / deltaTime;
    // print(deltaTime); print(" "); print(deltaTicks); print(" "); print(deltaTicks*1000 / deltaTime);

    lastTime = currentTime;
    lastTicks = currentTicks;
}

void Motor::updateData(){
    updateTicks();
    updateSpeed();
}

i32 Motor::getSpeed(){
    return speed;
}

i32 Motor::getFeedForwardDuty(i32 tps){ /// return pwm duty best match tps
    bool isNegative = tps < 0;
    tps = abs(tps);

    if(tps <= feedForwardMap[0].tps){
        if(isNegative) return -feedForwardMap[0].actualDuty;
        return feedForwardMap[0].actualDuty;
    }
    if(tps >= feedForwardMap[10].tps){
        if(isNegative) return -feedForwardMap[9].actualDuty;
        return feedForwardMap[10].actualDuty;
    }

    i32 res;
    for(int i = 1; i < 11; i++){
        if(feedForwardMap[i].tps >= tps){
            i32 x0 = feedForwardMap[i-1].tps;
            i32 x1 = feedForwardMap[i].tps;

            i32 y0 = feedForwardMap[i-1].actualDuty;
            i32 y1 = feedForwardMap[i].actualDuty;

            if(x1 == x0) return isNegative ? -y0 : y0;

            res = y0 + (tps - x0) * (y1 - y0) / (x1 - x0);
            break;
        }
    }
    if(isNegative) return -res;
    return res;
}




void Motor::speedPID() {
    float sumError = 0.0f;
    float lastError = 0.0f;

    uint32_t lastTime = micros();

    const float sumErrorLimit = 1000.0f;

    while (true) {
        updateData();

        uint32_t currentTime = micros();
        float dt = (currentTime - lastTime) * 0.001f;   // ms

        if (dt <= 0.0f) {
            continue;
        }

        float error = (float)targetTps - (float)speed;

        if (targetTps == 0) {
            sumError = 0.0f;
            lastError = error;
        }

        float derivative = (error - lastError) / dt;

        // P + D + FeedForward
        float outputNoI =
            (float)getFeedForwardDuty(targetTps) +
            error * kP +
            derivative * kD;

        // Anti-windup
        if (fabs(outputNoI) < 1023.0f) {
            sumError += error * dt;
            sumError = constrain(sumError,
                                 -sumErrorLimit,
                                  sumErrorLimit);
        }

        // Thêm thành phần I
        float output = outputNoI + sumError * kI;

        output = constrain(output, -1023.0f, 1023.0f);

        movePWM((int)output);

        lastError = error;
        lastTime = currentTime;

        delay(5); /// 200 Hz
    }
}

void Motor::SpeedPIDWrapper(void *pv) {
    Motor *motor = (Motor*)pv;
    motor->speedPID();
    vTaskDelete(nullptr);     // thực tế sẽ không chạy tới đây
}


void Motor::setTargetTPS(i32 tps){
    targetTps = tps;
}