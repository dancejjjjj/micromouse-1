#pragma once
#include <Simple_MultiVL53L0X.h>
using ui8 = uint8_t;
using i16 = int16_t;
using ui16 = uint16_t;


struct Simple_Controls{
    const i16 VL53L0X_OFFSET[4] = {-40, -15, -15, -24};
    const float VL53L0X_SCALE[4] = {0.94, 1, 1.14, 0.98};
    // const i16 VL53L0X_OFFSET[4] = {0, 0, 0, 0};
    
    Simple_MultiVL53L0X toFSensors;

    void begin();
    ui16 getDistance(ui8 id);

    ui16 averageDistance(ui8 id){ /// calculate average of latest 10 raw input
        static int average10[4][10] = {{0}};
        int sum = 0;
        for(int i = 1; i < 10; i++){
            average10[id][i-1] = average10[id][i];
            sum += average10[id][i-1];
        }
        average10[id][9] = toFSensors.getDistance(id);  
        sum += average10[id][9]; 
        return sum/10;
    }

};