#include <Simple_Controls.h>

void Simple_Controls::begin(){
    toFSensors.begin();
}

ui16 Simple_Controls::getDistance(ui8 id){
    ui16 distance = (toFSensors.getDistance(id) + VL53L0X_OFFSET[id]) / VL53L0X_SCALE[id];
    if(distance < 500)
     return  (distance);
    else{
        return 500;
    }
}

