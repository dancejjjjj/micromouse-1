#include <Simple_MultiVL53L0X.h>

Adafruit_VL53L0X Simple_MultiVL53L0X::sensorsArray[4];

void Simple_MultiVL53L0X::begin(){
    Wire.begin();
    PCF8574 pcf(0x20);


    if(pcf.begin()){
      println("PCF8574 [  OK  ]");
    }else{
      println("PCF8574 [FAILED]");
    }

    pcf.write8(0b11111111);    
    
    for(int i = 0; i < 4; i++){
      if (i == 1) continue;
      pcf.write(i, LOW);
      delay(15);
      pcf.write(i, HIGH);
      delay(15);
      sensorsArray[i].begin();
      sensorsArray[i].setAddress(0x30 + i);
      sensorsArray[i].setMeasurementTimingBudgetMicroSeconds(20000);
      sensorsArray[i].setDeviceMode(VL53L0X_DEVICEMODE_CONTINUOUS_RANGING);
      sensorsArray[i].startRangeContinuous();
    }
}

ui16 Simple_MultiVL53L0X::getDistance (ui8 id){
    return sensorsArray[id].readRange();
}