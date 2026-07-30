#include <Arduino.h>
#include <Adafruit_VL53L0X.h>
#include <Adafruit_NeoPixel.h>
#include <Wire.h>
#include <PCF8574.h>
#include <Motor.h>

/// STL Lib
#include <utility>
#include <vector>

#include <Simple_Controls.h>

Simple_Controls controls;

#define print_sensor sensor_print
#define print Serial.print
#define println Serial.println
using ui8 = uint8_t;
using ui16 = uint16_t;
using i32 = int32_t;
using ui32 = uint32_t;
using ui64 = uint64_t;

/// ================================================= JUST FOR FUN =================================================
#define RGB_PIN   48
#define NUM_PIXELS 1


ui8 ID = 0;
Adafruit_NeoPixel pixels(NUM_PIXELS, RGB_PIN, NEO_RBG + NEO_KHZ800);

void setColor(uint8_t r, uint8_t g, uint8_t b) {
  pixels.setPixelColor(0, pixels.Color(r, g, b));
  pixels.show();
}


void switch_led(uint8_t ID) {
  static uint8_t state = 0; // nhớ trạng thái giữa các lần gọi

  switch (state) {
    case 0: setColor(255, 0, 0); break;   // đỏ
    case 1: setColor(0, 255, 0); break;   // xanh lá
    case 2: setColor(0, 0, 255); break;   // xanh dương
    case 3: setColor(255, 255, 0); break; // vàng
    case 4: setColor(0, 255, 255); break; // cyan
    case 5: setColor(255, 0, 255); break; // tím
    //case 6: setColor(0, 0, 0); break;     // tắt
  }

  state = (state + 1) % 6; // quay vòng
}




/// ================================================= JUST FOR FUN END =================================================


/// ================================================= CONFIG =================================================


/// ================================================= ENCODER =================================================

// ticks
volatile int32_t ticksL = 0;
volatile int32_t ticksR = 0;

volatile i32 timeL;
volatile i32 timeR;
volatile i32 pastTimeL;
volatile i32 pastTimeR;
volatile i32 speedL;
volatile i32 speedR;

// nếu quay ngược thì đổi true/false
bool invertEncL = false;
bool invertEncR = true;

/// ================================================= MOTOR INSTANCE =================================================
Motor motorL( 5, 6,4, 1, 2,1, PCNT_UNIT_0);
Motor motorR(11,10,7,13,12,0, PCNT_UNIT_1);



/// ================================================= ToF SENSOR =================================================
#define SDA_PIN 8
#define SCL_PIN 9

// PCF8574 address
#define PCF_ADDR 0x20

// XSHUT mapping trên PCF (P0 → P3)
#define XSHUT_L   0
#define XSHUT_M1  1
#define XSHUT_M2  2
#define XSHUT_R   3

PCF8574 pcf(PCF_ADDR);

// Sensors
Adafruit_VL53L0X sensorL;
Adafruit_VL53L0X sensorM1;
Adafruit_VL53L0X sensorM2;
Adafruit_VL53L0X sensorR;

// Distance
volatile ui16 distL = 0;
volatile uint16_t distM1 = 0;
volatile uint16_t distM2 = 0;
volatile uint16_t distR = 0;

// Fiter
uint16_t lastL1=0, lastL2=0;
uint16_t lastM11=0, lastM12=0;
uint16_t lastM21=0, lastM22=0;
uint16_t lastR1=0, lastR2=0;

uint16_t median3(uint16_t a, uint16_t b, uint16_t c){
  if (a > b) { uint16_t t=a; a=b; b=t; }
  if (b > c) { uint16_t t=b; b=c; c=t; }
  if (a > b) { uint16_t t=a; a=b; b=t; }
  return b;
}

// ================================================= PCF CONTROL =================================================
void xshut_write(uint8_t pin, bool level){
  pcf.write(pin, level ? HIGH : LOW);
}

// ================================================= INIT =================================================
bool init_sensor(Adafruit_VL53L0X &sensor, uint8_t xshutPin, uint8_t addr, const char* name){
  xshut_write(xshutPin, HIGH);
  vTaskDelay(50); // ⚠️ quan trọng với PCF

  while(!sensor.begin()){
    print("Retry ");
    println(name);
    vTaskDelay(50);
  }

  sensor.setAddress(addr);

  print("[OK] ");
  print(name);
  print(" @ 0x");
  println(addr, HEX);

  return true;
}

void sensor_init(){
  println("=== Sensor init (PCF8574) ===");

  Wire.begin(SDA_PIN, SCL_PIN);
  vTaskDelay(10);

  pcf.begin();
  vTaskDelay(10);

  // Tắt hết
  for(int i = 0; i < 4; i++){
    xshut_write(i, LOW);
  }
  vTaskDelay(100);

  // Init từng sensor
  init_sensor(sensorL,  XSHUT_L,  0x30, "L");
  init_sensor(sensorM1, XSHUT_M1, 0x31, "M1");
  init_sensor(sensorM2, XSHUT_M2, 0x32, "M2");
  init_sensor(sensorR,  XSHUT_R,  0x33, "R");

  // Timing budget
  const ui32 budget = 5000;
  sensorL.setMeasurementTimingBudgetMicroSeconds(budget);
  sensorM1.setMeasurementTimingBudgetMicroSeconds(budget);
  sensorM2.setMeasurementTimingBudgetMicroSeconds(budget);
  sensorR.setMeasurementTimingBudgetMicroSeconds(budget);

  vTaskDelay(10);

  // Continuous mode
  sensorL.startRangeContinuous(5);
  sensorM1.startRangeContinuous(5);
  sensorM2.startRangeContinuous(5);
  sensorR.startRangeContinuous(5);

  println("=== Sensor READY ===");
}

// ================================================= READ =================================================
void sensor_read(){
  const ui16 calibrateL = 2;
  const ui16 calibrateM1 = -7;
  const ui16 calibrateM2 = 0;
  const ui16 calibrateR = -30;

  distL = controls.getDistance(0);
  //distM1 = controls.getDistance(1);
  distM2 = controls.getDistance(2);
  distR = controls.getDistance(3);

  // // ===== L =====
  // if(sensorL.isRangeComplete()){
  //   uint16_t raw = sensorL.readRangeResult() + calibrateL;

  //   uint16_t med = median3(lastL2, lastL1, raw);
  //   distL = distL * 0.7 + med * 0.3;

  //   lastL2 = lastL1;
  //   lastL1 = raw;
  // }

  // // ===== M1 =====
  // if(sensorM1.isRangeComplete()){
  //   uint16_t raw = sensorM1.readRangeResult() + calibrateM1;

  //   uint16_t med = median3(lastM12, lastM11, raw);
  //   distM1 = distM1 * 0.7 + med * 0.3;

  //   lastM12 = lastM11;
  //   lastM11 = raw;
  // }

  // // ===== M2 =====
  // if(sensorM2.isRangeComplete()){
  //   uint16_t raw = sensorM2.readRangeResult() + calibrateM2;

  //   uint16_t med = median3(lastM22, lastM21, raw);
  //   distM2 = distM2 * 0.7 + med * 0.3;

  //   lastM22 = lastM21;
  //   lastM21 = raw;
  // }

  // // ===== R =====
  // if(sensorR.isRangeComplete()){
  //   uint16_t raw = sensorR.readRangeResult() + calibrateR;

  //   uint16_t med = median3(lastR2, lastR1, raw);
  //   distR = distR * 0.7 + med * 0.3;

  //   lastR2 = lastR1;
  //   lastR1 = raw;
  // }
}

// void sensor_read(){
//   const ui16 calibrateL = -22;
//   const ui16 calibrateM1 = 0;
//   const ui16 calibrateM2 = 0;
//   const ui16 calibrateR = -20;
//   // ===== L =====
//   if(sensorL.isRangeComplete()){
//     distL = sensorL.readRangeResult() + calibrateL;
//   }

//   // ===== M1 =====
//   if(sensorM1.isRangeComplete()){
//     distM1 = sensorM1.readRangeResult() + calibrateM1;
//   }

//   // ===== M2 =====
//   if(sensorM2.isRangeComplete()){
//     distM2 = sensorM2.readRangeResult() + calibrateM2;
//   }

//   // ===== R =====
//   if(sensorR.isRangeComplete()){
//     distR = sensorR.readRangeResult() + calibrateR;
//   }
// }

// ================= DEBUG =================
void sensor_print(){
  print(">SensorL:"); print(distL); print(",");
  print("SensorM1:"); print(distM1); print(",");
  print("SensorM2:"); print(distM2); print(",");
  print("SensorR:"); print(distR); println(",");
}

/// ================================================= ToF SENSOR END =================================================

/// ================================================= Speed =================================================
void speedCalc(){
  uint32_t tL = timeL;
  uint32_t pL = pastTimeL;
  uint32_t dtL = tL - pL;

  if(micros() - tL > 30000) speedL = 0;  
  else if(dtL > 20){
    speedL = 1000000.0f / dtL;
  }
  
  
  uint32_t tR = timeR;
  uint32_t pR = pastTimeR;
  uint32_t dtR = tR - pR;
  if(micros() - tR > 30000) speedR = 0;  
  else if(dtR > 20){
    speedR = 1000000.0f / dtR;
  }
}
/// ================================================= Speed END =================================================

/// ================================================= Better controls =================================================

void print_motor(){
  speedCalc();
  print(">motorL:"); print(motorL.getTicks()); print(",");
  print("motorR:"); print(motorR.getTicks()); print(",");
  print("speedL:"); print(motorL.getSpeed()); print(",");
  print("speedR:"); print(motorR.getSpeed()); print(","); println("");
}


void moveByTicks(int targetTicks, int timeOut, int maxPWM) {
    // Reset ticks về 0 để tránh tích lũy sai số từ lần gọi trước
    motorL.resetTicks();
    motorR.resetTicks();
 
    unsigned long startTime = millis();
 
    // Tất cả biến PID khai báo LOCAL để mỗi lần gọi đều bắt đầu sạch
    float e_enc_prev = 0;
    float e_enc_sum  = 0;
    float Ki_enc     = 0.0f;
    float Kd_enc     = 0.5f;
 
    float Kp_enc   = 1.0f;
    float Kp_wall  = 1.0f; // Scaled up slightly for 10-bit PWM range
    float targetDist = 45.0f;

    // Minimum PWM threshold to overcome static friction (10-bit PWM: 0-1023)
    const int minStartPWM = 256; 
    const int minEndPWM   = 256;
 
    while (true) {
        motorL.updateData();
        motorR.updateData();
        int curL = abs(motorL.getTicks());
        int curR = abs(motorR.getTicks());
        int avgTicks = (curL + curR) / 2;
 
        // Điều kiện dừng: đạt số tick hoặc hết thời gian
        if (avgTicks >= targetTicks || (millis() - startTime) > (unsigned long)timeOut) break;
 
        // --- 1. TÍNH TOÁN BASE PWM (RAMPING) ---
        int currentBasePWM;
        
        if (avgTicks < targetTicks * 0.2f) {
            // Ramp up from starting torque threshold to maxPWM
            currentBasePWM = map(avgTicks, 0, targetTicks * 0.2f, minStartPWM, maxPWM);
        } else if (avgTicks > targetTicks * 0.7f) {
            // Ramp down to minimum stopping PWM
            currentBasePWM = map(avgTicks, targetTicks * 0.7f, targetTicks, maxPWM, minEndPWM);
        } else {
            currentBasePWM = maxPWM;
        }
 
        // --- 2. TÍNH TOÁN SAI SỐ ENCODER (PID) ---
        float e_enc = (float)(curL - curR);
        e_enc_sum += e_enc;
        e_enc_sum = constrain(e_enc_sum, -300.0f, 300.0f); 
        float d_enc = e_enc - e_enc_prev;
        e_enc_prev = e_enc;
 
        float corr = (Kp_enc * e_enc) + (Ki_enc * e_enc_sum) + (Kd_enc * d_enc);
 
        // --- 3. KẾT HỢP CẢM BIẾN TƯỜNG ---
        sensor_read(); // Cập nhật distL, distR
        
        bool hasL = distL < 120;
        bool hasR = distR < 120;
        float e_wall = 0;
        float w_wall = 0;
 
        if (hasL && hasR) {
            e_wall = distL - distR;
            w_wall = 1.0f;
        }
        else if (hasL) {
            e_wall = distL - targetDist;
            w_wall = 0.7f;
        }
        else if (hasR) {
            e_wall = targetDist - distR;
            w_wall = 0.7f;
        }
        
        // Cộng dồn sai số tường vào hệ số bù
        corr += (Kp_wall * e_wall * w_wall);
 
        // --- 4. ĐIỀU KHIỂN ĐỘNG CƠ ---
        int pwmL = currentBasePWM - (int)corr;
        int pwmR = currentBasePWM + (int)corr;
 
        // Constrain to 10-bit PWM limits (-1023 to 1023)
        motorL.movePWM(constrain(pwmL, -1023, 1023));
        motorR.movePWM(constrain(pwmR, -1023, 1023));
    }
    
    motorL.movePWM(0);
    motorR.movePWM(0);
}


void turnByTicks(i32 ticks, ui32 budget) {
    motorL.resetTicks();
    motorR.resetTicks();

    i32 targetAbs = abs(ticks);

    float Kp_enc  = 1.0f;
    float Ki_enc  = 0.02f;
    float Kd_enc  = 1.0f;

    float e_enc_prev = 0;
    float e_enc_sum  = 0;

    int maxPWM    = 512;   // Giới hạn maxPWM quay tại chỗ để tránh trượt bánh
    int minPWM    = 220;   // ĐÃ SỬA: Giảm từ 512 xuống 220 để xe chậm hẳn lại trước khi dừng
    int tolerance = 2;

    long startTime = millis();

    while (true) {
        motorL.updateData();
        motorR.updateData();
        
        i32 doneL = abs(motorL.getTicks());
        i32 doneR = abs(motorR.getTicks());

        i32 errL = targetAbs - doneL;
        i32 errR = targetAbs - doneR;

        if ((errL <= tolerance && errR <= tolerance) || (millis() - startTime) > budget) {
            break;
        }

        // --- 1. RAMPING ---
        int avgDone = (doneL + doneR) / 2;
        int basePWM;
        if (avgDone < targetAbs * 0.25f) {
            basePWM = map(avgDone, 0, (int)(targetAbs * 0.25f), minPWM, maxPWM);
        } else if (avgDone > targetAbs * 0.50f) {
            basePWM = map(avgDone, (int)(targetAbs * 0.50f), targetAbs, maxPWM, minPWM);
        } else {
            basePWM = maxPWM;
        }
        basePWM = constrain(basePWM, minPWM, maxPWM);

        // --- 2. PID ĐỒNG BỘ 2 MOTOR ---
        float e_enc = (float)(errL - errR);
        e_enc_sum += e_enc;
        e_enc_sum = constrain(e_enc_sum, -150, 150);
        float d_enc = e_enc - e_enc_prev;
        e_enc_prev = e_enc;

        float corr = Kp_enc * e_enc + Ki_enc * e_enc_sum + Kd_enc * d_enc;

        int pwmL = constrain(basePWM + (int)corr, minPWM, 1023);
        int pwmR = constrain(basePWM - (int)corr, minPWM, 1023);

        // --- 3. ÁP PWM ĐÚNG CHIỀU ---
        if (ticks > 0) {        // Quẹo phải
            motorL.forward(pwmL);
            motorR.backward(pwmR);
        } else {                // Quẹo trái
            motorL.backward(pwmL);
            motorR.forward(pwmR);
        }

        print_motor();
        vTaskDelay(5);
    }

    motorL.movePWM(0);
    motorR.movePWM(0);
    vTaskDelay(10);
}

/// ================================================= INTEGRATION =================================================

void moveForward(int cell) { 
    if (cell > 1) {
        moveByTicks(2150 * cell, 3500 * cell, 600);  
    } else {
        moveByTicks(2150 * cell, 3000 * cell, 600);
    }
}

void turnLeft() {
    turnByTicks(-800, 3000);
}

void turnRight() {
    turnByTicks(800, 3000);
}


bool wallFront() {
    sensor_read();
    const ui16 maxDistance = 100;
    //return (distM1 + distM2) < (maxDistance * 2);
    return distM2 < maxDistance;
}
  
bool wallLeft() {
    sensor_read();
    const ui16 maxDistance = 100;
    return distL < maxDistance; 
}
  
bool wallRight() {
    sensor_read();
    const ui16 maxDistance = 100;
    return distR < maxDistance; 
}



int N = 16; /// size of the Maze
int myPosX = 0;
int myPosY = 15;

struct static_queue{
    std::pair<ui8,ui8> array[255];
    ui8 capacity = 0;
    ui8 L = 0, R = 0;

    std::pair<ui8,ui8> front(){
        return array[R];
    }
    void pop(){
        if(R == 0){
            R = 254;
        }else{
            R --;
        }
        --capacity;
    }
    void push(std::pair<ui8,ui8> input){
        array[L] = input;
        if(L == 0){
            L = 254;
        }else{
            L --;
        }
        ++capacity;
    }

    ui8 empty(){
        return !capacity;
    }
};


// DEBUG
//ofstream out("output.txt");

struct cell {
    ui8 distance = 0;
	bool can_go_up = true;
	bool can_go_down = true;
	bool can_go_left = true;
	bool can_go_right = true;
    bool vis = 0;
};
std::vector<std::vector<cell>> Matrix;

// DEBUG
//void log(const std::string& text) {
//    std::cerr << text << std::endl;
//}

// DEBUG
/// Convert my coordinate to simulator coordinate
//int cvX(int X){
//    return X;
//}
//int cvY(int Y){
//    return N-1-Y;
//}

void makeBorder(int Size) {
    for (int i = 0; i < Size; i++) {
        Matrix[0][i].can_go_left = false;
		Matrix[Size - 1][i].can_go_right = false;
		Matrix[i][0].can_go_up = false;
		Matrix[i][Size - 1].can_go_down = false;
	}
}

// DUBUG
// void cellDebug(int X, int Y){
//     cell Cell = Matrix[X][Y];
//     out << X << ' ' << Y << endl;
//     out << "Distance: " << Cell.distance << endl;
//     out << "Up: " << Cell.can_go_up << endl;
//     out << "Down: " << Cell.can_go_down << endl;
//     out << "Left: " << Cell.can_go_left << endl;
//     out << "Right: " << Cell.can_go_right << endl;
//     out << "--------------\n";
//     out.flush();
// }

void clearMatrix(){
    cell cleanCell;
    for(auto& i:Matrix){
        for(auto& cell: i){
            cell.distance = 0;
        }
    }
}

// DEBUG
// void setText(){
//     for(int x = 0; x < N; x++){
//         for(int y = 0; y < N; y++){
//             setText(cvX(x), cvY(y),to_string(Matrix[x][y].distance));
//         }
//     }
// }

// DEBUG
// void setColor(){
//     int CenterPos1 = N / 2;
//     int CenterPos2 = N / 2 - 1;
    
//     setColor(cvX(CenterPos1), cvY(CenterPos1),  'g');
//     setColor(cvX(CenterPos1), cvY(CenterPos2),  'g');
//     setColor(cvX(CenterPos2), cvY(CenterPos1),  'g');
//     setColor(cvX(CenterPos2), cvY(CenterPos2),  'g');
// }

void bfsType02(int toX, int toY) {
    clearMatrix();
    /// set where to
    Matrix[toX][toY].distance = 1;
    
    /// bfs from center
    static_queue MyQueue;
    MyQueue.push({ toX, toY});
    
    while (MyQueue.empty() == false) { /// not empty
        ui8 CurX = MyQueue.front().first;
        ui8 CurY = MyQueue.front().second;
        MyQueue.pop();

        if (Matrix[CurX][CurY].can_go_up) {
            if (Matrix[CurX][CurY-1].distance == 0) { /// = 0 means not visited
                Matrix[CurX][CurY - 1].distance = Matrix[CurX][CurY].distance + 1;
                MyQueue.push({ CurX,CurY - 1 });
            }
        }
        if (Matrix[CurX][CurY].can_go_down) {
            if (Matrix[CurX][CurY+1].distance == 0) { /// = 0 means not visited
                Matrix[CurX][CurY + 1].distance = Matrix[CurX][CurY].distance + 1;
                MyQueue.push({ CurX,CurY + 1 });
            }
        }
        if (Matrix[CurX][CurY].can_go_left) {
            if (Matrix[CurX-1][CurY].distance == 0) { /// = 0 means not visited
                Matrix[CurX-1][CurY].distance = Matrix[CurX][CurY].distance + 1;
                MyQueue.push({ CurX-1,CurY});
            }
        }
        if (Matrix[CurX][CurY].can_go_right) {
            if (Matrix[CurX+1][CurY].distance == 0) { /// = 0 means not visited
                Matrix[CurX+1][CurY].distance = Matrix[CurX][CurY].distance + 1;
                MyQueue.push({ CurX+1,CurY});
            }
        }	
    }
}


// DEBUG
// void printDistance(vector<vector<cell>>& A) {
//     for(int x = 0; x < A.size(); x++){
//         for(int y = 0; y < A.size(); y++){
//             int i = cvX(x);
//             int j = cvY(y);
//             if(Matrix[x][y].can_go_up == false){
//                 setWall(i,j,'n');
//             }
//             if(Matrix[x][y].can_go_down == false){
//                 setWall(i,j,'s');
//             }
//             if(Matrix[x][y].can_go_left == false){
//                 setWall(i,j,'w');
//             }
//             if(Matrix[x][y].can_go_right == false){
//                 setWall(i,j,'e');
//             }
//         }
//     }
// }

char mouseDirection = 'n';

char findDirection(int x, int y) { // nếu có đường rẽ bằng đường đi thẳng thì chọn đường đi thẳng
    int minValue = INT_MAX;
    char minPath = 0;
    if(Matrix[x][y].can_go_up){
        if(Matrix[x][y-1].distance < minValue){
            minValue = Matrix[x][y-1].distance;
            minPath = 'n';
        }
    }
    if(Matrix[x][y].can_go_down){
        if(Matrix[x][y+1].distance < minValue){
            minValue = Matrix[x][y+1].distance;
            minPath = 's';
        }
    }
    if(Matrix[x][y].can_go_left){
        if(Matrix[x-1][y].distance < minValue){
            minValue = Matrix[x-1][y].distance;
            minPath = 'w';
        }
    }
    if(Matrix[x][y].can_go_right){
        if(Matrix[x+1][y].distance < minValue){
            minValue = Matrix[x+1][y].distance;
            minPath = 'e';
        }
    }
    return minPath;
}

void spinningBaby(char direction)
{
    switch (direction){
        case 'n': /// want to go up
            if(mouseDirection == 'e'){
                turnLeft();
            }else if(mouseDirection == 'w'){
                 turnRight();
            }else if(mouseDirection == 's'){
                turnLeft();
                turnLeft();
            }
            mouseDirection = 'n';
            break;
            case 's':
            if(mouseDirection == 'e'){
                turnRight();
            }else if(mouseDirection == 'w'){
                 turnLeft();
            }else if(mouseDirection == 'n'){
                turnLeft();
                turnLeft();
            }
            mouseDirection = 's';
            break;
        case 'e': /// to the right
            if(mouseDirection == 'n'){
                turnRight();
            }else if(mouseDirection == 's'){
                 turnLeft();
            }else if(mouseDirection == 'w'){
                turnLeft();
                turnLeft();
            }
            mouseDirection = 'e';
            break;
            case 'w': /// to the left
            if(mouseDirection == 'n'){
                turnLeft();
            }else if(mouseDirection == 's'){
                 turnRight();
            }else if(mouseDirection == 'e'){
                turnLeft();
                turnLeft();
            }
            mouseDirection = 'w';
            break;
        }
}


void move(int fromX, int fromY, char direction)
{
    int predPosX = myPosX;
    int predPosY = myPosY;
    int blocks = 0;
    while (Matrix[predPosX][predPosY].vis == 1 and findDirection(predPosX, predPosY) == mouseDirection){
        blocks++;
        switch (direction){
        case 'n': /// want to go up
            mouseDirection = 'n';
            predPosY-=1;
            break;
        case 's':
            mouseDirection = 's';
            predPosY+=1;
            break;
        case 'e': /// to the right
            mouseDirection = 'e';
            predPosX+=1;
            break;
        case 'w': /// to the left
            predPosX-=1;
            mouseDirection = 'w';
            break;
        }
    }
    myPosX = predPosX;
    myPosY = predPosY;
    moveForward(blocks);
}
    
void updateWall(int x, int y) {
    // Sample sensors 3 times for stability before checking walls (takes ~45ms total instead of 500ms
    if (mouseDirection == 'n') {
        if (wallFront()) Matrix[x][y].can_go_up = false;
        if (wallLeft())  Matrix[x][y].can_go_left = false;
        if (wallRight()) Matrix[x][y].can_go_right = false;  
    }
    else if (mouseDirection == 's') {
        if (wallFront()) Matrix[x][y].can_go_down = false;
        if (wallLeft())  Matrix[x][y].can_go_right = false;
        if (wallRight()) Matrix[x][y].can_go_left = false;  
    }
    else if (mouseDirection == 'e') {
        if (wallFront()) Matrix[x][y].can_go_right = false;
        if (wallLeft())  Matrix[x][y].can_go_up = false;
        if (wallRight()) Matrix[x][y].can_go_down = false;  
    }
    else if (mouseDirection == 'w') {
        if (wallFront()) Matrix[x][y].can_go_left = false;
        if (wallLeft())  Matrix[x][y].can_go_down = false;
        if (wallRight()) Matrix[x][y].can_go_up = false;  
    }

    cell currentCell = Matrix[x][y]; 
    if (!currentCell.can_go_up    && y - 1 >= 0) Matrix[x][y-1].can_go_down = false;
    if (!currentCell.can_go_down  && y + 1 < N)  Matrix[x][y+1].can_go_up   = false;
    if (!currentCell.can_go_left  && x - 1 >= 0) Matrix[x-1][y].can_go_right = false;
    if (!currentCell.can_go_right && x + 1 < N)  Matrix[x+1][y].can_go_left  = false;
}



void goTo(int x, int y){

while(myPosX != x || myPosY != y){ /// go back

updateWall(myPosX, myPosY);

Matrix[myPosX][myPosY].vis = 1;

bfsType02(x,y);

//setText();

char d = findDirection(myPosX, myPosY);

if (mouseDirection != d) spinningBaby(d);

move(myPosX, myPosY, d);

//API::setColor(cvX(myPosX), cvY(myPosY), 'c');

switch_led((ID++) % 6);

}

} 


/// ================================================= INTERGRATION END =================================================

void setup(){
  Serial.begin(115200);
  vTaskDelay(1000);


  Matrix.resize(N, std::vector<cell>(N));
  makeBorder(16);
  
  motorL.begin();
  motorR.begin();
  
  //sensor_init();
  controls.begin();
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);

  
  vTaskDelay(5000);

  while(true){
    goTo(7, 8);
    // vTaskDelay(15000);
    //vTaskDelay(100);
    goTo(0,15);
  }
  
}


void loop() {

    // sensor_read();
    // sensor_print();
    // print(wallFront); print(" ");
    // print(wallLeft); print(" ");
    // print(wallRight); println(" ");
    // turnLeft();
    // vTaskDelay(100);
    // turnRight();
    // vTaskDelay(100);
  
  

  //print_motor();
  
  // moveForward(1);

  // sensor_read();
  // sensor_print();

  // moveForward(4);
  // vTaskDelay(5000);
  // moveByTicks(280, 3000);

  // print(">");
  // print("L:"); print(wallLeft()); print(",");
  // print("R:"); print(wallRight()); print(",");
  // print("Front:"); print(wallFront()); print(",");
  // println("");
  // sensor_read();
  // print_sensor();

  // vTaskDelay(3000);
  // sensor_read();
  // sensor_print();
  // vTaskDelay(2);


}