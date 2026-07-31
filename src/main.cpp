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




/// ================================================= JUST FOR FUN END =================================================


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


// Distance
volatile ui16 distL = 0;
volatile uint16_t distM1 = 0;
volatile uint16_t distM2 = 0;
volatile uint16_t distR = 0;


// ================================================= READ =================================================
void sensor_read(){
  const ui16 calibrateL = 2;
  const ui16 calibrateM1 = -7;
  const ui16 calibrateM2 = 0;
  const ui16 calibrateR = -30;

  distL = controls.getDistance(0);
  distR = controls.getDistance(3);
  //distM1 = controls.getDistance(1);
  distM2 = controls.getDistance(2);
}

// ================= DEBUG =================
void sensor_print(){
  print(">SensorL:"); print(distL); print(",");
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
    motorL.updateData();
    motorR.updateData();
  speedCalc();
  print(">motorL:"); print(motorL.getTicks()); print(",");
  print("motorR:"); print(motorR.getTicks()); print(",");
  print("speedL:"); print(motorL.getSpeed()); print(",");
  print("speedR:"); print(motorR.getSpeed()); print(","); println("");
}

bool wallFront() {
    const ui16 maxDistance = 100;
    uint32_t sum = 0;
    for (int i = 0; i < 3; i++) {
        sum += controls.getDistance(2);
        vTaskDelay(1); // Small delay to prevent RMT buffer overflow
    }
    distM2 = sum / 3;
    return distM2 < maxDistance;
}

bool wallLeft() {
    const ui16 maxDistance = 100;
    uint32_t sum = 0;
    for (int i = 0; i < 3; i++) {
        sum += controls.getDistance(0);
        vTaskDelay(1);
    }
    distL = sum / 3;
    return distL < maxDistance;
}

bool wallRight() {
    const ui16 maxDistance = 100;
    uint32_t sum = 0;
    for (int i = 0; i < 3; i++) {
        sum += controls.getDistance(3);
        vTaskDelay(1);
    }
    distR = sum / 3;
    return distR < maxDistance;
}



void moveByTicks_encPID(int targetTicks, int timeOut, int maxPWM) {
    motorL.resetTicks();
    motorR.resetTicks();

    unsigned long startTime = millis();

    // --- ENCODER PD GAINS (ƯU TIÊN CHÍNH) ---
    const float Kp_enc = 0.8f;  // Tăng nhẹ để bám đường thẳng tốt hơn
    const float Kd_enc = 0.5f;  
    float e_prev_enc = 0.0f;

    // --- WALL PD GAINS & SAFETY LIMITS (CHỈ CANH NHẸ) ---
    const float Kp_wall = 0.25f;    // Giảm hệ số Kp tường để không bị giật
    const float Kd_wall = 0.15f;    // Giảm hệ số Kd tường
    const float targetDist = 45.0f; // Khoảng cách mục tiêu đến tường (mm)
    
    // Giới hạn can thiệp của tường: chỉ cho phép chỉnh nhẹ (ví dụ 80/1023)
    const int maxWallCorr = 80;     

    // Valid wall range thresholds (mm)
    const float wallMaxDist = 100.0f;

    float e_wall_prev = 0.0f;
    bool hadWallLastFrame = false;

    // Minimum PWM thresholds for 10-bit PWM (0-1023)
    const int minStartPWM = 256; 
    const int minEndPWM   = 200;

    while (true) {
        motorL.updateData();
        motorR.updateData();

        int curL = abs(motorL.getTicks());
        int curR = abs(motorR.getTicks());
        int avgTicks = (curL + curR) / 2;

        // Exit conditions
        if (avgTicks >= targetTicks || (millis() - startTime) > (unsigned long)timeOut) break;

        // --- 1. RAMPING ---
        int currentBasePWM;
        
        if (avgTicks < targetTicks * 0.15f) {
            currentBasePWM = map(avgTicks, 0, targetTicks * 0.15f, minStartPWM, maxPWM);
        } else if (avgTicks > targetTicks * 0.80f) {
            currentBasePWM = map(avgTicks, targetTicks * 0.80f, targetTicks, maxPWM, minEndPWM);
        } else {
            currentBasePWM = maxPWM;
        }

        // --- 2. ENCODER PD DIFFERENTIAL CONTROL (LUÔN HOẠT ĐỘNG) ---
        float error_enc = (float)(curL - curR);
        float derivative_enc = error_enc - e_prev_enc;
        e_prev_enc = error_enc;

        float corr_enc = (Kp_enc * error_enc) + (Kd_enc * derivative_enc);


        // --- 3. SAFE WALL PD (CANH NHẸ) ---
        float distL = controls.getDistance(0);
        float distR = controls.getDistance(3);

        bool hasL = distL <= wallMaxDist;
        bool hasR = distR <= wallMaxDist;

        float e_wall = 0.0f;
        float corr_wall = 0.0f;
        bool hasAnyWall = hasL || hasR;

        if (hasL && hasR) {
            // Không set corr_enc = 0 nữa
            e_wall = distL - distR; 
        } else if (hasL) {
            e_wall = (distL - targetDist);
        } else if (hasR) {
            e_wall = (targetDist - distR);
        }

        if (hasAnyWall) {
            float d_wall = 0.0f;
            
            if (hadWallLastFrame) {
                d_wall = e_wall - e_wall_prev;
            }
            e_wall_prev = e_wall;
            hadWallLastFrame = true;

            // Tính toán lực can thiệp từ tường
            corr_wall = (Kp_wall * e_wall) + (Kd_wall * d_wall);

            // Chặn giới hạn PWM của Wall PID ở mức rất nhỏ (maxWallCorr)
            corr_wall = constrain(corr_wall, (float)-maxWallCorr, (float)maxWallCorr);
        } else {
            hadWallLastFrame = false;
            e_wall_prev = 0.0f;
            corr_wall = 0.0f;
        }

        // --- 4. MOTOR DRIVE ---
        // Kết hợp cả 2: Encoder điều phối chính, Wall chỉ bù thêm hoặc bớt đi một lượng nhỏ
        float totalCorr = corr_enc + corr_wall;

        int pwmL = currentBasePWM - (int)totalCorr;
        int pwmR = currentBasePWM + (int)totalCorr;

        motorL.movePWM(constrain(pwmL, -1023, 1023));
        motorR.movePWM(constrain(pwmR, -1023, 1023));
    }
    
    // Stop motors
    motorL.movePWM(0);
    motorR.movePWM(0);
}


void turnByTicks(i32 ticks, ui32 budget) {
    motorL.resetTicks();
    motorR.resetTicks();

    i32 targetAbs = abs(ticks);

    float Kp_enc  = 2.0f;
    float Ki_enc  = 0.02f;
    float Kd_enc  = 0.0f;

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

        //print_motor();
        vTaskDelay(5);
    }

    motorL.movePWM(0);
    motorR.movePWM(0);

    vTaskDelay(10);

}

/// ================================================= INTEGRATION =================================================

void moveForwardMotor(int cell) { 
    if (cell > 1) {
        moveByTicks_encPID(2370 * cell, 3500 * cell, 600);  
    } else {
        moveByTicks_encPID(2100 * cell, 3000 * cell, 600);
    }

    //print_motor();

    motorL.resetTicks();
    motorR.resetTicks();
    delay(5);
}

void turnLeft() {
    turnByTicks(-810, 3000);
    delay(5);
}

void turnRight() {
    turnByTicks(810, 3000);
    delay(5);
}

#include <cstdint>
#include <vector>
#include <queue>
#include <stack>


using i8 = int8_t;
using i16 = int16_t;
using i32 = int32_t;
using ui8 = uint8_t;
using ui16 = uint16_t;
using ui32 = uint32_t;

using namespace std;

struct cell {
    ui8 mask = 0b00001111; // 000vnswe - mouse visited - north - east - south - west
    ui8 value = 0; // value of the cell - use for BFS
    ui32 mask2 = 0; // xxxx...yyyy... - bitmask for dijkstra

    bool direction(char d) { /// return if mouse can go to specific direction in this cell.
        if(d == 'n') return north();
        if(d == 'e') return east();
        if(d == 's') return south();
        if(d == 'w') return west();
    }

    bool moveX(int n){
        return getBit2(n);
    }

    void moveX(int n, bool value){
        changeBit2(n, value);
    }

    bool moveY(int n){
        return getBit2(n + 16);
    }

    void moveY(int n, bool value){
        changeBit2(n + 16, value);
    }

    /// change bit
    void visit(bool tf){ // Set whether the robot visited this cell.
        changeBit(4, tf);
    }
    void north(bool tf){ // Set whether the robot can move NORTH within that cell.
        changeBit(3, tf);
    }
    void east(bool tf){ // Set whether the robot can move SOUTH within that cell.
        changeBit(2, tf);
    }
    void south(bool tf){ // Set whether the robot can move WEST within that cell.
        changeBit(1, tf);
    }
    void west(bool tf){ // Set whether the robot can move EAST within that cell.
        changeBit(0, tf);
    }

    /// get bit
    bool visit(){ // Return True if the robot visited this cell.
        return getBit(4);
    }
    bool north(){ // Return True if the robot can go to the NORTH, vice versa.
        return getBit(3);
    }
    bool east(){ // Return True if the robot can go to the SOUTH, vice versa.
        return getBit(2);
    }
    bool south(){ // Return True if the robot can go to the WEST, vice versa.
        return getBit(1);
    }
    bool west(){ // Return True if the robot can go to the EAST, vice versa.
        return getBit(0);
    }
    
    bool getBit(ui8 n) const {
        return (mask >> n) & 1;
    }
    void changeBit(ui8 n, bool value) {
        if (value) mask |= (1 << n);
        else mask &= ~(1 << n);
    }

    bool getBit2(ui8 n) const {
        return (mask2 >> n) & 1;
    }
    void changeBit2(ui8 n, bool value) {
        if (value) mask2 |= (1 << n);
        else mask2 &= ~(1 << n);
    }
};

cell Matrix[16][16];

/// =============================== GUI ===============================
// void printWall(int x, int y){
//     if(Matrix[x][y].north() == false){
//         setWall(x,y,'n');
//     }
//     if(Matrix[x][y].south() == false){
//         setWall(x,y,'s');
//     }
//     if(Matrix[x][y].west() == false){
//         setWall(x,y,'w');
//     }
//     if(Matrix[x][y].east() == false){
//         setWall(x,y,'e');
//     }
// }

// void printAllWall(){
//     for(int i = 0; i < 16; i++){
//         for(int j = 0; j < 16; j++){
//             printWall(i,j);
//         }
//     }
// }

// void printNumber(int x, int y){
//     setText(x,y,to_string(Matrix[x][y].value));
// }

// void printAllNumber(){
//     for(int i = 0; i < 16; i++){
//         for(int j = 0; j < 16; j++){
//             printNumber(i,j);
//         }
//     }
// }
/// ============================= GUI END =============================

void setBorder(){
    for(int i = 0; i < 16; i++){
        Matrix[i][0].south(false);
        Matrix[i][15].north(false);
        Matrix[0][i].west(false);
        Matrix[15][i].east(false);
    }
}

char mouseDirection = 'n';
ui8 mouseX = 0, mouseY = 0;

void refine(ui8 x, ui8 y){ // set wall for the nearest 4 cells
    i8 dx[] = {-1,0,0,1};
    i8 dy[] = {0,-1,1,0};

    for(int i = 0; i < 4; i++){
        i8 nextX = x + dx[i];
        i8 nextY = y + dy[i];

        if(nextX >= 0 && nextX < 16 && nextY >= 0 && nextY < 16){
            if(i == 0) Matrix[nextX][nextY].east(Matrix[x][y].west());
            if(i == 1) Matrix[nextX][nextY].north(Matrix[x][y].south());
            if(i == 2) Matrix[nextX][nextY].south(Matrix[x][y].north());
            if(i == 3) Matrix[nextX][nextY].west(Matrix[x][y].east());
        }
    }
}

void getWall(){ // pure miracle
    ui8 magic_number;
    ui8 magic_map [][3] = {
        {0,3,2},
        {3,2,1},
        {2,1,0},
        {1,0,3}
    };

    if(mouseDirection == 'n') magic_number = 0;
    if(mouseDirection == 'e') magic_number = 1;
    if(mouseDirection == 's') magic_number = 2;
    if(mouseDirection == 'w') magic_number = 3;

    Matrix[mouseX][mouseY].changeBit(magic_map[magic_number][0], !wallLeft());
    Matrix[mouseX][mouseY].changeBit(magic_map[magic_number][1], !wallFront());
    Matrix[mouseX][mouseY].changeBit(magic_map[magic_number][2], !wallRight());

    refine(mouseX,mouseY);
}

void bfs (int startX, int startY){
    for(int i = 0; i < 16; i++){
        for(int j = 0; j < 16; j++){
            Matrix[i][j].value = 0;
        }
    }
    queue<pair<int,int>> q; // store x,y coordinate
    q.push({startX,startY});
    Matrix[startX][startY].value = 1;

    while(!q.empty()){
        int currentX = q.front().first;
        int currentY = q.front().second;
        cell currentCell = Matrix[currentX][currentY];
        
        q.pop();
        
        if(currentCell.north() && Matrix[currentX][currentY+1].value == 0){
            Matrix[currentX][currentY+1].value = currentCell.value + 1;
            q.push({currentX, currentY+1});
        }
        
        if(currentCell.east() && Matrix[currentX+1][currentY].value == 0){
            Matrix[currentX+1][currentY].value = currentCell.value + 1;
            q.push({currentX+1, currentY});
        }

        if(currentCell.south() && Matrix[currentX][currentY-1].value == 0){
            Matrix[currentX][currentY-1].value = currentCell.value + 1;
            q.push({currentX, currentY-1});
        }

        if(currentCell.west() && Matrix[currentX-1][currentY].value == 0){
            Matrix[currentX-1][currentY].value = currentCell.value + 1;
            q.push({currentX-1, currentY});
        }
    }
}

void turnClockwise(int n){
    if(n >= 0){
        while(n--) turnRight();
    }else{
        while(n++) turnLeft();
    }
}

void turnToDirection(char d){
    if(d == mouseDirection) return;

    ui8 magic_number;
    ui8 magic_number2;

    if(mouseDirection == 'n') magic_number = 0;
    if(mouseDirection == 'e') magic_number = 1;
    if(mouseDirection == 's') magic_number = 2;
    if(mouseDirection == 'w') magic_number = 3;

    if(d == 'n') magic_number2 = 0;
    if(d == 'e') magic_number2 = 1;
    if(d == 's') magic_number2 = 2;
    if(d == 'w') magic_number2 = 3;

    i8 magic_map [][4] = {
        {0, 1, 2, -1},
        {-1, 0, 1, 2},
        {-2,-1, 0, 1},
        { 1, 2,-1, 0}
    };

    turnClockwise(magic_map[magic_number][magic_number2]);
    mouseDirection = d;

}

char getDirection (int x, int y){ // return optimal direction 
    if(mouseDirection == 'n' && Matrix[x][y].north() && Matrix[x][y+1].value < Matrix[x][y].value) return 'n';
    if(mouseDirection == 'e' && Matrix[x][y].east() && Matrix[x+1][y].value < Matrix[x][y].value) return 'e';
    if(mouseDirection == 's' && Matrix[x][y].south() && Matrix[x][y-1].value < Matrix[x][y].value) return 's';
    if(mouseDirection == 'w' && Matrix[x][y].west() && Matrix[x-1][y].value < Matrix[x][y].value) return 'w';

    if(Matrix[x][y].north() && Matrix[x][y+1].value < Matrix[x][y].value) return 'n';
    if(Matrix[x][y].east() && Matrix[x+1][y].value < Matrix[x][y].value) return 'e';
    if(Matrix[x][y].south() && Matrix[x][y-1].value < Matrix[x][y].value) return 's';
    if(Matrix[x][y].west() && Matrix[x-1][y].value < Matrix[x][y].value) return 'w';

    return 'E'; // ERROR - cannot find optimal direction - may the robot is already in center
}

void moveForward(int steps = 1){
    moveForwardMotor(steps);
    if(mouseDirection == 'n') mouseY += steps;
    if(mouseDirection == 'e') mouseX += steps;
    if(mouseDirection == 's') mouseY -= steps;
    if(mouseDirection == 'w') mouseX -= steps;
}



void moveTo (int targetX, int targetY){ // simple function that helps the robot move to a specific location - prioritize moving forward
    while(true){
        setColor(mouseX, mouseY, 'g');
        getWall();
        Matrix[mouseX][mouseY].visit(true);
        
        // printAllWall();

        bfs(targetX,targetY);

        // printAllNumber();

        char d = getDirection(mouseX,mouseY);
        if(d == 'E') break;
        
        int steps = 1;
        if(d == 'n'){
            int pseudoY = mouseY + steps;
            while(Matrix[mouseX][pseudoY].visit() && getDirection(mouseX,pseudoY) == d){
                steps++;
                pseudoY = mouseY + steps;
            }
        }else if(d == 's'){
            int pseudoY = mouseY - steps;
            while(Matrix[mouseX][pseudoY].visit() && getDirection(mouseX,pseudoY) == d){
                steps++;
                pseudoY = mouseY - steps;
            }
        }else if(d == 'e'){
            int pseudoX = mouseX + steps;
            while(Matrix[pseudoX][mouseY].visit() && getDirection(pseudoX,mouseY) == d){
                steps++;
                pseudoX = mouseX + steps;
            }
        }else if(d == 'w'){
            int pseudoX = mouseX - steps;
            while(Matrix[pseudoX][mouseY].visit() && getDirection(pseudoX,mouseY) == d){
                steps++;
                pseudoX = mouseX - steps;
            }
        }
        turnToDirection(d);
        moveForward(steps);
    }
}

pair<int,int> getNearestUndiscovered(int startX, int startY){
    for(int i = 0; i < 16; i++){
        for(int j = 0; j < 16; j++){
            Matrix[i][j].value = 0;
        }
    }
    queue<pair<int,int>> q; // store x,y coordinate
    q.push({startX,startY});
    Matrix[startX][startY].value = 1;

    while(!q.empty()){
        int currentX = q.front().first;
        int currentY = q.front().second;
        cell currentCell = Matrix[currentX][currentY];

        if (currentCell.visit() == false){
            return {currentX, currentY};
        }
        q.pop();
        
        if(currentCell.north() && Matrix[currentX][currentY+1].value == 0){

            Matrix[currentX][currentY+1].value = 1;
            q.push({currentX, currentY+1});
        }
        
        if(currentCell.east() && Matrix[currentX+1][currentY].value == 0){
            Matrix[currentX+1][currentY].value = 1;
            q.push({currentX+1, currentY});
        }

        if(currentCell.south() && Matrix[currentX][currentY-1].value == 0){
            Matrix[currentX][currentY-1].value = 1;
            q.push({currentX, currentY-1});
        }

        if(currentCell.west() && Matrix[currentX-1][currentY].value == 0){
            Matrix[currentX-1][currentY].value = 1;
            q.push({currentX-1, currentY});
        }
    }

    return {0,0};
}

void findAllUndiscovered(){
    while (true){
        pair<int,int> coordinate = getNearestUndiscovered(mouseX,mouseY);
        if(coordinate == make_pair(0,0)) return;
        moveTo(coordinate.first, coordinate.second);
    }
}

/// ====================== Dijkstra ======================
void initDijkstra(){ // create data for dijkstra to work
    for(int x = 0; x < 16; x++){
        for(int y = 0; y < 16; y++){
            for(int r = x; r < 16; r++){
                if(Matrix[r][y].north() || Matrix[r][y].south()){
                    Matrix[x][y].moveX(r, true);
                    Matrix[r][y].moveX(x, true);
                    // setColor(r,y,'G');
                }
                if(Matrix[r][y].east() == false){
                    break;
                }
            }
            
            // std::this_thread::sleep_for(std::chrono::milliseconds(500));
            
            for(int u = y; u < 16; u++){
                if(Matrix[x][u].east() || Matrix[x][u].west()){
                    Matrix[x][y].moveY(u,true);
                    Matrix[x][u].moveY(y,true);
                    // setColor(x,u,'G');
                }
                if(Matrix[x][u].north() == false){
                    break;
                }
            }
        }
    }
}

struct xy{ /// my struct to store 4-bit xy coordinate into 8-bit interger 
    ui8 mask = 0; // xxxxyyyy
    ui8 x(){
        return mask >> 4;
    }
    ui8 y(){
        return mask & 0b00001111;
    }
    void x(ui8 n){
        mask = (mask & 0b00001111) | (n << 4);
    }
    void y(ui8 n){
        mask = (mask & 0b11110000) | n;
    }
};

pair<int,int> extractXY(xy a){ // return pair{x,y} from xy structure
    return {a.x(), a.y()};
}


xy par[16][16];

ui8 turnCost = 4;
ui8 distanceCost[16] = {1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16};
void dijkstra(int startX, int startY){
    for(int i = 0; i < 16; i++){
        for(int j = 0; j < 16; j++){
            par[i][j].x(i);
            par[i][j].y(j);
            Matrix[i][j].value = UINT8_MAX;
        }
    }

    xy coordinate;
    coordinate.x(startX);
    coordinate.y(startY);

    priority_queue<pair<int,int>, vector<pair<int,int>>, greater<pair<int,int>>> pq;
    Matrix[startX][startY].value = 0;
    pq.push({0, coordinate.mask});

    while(!pq.empty()){
        i16 currentDistance = pq.top().first;
        xy coordinate1; //
        xy coordinate2; // temp variable
        coordinate1.mask = pq.top().second;
        i16 currentX = coordinate1.x();
        i16 currentY = coordinate1.y();
        pq.pop();

        if(Matrix[currentX][currentY].value < currentDistance) continue;

        for(i16 i = 0; i < 16; i++){
            if(Matrix[currentX][currentY].moveX(i) == true){
                if(currentDistance + distanceCost[abs(i - currentX)] + turnCost < Matrix[i][currentY].value){
                    par[i][currentY] = coordinate1;

                    Matrix[i][currentY].value = currentDistance + distanceCost[abs(i - currentX)] + turnCost;
                    coordinate2.x(i);
                    coordinate2.y(currentY);
                    pq.push({Matrix[i][currentY].value, coordinate2.mask});
                }

            }
            if(Matrix[currentX][currentY].moveY(i) == true){
                if(currentDistance + distanceCost[abs(i - currentY)] + turnCost < Matrix[currentX][i].value){
                    par[currentX][i] = coordinate1;

                    Matrix[currentX][i].value = currentDistance + distanceCost[abs(i - currentY)] + turnCost;
                    coordinate2.x(currentX);
                    coordinate2.y(i);
                    pq.push({Matrix[currentX][i].value, coordinate2.mask});
                }
            }
        }
    }    
}

void dijkstraTo(int x, int y){
    initDijkstra();
    dijkstra(mouseX, mouseY);

    stack<pair<int,int>> stk;
    stk.push({x,y});
    while(true){
        if(stk.top() == extractXY(par[stk.top().first][stk.top().second])){
            break;
        }
        stk.push(extractXY(par[stk.top().first][stk.top().second]));
    }

    while(!stk.empty()){
        int x = stk.top().first;
        int y = stk.top().second;
        setColor(x, y, 'b');
        moveTo(x,y);
        stk.pop();
    }
}

/// ==================== Dijkstra END ====================



/// ================================================= INTERGRATION END =================================================

void setup(){
  Serial.begin(115200);
  vTaskDelay(1000);


  
  

  
  
  motorL.begin();
  motorR.begin();
  
  //sensor_init();
  controls.begin();
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);
  
  vTaskDelay(5000);

  //   while(true){
    //     goTo(7, 7);
    //     // vTaskDelay(15000);
    //     //vTaskDelay(100);
    //     goTo(0,15);
    //   }
    
    setBorder();
    // printAllWall();
    
    moveTo(7,7);
    findAllUndiscovered();
    moveTo(0,0);
}


void loop() {

    //print_motor();
    
    // sensor_read();
    // print_sensor();
    // moveForward(2);
    // vTaskDelay(5000);
    // turnRight();
    // vTaskDelay(1000);
    
    // vTaskDelay(1000);
    //vTaskDelay(1000);
        dijkstraTo(7,7);
        dijkstraTo(0,0);

    // motorL.movePWM(512);
    // motorR.movePWM(512);

    // vTaskDelay(1000);

    // motorL.movePWM(0);
    // motorR.movePWM(0);

    // print_motor();
    // vTaskDelay(2000);

    // motorL.resetTicks();
    // motorR.resetTicks();

}