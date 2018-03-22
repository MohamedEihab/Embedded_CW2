#include "mbed.h"
#include "SHA256.h"
#include "rtos.h"

//Photointerrupter input pins
#define I1pin D2
#define I2pin D11
#define I3pin D12

//Incremental encoder input pins
#define CHA   D7
#define CHB   D8  

//Motor Drive output pins   //Mask in output byte
#define L1Lpin D4           //0x01
#define L1Hpin D5           //0x02
#define L2Lpin D3           //0x04
#define L2Hpin D6           //0x08
#define L3Lpin D9           //0x10
#define L3Hpin D10          //0x20

//Mapping from sequential drive states to motor phase outputs
/*
State   L1  L2  L3
0       H   -   L
1       -   H   L
2       L   H   -
3       L   -   H
4       -   L   H
5       H   L   -
6       -   -   -
7       -   -   -
*/
//Drive state to output table
const int8_t driveTable[] = {0x12,0x18,0x09,0x21,0x24,0x06,0x00,0x00};

//Mapping from interrupter inputs to sequential rotor states. 0x00 and 0x07 are not valid
const int8_t stateMap[] = {0x07,0x05,0x03,0x04,0x01,0x00,0x02,0x07};  
//const int8_t stateMap[] = {0x07,0x01,0x03,0x02,0x05,0x00,0x04,0x07}; //Alternative if phase order of input or drive is reversed

//Phase lead to make motor spin
int8_t lead = 2;  //2 for forwards, -2 for backwards

//Status LED
DigitalOut led1(LED1);

//Photointerrupter inputs
InterruptIn I1(I1pin);
InterruptIn I2(I2pin);
InterruptIn I3(I3pin);

//Motor Drive outputs
PwmOut L1L(L1Lpin);
DigitalOut L1H(L1Hpin);
PwmOut L2L(L2Lpin);
DigitalOut L2H(L2Hpin);
PwmOut L3L(L3Lpin);
DigitalOut L3H(L3Hpin);

//

RawSerial pc(SERIAL_TX, SERIAL_RX);

//

Thread commOutT(osPriorityAboveNormal,1024);
Thread commInT(osPriorityAboveNormal,1024);
Thread motorCtrlT(osPriorityNormal,1024);
Thread BitcoinT(osPriorityNormal,1024);

//
void putMessage(uint8_t code, uint32_t data);
void motorISR();
void motorCtrlFn();
void motorCtrlTick();
void commOutFn();
void commInFn();
void setMotor();
int8_t motorHome();
void bitcoinFn();
void serialISR();

//

typedef struct{ 
    uint8_t code;
    uint32_t data; 
} message_t ;

//

Mail<message_t,16> outMessages;

//

Queue<void, 8> inCharQ;

//

int maxspeed = 0;
float speedout;
float setpos;
int8_t orState = 0; 
int8_t intState = 0;
int8_t intStateOld = 0;
uint32_t setTorque = 0;
uint32_t setTorqueS = 0;
uint32_t setTorqueR = 0;
char charBuffer[20];
volatile uint8_t charBufferIndex = 0;
bool termination = false;
bool command = false;
int32_t lastPosition;
int32_t velocity;
float lastErr;
float currentErr;
int32_t motorPosition; 
volatile uint64_t newKey;
Mutex newKey_mutex;
Mutex setTorque_mutex;
bool setSpeed = false;
int i = 0;

//

uint8_t sequence[] = {0x45,0x6D,0x62,0x65,0x64,0x64,0x65,0x64, 
0x20,0x53,0x79,0x73,0x74,0x65,0x6D,0x73, 0x20,0x61,0x72,0x65,0x20,
0x66,0x75,0x6E, 0x20,0x61,0x6E,0x64,0x20,0x64,0x6F,0x20, 0x61,0x77,
0x65,0x73,0x6F,0x6D,0x65,0x20, 0x74,0x68,0x69,0x6E,0x67,0x73,0x21,
0x20, 0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00, 0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00};
uint64_t* key = (uint64_t*)((int)sequence + 48); 
uint64_t* nonce = (uint64_t*)((int)sequence + 56); 
uint8_t hash[32];

//Main
int main() {
    L1L.period_us(2000);
    L2L.period_us(2000);
    L3L.period_us(2000);

    orState = motorHome();

    I1.rise(&motorISR);
    I2.rise(&motorISR);
    I3.rise(&motorISR);
    I1.fall(&motorISR);
    I2.fall(&motorISR);
    I3.fall(&motorISR);
    

//    L1L.write(0.25);
//    L2L.write(0.25);
//    L3L.write(0.25);


    pc.attach(&serialISR);
    
    commOutT.start(commOutFn);
    motorCtrlT.start(motorCtrlFn);
    commInT.start(commInFn);
    BitcoinT.start(bitcoinFn);
   
    
}

void bitcoinFn(){
    Timer count;
    count.start();
    int hash_count = 0;
    int current_time = 0;
    int previous_time = 0;
    int hashes_calculated = 0;

    while (1){
        newKey_mutex.lock();
        *key = newKey;
        newKey_mutex.unlock();
            
        SHA256::computeHash(&hash[0], &sequence[0] , sizeof(sequence)/sizeof(sequence[0]));
        hash_count += 1;
        
        if ((hash[0] == 0) && (hash[1] == 0)) {
            //pc.printf("========================= Success!\n\r ===========================");
            putMessage(0, *nonce);
        }
        
        current_time = count.read();

        if ((current_time - previous_time) >= 1){
            previous_time = current_time;
            float hash_rate = (hash_count - hashes_calculated);

            putMessage(1, hash_rate);
            hashes_calculated = hash_count;
        }
    
        *nonce = *nonce + 1;
    }
}


//Set a given drive state
void motorOut(int8_t driveState, uint32_t torque=2000){
    
    //Lookup the output byte from the drive state.
    int8_t driveOut = driveTable[driveState & 0x07];


    
    if (~driveOut & 0x01) L1L.pulsewidth_us(0);
    if (~driveOut & 0x02) L1H = 1;
    if (~driveOut & 0x04) L2L.pulsewidth_us(0);
    if (~driveOut & 0x08) L2H = 1;
    if (~driveOut & 0x10) L3L.pulsewidth_us(0);
    if (~driveOut & 0x20) L3H = 1;
    
    //Then turn on
    if (driveOut & 0x01) L1L.pulsewidth_us(torque);
    if (driveOut & 0x02) L1H = 0;
    if (driveOut & 0x04) L2L.pulsewidth_us(torque);
    if (driveOut & 0x08) L2H = 0;
    if (driveOut & 0x10) L3L.pulsewidth_us(torque);
    if (driveOut & 0x20) L3H = 0;
}
    
//Convert photointerrupter inputs to a rotor state
inline int8_t readRotorState(){
    return stateMap[I1 + 2*I2 + 4*I3];
}

//Basic synchronisation routine    
int8_t motorHome() {
    //Put the motor in drive state 0 and wait for it to stabilise
    motorOut(0);
    wait(2.0);
    
    //Get the rotor state
    return readRotorState();
}

void setMotor(){
           //Rotot offset at motor state 0
          // motorISR();

        intState = readRotorState();
        if (intState != intStateOld) {
            intStateOld = intState;
            motorOut((intState-orState+lead+6)%6, setTorque); //+6 to make sure the remainder is positive
        }
    }

void commOutFn(){
    while(1) {
        osEvent newEvent = outMessages.get();
        message_t *pMessage = (message_t*)newEvent.value.p; 
        //pc.printf("Message %d with data 0x%016x\n\r", pMessage->code,pMessage->data); 

        switch(pMessage->code){
            case 0:
                pc.printf("========================= Success! ===========================\n\r");
                break;
            case 1:
                pc.printf("Hashrate per second: %d\n\r", pMessage->data);
                break;
            case 2:
                pc.printf("New key inputted: %c\n\r", pMessage->data);
                break;
            case 3:
                pc.printf("New command inputted to change: %c\n\r", pMessage->data);
                break;
            case 4:
                pc.printf("Current Velocity: %d\n\r", pMessage->data);
                break;
            case 5:
                pc.printf("Current Position: %d\n\r", pMessage->data);
                break;
            case 6:
                pc.printf("Max Velocity: : %d\n\r", pMessage->data);
                break;
            case 7:
                pc.printf("Rotation Velocity: %d\n\r", pMessage->data);
                break;
            case 8:
                pc.printf("Max Velocity: %d\n\r", pMessage->data);
                break;                
        }
        outMessages.free(pMessage);
    }
}

void commInFn(){
    while(1){

        //putMessage(1, (uint32_t) nonce);
        uint32_t speed;
        osEvent newEvent = inCharQ.get();
        uint8_t newChar = (uint8_t) newEvent.value.p;        
        pc.putc(newChar);


        if (newChar == '\r'){
            charBuffer[charBufferIndex] = '\0';
            charBufferIndex = charBufferIndex + 1;
            if (charBuffer[0] == 'V'){
                setSpeed = true;
                sscanf(charBuffer, "V%d", &speed);
                putMessage(3,'S');

                //if(speed == 0){
//                    setTorque = 1000;
//                    lead = 2;
//                }
//                else{
                    maxspeed = speed; 
                
                putMessage(6,maxspeed);
            } else if (charBuffer[0] == 'K'){
                newKey_mutex.lock();
                sscanf(charBuffer, "K%x", &newKey);
                putMessage(3,newKey);
                newKey_mutex.unlock();

            } else if (charBuffer[0] == 'R'){
                
                setSpeed = false;
                sscanf(charBuffer, "R%f", &setpos);
                putMessage(3,'R');
                putMessage(7,setpos);
               //setpos = speed;     
               //pos=1;
            } else if (charBuffer[0] == 'T'){
                sscanf(charBuffer, "T%d", &setTorque);
                setTorque = speed;
            }
            charBufferIndex = 0;
        }else{
            charBuffer[charBufferIndex] = newChar;
            charBufferIndex = charBufferIndex + 1;
            putMessage(2, newChar);
        }
    }
}


void motorISR() {
    static int8_t oldRotorState;
    int8_t rotorState = readRotorState(); 
    motorOut((rotorState-orState+lead+6)%6,setTorque);
    if (rotorState - oldRotorState == 5) motorPosition--;
    else if (rotorState - oldRotorState == -5) motorPosition++; 
    else motorPosition += (rotorState - oldRotorState); 
    oldRotorState = rotorState;
}

void motorCtrlTick(){ 
    motorCtrlT.signal_set(0x1); 
}
    
void motorCtrlFn(){
    Ticker motorCtrlTicker; 
    motorCtrlTicker.attach_us(&motorCtrlTick,100000);
    while(1){
        
        motorCtrlT.signal_wait(0x1);

        velocity = (motorPosition - lastPosition)*10;
       // //putMessage(4, velocity);
        if(i < 10){
            i++;
            }else{
                i = 0;
                //printf("The velocity is123: %d %d %d and the current position is: %d\n\r", velocity, motorPosition, lastPosition, motorPosition);
                putMessage(4, velocity);
                putMessage(5, motorPosition);
                }
        if(maxspeed == 0){
            speedout = 1000;
            lead = 2;
            } else{
        speedout = (maxspeed*6-abs(velocity));
        
        }

        /*if((setpos - motorPosition) >=0){
            speedout = speedout;
            }else{
                speedout *= -1;
                }*/
        if(abs(speedout) < 1000){
        setTorqueS = abs(speedout);
        }else{
            setTorqueS = 1000;
            }
            
        //printf("The speed controller output is: %d", speedout);
        if(speedout < 0){
                lead = -2;
            }else{
                lead = 2;
            }
            //setTorque = setTorqueS;

            //STOP HERE


            
            //kp, kd
            //Er - currentError
            //yr - setTorque
            
            //
        //if(pos==1){
            
            currentErr = setpos - (motorPosition/6);
            //putMessage(4, currentErr - setpos);
            setTorqueR = (25*currentErr) + (currentErr - setpos);
            //putMessage(5, motorPosition);
            setpos = currentErr; 

            //setTorque = setTorqueR;
        //}
        
        if(velocity >= 0){
                if(setTorqueR <= setTorqueS){
                    //printf("v > 0; torque_q lower or equal\n\r");
                  //  setTorque_mutex.lock();
                    setTorque = setTorqueR;
                   // setTorque_mutex.unlock();
                    }
                else{
                    //printf("v > 0; torque_q higher\n\r");
//                    setTorque_mutex.lock();
                    setTorque = setTorqueS;
  //                  setTorque_mutex.unlock();
                    }}
        else{
                if(setTorqueR > setTorqueS){
                    //printf("v < 0; torque_q greater\n\r");
                    //setTorque_mutex.lock();
                    setTorque = setTorqueR;
                    //setTorque_mutex.unlock();
                    }
                else{
                  // printf("v < 0; torque_q lower\n\r");
                    //setTorque_mutex.lock();
                    setTorque = setTorqueS;
                    //setTorque_mutex.unlock();
                    }
            }
               
        /*
        if (setSpeed){
            setTorque = setTorqueS;
        }else{
            if(setTorqueR <= 1000){
            setTorque = setTorqueR;
            }
            else{
                setTorque = 1000;
                }    
        }*/
        
        //printf("The velocity is: %d %d %d\n\r", velocity, motorPosition, lastPosition);
        lastPosition = motorPosition;
    }
        
}

void putMessage(uint8_t code, uint32_t data){ 
    message_t *pMessage = outMessages.alloc(); 
    pMessage->code = code;
    pMessage->data = data; 
    outMessages.put(pMessage);
}

void serialISR(){
    uint8_t newChar = pc.getc();
    inCharQ.put((void*)newChar); 
}