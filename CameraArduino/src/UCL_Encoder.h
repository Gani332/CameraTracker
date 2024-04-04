#include "mbed.h"

#define PREV_MASK 0x1 //Mask for the previous state in determining direction of rotation.
#define CURR_MASK 0x2 //Mask for the current state in determining direction of rotation.
#define INVALID   0x3 //XORing two states where both bits have changed.

class QuadratureEncoder {
public:
    // Constructor
    QuadratureEncoder(PinName pinA, PinName pinB) : 
        encoderA(pinA), 
        encoderB(pinB),
        ledA(LED1),
        ledB(LED3),
        pulses_(0) {
        // Attach interrupts for both pins        
        encoderA.rise(callback(this, &QuadratureEncoder::encoderISR));
        encoderA.fall(callback(this, &QuadratureEncoder::encoderISR));
        encoderB.rise(callback(this, &QuadratureEncoder::encoderISR));
        encoderB.fall(callback(this, &QuadratureEncoder::encoderISR));
    }
    
    // Interrupt Service Routine (ISR) for encoder
    void encoderISR() {
        int change = 0;
        int chanA  = encoderA.read();
        int chanB  = encoderB.read();
        currState_ = (chanA << 1) | (chanB);
        if (((currState_ ^ prevState_) != INVALID) && (currState_ != prevState_)) {
            //2 bit state. Right hand bit of prev XOR left hand bit of current
            //gives 0 if clockwise rotation and 1 if counter clockwise rotation.
            change = (prevState_ & PREV_MASK) ^ ((currState_ & CURR_MASK) >> 1);
            // find direction
             if (change == 0) {
                 change = -1;
             }
            pulses_ += change;          // update pulse counter
        }
        prevState_ = currState_;        // update the state 
        // turn on respective LEDs
        ledA = chanA;
        ledB = chanB;
    }
    // Get current count
    int getCount() {
        return pulses_;
    }
    // Initialise the encoder in correct modes
    void Initialise(){
        encoderA.mode(PullUp);
        encoderB.mode(PullUp);
        int chanA  = encoderA.read();
        int chanB  = encoderB.read();
        currState_ = (chanA << 1) | (chanB);
        prevState_ = currState_; 
    }

private:
    InterruptIn encoderA;
    InterruptIn encoderB;  
    DigitalOut ledA;   
    DigitalOut ledB;     
    int          prevState_;
    int          currState_;
    volatile int PrevA = encoderA.read();
    volatile int PrevB = encoderB.read();
    volatile int pulses_;
};
