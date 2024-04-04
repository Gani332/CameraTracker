#ifdef NUCLEO_F303ZE

#include "mbed.h"
#include <UCL_Encoder.h>

PwmOut motor1_PWM(D5);      // Motor PWM pin
DigitalOut motor1_dir(D6);  // Motor direction pin
DigitalOut Enable_Motor_Shield(A0); // Pin for enable (1) or disable (0)
BufferedSerial pc(USBTX, USBRX, 921600); // Serial communication

DigitalOut led2(LED2);
DigitalIn user_btn(BUTTON1);
Timer printing_timer;

QuadratureEncoder encoder(PA_6, PA_7);  // Encoder variable

int gear_ratio = 19;
int motor1_CPT = 3; // Counts per turn (revolution) of the motor
int motor1_count = 0;
float motor1_rot = 0;
float pos_error = 0;

// PID controller parameters
float Kp = 0.0012;   // Proportional gain
float Ki = 0.0;  // Integral gain
float Kd = 0.0; // Derivative gain

float scalingFactor = 0.1;

float integral = 0;
float previous_error = 0;
float derivative = 0;
float output = 0;

// Calculate the encoder counts that correspond to 60 degrees rotation
const int countsFor75Degrees = (motor1_CPT * gear_ratio * 75) / 360;
int initialPosition = 0; // To store the encoder's initial position

void setup() {
    encoder.Initialise();
    Enable_Motor_Shield.write(1);
    motor1_PWM.period_us(50); // Set PWM period
    printing_timer.start();
    // Capture the initial position of the motor at startup
    initialPosition = encoder.getCount();
}

void loop() {

    static bool lastDirection = motor1_dir;
    char buf[32];
    if (pc.readable()) {
        if (ssize_t num = pc.read(buf, sizeof(buf) - 1)) {
            buf[num] = '\0'; // Null-terminate
            int x, y, w, h;
            if (sscanf(buf, "%d,%d,%d,%d", &x, &y, &w, &h) == 4) {
                // Assuming the frame width is 320 pixels, calculate error from center
                float faceCenterX = x + w / 2;
                pos_error = 160 - faceCenterX; // 160 is the center for 320 width

                printf("Face Center X: %d, pos_error: %d\n", (int)faceCenterX, (int)pos_error);
                
                // PID calculations
                integral += pos_error;
                derivative = pos_error - previous_error;
                output = Kp * pos_error + Ki * integral + Kd * derivative;
                previous_error = pos_error;
                
                int currentPos = encoder.getCount() - initialPosition;
                if ((currentPos > countsFor75Degrees && output > 0) || (currentPos < -countsFor75Degrees && output < 0)) {
                    output = 0; // Prevent further movement if limit reached
                }
                
                // Apply output to motor
                if (output > 0) {
                    motor1_dir = 1;
                    output *= 1.3;
                } else 
                {
                    motor1_dir = 0;
                    output *= 1.7;
                }
                //output *= scalingFactor;
                motor1_PWM.write(abs(output) < 1.0f ? abs(output) : 1.0f);
            }
        }
    }

    ThisThread::sleep_for(10ms); // Reduce CPU usage
}

int main() {
    setup();
    while (true) {
        loop();
    }
}

#endif