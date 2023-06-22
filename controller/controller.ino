#include "rbc.h"
#include "rbc_config.h"
#include <math.h>
#include <Arduino.h>

// Limits
#define RPS_LIMIT 80.
#define PWM_LIMIT 255.

ROSSerialClient* client;

void isr1() {
    client->isr1();
}

void isr2() {
    client->isr2();
}

void isr3() {
    client->isr3();
}

void isr4() {
    client->isr4();
}

void setup() {
    // PWM frequency configuration
    TCCR1B = TCCR1B & B11111000 | B00000001;  // for PWM frequency of 31372.55 Hz on D11, D12
    TCCR3B = TCCR3B & B11111000 | B00000001;   // for PWM frequency of 31372.55 Hz on D2, D3, D5
    TCCR4B = TCCR4B & B11111000 | B00000001;   // for PWM frequency of 31372.55 Hz on D6, D7, D8

    // Start serial
    Serial.begin(57600);

    // Initial PID parameters (0,0,0) so that the wheels don't move on startup, instead we wait for the first message
    double kp = 0.0;
    double ki = 0.0;
    double kd = 0.0;

    // Motor configurations
    double gear_ratio = 1./82.;
    int ppr = 16;
    int smoothener_window_size = 1;

    // Pinouts: { lpwm_pin, rpwm_pin, hall_a_pin, hall_b_pin }
    int motor_pinouts[4][4] = {
        { 2, 7, 18, 30 },   // Wheel 1
        { 3, 8, 19, 31 },   // Wheel 2
        { 5, 11, 20, 32 },  // Wheel 3
        { 6, 12, 21, 33 }   // Wheel 4
    };

    // Create config
    RBCConfig config = createConfig(
        // Pinouts
        motor_pinouts,
        // Wheel parameters 
        ppr, 
        gear_ratio, 
        // Smoothener parameters
        smoothener_window_size,
        // PID parameters
        kp, 
        ki, 
        kd,
        // Minimum startup PWM  
        50
    );

    // Create client
    client = new ROSSerialClient(config);

    // Attach interrupts
    attachInterrupt(digitalPinToInterrupt(motor_pinouts[0][2]), isr1, RISING);
    attachInterrupt(digitalPinToInterrupt(motor_pinouts[1][2]), isr2, RISING);
    attachInterrupt(digitalPinToInterrupt(motor_pinouts[2][2]), isr3, RISING);
    attachInterrupt(digitalPinToInterrupt(motor_pinouts[3][2]), isr4, RISING);
}

void loop() {
    client->update();
}