#include <Arduino.h>
#include "src/updaters/robot_core.h"
#include "src/utils/time.h"

// Global variables

RobotCore* core;
RobotState state;
Clock clock;
Tick tick;

// Setting up the interrupts

void isr1() {
    core->motors[0]->encoder.hall_a_interrupt();
}

void isr2() {
    core->motors[1]->encoder.hall_a_interrupt();
}

void isr3() {
    core->motors[2]->encoder.hall_a_interrupt();
}

void isr4() {
    core->motors[3]->encoder.hall_a_interrupt();
}


void setup() {

    // PWM frequency configuration
    TCCR1B = TCCR1B & B11111000 | B00000001;  // for PWM frequency of 31372.55 Hz on D11, D12
    TCCR3B = TCCR3B & B11111000 | B00000001;   // for PWM frequency of 31372.55 Hz on D2, D3, D5
    TCCR4B = TCCR4B & B11111000 | B00000001;   // for PWM frequency of 31372.55 Hz on D6, D7, D8

    // Configure the robot state
    state.motors[0].lpwm_pin = 2;
    state.motors[0].rpwm_pin = 7;
    state.motors[0].hall_a_pin = 18;
    state.motors[0].hall_b_pin = 30;
    state.motors[0].gear_ratio = 1./82.;
    state.motors[0].ppr = 16;

    state.motors[1].lpwm_pin = 3;
    state.motors[1].rpwm_pin = 8;
    state.motors[1].hall_a_pin = 19;
    state.motors[1].hall_b_pin = 31;
    state.motors[1].gear_ratio = 1./82.;
    state.motors[1].ppr = 16;

    state.motors[2].lpwm_pin = 5;
    state.motors[2].rpwm_pin = 11;
    state.motors[2].hall_a_pin = 20;
    state.motors[2].hall_b_pin = 32;
    state.motors[2].gear_ratio = 1./82.;
    state.motors[2].ppr = 16;

    state.motors[3].lpwm_pin = 6;
    state.motors[3].rpwm_pin = 12;
    state.motors[3].hall_a_pin = 21;
    state.motors[3].hall_b_pin = 33;
    state.motors[3].gear_ratio = 1./82.;
    state.motors[3].ppr = 16;

    Serial.begin(57600);
    core = new RobotCore(state);

    // Attach interrupts
    attachInterrupt(digitalPinToInterrupt(state.motors[0].hall_a_pin), isr1, RISING);
    attachInterrupt(digitalPinToInterrupt(state.motors[1].hall_a_pin), isr2, RISING);
    attachInterrupt(digitalPinToInterrupt(state.motors[2].hall_a_pin), isr3, RISING);
    attachInterrupt(digitalPinToInterrupt(state.motors[3].hall_a_pin), isr4, RISING);

}

void loop() {
    // Update the robot
    tick = clock.update();
    core->update(tick);
}