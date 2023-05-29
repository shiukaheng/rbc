#include "client2.h"

// PID client for robocock wheels

SerialPIDClient2::SerialPIDClient2(RBCConfig config) {
    _inputBuffer = "";
    _motor1 = new PIDMotor(config.motor_configs[0]);
    _motor2 = new PIDMotor(config.motor_configs[1]);
    _motor3 = new PIDMotor(config.motor_configs[2]);
    _motor4 = new PIDMotor(config.motor_configs[3]);
}

SerialPIDClient2::~SerialPIDClient2() {
    delete _motor1;
    delete _motor2;
    delete _motor3;
    delete _motor4;
}

void SerialPIDClient2::isr1() {
    _motor1->isr();
}

void SerialPIDClient2::isr2() {
    _motor2->isr();
}

void SerialPIDClient2::isr3() {
    _motor3->isr();
}

void SerialPIDClient2::isr4() {
    _motor4->isr();
}

void SerialPIDClient2::update() {
    _motor1->update();
    _motor2->update();
    _motor3->update();
    _motor4->update();

    // Print the RPMs in format: "rpm1,rpm2,rpm3,rpm4" then a newline
    Serial.print(_motor1->getRPM());
    Serial.print(",");
    Serial.print(_motor2->getRPM());
    Serial.print(",");
    Serial.print(_motor3->getRPM());
    Serial.print(",");
    Serial.print(_motor4->getRPM());
    Serial.println();

    while (Serial.available() > 0) {
        // Parse the input buffer in format: "rpm1,rpm2,rpm3,rpm4" of which each is a float
        // Lets first parse them into the _rpm_targets array
        char inChar = (char)Serial.read(); // Read a character
        if (inChar != '\n') {
            _inputBuffer += inChar; // Add it to the input buffer
        } else {
            // We have a full line, lets parse it
            int i = 0;
            int commaIndex = _inputBuffer.indexOf(','); // Find the first comma
            while (commaIndex != -1) {
                // Parse the float between the start of the string and the comma
                _rpm_target_strings[i] = _inputBuffer.substring(0, commaIndex);
                _draft_rpm_targets[i] = _rpm_target_strings[i].toFloat();
                // Remove the parsed float and the comma from the input buffer
                _inputBuffer = _inputBuffer.substring(commaIndex + 1);
                // Find the next comma
                commaIndex = _inputBuffer.indexOf(',');
                i++;
            }
            // Handle the last float
            if (_inputBuffer.length() > 0) {
                _rpm_target_strings[i] = _inputBuffer;
                _draft_rpm_targets[i] = _rpm_target_strings[i].toFloat();
            }
            // Check if all values have been parsed successfully by seeing if we have 4 values
            if (i == 3) {
                // We have 4 values, lets copy them into the _rpm_targets array
                for (int j = 0; j < 4; j++) {
                    _rpm_targets[j] = _draft_rpm_targets[j];
                }
                // Set the RPM targets of the motors
                _motor1->setRPM(_rpm_targets[0]);
                _motor2->setRPM(_rpm_targets[1]);
                _motor3->setRPM(_rpm_targets[2]);
                _motor4->setRPM(_rpm_targets[3]);
            }

            // Clear the input buffer
            _inputBuffer = "";
        }
    }
}
