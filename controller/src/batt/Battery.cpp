/*
 Battery.cpp - Battery library
 Copyright (c) 2014 Roberto Lo Giacco.

 This program is free software: you can redistribute it and/or modify
 it under the terms of the GNU Lesser General Public License as 
 published by the Free Software Foundation, either version 3 of the 
 License, or (at your option) any later version.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.

 You should have received a copy of the GNU General Public License
 along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "Battery.h"
#include <Arduino.h>

Battery::Battery(uint16_t minVoltage, uint16_t maxVoltage, uint8_t sensePin) {
	this->sensePin = sensePin;
	this->activationPin = 0xFF;
	this->minVoltage = minVoltage;
	this->maxVoltage = maxVoltage;
}

void Battery::begin(uint16_t refVoltage, float dividerRatio, mapFn_t mapFunction) {
	this->refVoltage = refVoltage;
	this->dividerRatio = dividerRatio;
	pinMode(this->sensePin, INPUT);
	if (this->activationPin < 0xFF) {
		pinMode(this->activationPin, OUTPUT);
	}
	this->mapFunction = mapFunction ? mapFunction : &linear;
}

void Battery::onDemand(uint8_t activationPin, uint8_t activationMode) {
	if (activationPin < 0xFF) {
		this->activationPin = activationPin;
		this->activationMode = activationMode;
		pinMode(this->activationPin, OUTPUT);
		digitalWrite(activationPin, !activationMode);
	}
}

uint8_t Battery::level() {
	return this->level(this->voltage());
}

uint8_t Battery::level(uint16_t voltage) {
	if (voltage <= minVoltage) {
		return 0;
	} else if (voltage >= maxVoltage) {
		return 100;
	} else {
		return (*mapFunction)(voltage, minVoltage, maxVoltage);
	}
}

uint16_t Battery::voltage() {
	if (activationPin != 0xFF) {
		digitalWrite(activationPin, activationMode);
		delayMicroseconds(10); // copes with slow switching activation circuits
	}
	analogRead(sensePin);
	delay(2); // allow the ADC to stabilize
	uint16_t reading = analogRead(sensePin) * dividerRatio * refVoltage / 1024;
	if (activationPin != 0xFF) {
		digitalWrite(activationPin, !activationMode);
	}
	return reading;
}

// Function that light up LEDs according to the battery level
// LED 1: >10% LED 2: >20% LED 3: >40% LED 4: >60% LED 5: >80%
void Battery::leds(uint8_t led1, uint8_t led2, uint8_t led3, uint8_t led4, uint8_t led5) {
	uint8_t level = this->level();
	digitalWrite(led1, (level > 80) ? HIGH : LOW);
    digitalWrite(led2, (level > 60) ? HIGH : LOW);
    digitalWrite(led3, (level > 40) ? HIGH : LOW);
    digitalWrite(led4, (level > 20) ? HIGH : LOW);
    digitalWrite(led5, (level > 10) ? HIGH : LOW);
} 