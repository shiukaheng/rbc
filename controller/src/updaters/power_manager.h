#pragma once

#include "../utils/time.h"
#include "../utils/base_state_updater.h"

#include "../states/motor_state.h"
#include "../states/robot_state.h"

#include "../utils/Battery.h"

class PowerManager : public BaseStateUpdater<RobotState> {

    private:
        Battery battery;
        bool is_first_read = true;
        long read_period = 5000000; // us (5 seconds)
        int read_time_elapsed = 0; // us

    public:

        PowerManager(RobotState& state) : BaseStateUpdater(state) {

            pinMode(22, OUTPUT);
            pinMode(23, OUTPUT);
            pinMode(24, OUTPUT);
            pinMode(25, OUTPUT);
            pinMode(26, OUTPUT);

            battery = Battery(
                10000, // minV = 10000 mV
                14000, // maxV = 14000 mV
                A4    // battery voltage sense pin
            );
            // Alternatively, (17400, 25200, A4) works for 6S LiPo (minV = 17.4V; maxV = 25.2V)

            battery.begin(
                5000, // Reference voltage = 5000 mV
                5.55555555556, // Divider ratio
                &sigmoidal
            )

        }

        void update(Tick& tick) {

            // Add time elapsed since last update
            read_time_elapsed += tick.dt;
            // If the read period has elapsed, read the battery voltage
            if (read_time_elapsed >= read_period) {
                // Reset the time elapsed
                read_time_elapsed = 0;
                // Read the battery voltage
                state.battery_voltage = battery.voltage();
                // Calculate the battery level
                state.battery_level = battery.level(state.battery_voltage);
                if (!is_first_read) {
                    // We trigger onMeasure
                    onMeasure();
                }
                // Set to not first read
                is_first_read = false;
            }

        }

        void onMeasure() {
            int level = state.battery_level;

            // Define LED pins
            int ledPins[] = {22, 23, 24, 25, 26};
            int numLeds = sizeof(ledPins) / sizeof(ledPins[0]);

            // Map battery level to number of LEDs (assuming each LED represents 20%)
            int ledsToLight = map(level, 0, 100, 0, numLeds);

            // Light up LEDs based on battery level
            for (int i = 0; i < numLeds; ++i) {
                if (i < ledsToLight) {
                    digitalWrite(ledPins[i], HIGH); // Turn on LED
                } else {
                    digitalWrite(ledPins[i], LOW);  // Turn off LED
                }
            }
        }
}