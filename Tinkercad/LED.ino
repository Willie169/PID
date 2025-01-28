/**
 * @brief Controls a set of LEDs with a random blinking pattern.
 *
 * This Arduino sketch configures a set of LEDs connected to specific pins on the
 * microcontroller and controls them with a random blinking pattern. 
 * 
 * **Pattern Explanation:**
 * - The LED at the current index `i` is turned on, then turned off after a delay.
 * - The delay alternates between 100 milliseconds (ms) and 200 ms based on the 
 *   parity of the index `i` (even indices have a 100 ms delay, odd indices have 
 *   a 200 ms delay).
 * - After turning off the LED, a random number is generated to decide how much to 
 *   increment the index `i` for the next LED to be activated.
 * - The increment value is determined as follows:
 *   - If the random number is 1, the increment is a random number between 1 and 2.
 *   - If the random number is 0, the increment is a random number between 4 and 5.
 * - The index `i` is updated by adding this increment value, and then wrapped around 
 *   using modulo 6 to ensure it stays within the valid range of LED indices (0 to 5).
 * 
 * As a result, the pattern of which LED is lit and the timing between LEDs is 
 * randomized, creating a dynamic and unpredictable LED display.
 */

const uint8_t LEDs[] = {0b1000, 0b1001, 0b1010, 0b1011, 0b1100, 0b1101}; ///< Array of LED pin numbers.
const uint8_t total = sizeof(LEDs); ///< Total number of LEDs.
uint8_t i = 0; ///< Index of the currently active LED.

void setup() {
    /**
     * @brief Sets up the LED pins.
     * 
     * Configures each LED pin as an OUTPUT and initializes each LED to LOW
     * (off) state.
     */
    for (uint8_t i = 0; i < total; i++) {
        pinMode(LEDs[i], OUTPUT); ///< Set the LED pin as an OUTPUT.
        digitalWrite(LEDs[i], LOW); ///< Initialize the LED pin to LOW (off).
    }
}

void loop() {
    /**
     * @brief Controls the LEDs with a blinking pattern.
     * 
     * Turns on the LED at the current index, waits for a variable amount of time,
     * then turns it off. The delay time alternates between 100 ms and 200 ms based
     * on whether the current index is even or odd. After turning off the LED, a random
     * number is generated to decide how much to increment the index `i` and which LED
     * to activate next.
     */
    digitalWrite(LEDs[i], HIGH); ///< Turn on the current LED.
    delay(100 + i % 2 * 100); ///< Delay for 100 ms or 200 ms based on index `i`.
    digitalWrite(LEDs[i], LOW); ///< Turn off the current LED.
    
    // Generate a random number between 0 and 1.
    uint8_t temp = random(0, 2);
    
    // Generate a random increment based on the value of `temp`.
    if (temp) {
        temp = random(1, 3); ///< Random increment between 1 and 2.
    } else {
        temp = random(4, 6); ///< Random increment between 4 and 5.
    }
    
    // Update the index `i` and ensure it wraps around using modulo operation.
    i = (i + temp) % 6;
}