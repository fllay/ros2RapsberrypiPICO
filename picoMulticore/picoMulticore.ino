#include <Arduino.h>
#include "pico/multicore.h"
#include "hardware/irq.h"
#include "hardware/timer.h"

// GPIO pins for interrupts
const uint8_t gpioPin1 = 2;
const uint8_t gpioPin2 = 3;

// Shared variable for interrupt status
volatile bool interrupt_flag = false;

// Mutex for synchronization
mutex_t interrupt_mutex;

// GPIO interrupt handlers
void gpioInterruptHandler1() {
    mutex_enter_blocking(&interrupt_mutex);
    Serial.println("Core1: GPIO Interrupt 1 triggered!");
    mutex_exit(&interrupt_mutex);
}

void gpioInterruptHandler2() {
    mutex_enter_blocking(&interrupt_mutex);
    Serial.println("Core1: GPIO Interrupt 2 triggered!");
    mutex_exit(&interrupt_mutex);
}



// Core1 entry function
void core1_entry() {
    // Initialize the mutex
    mutex_init(&interrupt_mutex);

    // Set up GPIO interrupts
    pinMode(gpioPin1, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(gpioPin1), gpioInterruptHandler1, FALLING);

    pinMode(gpioPin2, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(gpioPin2), gpioInterruptHandler2, FALLING);


    while (1) {
        // Core1 main loop
        tight_loop_contents(); // This can be replaced with other tasks for Core1
    }
}

void setup() {
    Serial.begin(115200);
    while (!Serial); // Wait for the serial monitor to open

    // Start Core1
    multicore_launch_core1(core1_entry);
}

void loop() {
    // Core0 main loop
    Serial.println("Core0: Running...");
    delay(2000); // Adjust this delay as needed
}
