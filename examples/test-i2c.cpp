// Sample main.cpp file. Blinks an LED, sends a message out USART2
// and turns on PWM on pin 2

#include "wirish.h"
#include "i2c.h"

#define LED_PIN 13
#define PWM_PIN  2

void setup() {
    /* Set up the LED to blink  */
    pinMode(LED_PIN, OUTPUT);

    // get started
    Serial2.begin(9600);

    i2c_init(I2C1,10000);
}

int toggle = 0;
uint8 d = 3;

float t = 12.34;

void loop() {
    toggle ^= 1;
    digitalWrite(LED_PIN, toggle);
    delay(100);

    Serial2.print("Before:\t");
    Serial2.println(i2c_status(I2C1),DEC);
    i2c_master_start_write(I2C1, 12, &d, 1);
    Serial2.print("After:\t");
    Serial2.println(i2c_status(I2C1),DEC);

    //i2c_send1(12,5);

    t = sqrt(t);
    SerialUSB.println(t);

}

// Force init to be called *first*, i.e. before static object allocation.
// Otherwise, statically allocated object that need libmaple may fail.
 __attribute__(( constructor )) void premain() {
    init();
}

int main(void)
{
    setup();

    while (1) {
        loop();
    }
    return 0;
}
