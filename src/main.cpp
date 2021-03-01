#include <Arduino.h>
#include <avr/sleep.h>
#include <avr/wdt.h>
#include <avr/power.h>
#include <SoftwareSerial.h>

#define rxPin 5  // We use a nonexistent pin as we are not interested in receiving data
#define txPin 1

SoftwareSerial serial(rxPin, txPin);

#define EN3 4 // ATTiny pin 3 (PB4)
#define EN5 3 // ATTiny pin 2 (PB3)
#define SLEEPSIG 0  // ATTiny Pin 5 (PB0)

bool debug = true;  // runstate serial output
bool low_bat = false; // low battery flag

#define BATTERYMIN              2400 // Minimum battery startup voltage 2.4v
#define BATTERYRESET            2650 // Battery restart voltage 2.65v
#define BATTERYOVERCHARGELIMIT  3050 // set overcharge above 3.05v
#define BATTERYOVERCHARGECLEAR  2900 // clear overcharge once we fall below 2.9v once we've been in an overcharge state

bool overcharge = false;  // flag to capture overcharge battery state

// Analog sensing pin
int VBatPin = A1;    // Reads in the analogue number of voltage
unsigned long VBat = 0; // This will hold the batery pack voltage 2000->3000mv

// values for the voltage divider resistors
#define VOLTAGE_DIVIDER_R1 554000
#define VOLTAGE_DIVIDER_R2 224500

// sleep bit patterns
#define SLEEP1 0b000110 // 1 second
#define SLEEP2 0b000111 // 2 seconds
#define SLEEP4 0b100000 // 4 seconds
#define SLEEP8 0b100001 // 8 seconds

// state machine
int run_state = 0;

bool awake = false;


ISR(WDT_vect) {
    wdt_disable();  
}


void myWatchdogEnable(const byte interval) {
    wdt_reset();

    MCUSR = 0;                      // reset various flags
    WDTCR |= 0b00011000;            // see docs, set WDCE, WDE
    WDTCR =  0b01000000 | interval; // set WDIE, and appropriate delay

    byte old_ADCSRA = ADCSRA;
    ADCSRA = 0;
    power_all_disable(); // shut down ADC, Timer 0 and 1, serial etc
  
    set_sleep_mode (SLEEP_MODE_PWR_DOWN);
    sleep_bod_disable();
    sei();
    sleep_mode();

    power_all_enable();
    ADCSRA = old_ADCSRA;
}


unsigned long readBatteryVoltage() {
    int Vanalog = 0; // Raw ADC readings of battery voltage

    //Let's check the battery off load before we go any further
    Vanalog = analogRead(VBatPin);
    serial.print("ADCraw: ");
    serial.println(Vanalog);

    // Calculate voltage: Internal Ref 1060mV..   VBAT---560k--^---220k---GND
    // Adjusted for actual reading but need more accurate resistors really! - 5% LOL.
    //
    // Vout = Vs x R2 / (R1 + R2)
    // Vs = ( Vout x (R1 + R2) ) / R2
    // Vout = Analogue / 1023 * 1.1 * 1000 (multiply by 1000 as we want mV units)

    return (((double)Vanalog / 1023 * 1.1 * 1000) * (VOLTAGE_DIVIDER_R1 + VOLTAGE_DIVIDER_R2)) / VOLTAGE_DIVIDER_R2;
}

void setup() {
    // Set up IO pins
    pinMode(rxPin, INPUT);
    pinMode(txPin, OUTPUT);
    pinMode (EN3, OUTPUT);
    pinMode (EN5, OUTPUT);
    pinMode (SLEEPSIG, INPUT_PULLUP);
    digitalWrite(EN3, LOW);
    digitalWrite(EN5, LOW);

    analogReference(INTERNAL1V1);
    // read a sample and discard it
    analogRead(VBatPin);

    // Start the software serial
    serial.begin(4800);
    delay(100);
    serial.println("reset");
}  

/*
 * State machine
 * 0 = Initial power on, check battery voltage, if all OK power up the DC boosts and wait
 * 1 = Wait for 10 seconds for the device to wake up else go into power down forever (charging state)
 * 2 = Device has woken up, so wait for device to ask for power off sleep then go back to state 0
 * 3 = No devices were detected, power down and go into a solar charging state until reset.
 */

void loop() {
    switch(run_state) {
        case 0:
            if (debug)
                serial.println(run_state);

            // Let's check the battery off load before we go any further
            VBat = readBatteryVoltage();

            serial.print("VBat: ");
            serial.print(VBat);
            serial.println("mV");

            if (VBat < BATTERYMIN) { // Batteries critically low @ <1.2v per cell.  Abort & sleep to allow solar charging
                // disable boosts and go back to sleep
                digitalWrite (EN5, LOW);
                digitalWrite (EN3, LOW);
                low_bat = true;
                run_state = 2;
            } else if (low_bat && (VBat < BATTERYRESET) )
                run_state = 2;
            else { // Batteries above minimum cut-off voltage, go for main startup
                // Now lets switch on the loads
                low_bat = false;
                digitalWrite (EN3,HIGH); // switch on 3.3v rail
                digitalWrite (EN5, HIGH); // switch on 5v rail
                run_state = 1; // initial power on completed, wait for device to sleep
                myWatchdogEnable (SLEEP1); // 1 second short sleep to save power whilst waiting
            }

            if (overcharge && (VBat < BATTERYOVERCHARGECLEAR) ) { // clear overcharge state, with some hysteresis protection
                overcharge = false;
                serial.println("Overcharge cleared");
            }

            if (VBat > BATTERYOVERCHARGELIMIT) { // Batteries are at risk of overcharging.  STAY AWAKE
                overcharge = true;
                serial.println("Overcharge");
            }

            if (debug)
                serial.println(run_state);

            break;

        case 1:
            if (debug)
                serial.println(run_state);

            awake = false;

            for (int i = 0; i < 10; i++) { // wait for 10 seconds before shutting down and going into charge only mode
                if (debug) {
                    serial.print("waiting ");
                    serial.println(i);
                }
                if ( digitalRead (SLEEPSIG) == HIGH)
                    // the companion device hasn't woke up yet;
                    myWatchdogEnable (SLEEP1);  // device not yet fully awake go back to sleep to save power
                else {
                    awake = true;
                    i = 60; // break out of for loop - case break; didn't seem to work ??
                    if (debug)
                        serial.println("leaving");
                }
            }

            // Sample battery voltage under load
            VBat = readBatteryVoltage();

            serial.print("VBat: ");
            serial.print(VBat);
            serial.println("mV");
        
            if (awake)
                run_state = 2; // the device has pulled the SLEEPSIG pin low so is now awake
            else {
                // device didn't wake up so we shut down the boost rails and move to sleep forever
                digitalWrite (EN5, LOW);
                digitalWrite (EN3, LOW);
                // run_state = 3;
                run_state = 2;  // (test) actually don't sleep forever just wait 5 minutes, try again
            }

            if (debug)
                serial.println(run_state);

            break;

        case 2:
            if (debug)
                serial.println(run_state);

            if (digitalRead (SLEEPSIG) == HIGH || run_state == 4) {
                // SLEEPSIG has gone HIGH again - device has signalled to sleep, or battery critically low
                // but if we're in a state of overcharge we're going to stay awake for the sleep period
                if (!overcharge) {
                    digitalWrite (EN5, LOW); // switch off 5v rail
                    digitalWrite (EN3, LOW) ; // switch off 3v3 rail
                }

                for (int i = 0; i < 75; i++) { // power down sleep for 5 minutes ( 75 x 4 seconds)
                    //for (int i=0; i < 3; i++) { // power down sleep for a few seconds (testing) ( 3 x 4 seconds)
                    myWatchdogEnable (SLEEP4); 
                }
                run_state = 0;  // time to wake up again
            } else
                // device still awake - sleep then check again in a second
                myWatchdogEnable (SLEEP1);

            if (debug)
                serial.println(run_state);

            break;

        case 3:
            if (debug)
                serial.println(run_state);

            serial.println("forever");
            while (1) { // forever
                myWatchdogEnable (SLEEP8);
                if (debug)
                    serial.println(run_state);
            }
            // no change of runstate we keep doing this forever until reset
            break;
    }
}