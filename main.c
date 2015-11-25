#include <msp430.h>

#define VOLUME_UP 0x20DF40BF
#define VOLUME_DOWN 0x20DFC03F
#define CHANNEL_UP 0x20df00ff
#define CHANNEL_DOWN 0x20DF807F

#define ms_20        655   // 20.0 ms
#define ms_0point75  25    // 0.75 ms
#define ms_1point50  49    // 1.50 ms
#define ms_2point25  73    // 2.25 ms

#define RESET_COUNT = 30000

int done = 1;
long buf = 0;
int index = 0;
long timeCounter = 0;
long resetCounter = 0;

#define SERVO_PERIOD_COUNTS 40

#define SERVO_MINUS_45 2
#define SERVO_NEUTRAL 3
#define SERVO_PLUS_45 4

int servo_count = 0;
int low_servo = SERVO_NEUTRAL;
int servo_on = 0;

void set_servo_length(int duty_cycle) {
    servo_count = 0;
    low_servo = duty_cycle;
}

/*
 * main.c
 */
int main(void) {
    WDTCTL = WDTPW | WDTHOLD;          // Stop watchdog timer

    if (CALBC1_16MHZ==0xFF)              // If calibration constant erased
    {
        while(1);                         // do not load, trap CPU!!
    }
    DCOCTL = 0;                           // Select lowest DCOx and MODx settings
    BCSCTL1 = CALBC1_1MHZ;                // Set range
    DCOCTL = CALDCO_1MHZ;

    CCTL0 |= CCIE;
    TACTL = TASSEL_2 + MC_1;
    TACCR0 = 500;

    P1DIR |= BIT6;                        // UsingP1.6 for LED.

    P1IES |= BIT1;                        // Set P1.1 to trigger on a falling edge.
    P1IES &= ~BIT0;                       // Set P1.0 to trigger on a rising edge
    P1IE |= BIT0 + BIT1;

    P1DIR |= BIT5; // servo PWM control
    P1DIR |= BIT3; // bit to look at amount of time taken by ISR's

    __delay_cycles(250000);
    P1IFG = 0;                            // Clear out unintended GPIO triggerings.

    __enable_interrupt();

    servo_on = 1;

    while(1) {
    buf = 0;                          // Reset buffer.
        while(index < 32)                 // Sit here until we decode a 32-bit instruction
        ;                             // from TV remote.
        done = 1;                         // that the next faling edge is not a data bit.

        switch(buf) {                     // Perform some action based on the code
        case CHANNEL_UP:                  // sent by the TV remote.
            set_servo_length(SERVO_MINUS_45);
        break;

        case CHANNEL_DOWN:
            set_servo_length(SERVO_NEUTRAL);
        break;

        case VOLUME_UP:
            set_servo_length(SERVO_PLUS_45);
        break;

        case VOLUME_DOWN:
        P1OUT |= BIT6;
        __delay_cycles(400000);
        P1OUT &= ~BIT6;
        break;
        default:                           // Error condition. Read an invalid code.
        index = 0;
        break;
        }
    }
    P1IFG = 0;

return 0;
}

/* Increments timeCounter, which counts up with a 100us period. Used to decode IR. */
#pragma vector=TIMER0_A0_VECTOR
__interrupt void something(void) {
    P1OUT |= BIT5;
    timeCounter++;
    servo_count++;

    if(servo_count == low_servo) {
        P1OUT &= ~BIT3;
    }
    else if(servo_count == SERVO_PERIOD_COUNTS) {
        servo_count = 0;
        P1OUT |= BIT3;
    }
    if(resetCounter++ == 6000) {
        index = 0;
        buf = 0;
        resetCounter = 0;
    }
    P1OUT &= ~BIT5;
}

/* Reads the next data bit from IR sensor and stores it in a buffer.
 * Using NEC protocol, so decodes '1's or '0's based on width of high signal.
 */
#pragma vector=PORT1_VECTOR
__interrupt void button(void) {
    if(P1IFG & BIT0 && P1IFG & BIT1) {    // Check error condition.
        P1OUT ^= BIT6;                    // Shouldn't have interrupts for both high to low
        __delay_cycles(250000);           // and low to high transitions.
        P1OUT ^= BIT6;                    // Toggle LED to indicate error.
        __delay_cycles(250000);
        P1OUT ^= BIT6;
    }
    else if(P1IFG & BIT0) {               // Rising edge from IR receiver digital output.
        if(done) {                        // Indicate that subsequent edges are of interest.
            done = 0;
        }
        timeCounter = 0;                  // Reset time counter to zero on rising edge.
    }
    else if(P1IFG & BIT1 && !done) {      // Falling edge from IR receiver digital output.
        long diff = timeCounter;          // See how many 100us periods have gone by since
                                          // the previous rising edge.
        if(diff <= 2) {                    // This high signal was less than 800us long,                          // '0'
            index++;                      // which inidicates a '0' with NEC protocol.
            buf <<= 1;                    // Add in a '0' bit to the buffer.
        }
        else if(diff <= 5) {              // This high signal is between 800us and 2ms,
            index++;                      // which inidicates a '1' bt by NEC protocol.
            buf <<= 1;                    // Add in a '1' bit to the buffer.
            buf |= 1;
        }
        else if(diff <= 10) {              // This high signal is between 2ms and 6ms,
            if(buf | index != 0) {        // which inidicates that this was the data header.
                P1OUT |= BIT6;            // If buffer and index and not both set to zero,
            }
            index = 0;
            buf = 0;
        }                             // then this is an error condition, as these should
    }                                  // be reset before data capture.
    P1IFG = 0;                            // Clear Port 1 GPIO interrup flags
}
