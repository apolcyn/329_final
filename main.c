#include <msp430.h>

#define VOLUME_UP 0x20DF40BF      // 32 bit values of commands sent LG TV
#define VOLUME_DOWN 0x20DFC03F    // remote for different button presses.
#define CHANNEL_UP 0x20df00ff
#define CHANNEL_DOWN 0x20DF807F
#define REMOTE_ONE 0x20DF8877
#define REMOTE_TWO 0x20DF48B7
#define REMOTE_THREE 0x20DFC837
#define REMOTE_FOUR 0x20DF28D7
#define REMOTE_FIVE 0x20DFA857
#define REMOTE_SIX 0x20DF6897

// Enum values of different actions for the main motor
// that drives the rear wheels to perform.
#define DRIVE_FORWARD 0
#define DRIVE_REVERSE 1
#define DRIVE_STOP 2
#define DRIVE_STANDBY 3
#define INCREASE_SPEED 4
#define DECREASE_SPEED 5

// Number of clock ticks in entire PWM period for
// motors that drive the rear wheels.
#define MOTOR_DUTY_HIGH_MAX 4

int motor_duty_low = 0;                    // Clock tick number in a PWM
                                           // cycle at which duty cycle line goes low.

int motor_duty_high = MOTOR_DUTY_HIGH_MAX; // Total number of clock ticks
                                           // in a PWM duty cycle.

int motor_duty_count = 0;                  // Number of clock ticks that have
                                           // gone by so far in the current PWM cycle.

int motor_duty_state_tracker = 0;          // A state variable to keep track of
                                           // what direction an speed the motors
                                           // are moving at.

#define MOTOR_AIN_0 BIT0                   // P1 ports of outputs to H-bridge
#define MOTOR_AIN_1 BIT1                   // that drives the rear motors.
#define MOTOR_PWM_A BIT2
#define MOTOR_STANDBY_A BIT3

#define RESET_COUNT = 30000                // Clock tick number at which the IR
                                           // buffers are cleared and reset. Used to
                                           // recover from errors in IR decoding.
                                           // This resets it every three seconds.

int done = 1;                              // Flag to tell when a 32 bit command from
                                           //  a remote has been received.
long buf = 0;  // Buffer that stores the 32 bit remote command as it gets received.
int index = 0; // Current bit that we're on, when receiving a remote command.
long timeCounter = 0;  // A number used to record the number of clock ticks that
                       // the last bit took, when decoding IR.
long resetCounter = 0; // Value that increments on each clock tick
                       // and resets the IR buffers when it reaches its max.

#define SERVO_PERIOD_COUNTS 40   // Number of clock ticks in the servo's PWM cycle.

// Number of clock ticks of different duty cycles
// for servo control.
#define SERVO_MINUS_45 2
#define SERVO_NEUTRAL 3
#define SERVO_PLUS_45 4

int servo_degree_options[] = {SERVO_MINUS_45   // Array that's used to shift between
        , SERVO_NEUTRAL                        // servo positions.
        , SERVO_PLUS_45
};

#define SERVO_NEUTRAL_INDEX 1                // PWM servo duty cycle for neutral position.
#define NUM_SERVO_DEGREE_OPTIONS 3           // Number of options servo has for positions.

int servo_options_index = SERVO_NEUTRAL_INDEX;   // Index into servo_options array.
int servo_count = 0;                             // Clock counter for servo PWM duty cycle.
int low_servo = SERVO_NEUTRAL;                   // Clock tick in PWM cycle where
                                                 // line goes low.
int servo_on = 0;                                // Flag to tell if servo is on or not.

int abs_value(int num) {
    if(num < 0) {
        return num * -1;
    }
    else {
        return num;
    }
}

/* Carries out some new motor action.
 *  */
void do_motor_action(int motor_action) {
    switch(motor_action) {
    case DRIVE_FORWARD:
        P2OUT |= MOTOR_AIN_0;
        P2OUT &= ~MOTOR_AIN_1;
        P2OUT |= MOTOR_STANDBY_A;
        break;
    case DRIVE_REVERSE:
        P2OUT &= ~MOTOR_AIN_0;
        P2OUT |= MOTOR_AIN_1;
        P2OUT |= MOTOR_STANDBY_A;
        break;
    case DRIVE_STOP:
        P2OUT &= ~(MOTOR_AIN_0 + MOTOR_AIN_1);
        P2OUT |= MOTOR_PWM_A;
        P2OUT |= MOTOR_STANDBY_A;
        motor_duty_low = 0;
        motor_duty_state_tracker = 0;
        break;
    case DRIVE_STANDBY:
        do_motor_action(DRIVE_STOP);
        P2OUT &= ~MOTOR_STANDBY_A;
        break;
    case INCREASE_SPEED:
        if(motor_duty_state_tracker == 0) {   // Flip to "forwards" if stopped.
            do_motor_action(DRIVE_FORWARD);
        }
        if(motor_duty_state_tracker < MOTOR_DUTY_HIGH_MAX) {
            // Speed up in "forwards" if possible.
            motor_duty_low = abs_value(++motor_duty_state_tracker);
        }
        break;
    case DECREASE_SPEED:
        if(motor_duty_state_tracker == 0) {   // Flip to "reverse" if stopped.
            do_motor_action(DRIVE_REVERSE);
        }
        if(motor_duty_state_tracker > -(MOTOR_DUTY_HIGH_MAX)) {
            // Speed up in "backwards" if possible.
            motor_duty_low = abs_value(--motor_duty_state_tracker);
        }
        break;
    default:
        break;
    }
}

/* Sets the length of the high part of the servo's duty cycle,
 * which controls the position of the servo.
 */
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

    P2DIR |= BIT0 + BIT1 + BIT2 + BIT3;   // USing lower nibble of P2 to control
                                          // the H-bridge motor controller of rear wheels.

    do_motor_action(DRIVE_FORWARD);       // Start out in the "forwards" direction,
                                          // but still stopped.
    servo_on = 1;

    while(1) {
        buf = 0;                          // Reset buffer and the bit index into it.
        index = 0;
        while(index < 32)                 // Sit here until we decode a 32-bit instruction
        ;                             // from TV remote.
        done = 1;                         // that the next faling edge is not a data bit.

        switch(buf) {                     // Perform some action based on the code
                                          // sent by the TV remote.
        case VOLUME_DOWN:                 // Simple LED toggle indicates that system is working.
            P1OUT |= BIT6;
            __delay_cycles(400000);
            P1OUT &= ~BIT6;
            break;

        case REMOTE_TWO:                      // Turn left if possible
            if(servo_options_index > 0) {
                set_servo_length(servo_degree_options[--servo_options_index]);
            }
            break;

        case REMOTE_ONE:                      // Turn right if possible
            if(servo_options_index < NUM_SERVO_DEGREE_OPTIONS) {
                set_servo_length(servo_degree_options[++servo_options_index]);
            }
            break;

        case REMOTE_FOUR:                      // Set servo to a neutral position
            servo_options_index = SERVO_NEUTRAL_INDEX;
            set_servo_length(SERVO_NEUTRAL);
            break;

        case REMOTE_FIVE:                     // Stop the rear wheel motors.
            do_motor_action(DRIVE_STOP);
            break;

        case REMOTE_THREE:                     // Make the vehicle go more in the
            do_motor_action(INCREASE_SPEED);   // "forwards" direction.
            break;

        case REMOTE_SIX:                     // Make the vehicle go more in the
            do_motor_action(DECREASE_SPEED); // "reverse" direction.
            break;

        default:                           // Error condition. An invalid code was received.
            index = 0;
            break;
        }
    }
    return 0;
}

/* Increments timeCounter, which counts up with a 100us period. Used to decode IR. */
#pragma vector=TIMER0_A0_VECTOR
__interrupt void something(void) {
    timeCounter++;
    servo_count++;

    if(servo_count == low_servo) {                // Handle the PWM cycle of the servo.
        P1OUT &= ~BIT3;
    }
    if(servo_count == SERVO_PERIOD_COUNTS) {
        servo_count = 0;
        P1OUT |= BIT3;
    }

    if(motor_duty_count == MOTOR_DUTY_HIGH_MAX) {  // Handle the PWM cycle of the
        motor_duty_count = 0;                      // rear wheel motors.
        P2OUT |= MOTOR_PWM_A;
    }
    if(motor_duty_count == motor_duty_low) {
        P2OUT &= ~MOTOR_PWM_A;
    }

    motor_duty_count++;

    if(resetCounter++ == 6000) {                   // Reset the IR sensor buffer
        index = 0;                                 // every three seconds to recover
        buf = 0;                                   // from potential errors.
        resetCounter = 0;
    }
}

/* Reads the next data bit from IR sensor and stores it in a buffer.
 * Implements receiver of NEC protocol transmissions
 * , so this decodes '1's or '0's based on width of each 'high' section
 * of the demodulated IR signal that is toggling.
 * (Note that the IR receiver is idle 'high'.
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
        timeCounter = 0;                  // Reset time counter to zero on rising edge, so that
                                          // we can count the length of time each one takes.
    }
    else if(P1IFG & BIT1 && !done) {      // Falling edge from IR receiver digital output.
        long diff = timeCounter;          // See how many 100us periods have gone by since
                                          // the previous rising edge (stored in 'timeCounter').
        if(diff <= 2) {                    // Previous high signal was less than 1ms long,                          // '0'
            index++;                      // which inidicates a '0' with NEC protocol.
            buf <<= 1;                    // Add in a '0' bit to the buffer.
        }
        else if(diff <= 5) {              // Previous high signal is between 1ms and 2.5ms long,
            index++;                      // which inidicates a '1' by the NEC protocol.
            buf <<= 1;                    // Add in a '1' bit to the buffer.
            buf |= 1;
        }
        else if(diff <= 10) {              // This high signal is between 2.5ms and 5ms,
            if(buf | index != 0) {        // which inidicates that this was the data header.
                P1OUT |= BIT6;            // If 'buffer' and 'index' are not cleared and set
            }                             // to zero, then indicate error with LED.
            index = 0;                    // Header comes first, so both should be reset.
            buf = 0;
        }
    }
    P1IFG = 0;                            // Clear Port 1 GPIO interrup flags
}
