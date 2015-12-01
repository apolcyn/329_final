#include <msp430.h>

#define VOLUME_UP 0x20DF40BF
#define VOLUME_DOWN 0x20DFC03F
#define CHANNEL_UP 0x20df00ff
#define CHANNEL_DOWN 0x20DF807F

#define REMOTE_ONE 0x20DF8877
#define REMOTE_TWO 0x20DF48B7
#define REMOTE_THREE 0x20DFC837
#define REMOTE_FOUR 0x20DF28D7
#define REMOTE_FIVE 0x20DFA857
#define REMOTE_SIX 0x20DF6897

#define ms_20        655   // 20.0 ms
#define ms_0point75  25    // 0.75 ms
#define ms_1point50  49    // 1.50 ms
#define ms_2point25  73    // 2.25 ms

// actions to take on the main motor that drives the rear wheels
#define DRIVE_FORWARD 0
#define DRIVE_REVERSE 1
#define DRIVE_STOP 2
#define DRIVE_STANDBY 3
#define INCREASE_SPEED 4
#define DECREASE_SPEED 5

#define MOTOR_DUTY_HIGH_MAX 4

int motor_duty_low = 0;
int motor_duty_high = MOTOR_DUTY_HIGH_MAX;
int motor_duty_count = 0;
int motor_duty_state_tracker = 0;

#define MOTOR_AIN_0 BIT0
#define MOTOR_AIN_1 BIT1
#define MOTOR_PWM_A BIT2
#define MOTOR_STANDBY_A BIT3

#define RESET_COUNT = 30000

int done = 1;
long buf = 0;
int index = 0;
long timeCounter = 0;
long resetCounter = 0;

#define SERVO_PERIOD_COUNTS 40

#define SERVO_MINUS_90 1
#define SERVO_MINUS_45 2
#define SERVO_NEUTRAL 3
#define SERVO_PLUS_45 4
#define SERVO_PLUS_90 5

int servo_degree_options[] = {SERVO_MINUS_90
        , SERVO_MINUS_45
        , SERVO_NEUTRAL
        , SERVO_PLUS_45
        , SERVO_PLUS_90
};

#define NUM_SERVO_DEGREE_OPTIONS 5

int servo_options_index = 2;
int servo_count = 0;
int low_servo = SERVO_NEUTRAL;
int servo_on = 0;

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
        if(motor_duty_state_tracker == 0) {
            do_motor_action(DRIVE_FORWARD);
        }
        if(motor_duty_state_tracker < MOTOR_DUTY_HIGH_MAX) {
            motor_duty_low = abs_value(++motor_duty_state_tracker);
        }
        break;
    case DECREASE_SPEED:
        if(motor_duty_state_tracker == 0) {
            do_motor_action(DRIVE_REVERSE);
        }
        if(motor_duty_state_tracker > -(MOTOR_DUTY_HIGH_MAX)) {
            motor_duty_low = abs_value(--motor_duty_state_tracker);
        }
        break;
    default:
        break;
    }
}

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

    P2DIR |= BIT0 + BIT1 + BIT2 + BIT3;

    do_motor_action(DRIVE_FORWARD);

    servo_on = 1;

    while(1) {
    buf = 0;                          // Reset buffer.
        while(index < 32)                 // Sit here until we decode a 32-bit instruction
        ;                             // from TV remote.
        done = 1;                         // that the next faling edge is not a data bit.

        switch(buf) {                     // Perform some action based on the code
        case VOLUME_DOWN:                 // sent by the TV remote.
            P1OUT |= BIT6;
            __delay_cycles(400000);
            P1OUT &= ~BIT6;
            break;

        case REMOTE_TWO:
            if(servo_degree_options > 0) {
                set_servo_length(servo_degree_options[--servo_options_index]);
            }
            break;

        case REMOTE_ONE:
            if(servo_options_index < NUM_SERVO_DEGREE_OPTIONS) {
                set_servo_length(servo_degree_options[++servo_options_index]);
            }
            break;

        case REMOTE_FOUR:
            servo_options_index = 2;
            set_servo_length(SERVO_NEUTRAL);
            break;

        case REMOTE_FIVE:
            do_motor_action(DRIVE_STOP);
            break;

        case REMOTE_THREE:
            do_motor_action(INCREASE_SPEED);
            break;

        case REMOTE_SIX:
            do_motor_action(DECREASE_SPEED);
            break;

        default:                           // Error condition. Read an invalid code.
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

    if(servo_count == low_servo) {
        P1OUT &= ~BIT3;
    }
    if(servo_count == SERVO_PERIOD_COUNTS) {
        servo_count = 0;
        P1OUT |= BIT3;
    }

    if(motor_duty_count == MOTOR_DUTY_HIGH_MAX) {
        motor_duty_count = 0;
        P2OUT |= MOTOR_PWM_A;
    }
    if(motor_duty_count == motor_duty_low) {
        P2OUT &= ~MOTOR_PWM_A;
    }

    motor_duty_count++;

    if(resetCounter++ == 6000) {
        index = 0;
        buf = 0;
        resetCounter = 0;
    }
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
