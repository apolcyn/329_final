#include <msp430.h> 

#define VOLUME_UP 0x20DF40BF
#define VOLUME_DOWN 0x20DFC03F
#define CHANNEL_UP 0x20df00ff
#define CHANNEL_DOWN 0x20DF807F



int done = 1;
long buf = 0;
int index = 0;
long timeCounter = 0;

/*
 * main.c
 */
int main(void) {
    WDTCTL = WDTPW | WDTHOLD;	// Stop watchdog timer
	
    if (CALBC1_16MHZ==0xFF)	     // If calibration constant erased
    {
        while(1);                // do not load, trap CPU!!
    }
    DCOCTL = 0;                  // Select lowest DCOx and MODx settings
    BCSCTL1 = CALBC1_1MHZ;      // Set range
    DCOCTL = CALDCO_1MHZ;

    CCTL0 |= CCIE;
    TACTL = TASSEL_2 + MC_1;
    TACCR0 = 100;

    P1DIR |= BIT6;

   // P1REN |= BIT0 + BIT1; // enable resistors on P1.1 and P1.0
   // P1OUT |= BIT1;  // use a pull-up resistor on P1.1
   // P1OUT &= ~BIT0;  // use a pull-down resistor on P1.0

    P1IES |= BIT1;   //  This sets P1.1 to trigger on a falling edge.
    P1IES &= ~BIT0;  //  This sets P1.0 to trigger on a rising edge
    P1IE |= BIT0 + BIT1;

    P1DIR |= BIT4;

    __delay_cycles(250000);
    P1IFG = 0;

    __enable_interrupt();


   // 00100000 11011111 00000000 11111111
    // 0x20df00ff == channel up
    // 0x20DF807F == channel down
    // 0x20DF40BF == volume up
    // 0x20DFC03F == volume down

    while(1) {
    	int temp = P1IFG;
    	temp = temp;
    	P1IFG = 0;
        while(index < 32)
        	;
        index = 0;
        done = 1;

        switch(buf) {
        case CHANNEL_UP:
        	P1OUT |= BIT6;
        	break;

        case CHANNEL_DOWN:
        	P1OUT &= ~BIT6;
        	break;

        case VOLUME_UP:
        	P1OUT |= BIT6;
        	__delay_cycles(500000);
        	P1OUT &= ~BIT6;
        	__delay_cycles(500000);
        	P1OUT |= BIT6;
        	__delay_cycles(500000);
        	P1OUT &= ~BIT6;
        	break;

        case VOLUME_DOWN:
        	P1OUT ^= BIT6;
        	break;
        default:
        	//error
        	index = 0;
        	break;
        }
    }

    //TACCR0 = 1;
    //CCTL0 = CCIE; 24 bits in 14ms.
	return 0;
}

#pragma vector=TIMER0_A0_VECTOR
__interrupt void something(void) {
	timeCounter++;
}

#pragma vector=PORT1_VECTOR
__interrupt void button(void) {
	P1OUT |= BIT4;
	if(P1IFG & BIT0 && P1IFG & BIT1) {
		// error condition. should not be both high at same time.
		P1OUT ^= BIT6;
	}
	else if(P1IFG & BIT0) { //rising edge
		if(done) {
			done = 0;
		}
		timeCounter = 0;
	}
	else if(P1IFG & BIT1 && !done) { // falling edge
		long diff = timeCounter;

		if(diff < 8) {
			// '0'
			index++;
			buf <<= 1;
		}
		else if(diff < 20) {
			// '1'
			index++;
			buf <<= 1;
			buf |= 1;
		}
		else if(diff < 60) {
			// header
			if(buf | index != 0) {
				P1OUT |= BIT6;
			}
		}
		else {
			buf = 0;
			index = 0;
		}
	}
	__delay_cycles(100); // Delay a short bit just to prevent possible unintended edge triggerings
	P1OUT &= ~BIT4;
	P1IFG = 0;
}



//1.714 bits per ms
//1714.3 bits per sec
//3428 bits per sec

// 1888 bits per sec
// 3600 Hz
