#include <xc.h>

#pragma config FOSC=INTRCIO, WDTE=OFF, PWRTE=OFF, MCLRE=ON, CP=OFF, \
                CPD=OFF, BOREN=OFF, IESO=OFF, FCMEN=OFF

#define TIMER_RESET_VALUE 6 //Calculated by the formula
                            //TMR0 = 256 - (timerPeriod*Fosc)/(4*prescaler) + x

                            //In this case, timerPeriod = 0.001s
                            //              Fosc = 4,000,000
                            //              prescaler = 4
                            //              x = 0 because PS is > 2

                            //So,   TMR0 = 256 - (0.001*4000000)/(4*4)
                            //      TMR0 = 256 - 4000/16
                            //      TMR0 = 256 - 250
                            //      TMR0 = 6

#define _XTAL_FREQ 4000000


volatile int adcDelay;

void delay(int ticks)
{
    while(ticks-->0)
        NOP();
}

void initADC()
{
    TRISA = 0xFF;   //set all digital I/O to inputs
    ANSEL = 0x00;   //disable all analog ports
    ANSELH = 0x00;

    TRISAbits.TRISA0 = 1;   //Disable the output driver for pin RA2/AN2
    ANSELbits.ANS0 = 1;     //set RA2/AN2 to analog mode
    
    ADCON0bits.ADFM = 1;        //ADC result is right justified
    ADCON0bits.VCFG = 0;        //Vdd is the +ve reference
    ADCON1bits.ADCS = 0b001;    //Fosc/8 is the conversion clock
                                //This is selected because the conversion
                                //clock period (Tad) must be greater than 1.5us.
                                //With a Fosc of 4MHz, Fosc/8 results in a Tad
                                //of 2us.
    ADCON0bits.CHS = 0;         //select analog input, AN2
    ADCON0bits.ADON = 1;        //Turn on the ADC
}

void initTIMER()
{
    ///////////////////
    // Timer 0 Setup //
    ///////////////////
    OPTION_REGbits.PSA = 0; //Prescaler assigned to Timer 0 (other option is to
                            //the Watchdog timer (WDT))

    OPTION_REGbits.PS = 0b001;  //Set the prescaler to 1:256
    OPTION_REGbits.T0CS = 0;    //Use the instruction clock (Fcy/4) as the timer
                                //clock. Other option is an external oscillator
                                //or clock on the T0CKI pin.
    INTCONbits.T0IF = 0;        //Clear the Timer 0 interrupt flag
    TMR0 = TIMER_RESET_VALUE;                   //Load a value of 0 into the timer
    INTCONbits.T0IE = 1;        //Enable the Timer 0 interrupt    
    INTCONbits.GIE = 1;         //Set the Global Interrupt Enable
}

void interrupt isr()
{
    static int counter = 0;
    static int state = 1;
    counter++;
    INTCONbits.T0IF = 0;        //Clear the Timer 0 interrupt flag
    TMR0 = TIMER_RESET_VALUE;                   //Load a value of 0 into the timer
                                //This is actually not necessary since the
                                //register will be at 0 anyway after rolling
                                //over
    if(counter >= adcDelay)
    {
        PORTC = state;
        state <<= 1;
        if(state == 0x10)
            state = 0x01;

        counter=0;
    }
}


int readADC()
{
    int result;
    __delay_us(5);              //Wait the acquisition time (about 5us).
    ADCON0bits.GO = 1;          //start the conversion
    while(ADCON0bits.GO==1){};  //wait for the conversion to end
    result = (ADRESH<<8)+ADRESL;	//combine the 10 bits of the conversion 
    return result;
}

void initUART()
{
    int uartBaudRate = 9600;
    BAUDCTLbits.SCKP = 0;   //non-inverted data
    BAUDCTLbits.BRG16 = 0;  //8-bit Bud rate generator
    BAUDCTLbits.ABDEN = 0;  //no auto-baud
    SPBRGH = 0; //8-bit BRG, so high byte is 0
    SPBRG = (_XTAL_FREQ/uartBaudRate)/16-1; //See datasheet for BRG formula
    TXSTAbits.TX9 = 0; //8-bit transmitter
    TXSTAbits.TXEN = 1; //enable transmitter
    TXSTAbits.SYNC = 0; //asynchronous operation
    TXSTAbits.BRGH = 1; //high speed BRG
    RCSTAbits.RX9 = 0; //8-bit receiver
    RCSTAbits.CREN = 1; //enable receiver

    PIE1bits.TXIE = 0;  //disable transmit interrupt
    PIE1bits.RCIE = 0;  //disable receive interrupts

    RCSTAbits.SPEN = 1; //enable UART
    
    //ToDo:
    //https://bitbucket.org/dmorrish/embeddedlibrary/src
    
}



void main(void) {
        
    initADC();
    initTIMER();
    PORTC = 0x00;
    TRISC = 0xf0;

    while(1)
    {
        adcDelay = readADC();
//        delay(delayLeds);
    }
}

