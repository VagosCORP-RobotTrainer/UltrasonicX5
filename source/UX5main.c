#define    FCY    16000000UL    // Instruction cycle frequency, Hz - required for __delayXXX() to work
#include <xc.h>
#include <libpic30.h>        // __delayXXX() functions macros defined here
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <stdint.h>

// CONFIG4
#pragma config DSWDTPS = DSWDTPSB       // DSWDT Postscale Select (1:8,388,608 (2.4 hours))
#pragma config DSWDTOSC = LPRC          // Deep Sleep Watchdog Timer Oscillator Select (DSWDT uses Low Power RC Oscillator (LPRC))
#pragma config RTCOSC = LPRC            // RTCC Reference Oscillator Select (RTCC uses Low Power RC Oscillator (LPRC))
#pragma config DSBOREN = OFF            // Deep Sleep BOR Enable bit (BOR enabled in Deep Sleep)
#pragma config DSWDTEN = OFF            // Deep Sleep Watchdog Timer (DSWDT disabled)

// CONFIG3
#pragma config WPFP = WPFP63            // Write Protection Flash Page Segment Boundary (Highest Page (same as page 21))
#pragma config SOSCSEL = IO             // Secondary Oscillator Pin Mode Select (SOSC pins have digital I/O functions (RA4, RB4))
#pragma config WUTSEL = LEG             // Voltage Regulator Wake-up Time Select (Default regulator start-up time used)
#pragma config WPDIS = WPDIS            // Segment Write Protection Disable (Segmented code protection disabled)
#pragma config WPCFG = WPCFGDIS         // Write Protect Configuration Page Select (Last page and Flash Configuration words are unprotected)
#pragma config WPEND = WPENDMEM         // Segment Write Protection End Page Select (Write Protect from WPFP to the last page of memory)

// CONFIG2
#pragma config POSCMOD = NONE           // Primary Oscillator Select (Primary Oscillator disabled)
#pragma config I2C1SEL = PRI            // I2C1 Pin Select bit (Use default SCL1/SDA1 pins for I2C1 )
#pragma config IOL1WAY = OFF            // IOLOCK One-Way Set Enable (The IOLOCK bit can be set and cleared using the unlock sequence)
#pragma config OSCIOFNC = ON            // OSCO Pin Configuration (OSCO pin functions as port I/O (RA3))
#pragma config FCKSM = CSDCMD           // Clock Switching and Fail-Safe Clock Monitor (Sw Disabled, Mon Disabled)
#pragma config FNOSC = FRCPLL           // Initial Oscillator Select (Fast RC Oscillator with Postscaler and PLL module (FRCPLL))
#pragma config PLL96MHZ = ON            // 96MHz PLL Startup Select (96 MHz PLL Startup is enabled automatically on start-up)
#pragma config PLLDIV = DIV2            // USB 96 MHz PLL Prescaler Select (Oscillator input divided by 2 (8 MHz input))
#pragma config IESO = OFF               // Internal External Switchover (IESO mode (Two-Speed Start-up) disabled)

// CONFIG1
#pragma config WDTPS = PS256            // Watchdog Timer Postscaler (1:256)
#pragma config FWPSA = PR128            // WDT Prescaler (Prescaler ratio of 1:128)
#pragma config WINDIS = OFF             // Windowed WDT (Standard Watchdog Timer enabled,(Windowed-mode is disabled))
#pragma config FWDTEN = OFF             // Watchdog Timer (Watchdog Timer is disabled)
#pragma config ICS = PGx1               // Emulator Pin Placement Select bits (Emulator functions are shared with PGEC1/PGED1)
#pragma config GWRP = OFF               // General Segment Write Protect (Writes to program memory are allowed)
#pragma config GCP = OFF                // General Segment Code Protect (Code protection is disabled)
#pragma config JTAGEN = ON              // JTAG Port Enable (JTAG port is enabled)

#define delay_us    __delay_us
#define delay_ms    __delay_ms

//#define led         LATBbits.LATB5
char concon = 0;
char contsens = 0;
short clean;

char t1of = 0;
char s1sub = 0;
char s2sub = 0;
char s3sub = 0;
char s4sub = 0;
char s5sub = 0;
unsigned short s1tinit = 0;
unsigned short s2tinit = 0;
unsigned short s3tinit = 0;
unsigned short s4tinit = 0;
unsigned short s5tinit = 0;
unsigned short s1th = 0;
unsigned short s2th = 0;
unsigned short s3th = 0;
unsigned short s4th = 0;
unsigned short s5th = 0;
unsigned short s1tt = 0;
unsigned short s2tt= 0;
unsigned short s3tt = 0;
unsigned short s4tt = 0;
unsigned short s5tt = 0;

void putch(char val) {
    while(U1STAbits.UTXBF);
    U1TXREG = val;
}

void send_float(float valFloat){//funcion pensada para enviar los diferentes elementos de una variable tipo flotante de 3 bytes
    signed char i;
    for(i = 3; i >= 0; i--){
        char byte = *((unsigned char *) & valFloat + i);
        putch(byte);
    }
}

void send_int32(long valLong){
    signed char i;
    for(i = 3; i >= 0; i--){
        char byte = *((unsigned char *) & valLong + i);
        putch(byte);
    }
}

void send_int16(int valInt){////funcion pensada para enviar los diferentes elementos de una variable int (2 bytes)
    signed char i;
    for(i = 1; i >= 0; i--){
        char byte = *((unsigned char *) & valInt + i);
        putch(byte);
    }
}

void __attribute__((__interrupt__, no_auto_psv)) _T1Interrupt(void) {
//    t1of = 1;
    IFS0bits.T1IF = 0; // Clear Timer1 Interrupt Flag}
}

void __attribute__((__interrupt__, no_auto_psv)) _U1RXInterrupt(void) {
//    putch(U1RXREG);
    IFS0bits.U1RXIF = 0; // Clear Timer1 Interrupt Flag}
}

void __attribute__((__interrupt__, no_auto_psv)) _IC1Interrupt(void) {
    if (!s1sub) {
        if (!PORTBbits.RB7)
            clean = IC1BUF;
        else
            s1sub = 1;
    } else /*if (PORTBbits.RB7)*/ {
//        IEC0bits.IC1IE = 0;
        s1tinit = IC1BUF;
        s1tt = IC1BUF;
        s1th = s1tt - s1tinit;
        s1sub = 0;
    }
    IFS0bits.IC1IF = 0; // Clear IC1 Interrupt Flag
}

void __attribute__((__interrupt__, no_auto_psv)) _IC2Interrupt(void) {
    if (!s2sub) {
        if (!PORTBbits.RB8)
            clean = IC2BUF;
        else
            s2sub = 1;
    } else /*if (PORTBbits.RB8)*/ {
//        IEC0bits.IC2IE = 0;
        s2tinit = IC2BUF;
        s2tt = IC2BUF;
        s2th = s2tt - s2tinit;
        s2sub = 0;
    }
    IFS0bits.IC2IF = 0; // Clear IC2 Interrupt Flag
}

void __attribute__((__interrupt__, no_auto_psv)) _IC3Interrupt(void) {
    if (!s3sub) {
        if (!PORTBbits.RB9)
            clean = IC3BUF;
        else
            s3sub = 1;
    } else /*if (PORTBbits.RB9)*/ {
//        IEC2bits.IC3IE = 0;
        s3tinit = IC3BUF;
        s3tt = IC3BUF;
        s3th = s3tt - s3tinit;
        s3sub = 0;
    }
    IFS2bits.IC3IF = 0; // Clear IC3 Interrupt Flag
}

void __attribute__((__interrupt__, no_auto_psv)) _IC4Interrupt(void) {
    if (!s4sub) {
        if (!PORTBbits.RB10)
            clean = IC4BUF;
        else
            s4sub = 1;
    } else /*if (PORTBbits.RB10)*/ {
//        IEC2bits.IC4IE = 0;
        s4tinit = IC4BUF;
        s4tt = IC4BUF;
        s4th = s4tt - s4tinit;
        s4sub = 0;
    }
    IFS2bits.IC4IF = 0; // Clear IC4 Interrupt Flag
}

void __attribute__((__interrupt__, no_auto_psv)) _IC5Interrupt(void) {
    if (!s5sub) {
        if (!PORTBbits.RB11)
            clean = IC5BUF;
        else
            s5sub = 1;
    } else /*if (PORTBbits.RB5)*/ {
//        IEC2bits.IC5IE = 0;
        s5tinit = IC5BUF;
        s5tt = IC5BUF;
        s5th = s5tt - s5tinit;
        s5sub = 0;
    }
    IFS2bits.IC5IF = 0; // Clear IC4 Interrupt Flag
}

int main(int argc, char** argv) {
    //    WDT resetea pic, pic revive y reset antena, PWRGD reset pic y jejeje
    /////////Configuración de Clock/////////
    CLKDIVbits.ROI = 0;
    CLKDIVbits.DOZEN = 0;
    CLKDIVbits.RCDIV = 0b000;
    CLKDIVbits.CPDIV = 0b00;
    CLKDIVbits.PLLEN = 1;

    /////////Secuencia Cambio de Oscilador/////////
    //    OSCCONH = 0x78;
    //    OSCCONH = 0x9A;
    //    OSCCONbits.NOSC = 0b001;
    //    OSCCONL = 0x46;
    //    OSCCONL = 0x57;
    //    OSCCONbits.OSWEN = 1;
    //    __delay_ms(500);
    //    while(OSCCONbits.OSWEN);
    //////////////////////////////////////////////

    //Inicialización de Variables//

//    ///////////////////////////////
//    TRISBbits.TRISB5 = 1;
////    LATBbits.LATB5 = 0;
    AD1PCFGbits.PCFG11 = 1;
    AD1PCFGbits.PCFG10 = 1;
    AD1PCFGbits.PCFG9 = 1;
    AD1PCFGbits.PCFG8 = 1;
    AD1PCFGbits.PCFG7 = 1;
    AD1PCFGbits.PCFG5 = 1;
    TRISAbits.TRISA2 = 0;
    TRISAbits.TRISA3 = 0;
    TRISBbits.TRISB4 = 0;
    TRISAbits.TRISA4 = 0;
    TRISBbits.TRISB3 = 0;
    ///////////Asignación de Pines////////////
    OSCCON = 0x46; //Desbloqueo Fase 1
    OSCCON = 0x57; //Desbloqueo Fase 2
    OSCCONbits.IOLOCK = 0; //Desbloqueo Fase 3
    RPINR18bits.U1RXR = 5; //Asignación Input U1RX a RP5
    RPOR6bits.RP13R = 3; //Asignación Output RP13 a U1TX
    RPINR7bits.IC1R = 7; //Asignación Input IC1 a RP7
    RPINR7bits.IC2R = 8; //Asignación Input IC2 a RP8
    RPINR8bits.IC3R = 9; //Asignación Input IC3 a RP9
    RPINR8bits.IC4R = 10; //Asignación Input IC4 a RP10
    RPINR9bits.IC5R = 11; //Asignación Input IC5 a RP11
    OSCCON = 0x46; //Rebloqueo Fase 1
    OSCCON = 0x57; //Rebloqueo Fase 2
    OSCCONbits.IOLOCK = 1; //Rebloqueo Fase 3

    /////////Configuración timer 1/////////
    T1CONbits.TSIDL = 1;
    T1CONbits.TCS = 0;
    T1CONbits.TGATE = 0;
    T1CONbits.TCKPS = 0b01; //1:8 PreScaller 0.5 us per ++
    PR1 = 65535; //periodo del timer 1
    IPC0bits.T1IP = 7; // Prioridad 1 para inttimer1
    IFS0bits.T1IF = 0; // limpiar flag de interrupcion 1
    IEC0bits.T1IE = 0; // habilitar interrupcion del timer1
    T1CONbits.TON = 1; // iniciar timer 1

//    /////////Configuración timer 2/////////
//    //    T2CONbits.TSIDL = 1;
//    //    T2CONbits.TCS = 0;
//    //    T2CONbits.TGATE = 0;
//    //    T2CONbits.TCKPS = 0b01; //1:8 PreScaller 0.5us per ++
//    //    PR2 = 10000; //periodo del timer 2 = 5 _ms
//    //    IPC1bits.T2IP = 2; // Prioridad 1 para inttimer2
//    //    IFS0bits.T2IF = 0; // limpiar flag de interrupcion 2
//    //    IEC0bits.T2IE = 0; // habilitar interrupcion del timer2
//    //    T2CONbits.TON = 1; // iniciar timer 2

    ////////////configuración de ICx///////
    IC1CON2bits.SYNCSEL = 0b00000; //free-Running Mode  //0b10100; //IC1 Trig Souce  = IC1 pin
    IC2CON2bits.SYNCSEL = 0b00000; //free-Running Mode  //0b10101; //IC2 Trig Souce  = IC2 pin
    IC3CON2bits.SYNCSEL = 0b00000; //free-Running Mode  //0b10101; //IC3 Trig Souce  = IC3 pin
    IC4CON2bits.SYNCSEL = 0b00000; //free-Running Mode  //0b10101; //IC4 Trig Souce  = IC4 pin
    IC5CON2bits.SYNCSEL = 0b00000; //free-Running Mode  //0b10101; //IC5 Trig Souce  = IC5 pin
    IC1CON1bits.ICSIDL = 1; //IC1 halts on idle
    IC2CON1bits.ICSIDL = 1; //IC2 halts on idle
    IC3CON1bits.ICSIDL = 1; //IC3 halts on idle
    IC4CON1bits.ICSIDL = 1; //IC4 halts on idle
    IC5CON1bits.ICSIDL = 1; //IC5 halts on idle
    IC1CON1bits.ICTSEL = 0b100; //IC1 Captute Timer = Timer1
    IC2CON1bits.ICTSEL = 0b100; //IC2 Captute Timer = Timer1
    IC3CON1bits.ICTSEL = 0b100; //IC3 Captute Timer = Timer1
    IC4CON1bits.ICTSEL = 0b100; //IC4 Captute Timer = Timer1
    IC5CON1bits.ICTSEL = 0b100; //IC4 Captute Timer = Timer1
    IC1CON1bits.ICI = 0b00; //Interrupt on every capture event
    IC2CON1bits.ICI = 0b00; //Interrupt on every capture event
    IC3CON1bits.ICI = 0b00; //Interrupt on every capture event
    IC4CON1bits.ICI = 0b00; //Interrupt on every capture event
    IC5CON1bits.ICI = 0b00; //Interrupt on every capture event
    IC1CON2bits.ICTRIG = 0; //free-Running Mode  //1; //Trigger IC1 from Source Designated in SYNCSEL
    IC2CON2bits.ICTRIG = 0; //free-Running Mode  //1; //Trigger IC2 from Source Designated in SYNCSEL
    IC3CON2bits.ICTRIG = 0; //free-Running Mode  //1; //Trigger IC3 from Source Designated in SYNCSEL
    IC4CON2bits.ICTRIG = 0; //free-Running Mode  //1; //Trigger IC4 from Source Designated in SYNCSEL
    IC5CON2bits.ICTRIG = 0; //free-Running Mode  //1; //Trigger IC4 from Source Designated in SYNCSEL
    IC1CON2bits.TRIGSTAT = 0; //Clear TRIGSTAT
    IC2CON2bits.TRIGSTAT = 0; //Clear TRIGSTAT
    IC3CON2bits.TRIGSTAT = 0; //Clear TRIGSTAT
    IC4CON2bits.TRIGSTAT = 0; //Clear TRIGSTAT
    IC5CON2bits.TRIGSTAT = 0; //Clear TRIGSTAT
    IC1TMR = 0;
    IC2TMR = 0;
    IC3TMR = 0;
    IC4TMR = 0;
    IC5TMR = 0;
    IPC0bits.IC1IP = 5; // Prioridad 1 para Captura1
    IPC1bits.IC2IP = 5; // Prioridad 1 para Captura2
    IPC9bits.IC3IP = 5; // Prioridad 1 para Captura3
    IPC9bits.IC4IP = 5; // Prioridad 1 para Captura4
    IPC9bits.IC5IP = 5; // Prioridad 1 para Captura4
    IFS0bits.IC1IF = 0; // limpiar flag de interrupcion IC1
    IFS0bits.IC2IF = 0; // limpiar flag de interrupcion IC2
    IFS2bits.IC3IF = 0; // limpiar flag de interrupcion IC3
    IFS2bits.IC4IF = 0; // limpiar flag de interrupcion IC4
    IFS2bits.IC5IF = 0; // limpiar flag de interrupcion IC4
    IEC0bits.IC1IE = 1; // inhabilitar interrupcion de Captura1
    IEC0bits.IC2IE = 1; // inhabilitar interrupcion de Captura2
    IEC2bits.IC3IE = 1; // inhabilitar interrupcion de Captura3
    IEC2bits.IC4IE = 1; // inhabilitar interrupcion de Captura4
    IEC2bits.IC5IE = 1; // inhabilitar interrupcion de Captura4
    IC1CON1bits.ICM = 0b001; //Capture on every Edge (R/F)
    IC2CON1bits.ICM = 0b001; //Capture on every Edge (R/F)
    IC3CON1bits.ICM = 0b001; //Capture on every Edge (R/F)
    IC4CON1bits.ICM = 0b001; //Capture on every Edge (R/F)
    IC5CON1bits.ICM = 0b001; //Capture on every Edge (R/F)
    while (IC1CON1bits.ICBNE) clean = IC1BUF;
    while (IC2CON1bits.ICBNE) clean = IC2BUF;
    while (IC3CON1bits.ICBNE) clean = IC3BUF;
    while (IC4CON1bits.ICBNE) clean = IC4BUF;
    while (IC5CON1bits.ICBNE) clean = IC5BUF;

    /////////Configuración UART 1/////////
    //    U1BRG = 25; //BaudRate = 38400;
    U1BRG = 8; //BaudRate = 115200;
    U1MODEbits.UARTEN = 1;
    U1MODEbits.USIDL = 0;
    U1MODEbits.IREN = 0;
    U1MODEbits.RTSMD = 1;
    U1MODEbits.UEN = 0b00;
    U1MODEbits.WAKE = 0;
    U1MODEbits.LPBACK = 0;
    U1MODEbits.ABAUD = 0;
    U1MODEbits.RXINV = 0;
    U1MODEbits.BRGH = 0;
    U1MODEbits.PDSEL = 0b00;
    U1MODEbits.STSEL = 0;
    U1STAbits.UTXISEL1 = 0;
    U1STAbits.UTXISEL0 = 1;
    U1STAbits.UTXINV = 0;
    U1STAbits.UTXBRK = 0;
    U1STAbits.UTXEN = 1;
    U1STAbits.URXISEL = 0b01;
    U1STAbits.ADDEN = 0;
    U1STAbits.RIDLE = 0;
    IPC2bits.U1RXIP = 2; // Prioridad 1 para inttimer1
    IFS0bits.U1RXIF = 0;
    IEC0bits.U1RXIE = 1;
    //    U1STAbits.UTXBF //recurso, buffer vacio
    //    U1STAbits.URXDA //recurso, datos recibidos

//    //Protocolo Inicial
//    printf("\r\nLT8-WL");
//    printf("\r\nSensores:");
//    printf("\r\nFuncionamiento Correcto");
//    printf("\r\nEsperando Comandos.....");
//    ///////////////////

    while (1) {
//        putch(65);
//        delay_ms(500);
        LATAbits.LATA2 = 1;
        LATAbits.LATA4 = 1;
        delay_us(8);
        IC1TMR = 0;
        IC2TMR = 0;
        IC3TMR = 0;
        IC4TMR = 0;
        IC5TMR = 0;
        delay_us(2);
        LATAbits.LATA2 = 0;
        LATAbits.LATA4 = 0;
        delay_ms(40);
        LATBbits.LATB4 = 1;
        LATAbits.LATA3 = 1;
        delay_us(8);
        IC1TMR = 0;
        IC2TMR = 0;
        IC3TMR = 0;
        IC4TMR = 0;
        IC5TMR = 0;
        delay_us(2);
        LATBbits.LATB4 = 0;
        LATAbits.LATA3 = 0;
        delay_ms(40);
        LATBbits.LATB3 = 1;
        delay_us(8);
        IC1TMR = 0;
        IC2TMR = 0;
        IC3TMR = 0;
        IC4TMR = 0;
        IC5TMR = 0;
        delay_us(2);
        LATBbits.LATB3 = 0;
        delay_ms(40);
        putch(13);
        send_int16((unsigned short)(float)s1th / 11.6);
//        putch(10);
//        putch(13);
        send_int16((unsigned short)(float)s2th / 11.6);
//        putch(10);
//        putch(13);
        send_int16((unsigned short)(float)s3th / 11.6);
//        putch(10);
//        putch(13);
        send_int16((unsigned short)(float)s4th / 11.6);
//        putch(10);
//        putch(13);
        send_int16((unsigned short)(float)s5th / 11.6);
        putch(10);
    }
    return (EXIT_SUCCESS);
}

