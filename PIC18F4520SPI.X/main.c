/*
 * File:   main.c
 * Author: c13321
 *
 * Created on July 11, 2018, 2:15 PM
 */




// PIC18F4520 Configuration Bit Settings

// 'C' source line config statements

// CONFIG1H
#pragma config OSC = INTIO7     // Oscillator Selection bits (Internal oscillator block, CLKO function on RA6, port function on RA7)
#pragma config FCMEN = OFF      // Fail-Safe Clock Monitor Enable bit (Fail-Safe Clock Monitor disabled)
#pragma config IESO = OFF       // Internal/External Oscillator Switchover bit (Oscillator Switchover mode disabled)

// CONFIG2L
#pragma config PWRT = OFF       // Power-up Timer Enable bit (PWRT disabled)
#pragma config BOREN = SBORDIS  // Brown-out Reset Enable bits (Brown-out Reset enabled in hardware only (SBOREN is disabled))
#pragma config BORV = 3         // Brown Out Reset Voltage bits (Minimum setting)

// CONFIG2H
#pragma config WDT = ON         // Watchdog Timer Enable bit (WDT enabled)
#pragma config WDTPS = 32768    // Watchdog Timer Postscale Select bits (1:32768)

// CONFIG3H
#pragma config CCP2MX = PORTC   // CCP2 MUX bit (CCP2 input/output is multiplexed with RC1)
#pragma config PBADEN = ON      // PORTB A/D Enable bit (PORTB<4:0> pins are configured as analog input channels on Reset)
#pragma config LPT1OSC = OFF    // Low-Power Timer1 Oscillator Enable bit (Timer1 configured for higher power operation)
#pragma config MCLRE = ON       // MCLR Pin Enable bit (MCLR pin enabled; RE3 input pin disabled)

// CONFIG4L
#pragma config STVREN = ON      // Stack Full/Underflow Reset Enable bit (Stack full/underflow will cause Reset)
#pragma config LVP = ON         // Single-Supply ICSP Enable bit (Single-Supply ICSP enabled)
#pragma config XINST = OFF      // Extended Instruction Set Enable bit (Instruction set extension and Indexed Addressing mode disabled (Legacy mode))

// CONFIG5L
#pragma config CP0 = OFF        // Code Protection bit (Block 0 (000800-001FFFh) not code-protected)
#pragma config CP1 = OFF        // Code Protection bit (Block 1 (002000-003FFFh) not code-protected)
#pragma config CP2 = OFF        // Code Protection bit (Block 2 (004000-005FFFh) not code-protected)
#pragma config CP3 = OFF        // Code Protection bit (Block 3 (006000-007FFFh) not code-protected)

// CONFIG5H
#pragma config CPB = OFF        // Boot Block Code Protection bit (Boot block (000000-0007FFh) not code-protected)
#pragma config CPD = OFF        // Data EEPROM Code Protection bit (Data EEPROM not code-protected)

// CONFIG6L
#pragma config WRT0 = OFF       // Write Protection bit (Block 0 (000800-001FFFh) not write-protected)
#pragma config WRT1 = OFF       // Write Protection bit (Block 1 (002000-003FFFh) not write-protected)
#pragma config WRT2 = OFF       // Write Protection bit (Block 2 (004000-005FFFh) not write-protected)
#pragma config WRT3 = OFF       // Write Protection bit (Block 3 (006000-007FFFh) not write-protected)

// CONFIG6H
#pragma config WRTC = OFF       // Configuration Register Write Protection bit (Configuration registers (300000-3000FFh) not write-protected)
#pragma config WRTB = OFF       // Boot Block Write Protection bit (Boot block (000000-0007FFh) not write-protected)
#pragma config WRTD = OFF       // Data EEPROM Write Protection bit (Data EEPROM not write-protected)

// CONFIG7L
#pragma config EBTR0 = OFF      // Table Read Protection bit (Block 0 (000800-001FFFh) not protected from table reads executed in other blocks)
#pragma config EBTR1 = OFF      // Table Read Protection bit (Block 1 (002000-003FFFh) not protected from table reads executed in other blocks)
#pragma config EBTR2 = OFF      // Table Read Protection bit (Block 2 (004000-005FFFh) not protected from table reads executed in other blocks)
#pragma config EBTR3 = OFF      // Table Read Protection bit (Block 3 (006000-007FFFh) not protected from table reads executed in other blocks)

// CONFIG7H
#pragma config EBTRB = OFF      // Boot Block Table Read Protection bit (Boot block (000000-0007FFh) not protected from table reads executed in other blocks)

// #pragma config statements should precede project file includes.
// Use project enums instead of #define for ON and OFF.

#include <xc.h>

#define _XTAL_FREQ 8000000

char dummy;

void writeEEPROM(char uLowerAddress, char uData);

char readEEPROM(char uLowerAddress);

void main(void) {
    
    TRISD=0x00;
    
    LATD=0x00;
    
    //Set up SPI Master mode
    
    //SPI Master mode, clock fosc/4
    SSPCON1bits.SSPM=0x0;
    
    //Set the SPI CKP and CKE bits
    SSPCON1bits.CKP=0;
    SSPSTATbits.CKE=0;
    
    //Serial port enable bit. SCK TRIS bit must be cleared. SDO TRIS bit must be cleared. SDI is automatically controlled by the SPI module
    SSPCON1bits.SSPEN=1;
    
    //SCK=RC3, SDO=RC5, SDI=RC4, RC1= CS, RC2= WP (EEPROM module)
    
    TRISCbits.RC3=0; //SCK
    TRISCbits.RC5=0; //SDO
    
    TRISCbits.RC1=0; //CS
    TRISCbits.RC2=0; //WP (EEPROM write protect pin)
    
    LATCbits.LATC2=1; //WP high for writes
    
    //Write data 0x07 to address 0x12
    writeEEPROM(0x12,0x07);
    
    //Wait 5 ms. This delay is required. You shouldn't read an EEPROM address immediately after writing it. 
    __delay_ms(5);
    
    //Read the data at address 0x12
    LATD=readEEPROM(0x12);
    
    while(1);
    
}

void writeEEPROM(char uLowerAddress, char uData){
    
    //WREN Instruction 
    char wrenInstruction=0x06;
    
    //WRITE Instruction
    char writeInstruction=0x02;
    
    //Before you write to an EEPROm, you must initiate a Write Latch Sequence
    
    //START WRITE LATCH Sequence
    
    //bring CS LOW
    LATCbits.LATC1=0;
    
    //load the WREN Instruction into the SSPBUF
    SSPBUF=wrenInstruction;
    
    //wait for transmission to complete
    while(!SSPSTATbits.BF);
    
    //Read dummy data
    dummy=SSPBUF;
    
    //bring CS HIGH
    LATCbits.LATC1=1;
    
    //END WRITE LATCH Sequence
    
    //START WRITE INSTRUCTION
    
    //bring CS LOW
    LATCbits.LATC1=0;
    
    //Let the EEPROM device know that you want to start writing data into it.
    
    SSPBUF=writeInstruction;
    
    //wait for transmission to complete
    while(!SSPSTATbits.BF);
    
    //Read dummy data
    dummy=SSPBUF;
    
    //Send upper address
    
    SSPBUF=0x00;
    
    //wait for transmission to complete
    while(!SSPSTATbits.BF);
    
    //Read dummy data
    dummy=SSPBUF;
    
    //END WRITE INSTRUCTION
    
    //START WRITING SEQUENCE
    
    //send lower address
    
    SSPBUF=uLowerAddress;
    
    //wait for transmission to complete
    while(!SSPSTATbits.BF);
    
    //Read dummy data
    dummy=SSPBUF;
    
    //send data to write
    
    SSPBUF=uData;
    
    //wait for transmission to complete
    while(!SSPSTATbits.BF);
    
    //Read dummy data
    dummy=SSPBUF;
    
    //bring CS HIGH
    LATCbits.LATC1=1;
    
    //END WRITE SEQUENCE
    
}

char readEEPROM(char uLowerAddress){
    
    //READ Instruction
    char readInstruction=0x03;

    //START READ Instruction
    
    //bring CS LOW
    LATCbits.LATC1=0;
    
    SSPBUF=readInstruction;
    
    //wait for transmission to complete
    while(!SSPSTATbits.BF);
    
    //Read dummy data
    dummy=SSPBUF;
    
    //END READ instruction
    
    //START READING at Location
    
    //Send upper address
    
    SSPBUF=0x00;
    
    //wait for transmission to complete
    while(!SSPSTATbits.BF);
    
    //Read dummy data
    dummy=SSPBUF;
    
    //send lower address
    
    SSPBUF=uLowerAddress;
    
    //wait for transmission to complete
    while(!SSPSTATbits.BF);
    
    //Read dummy data
    dummy=SSPBUF;
    
    //send dummy data
    SSPBUF=0x00;
    
    //wait for transmission to complete
    while(!SSPSTATbits.BF);
    
    //Read EEPROM DATA
    dummy=SSPBUF;
    
    return dummy;
    
}