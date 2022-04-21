#include<xc.h>           // processor SFR definitions
#include<sys/attribs.h>  // __ISR macro
#include<stdio.h>
#include"i2c_master_noint.h"

// DEVCFG0
#pragma config DEBUG = OFF // disable debugging
#pragma config JTAGEN = OFF // disable jtag
#pragma config ICESEL = ICS_PGx1 // use PGED1 and PGEC1
#pragma config PWP = OFF // disable flash write protect
#pragma config BWP = OFF // disable boot write protect
#pragma config CP = OFF // disable code protect

// DEVCFG1
#pragma config FNOSC = FRCPLL // use fast frc oscillator with pll
#pragma config FSOSCEN = OFF // disable secondary oscillator
#pragma config IESO = OFF // disable switching clocks
#pragma config POSCMOD = OFF // primary osc disabled
#pragma config OSCIOFNC = OFF // disable clock output
#pragma config FPBDIV = DIV_1 // divide sysclk freq by 1 for peripheral bus clock
#pragma config FCKSM = CSDCMD // disable clock switch and FSCM
#pragma config WDTPS = PS1048576 // use largest wdt value
#pragma config WINDIS = OFF // use non-window mode wdt
#pragma config FWDTEN = OFF // wdt disabled
#pragma config FWDTWINSZ = WINSZ_25 // wdt window at 25%

// DEVCFG2 - get the sysclk clock to 48MHz from the 8MHz fast rc internal oscillator
#pragma config FPLLIDIV = DIV_2 // divide input clock to be in range 4-5MHz
#pragma config FPLLMUL = MUL_24 // multiply clock after FPLLIDIV
#pragma config FPLLODIV = DIV_2 // divide clock after FPLLMUL to get 48MHz

// DEVCFG3
#pragma config USERID = 00000000 // some 16bit userid, doesn't matter what
#pragma config PMDL1WAY = OFF // allow multiple reconfigurations
#pragma config IOL1WAY = OFF // allow multiple reconfigurations

void readUART1(char * string, int maxLength);
void writeUART1(const char * string);

void writePin(unsigned char reg, unsigned char value);
unsigned char readPin(unsigned char reg);

int main() {

    __builtin_disable_interrupts(); // disable interrupts while initializing things

    // set the CP0 CONFIG register to indicate that kseg0 is cacheable (0x3)
    __builtin_mtc0(_CP0_CONFIG, _CP0_CONFIG_SELECT, 0xa4210583);

    // 0 data RAM access wait states
    BMXCONbits.BMXWSDRM = 0x0;

    // enable multi vector interrupts
    INTCONbits.MVEC = 0x1;

    // disable JTAG to get pins back
    DDPCONbits.JTAGEN = 0;

    // do your TRIS and LAT commands here
    TRISAbits.TRISA4 = 0; // set A4 as output
    TRISBbits.TRISB4 = 1; // set B4 as input
    LATAbits.LATA4 = 0; //off
    
    U1RXRbits.U1RXR = 0b0000; // set A2 to U1RX
    RPB3Rbits.RPB3R = 0b0001; // set B3 to U1TX
    
    __builtin_enable_interrupts();
    
    // turn on UART1 without an interrupt
    U1MODEbits.BRGH = 0; // set baud to 230400
    U1BRG = ((48000000 / 230400) / 16) - 1;

    // 8 bit, no parity bit, and 1 stop bit (8N1 setup)
    U1MODEbits.PDSEL = 0;
    U1MODEbits.STSEL = 0;

    // configure TX & RX pins as output & input pins
    U1STAbits.UTXEN = 1;
    U1STAbits.URXEN = 1;

    // enable the uart
    U1MODEbits.ON = 1;
    
    char m[100];
    
    i2c_master_setup(); 
    writePin(0x00, 0b01111111); // IODIR: set GP7 as input, others as output
    
    unsigned char st = 0b00000000;    
        
    //st = readPin(0x09); // GPIO: get 1 if high, 0 if low
    //sprintf(m, "Button pushed\r\n");
    //sprintf(m,"%x", st);
    //writeUART1(m);
    
    while (1) {
        
        st = readPin(0x09); // GPIO: get 1 if high, 0 if low
        if ((st & 0b00000001) == 0b00000000) { // if GP0 is low
            writePin(0x0A, 0b10000000); // OLAT: set GP7 as high, others not outputs
        } else {
            writePin(0x0A, 0b00000000); // OLAT: set GP7 as low
        } 

        _CP0_SET_COUNT(0);
        LATAbits.LATA4 = 1; //led on
        while(_CP0_GET_COUNT() < 1200000) {} // 0.05 s
        LATAbits.LATA4 = 0; //led off
        while(_CP0_GET_COUNT() < 2400000) {} 
            
    }
}


// Read from UART1
// block other functions until you get a '\r' or '\n'
// send the pointer to your char array and the number of elements in the array
void readUART1(char * message, int maxLength) {
  char data = 0;
  int complete = 0, num_bytes = 0;
  // loop until you get a '\r' or '\n'
  while (!complete) {
    if (U1STAbits.URXDA) { // if data is available
      data = U1RXREG;      // read the data
      if ((data == '\n') || (data == '\r')) {
        complete = 1;
      } else {
        message[num_bytes] = data;
        ++num_bytes;
        // roll over if the array is too small
        if (num_bytes >= maxLength) {
          num_bytes = 0;
        }
      }
    }
  }
  // end the string
  message[num_bytes] = '\0';
}

// Write a character array using UART1
void writeUART1(const char * string) {
  while (*string != '\0') {
    while (U1STAbits.UTXBF) {
      ; // wait until tx buffer isn't full
    }
    U1TXREG = *string;
    ++string;
  }
}

// Write I2C
void writePin(unsigned char reg, unsigned char value) {
    i2c_master_start();
    i2c_master_send(0b01000000);
    i2c_master_send(reg);
    i2c_master_send(value);
    i2c_master_stop();
}

// Read I2C
unsigned char readPin(unsigned char reg) {
    i2c_master_start();
    i2c_master_send(0b01000000); //write
    i2c_master_send(reg);
    i2c_master_restart();
    i2c_master_send(0b01000001); //read
    unsigned char rec = 0b00000000;
    rec = i2c_master_recv();
    i2c_master_ack(1);
    i2c_master_stop();
    
    return rec;
}