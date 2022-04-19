#include<xc.h>           // processor SFR definitions
#include<sys/attribs.h>  // __ISR macro
#include<stdio.h>
#include<math.h> 

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
unsigned char spi_io(unsigned char o);

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
    
    // setup SPI1
    SPI1CON = 0; // turn off the spi module and reset it
    SPI1BUF; // clear the rx buffer by reading from it
    SPI1BRG = 1; // 1000 for 24kHz, 1 for 12MHz; // baud rate to 10 MHz [SPI1BRG = (48000000/(2*desired))-1]
    SPI1STATbits.SPIROV = 0; // clear the overflow bit
    SPI1CONbits.CKE = 1; // data changes when clock goes from hi to lo (since CKP is 0)
    SPI1CONbits.MSTEN = 1; // master operation
    SPI1CONbits.ON = 1; // turn on spi 
    
    // SPI pins
    RPB15Rbits.RPB15R = 0b0011; // set B15 to SS1
    RPB13Rbits.RPB13R = 0b0011; // set B13 to SD01
    TRISBbits.TRISB15 = 0; // set SS1 as output
    TRISBbits.TRISB13 = 0; // set SDO1 as output
    TRISBbits.TRISB14 = 0; // set SCK1 as output
    LATBbits.LATB15 = 1; //SS1 starts high


    char m[100];
    
    unsigned short s=0;
    
    unsigned short make_num(unsigned char a_or_b, unsigned char v) {
        // a_or_b: a=0, b=1
        // v is voltage
        s=0;
        s = s | (a_or_b << 15);
        s = s | (0b111 << 12);
        s = s | (v << 4);
        return s;
    }
    
    _CP0_SET_COUNT(0);
    int tnext = 240000; // 0.01 sec
    float vsin;
    char vsinchar; // 0 to 255
    float vtri = 0;
    char vtrichar; // 0 to 255
            
    while (1) {
        
        //sine wave
        vsin = 1.65*sin(2*3.14*2*_CP0_GET_COUNT()/24000000)+1.65; // 0 to 3.3 V
        vsinchar = vsin*255/3.3; 
        s = make_num(0,vsinchar);
        LATBbits.LATB15 = 0; //SS1 low
        spi_io(s>>8);
        spi_io(s);
        LATBbits.LATB15 = 1; // SS1 high
        
        // triangle 
        vtri = 6.6*abs(((_CP0_GET_COUNT()-6000000) % 24000000)-12000000)/24000000;
        vtrichar = vtri*255/3.3;
        s = make_num(1,vtrichar);
        LATBbits.LATB15 = 0; //SS1 low
        spi_io(s>>8);
        spi_io(s);
        LATBbits.LATB15 = 1; // SS1 high
 
        while(_CP0_GET_COUNT() < tnext) {}// wait 0.01 sec
        tnext = tnext + 240000;   
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

// send a byte via spi and return the response
unsigned char spi_io(unsigned char o) {
  SPI1BUF = o;
  while(!SPI1STATbits.SPIRBF) { // wait to receive the byte
    ;
  }
  return SPI1BUF;
}