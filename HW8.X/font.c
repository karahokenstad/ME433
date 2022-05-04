#include<xc.h>           // processor SFR definitions
#include<sys/attribs.h>  // __ISR macro
#include<stdio.h>
#include"ssd1306.h"
#include"font.h"

char m[100];

// draw character starting at (x,y) coordinates
void drawChar(int x, int y, char ch) {
    unsigned char colm;
    int chASCII = ch;
    for (int i=0; i<5; i++) {
        colm = ASCII[chASCII-32][i];
        for (int j=0; j<8; j++) {
            if ((colm & 0b00000001) == 0b00000001) {
                ssd1306_drawPixel(x+i,y+j,1); // turn on
            }
            colm = colm >> 1;

        }
    }
}

// draw string of chars starting at (x,y)
void drawString(int x, int y, char message[]) {
    int xpos = x;
    int ypos = y;
    for (int i=0; message[i] != NULL; i++) {
        drawChar(xpos,ypos,message[i]);
        xpos = xpos + 5;
        if (xpos > 124) {
            ypos = ypos + 8; // move to next line
            xpos = 0;
        }
    }
}