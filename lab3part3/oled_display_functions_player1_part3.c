/*
 * Ajit Punj and Baban Malhi
 * lab3 part3-wireless pong game communicating via XBee module
 *
 * THIS IS PLAYER 1 CODE. This code controls the calculations for ball movements and sends
 * ball coordinates to remote processor, and
 * moves the left paddle, and sends paddle coordinates to remote processor.
 */

// Timing Delays
#define SSD1351_DELAYS_HWFILL	    (3)
#define SSD1351_DELAYS_HWLINE       (1)

//colors
#define	BLACK           0x0000
#define	BLUE            0x001F
#define	RED             0xF800
#define	GREEN           0x07E0
#define CYAN            0x07FF
#define MAGENTA         0xF81F
#define YELLOW          0xFFE0  
#define WHITE           0xFFFF

// SSD1351 Commands
#define SSD1351_CMD_SETCOLUMN 		0x15
#define SSD1351_CMD_SETROW    		0x75
#define SSD1351_CMD_WRITERAM   		0x5C
#define SSD1351_CMD_READRAM   		0x5D
#define SSD1351_CMD_SETREMAP 		0xA0
#define SSD1351_CMD_STARTLINE 		0xA1
#define SSD1351_CMD_DISPLAYOFFSET 	0xA2
#define SSD1351_CMD_DISPLAYALLOFF 	0xA4
#define SSD1351_CMD_DISPLAYALLON  	0xA5
#define SSD1351_CMD_NORMALDISPLAY 	0xA6
#define SSD1351_CMD_INVERTDISPLAY 	0xA7
#define SSD1351_CMD_FUNCTIONSELECT 	0xAB
#define SSD1351_CMD_DISPLAYOFF 		0xAE
#define SSD1351_CMD_DISPLAYON     	0xAF
#define SSD1351_CMD_PRECHARGE 		0xB1
#define SSD1351_CMD_DISPLAYENHANCE	0xB2
#define SSD1351_CMD_CLOCKDIV 		0xB3
#define SSD1351_CMD_SETVSL 		0xB4
#define SSD1351_CMD_SETGPIO 		0xB5
#define SSD1351_CMD_PRECHARGE2 		0xB6
#define SSD1351_CMD_SETGRAY 		0xB8
#define SSD1351_CMD_USELUT 		0xB9
#define SSD1351_CMD_PRECHARGELEVEL 	0xBB
#define SSD1351_CMD_VCOMH 		0xBE
#define SSD1351_CMD_CONTRASTABC		0xC1
#define SSD1351_CMD_CONTRASTMASTER	0xC7
#define SSD1351_CMD_MUXRATIO            0xCA
#define SSD1351_CMD_COMMANDLOCK         0xFD
#define SSD1351_CMD_HORIZSCROLL         0x96
#define SSD1351_CMD_STOPSCROLL          0x9E
#define SSD1351_CMD_STARTSCROLL         0x9F
#define SSD1351WIDTH 128
#define SSD1351HEIGHT 128  

int WIDTH=128;
int HEIGHT=128;

#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_gpio.h"
#include "inc/hw_ints.h"
#include "inc/hw_nvic.h"
#include "driverlib/debug.h"
#include "driverlib/fpu.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include "driverlib/rom.h"
#include "driverlib/sysctl.h"
#include "driverlib/interrupt.h"
#include "driverlib/uart.h"
#include "utils/uartstdio.h"

#include "inc/tm4c123gh6pm.h"
#include "driverlib/timer.h"
#include "driverlib/systick.h"

#include "driverlib/ssi.h"

#include "glcdfont.c"
volatile int edge1, edge2, counter, pulse, arrayVals;
volatile int iTick;
volatile int arrayValues[32];
volatile unsigned char fromUART;
//char positions for bottom half of screen
volatile int remoteCharXValue=0;
volatile int remoteCharYValue=128/2+5;
//char buffer
volatile char charBuffer[200];
volatile int charBufferIndex=0;
volatile bool Tx_done;
//paddle and ball globals
const int paddle1X=10;
//paddleX variables are const since X coordinate will never change
int paddle1Y=128/2-20/2;//halfway through screen
const int paddle2X=128-10;//padding of 10 px
int paddle2Y=128/2-20/2;
int ballX=128/2;
int ballY=128/2;//middle of screen
int ballXDirection=0;//ball direction either 0 (left) or 1 (right)
int ballYDirection=0;//ball direction either 0 (down) or 1 (up)
const int dx=3;
const int dy=3;

volatile bool systickDone;

void
InitConsole(void)
{
    // Enable GPIO port A which is used for UART0 pins.
    // Enable UART0.
    //
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);

    // Configure the pin muxing for UART0 functions on port A0 and A1.
    // Select the alternate (UART) function for these pins.
    //
    GPIOPinConfigure(GPIO_PA0_U0RX);
    GPIOPinConfigure(GPIO_PA1_U0TX);
    GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);

    // Use the internal 16MHz oscillator as the UART clock source.
    //
    UARTClockSourceSet(UART0_BASE, UART_CLOCK_PIOSC);

    // Initialize the UART for console I/O.
    //
    UARTStdioConfig(0, 115200, 16000000);
}

void writeCommand(int c) {
    //wait until tx fifo empty
		while(SSIBusy(SSI0_BASE))
    {
    }
		//set data command signal low
		GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_7, 0);
		//send 8 bit command
		SSIDataPut(SSI0_BASE, c);
		//wait until fifo empty and transmit data
		while(SSIBusy(SSI0_BASE))
    {
    }
}


void writeData(int c) {
    //set dc high
		GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_7, GPIO_PIN_7);
		//send 8 bit data using blocking put function
		SSIDataPut(SSI0_BASE, c);
} 

void initializePins(){

	SysCtlClockSet(SYSCTL_SYSDIV_2_5 | SYSCTL_USE_PLL | SYSCTL_XTAL_16MHZ | SYSCTL_OSC_MAIN);


    // Display the setup on the console.
    //
    UARTprintf("\nSSI ->\n");
    UARTprintf("  Mode: SPI\n");
    UARTprintf("  Data: 8-bit\n\n");

    // The SSI0 peripheral must be enabled for use.
    //
    SysCtlPeripheralEnable(SYSCTL_PERIPH_SSI0);

    // For this example SSI0 is used with PortA[5:2].
		// GPIO port A needs to be enabled so these pins can be used.
    //
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);

    // Configure the pin muxing for SSI0 functions on port A2, A3, A4, and A5.
    //
    GPIOPinConfigure(GPIO_PA2_SSI0CLK);//pa2 is clock
    GPIOPinConfigure(GPIO_PA3_SSI0FSS);//pa3 is frameslave select
    GPIOPinConfigure(GPIO_PA4_SSI0RX);//pa4 is receive
    GPIOPinConfigure(GPIO_PA5_SSI0TX);//pa5 is transmit
		
		GPIOPinTypeGPIOOutput(GPIO_PORTA_BASE, GPIO_PIN_6 | GPIO_PIN_7);//PA6=reset, PA7=DC

    //
    // Configure the GPIO settings for the SSI pins.  This function also gives
    // control of these pins to the SSI hardware.  Consult the data sheet to
    // see which functions are allocated per pin.
    // The pins are assigned as follows:
    //      PA5 - SSI0Tx
    //      PA4 - SSI0Rx
    //      PA3 - SSI0Fss
    //      PA2 - SSI0CLK
		//			PA6=reset
		//			PA7=DC
    //
    GPIOPinTypeSSI(GPIO_PORTA_BASE, GPIO_PIN_5 | GPIO_PIN_4 | GPIO_PIN_3 |
                   GPIO_PIN_2);

    //
    // Configure and enable the SSI port for SPI master mode.  Use SSI0,
    // system clock supply, idle clock level low and active low clock in
    // freescale SPI mode, master mode, 1MHz SSI frequency, and 8-bit data.
    // For SPI mode, you can set the polarity of the SSI clock when the SSI
    // unit is idle.  You can also configure what clock edge you want to
    // capture data on.  Please reference the datasheet for more information on
    // the different SPI modes.
    //
    SSIConfigSetExpClk(SSI0_BASE, SysCtlClockGet(), SSI_FRF_MOTO_MODE_0,
                       SSI_MODE_MASTER, 1000000, 8);

    //
    // Enable the SSI0 module.
    //
    SSIEnable(SSI0_BASE);
}

//UART0 used for console, UART1 used for remote processor
void configureUART1(void){
    // Enable the GPIO Peripheral used by the UART1 and enable UART1
    //
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART1);
    
	
    // Configure GPIO Pins for UART mode.
    //
    GPIOPinConfigure(GPIO_PB0_U1RX);
    GPIOPinConfigure(GPIO_PB1_U1TX);
    GPIOPinTypeUART(GPIO_PORTB_BASE, GPIO_PIN_0 | GPIO_PIN_1);
    
    // Use the internal 16MHz oscillator as the UART clock source.
    //
    UARTClockSourceSet(UART1_BASE, UART_CLOCK_PIOSC);
    
    // Initialize UART1 with 9600 baud, 8 bit data, one stop, no parity
    UARTConfigSetExpClk(UART1_BASE, 16000000, 9600,
                            (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE |
                             UART_CONFIG_PAR_NONE));
    IntEnable(INT_UART1);
    UARTIntEnable(UART1_BASE, UART_INT_RX | UART_INT_RT);
    
    
    IntMasterEnable();
}

//begin OLED display functions
void goTo(int x, int y) {
  if ((x >= SSD1351WIDTH) || (y >= SSD1351HEIGHT)) return;
  
  // set x and y coordinate
  writeCommand(SSD1351_CMD_SETCOLUMN);
  writeData(x);
  writeData(SSD1351WIDTH-1);

  writeCommand(SSD1351_CMD_SETROW);
  writeData(y);
  writeData(SSD1351HEIGHT-1);

  writeCommand(SSD1351_CMD_WRITERAM);  
}

int Color565(int r, int g, int b) {
  int c;
  c = r >> 3;
  c <<= 6;
  c |= g >> 2;
  c <<= 5;
  c |= b >> 3;

  return c;
}

void fillScreen(int fillcolor) {
  fillRect(0, 0, SSD1351WIDTH, SSD1351HEIGHT, fillcolor);
}

// Draw a filled rectangle with no rotation.
void rawFillRect(int x, int y, int w, int h, int fillcolor) {
  // Bounds check
  if ((x >= SSD1351WIDTH) || (y >= SSD1351HEIGHT))
    return;

  // Y bounds check
  if (y+h > SSD1351HEIGHT)
  {
    h = SSD1351HEIGHT - y - 1;
  }

  // X bounds check
  if (x+w > SSD1351WIDTH)
  {
    w = SSD1351WIDTH - x - 1;
  }
  
  /*
  Serial.print(x); Serial.print(", ");
  Serial.print(y); Serial.print(", ");
  Serial.print(w); Serial.print(", ");
  Serial.print(h); Serial.println(", ");
*/

  // set location
  writeCommand(SSD1351_CMD_SETCOLUMN);
  writeData(x);
  writeData(x+w-1);
  writeCommand(SSD1351_CMD_SETROW);
  writeData(y);
  writeData(y+h-1);
  // fill!
  writeCommand(SSD1351_CMD_WRITERAM);  

  for (int i=0; i < w*h; i++) {
    writeData(fillcolor >> 8);
    writeData(fillcolor);
  }
}

void swap(int x, int y){
	int temp=x;
	x=y;
	y=temp;
}

int getRotation(void){
	return 0;
}

/**************************************************************************/
/*!
    @brief  Draws a filled rectangle using HW acceleration
*/
/**************************************************************************/
int fillRect(int x, int y, int w, int h, int fillcolor) {
	//int temp;
  // Transform x and y based on current rotation.
  switch (getRotation()) {
	//switch(0){
  case 0:  // No rotation
          //UARTprintf("case0 for fill rect");
    rawFillRect(x, y, w, h, fillcolor);
    break;
  case 1:  // Rotated 90 degrees clockwise.
    swap(x, y);
//		temp=x;
//		x=y;
//		y=temp;
    x = WIDTH - x - h;
    rawFillRect(x, y, h, w, fillcolor);
    break;
  case 2:  // Rotated 180 degrees clockwise.
    x = WIDTH - x - w;
    y = HEIGHT - y - h;
    rawFillRect(x, y, w, h, fillcolor);
    break;
  case 3:  // Rotated 270 degrees clockwise.
    swap(x, y);
    y = HEIGHT - y - w;
    rawFillRect(x, y, h, w, fillcolor);
    break;
  }
	return 0;
}

// Draw a horizontal line ignoring any screen rotation.
void rawFastHLine(int x, int y, int w, int color) {
  // Bounds check
  if ((x >= SSD1351WIDTH) || (y >= SSD1351HEIGHT))
    return;

  // X bounds check
  if (x+w > SSD1351WIDTH)
  {
    w = SSD1351WIDTH - x - 1;
  }

  if (w < 0) return;

  // set location
  writeCommand(SSD1351_CMD_SETCOLUMN);
  writeData(x);
  writeData(x+w-1);
  writeCommand(SSD1351_CMD_SETROW);
  writeData(y);
  writeData(y);
  // fill!
  writeCommand(SSD1351_CMD_WRITERAM);  

  for (int i=0; i < w; i++) {
    writeData(color >> 8);
    writeData(color);
  }
}

// Draw a vertical line ignoring any screen rotation.
void rawFastVLine(int x, int y, int h, int color) {
  // Bounds check
  if ((x >= SSD1351WIDTH) || (y >= SSD1351HEIGHT))
  return;

  // X bounds check
  if (y+h > SSD1351HEIGHT)
  {
    h = SSD1351HEIGHT - y - 1;
  }

  if (h < 0) return;

  // set location
  writeCommand(SSD1351_CMD_SETCOLUMN);
  writeData(x);
  writeData(x);
  writeCommand(SSD1351_CMD_SETROW);
  writeData(y);
  writeData(y+h-1);
  // fill!
  writeCommand(SSD1351_CMD_WRITERAM);  

  for (int i=0; i < h; i++) {
    writeData(color >> 8);
    writeData(color);
  }
}



void drawFastVLine(int x, int y, int h, int color) {
  // Transform x and y based on current rotation.
  switch (getRotation()) {
  case 0:  // No rotation
    rawFastVLine(x, y, h, color);
    break;
  case 1:  // Rotated 90 degrees clockwise.
    swap(x, y);
    x = WIDTH - x - h;
    rawFastHLine(x, y, h, color);
    break;
  case 2:  // Rotated 180 degrees clockwise.
    x = WIDTH - x - 1;
    y = HEIGHT - y - h;
    rawFastVLine(x, y, h, color);
    break;
  case 3:  // Rotated 270 degrees clockwise.
    swap(x, y);
    y = HEIGHT - y - 1;
    rawFastHLine(x, y, h, color);
    break;
  }
}

void drawFastHLine(int x, int y, int w, int color) {
  // Transform x and y based on current rotation.
  switch (getRotation()) {
  case 0:  // No rotation.
    rawFastHLine(x, y, w, color);
    break;
  case 1:  // Rotated 90 degrees clockwise.
    swap(x, y);
    x = WIDTH - x - 1;
    rawFastVLine(x, y, w, color);
    break;
  case 2:  // Rotated 180 degrees clockwise.
    x = WIDTH - x - w;
    y = HEIGHT - y - 1;
    rawFastHLine(x, y, w, color);
    break;
  case 3:  // Rotated 270 degrees clockwise.
    swap(x, y);
    y = HEIGHT - y - w;
    rawFastVLine(x, y, w, color);
    break;
  }
}

// Used to do circles and roundrects
void fillCircleHelper(int x0, int y0, int r,
                      int cornername, int delta, int color) {
    
    int f     = 1 - r;
    int ddF_x = 1;
    int ddF_y = -2 * r;
    int x     = 0;
    int y     = r;
    
    while (x<y) {
        if (f >= 0) {
            y--;
            ddF_y += 2;
            f     += ddF_y;
        }
        x++;
        ddF_x += 2;
        f     += ddF_x;
        
        if (cornername & 0x1) {
            drawFastVLine(x0+x, y0-y, 2*y+1+delta, color);
            drawFastVLine(x0+y, y0-x, 2*x+1+delta, color);
        }
        if (cornername & 0x2) {
            drawFastVLine(x0-x, y0-y, 2*y+1+delta, color);
            drawFastVLine(x0-y, y0-x, 2*x+1+delta, color);
        }
    }
}

//used for drawing the ball
void fillCircle(int x0, int y0, int r,
                              int color) {
    drawFastVLine(x0, y0-r, 2*r+1, color);
    fillCircleHelper(x0, y0, r, 3, 0, color);
}


void begin(void) {

    // Initialization Sequence
    writeCommand(SSD1351_CMD_COMMANDLOCK);  // set command lock
    writeData(0x12);  
    writeCommand(SSD1351_CMD_COMMANDLOCK);  // set command lock
    writeData(0xB1);

    writeCommand(SSD1351_CMD_DISPLAYOFF);  		// 0xAE

    writeCommand(SSD1351_CMD_CLOCKDIV);  		// 0xB3
    writeCommand(0xF1);  						// 7:4 = Oscillator Frequency, 3:0 = CLK Div Ratio (A[3:0]+1 = 1..16)
    
    writeCommand(SSD1351_CMD_MUXRATIO);
    writeData(127);
    
    writeCommand(SSD1351_CMD_SETREMAP);
    writeData(0x74);
  
    writeCommand(SSD1351_CMD_SETCOLUMN);
    writeData(0x00);
    writeData(0x7F);
    writeCommand(SSD1351_CMD_SETROW);
    writeData(0x00);
    writeData(0x7F);

    writeCommand(SSD1351_CMD_STARTLINE); 		// 0xA1
    if (SSD1351HEIGHT == 96) {
      writeData(96);
    } else {
      writeData(0);
    }


    writeCommand(SSD1351_CMD_DISPLAYOFFSET); 	// 0xA2
    writeData(0x0);

    writeCommand(SSD1351_CMD_SETGPIO);
    writeData(0x00);
    
    writeCommand(SSD1351_CMD_FUNCTIONSELECT);
    writeData(0x01); // internal (diode drop)
    //writeData(0x01); // external bias

//    writeCommand(SSSD1351_CMD_SETPHASELENGTH);
//    writeData(0x32);

    writeCommand(SSD1351_CMD_PRECHARGE);  		// 0xB1
    writeCommand(0x32);
 
    writeCommand(SSD1351_CMD_VCOMH);  			// 0xBE
    writeCommand(0x05);

    writeCommand(SSD1351_CMD_NORMALDISPLAY);  	// 0xA6

    writeCommand(SSD1351_CMD_CONTRASTABC);
    writeData(0xC8);
    writeData(0x80);
    writeData(0xC8);

    writeCommand(SSD1351_CMD_CONTRASTMASTER);
    writeData(0x0F);

    writeCommand(SSD1351_CMD_SETVSL );
    writeData(0xA0);
    writeData(0xB5);
    writeData(0x55);
    
    writeCommand(SSD1351_CMD_PRECHARGE2);
    writeData(0x01);
    
    writeCommand(SSD1351_CMD_DISPLAYON);		//--turn on oled panel    
}
void drawPixel(int x, int y, int color)
{
  // Transform x and y based on current rotation.
  switch (getRotation()) {
  // Case 0: No rotation
  case 1:  // Rotated 90 degrees clockwise.
    swap(x, y);
    x = WIDTH - x - 1;
    break;
  case 2:  // Rotated 180 degrees clockwise.
    x = WIDTH - x - 1;
    y = HEIGHT - y - 1;
    break;
  case 3:  // Rotated 270 degrees clockwise.
    swap(x, y);
    y = HEIGHT - y - 1;
    break;
  }

  // Bounds check.
  if ((x >= SSD1351WIDTH) || (y >= SSD1351HEIGHT)) return;
  if ((x < 0) || (y < 0)) return;

  goTo(x, y);
  
  // setup for data
  //*rsport |= rspinmask;
  //*csport &= ~ cspinmask;
  
  writeData(color >> 8);    
  writeData(color);
  
 // *csport |= cspinmask;
}
void drawCircle(int x0, int y0, int r,
                int color) {
    int f = 1 - r;
    int ddF_x = 1;
    int ddF_y = -2 * r;
    int x = 0;
    int y = r;
    
    drawPixel(x0  , y0+r, color);
    drawPixel(x0  , y0-r, color);
    drawPixel(x0+r, y0  , color);
    drawPixel(x0-r, y0  , color);
    
    while (x<y) {
        if (f >= 0) {
            y--;
            ddF_y += 2;
            f += ddF_y;
        }
        x++;
        ddF_x += 2;
        f += ddF_x;
        
        drawPixel(x0 + x, y0 + y, color);
        drawPixel(x0 - x, y0 + y, color);
        drawPixel(x0 + x, y0 - y, color);
        drawPixel(x0 - x, y0 - y, color);
        drawPixel(x0 + y, y0 + x, color);
        drawPixel(x0 - y, y0 + x, color);
        drawPixel(x0 + y, y0 - x, color);
        drawPixel(x0 - y, y0 - x, color);
    }
}

void drawChar(int x, int y, char c,
			    int color, int bg, int size) {

  if((x >= WIDTH)            || // Clip right
     (y >= HEIGHT)           || // Clip bottom
     ((x + 6 * size - 1) < 0) || // Clip left
     ((y + 8 * size - 1) < 0))   // Clip top
    return;
	int i;
  for (i=0; i<6; i++ ) {
    int line;
    if (i == 5) 
      line = 0x0;
    else 
      line = font[(c*5)+i];
    for (int8_t j = 0; j<8; j++) {
      if (line & 0x1) {
        if (size == 1) // default size
          drawPixel(x+i, y+j, color);
        else {  // big size
          fillRect(x+(i*size), y+(j*size), size, size, color);
        } 
      } else if (bg != color) {
        if (size == 1) // default size
          drawPixel(x+i, y+j, bg);
        else {  // big size
          fillRect(x+i*size, y+j*size, size, size, bg);
        }
      }
      line >>= 1;
    }
  }
}
					
void drawLine(int x0, int y0,
			    int x1, int y1,
			    int color) {
  int steep = abs(y1 - y0) > abs(x1 - x0);
  if (steep) {
    swap(x0, y0);
    swap(x1, y1);
  }

  if (x0 > x1) {
    swap(x0, x1);
    swap(y0, y1);
  }

  int dx, dy;
  dx = x1 - x0;
  dy = abs(y1 - y0);

  int err = dx / 2;
  int ystep;

  if (y0 < y1) {
    ystep = 1;
  } else {
    ystep = -1;
  }

  for (; x0<=x1; x0++) {
    if (steep) {
      drawPixel(y0, x0, color);
    } else {
      drawPixel(x0, y0, color);
    }
    err -= dy;
    if (err < 0) {
      y0 += ystep;
      err += dx;
    }
  }
}//end oled functions

//interrupt handler for uart1 (Xbee) and gets paddle coordinates from remote
//processor. This handler will not recieve ball coordinates because
//player1 code controlls ball movements.
void UART1IntHandler(void) {
    int32_t ui32Status;
    
    //
    // Get the interrrupt status.
    //
    ui32Status = UARTIntStatus(UART1_BASE, true);
    
    //
    // Clear the asserted interrupts.
    //
    UARTIntClear(UART1_BASE, ui32Status);
    
    //
    // Loop while there are characters in the receive FIFO.
    //
    
   // bool donePrinting=false;
    char firstChar;
    int fromUARTChar1, fromUartChar2;
    while(UARTCharsAvail(UART1_BASE))
    {
        //get uart char from rx fifo
        firstChar=UARTCharGet(UART1_BASE);
        //get char 2 as integer value
        fromUartChar2=UARTCharGet(UART1_BASE);
        if(firstChar=='p'){//if first value received is char 'p', move remote paddle (right paddle)
            fillRect(paddle2X,paddle2Y,6,20,WHITE);
            paddle2Y=fromUartChar2;
            fillRect(paddle2X,paddle2Y,6,20,BLACK);
            
        }
        /* don't really need this in player 1 code since player 2 always sends
         only paddle2 position, and never sends ball movment
        else{//else first char not 'p', move ball
            fromUARTChar1=(int *) firstChar;
            //fill ball white
            drawCircle(ballX,ballY,4,WHITE);
            ballX=fromUARTChar1;
            ballY=fromUartChar2;
            drawCircle(ballX,ballY,4,BLACK);
        }*/
        
    }
    
}
					
void initializeIRInterrupts(void){
				//enable ports F and B for LEDs and interrupt port
        SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
        SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);    
        //configure PB5 as input 
        GPIOPinTypeGPIOInput(GPIO_PORTB_BASE, GPIO_PIN_5);
    
        //enable LED outputs
        GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3);
    
        //configure interrupt type for PB5 (falling edge)
		GPIOIntTypeSet(GPIO_PORTB_BASE, GPIO_PIN_5, GPIO_FALLING_EDGE);
        //enable interrupts after configuring
		GPIOIntEnable(GPIO_PORTB_BASE, GPIO_INT_PIN_5);
		IntEnable(INT_GPIOB);
		IntMasterEnable();
        //initialize edge global variables and turn on the red LED 
		edge1=0;
		edge2=0;
		GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3, GPIO_PIN_1);//red led
        
        //set the systick period to close to the max value and enable it
		SysTickPeriodSet (16777000);
		SysTickEnable();
		int resetInt=0;
		iTick=0;
		arrayVals=0;
}

void SysTick_Handler (void) {
 
    //clear interrupts
    GPIOIntClear(GPIO_PORTB_BASE, GPIO_INT_PIN_5);
	counter++;
		//UARTprintf("enters handler\n");
    //first fallin edge of the first pulse hit since iTick == 0
	if(iTick==0){
		//reset systick, set edge1, set iTick=1, then edge2 gets set at the next falling edge
		SysTickPeriodSet (16777000);
		NVIC_ST_CURRENT_R=0;
		edge1=SysTickValueGet();
		iTick=1;
        systickDone=false;//global boolean to disable ball movement while pulses are measured to eliminate errors
	}
    //every pulse after the first is handled here
	else if(iTick==1){
        //UARTprintf("hit int arrayVals=%d\n",arrayVals);
		//get edge2 value and store it before re-clearing systick and getting the cleared value again for the first edge of the next pulse
		edge2=SysTickValueGet();
        //since both edge values are obtained, calculate the pulse width
		pulse=edge1-edge2;
        //store the pulse width raw value to the array, and increment the array position tracker
		arrayValues[arrayVals]=pulse;
		arrayVals++;
		//reset systick and get edge1, on next falling edge it will get edge2 and calculate from here to next falling edge
		SysTickPeriodSet (16777000);
		NVIC_ST_CURRENT_R=0;
		edge1=SysTickValueGet();
	}
	
}

//send coordinates from positions[] array (will always be size 2)
void SendPositions(int positions[]) {
    
    for (int i=0; i<2; i++) {
        UARTCharPut(UART1_BASE,positions[i]);
    }
}
					
int main(void){
	InitConsole();//uart
	initializePins();//set spi/ssi pins
    //toggle reset on display to turn on, set to high during actions
    GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_6, GPIO_PIN_6);
	GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_6, 0);//reset is low
	GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_6, GPIO_PIN_6);//reset is low
	begin();
	initializeIRInterrupts();//lab1 code
    configureUART1();
	int buttonPressed;
	int oldButtonPressed=-1;
	int amountPressed=0;
	char toPrint;
	int charXValue=0;
	int charYValue=0;
   
	int bitValues[32];
        int pulseWidth=0;
        int arrayPos=0;
    int buttonBreak=0;
   
	iTick=0;
	GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_6, GPIO_PIN_6);
	GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_6, 0);//reset is low
	GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_6, GPIO_PIN_6);//reset is low
    //fill screen white and draw a line halfway through the screen
	fillScreen(0xffff);
    
    //draw paddles
    fillRect(paddle1X,paddle1Y,6,20,BLACK);//paddle1 x,y,w,h,color
    fillRect(paddle2X,paddle2Y,6,20,BLACK);//paddle2 x,y,w,h,color
    //draw ball
    fillCircle(ballX,ballY,4,BLACK);
    
    const int PLAYER=1;
    /*
     * player integer controls which paddle is moved: player 1 moves left
     * player 2 moves right, and sends paddle movement to remote processor.
     * player 1 will be the MASTER in terms of the ball, so player 2 processor
     * only worries about drawing the ball based on calculate coordinates sent
     * to player2. 
     */
    int positions[2];
    systickDone=true;
    SysCtlDelay(SysCtlClockGet()/3/30);// delay before serve
	while(1){
		//keep remote code the same to check for button presses, add in ball and
        //paddle movement
        
        if(systickDone==true){//if systick is not measuring remote pulses, move the ball, else ignore this code to ensure interrupt handler works
            arrayVals=0;
        //control direction of movement
        
        if(ballXDirection==1){//movement is to the right
            //fill circle white
            fillCircle(ballX,ballY,4,WHITE);
            //move coordinates
            ballX=ballX+dx;
            //redraw moved ball
            fillCircle(ballX,ballY,4,BLACK);//paddle1 x,y,w,h,color
        }
        else if(ballXDirection==0){//movement is to the left
            //fill circle white
            fillCircle(ballX,ballY,4,WHITE);//paddle1 x,y,w,h,color
            //move coordinates
            ballX=ballX-dx;
            //redraw moved ball
            fillCircle(ballX,ballY,4,BLACK);//paddle1 x,y,w,h,color
        }
        if(ballYDirection==0){//move up
            //fill circle white
            fillCircle(ballX,ballY,4,WHITE);//paddle1 x,y,w,h,color
            //move coordinates
            //ballX=ballX-dx;
            ballY=ballY-dy;
            //redraw moved ball
            fillCircle(ballX,ballY,4,BLACK);//paddle1 x,y,w,h,color
        }
        else if(ballYDirection==1){//move down
            //fill circle white
            fillCircle(ballX,ballY,4,WHITE);//paddle1 x,y,w,h,color
            //move coordinates
            ballY=ballY+dy;
            //redraw moved ball
            fillCircle(ballX,ballY,4,BLACK);//paddle1 x,y,w,h,color
        }
        
        if (ballX-4>18 && ballX-4<24) {//hit left edge where paddle should be
            if(ballY>=paddle1Y && ballY<=paddle1Y+20){//if it hits within the paddle's range, bounce off
                ballXDirection=1;//change direction of movement
            }
            else{//paddle misses, game over
                fillCircle(ballX,ballY,4,WHITE);
                ballX=128/2;
                ballY=128/2;
                fillCircle(ballX,ballY,4,BLACK);
                ballXDirection=0;//return to middle and serve towards loser
            }
            
        }
        else if(ballX+4<128-18 && ballX+4>128-24){
            //hit right edge
            //ballXDirection=0;
            if(ballY>=paddle2Y && ballY<=paddle2Y+20){//if it hits within the paddle's range, bounce off
                ballXDirection=0;//change direction of movement
            }
            else{//paddle misses, game over
                fillCircle(ballX,ballY,4,WHITE);
                ballX=128/2;
                ballY=128/2;
                fillCircle(ballX,ballY,4,BLACK);
                ballXDirection=1;//return to middle and serve towards loser
            }
        }
        
        //bounce off top and bottom
        if(ballY==4){//hits top
            ballYDirection=1;
        }
        else if(ballY==128-4){
            ballYDirection=0;
        }
            //transmit ballX and ballY in positions array by calling sendPositions
            positions[0]=ballX;
            positions[1]=ballY;
            //positionArray[2]=paddle1Y;
            SendPositions(positions);
            //delay for ball movment
            SysCtlDelay(SysCtlClockGet()/3/80);
            
        }//systickDone end
        
        //when all 32 bits are recorded, find which button was pressed and put it on the display
        //multiple button presses trigger character switching, new button moves cursor over to print new char set
        if(arrayVals==32){
            //clear interrupts and disable interrupts while printing
            GPIOIntClear(GPIO_PORTB_BASE, GPIO_INT_PIN_5);
            GPIOIntDisable(GPIO_PORTB_BASE, GPIO_INT_PIN_5);
            //loop through bit values
            //buttonBreak=0;
            int loopcount;
            for(loopcount=0;loopcount<32;loopcount++){
                //take the current raw pulse width value, and decide if it is a 1 or 0
                int tempbitval=arrayValues[loopcount];
                //if pulse time is in the range 178000<width<180000, it is a 1 since 180000/80000000=2.25 ms which is the recorded period for a 1 from the logic analyzer
                if(tempbitval>178000 && tempbitval<180000)
                    arrayValues[loopcount]=1;
                //likewise, if pulse time is in the range 87000<width<89000 it is a 0 since 89000/80000000=1.1125 which is the recorded period for a 0 from the logic analyzer
                else if(tempbitval>87000 && tempbitval<89000)
                    arrayValues[loopcount]=0;
            }
            //calculate the binary value of the last 8 bits from the array in a decimal representation by taking the value from the array
            //multiplying remoteVal by 10 and then adding the array value to remoteVal
            int remoteVal=0;
            for(loopcount=24;loopcount<32;loopcount++){
                remoteVal*=10;
                remoteVal+=arrayValues[loopcount];
                
            }
            UARTprintf("remote val=%d\n",remoteVal);
            //remoteVal is a 8 bit number represented in decimal (so beginning 0's omitted)
            //which represents the last 8 bits of the pattern, minus the final bit (so bits 23:30)
            //and map these using a switch case to the recorded logic analyzer values
            //set toPrint (char) to the correct value within the array of characters
            switch(remoteVal){
                    
                    case 10111:
                        UARTprintf("Down\n");
                        if(paddle1Y+20<128){
                            //erase, increment, redraw, send char to remote
                            fillRect(paddle1X,paddle1Y,6,20,WHITE);
                            paddle1Y+=6;
                            fillRect(paddle1X,paddle1Y,6,20,BLACK);
                            positions[0]='p';//send char 'p' in first array spot to tell remote processor this is a paddle movement
                            positions[1]=paddle1Y;
                            SendPositions(positions);//send local paddle movement
                        }
                        
                        buttonPressed=1;
                        break;
                    case 1111:
                        UARTprintf("Up\n");
                        if(paddle1Y>0){
                            //erase, decrement, redraw, send char to remote
                            fillRect(paddle1X,paddle1Y,6,20,WHITE);
                            paddle1Y-=6;
                            fillRect(paddle1X,paddle1Y,6,20,BLACK);
                            positions[0]='p';
                            positions[1]=paddle1Y;
                            SendPositions(positions);
                        }
                        buttonPressed=2;
                        break;
                case 111:
                    UARTprintf("Mute\n");
                    toPrint=0x3;//end of transmission
                    charBufferIndex++;
                    buttonPressed=10;//10 is for mute/enter now
                    break;
                default://default case when an error is found due to invalid button pressed or noise detected
                    UARTprintf("error, try again\n");
                    //buttonPressed=-1;
                    break;
                 
            }
            
            //reset flags and necessary variables for next measurement
            //arrayPos=0;
            arrayVals=0;
            iTick=0;
            systickDone=true;
            //clear interrupts and enable again to calculate next value
            GPIOIntClear(GPIO_PORTB_BASE, GPIO_INT_PIN_5);
            GPIOIntEnable(GPIO_PORTB_BASE, GPIO_INT_PIN_5);
				}
        
	}
	return 0;
}