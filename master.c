/*
 * MasterMind implementation: template; see comments below on which parts need to be completed
 * CW spec: https://www.macs.hw.ac.uk/~hwloidl/Courses/F28HS/F28HS_CW2_2022.pdf
 * This repo: https://gitlab-student.macs.hw.ac.uk/f28hs-2021-22/f28hs-2021-22-staff/f28hs-2021-22-cwk2-sys

 * Compile: 
 gcc -c -o lcdBinary.o lcdBinary.c
 gcc -c -o master-mind.o master-mind.c
 gcc -o master-mind master-mind.o lcdBinary.o
 * Run:     
 sudo ./master-mind

 OR use the Makefile to build
 > make all
 and run
 > make run
 and test
 > make test

 ***********************************************************************
 * The Low-level interface to LED, button, and LCD is based on:
 * wiringPi libraries by
 * Copyright (c) 2012-2013 Gordon Henderson.
 ***********************************************************************
 * See:
 *  https://projects.drogon.net/raspberry-pi/wiringpi/
 *
 *    wiringPi is free software: you can redistribute it and/or modify
 *    it under the terms of the GNU Lesser General Public License as published by
 *    the Free Software Foundation, either version 3 of the License, or
 *    (at your option) any later version.
 *
 *    wiringPi is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *    GNU Lesser General Public License for more details.
 *
 *    You should have received a copy of the GNU Lesser General Public License
 *    along with wiringPi.  If not, see <http://www.gnu.org/licenses/>.
 ***********************************************************************
*/

/* ======================================================= */
/* SECTION: includes                                       */
/* ------------------------------------------------------- */

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdarg.h>

#include <unistd.h>
#include <string.h>
#include <time.h>

#include <errno.h>
#include <fcntl.h>
#include <pthread.h>
#include <sys/time.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <sys/wait.h>
#include <sys/ioctl.h>

/* --------------------------------------------------------------------------- */
/* Config settings */
/* you can use CPP flags to e.g. print extra debugging messages */
/* or switch between different versions of the code e.g. digitalWrite() in Assembler */
#define DEBUG
#undef ASM_CODE

// =======================================================
// Tunables
// PINs (based on BCM numbering)
// For wiring see CW spec: https://www.macs.hw.ac.uk/~hwloidl/Courses/F28HS/F28HS_CW2_2022.pdf
// GPIO pin for green LED
#define LED 26
// GPIO pin for red LED
#define LED2 5
// GPIO pin for button
#define BUTTON 19
// =======================================================
// delay for loop iterations (mainly), in ms
// in mili-seconds: 0.2s
#define DELAY   200
// in micro-seconds: 3s
#define TIMEOUT 3000000
// =======================================================
// APP constants   ---------------------------------
// number of colours and length of the sequence
#define COLS 3
#define SEQL 3
// =======================================================

// generic constants

#ifndef TRUE
#define TRUE  (1==1)
#define FALSE (1==2)
#endif

#define PAGE_SIZE   (4*1024)
#define BLOCK_SIZE    (4*1024)

#define INPUT      0
#define OUTPUT       1

#define LOW      0
#define HIGH       1

#define NAN1 8
#define NAN2 9

// =======================================================
// Wiring (see inlined initialisation routine)

#define STRB_PIN 24
#define RS_PIN   25
#define DATA0_PIN 23
#define DATA1_PIN 10
#define DATA2_PIN 27
#define DATA3_PIN 22

/* ======================================================= */
/* SECTION: constants and prototypes                       */
/* ------------------------------------------------------- */

// =======================================================
// char data for the CGRAM, i.e. defining new characters for the display

static unsigned char newChar [8] = 
{
  0b11111,
  0b10001,
  0b10001,
  0b10101,
  0b11111,
  0b10001,
  0b10001,
  0b11111,
} ;

/* Constants */

static const int colors = COLS;
static const int seqlen = SEQL;

static char* color_names[] = { "red", "green", "blue" };

static int* theSeq = NULL;

static int *seq1, *seq2, *cpy1, *cpy2;


/* --------------------------------------------------------------------------- */

// data structure holding data on the representation of the LCD
struct lcdDataStruct
{
  int bits, rows, cols ;
  int rsPin, strbPin ;
  int dataPins [8] ;
  int cx, cy ;
} ;

static int lcdControl ;

/* ***************************************************************************** */
/* INLINED fcts from wiringPi/devLib/lcd.c: */
// HD44780U Commands (see Fig 11, p28 of the Hitachi HD44780U datasheet)

#define LCD_CLEAR 0x01
#define LCD_HOME  0x02
#define LCD_ENTRY 0x04
#define LCD_CTRL  0x08
#define LCD_CDSHIFT 0x10
#define LCD_FUNC  0x20
#define LCD_CGRAM 0x40
#define LCD_DGRAM 0x80

// Bits in the entry register

#define LCD_ENTRY_SH    0x01
#define LCD_ENTRY_ID    0x02

// Bits in the control register

#define LCD_BLINK_CTRL    0x01
#define LCD_CURSOR_CTRL   0x02
#define LCD_DISPLAY_CTRL  0x04

// Bits in the function register

#define LCD_FUNC_F  0x04
#define LCD_FUNC_N  0x08
#define LCD_FUNC_DL 0x10

#define LCD_CDSHIFT_RL  0x04

// Mask for the bottom 64 pins which belong to the Raspberry Pi
//  The others are available for the other devices

#define PI_GPIO_MASK  (0xFFFFFFC0)

static unsigned int gpiobase ;
static uint32_t *gpio ;

static int timed_out = 0;

/* ------------------------------------------------------- */
// misc prototypes

int failure (int fatal, const char *message, ...);
void waitForEnter (void);
void waitForButton (uint32_t *gpio, int button);

/* ======================================================= */
/* SECTION: hardware interface (LED, button, LCD display)  */
/* ------------------------------------------------------- */
/* low-level interface to the hardware */

/* ********************************************************** */
/* COMPLETE the code for all of the functions in this SECTION */
/* Either put them in a separate file, lcdBinary.c, and use   */
/* inline Assembler there, or use a standalone Assembler file */
/* You can also directly implement them here (inline Asm).    */
/* ********************************************************** */

/* These are just prototypes; you need to complete the code for each function */

/* send a @value@ (LOW or HIGH) on pin number @pin@; @gpio@ is the mmaped GPIO base address */
void digitalWrite (uint32_t *gpio, int pin, int value);

/* set the @mode@ of a GPIO @pin@ to INPUT or OUTPUT; @gpio@ is the mmaped GPIO base address */
void pinMode(uint32_t *gpio, int pin, int mode);

/* send a @value@ (LOW or HIGH) on pin number @pin@; @gpio@ is the mmaped GPIO base address */
/* can use digitalWrite(), depending on your implementation */
void writeLED(uint32_t *gpio, int led, int value);

/* read a @value@ (LOW or HIGH) from pin number @pin@ (a button device); @gpio@ is the mmaped GPIO base address */
int readButton(uint32_t *gpio, int button);

/* wait for a button input on pin number @button@; @gpio@ is the mmaped GPIO base address */
/* can use readButton(), depending on your implementation */
void waitForButton (uint32_t *gpio, int button);

void showMatches( int *code,int *seq1, int *seq2,int lcd_format);

/* ======================================================= */
/* SECTION: game logic                                     */
/* ------------------------------------------------------- */
/* AUX fcts of the game logic */

/* ********************************************************** */
/* COMPLETE the code for all of the functions in this SECTION */
/* Implement these as C functions in this file                */
/* ********************************************************** */

void digitalWrite(uint32_t *gpio, int pin, int value)
{
    int off, res;
    off = (value == LOW) ? 10 : 7;

    asm volatile(
        "\tLDR R1, %[gpio]\n"
        "\tADD R0, R1, %[off]\n"
        "\tMOV R2, #1\n"
        "\tMOV R1, %[pin]\n"
        "\tAND R1, #31\n"
        "\tLSL R2, R1\n"
        "\tSTR R2, [R0, #0]\n"
        "\tMOV %[result], R2\n"
        : [result] "=r"(res)
        : [pin] "r"(pin), [gpio] "m"(gpio), [off] "r"(off * 4)
        : "r0", "r1", "r2", "cc");
}

void pinMode(uint32_t *gpio, int pin, int mode)
{
    int rslt;
    //setting the register for the pin
    int fSel = pin/10;
    // setting the value to shift 
    int shift =  (pin%10)*3;
  asm
  (
    /* inline assembler version of setting LED to ouput" */
    // Putting the gpio value into R1 register
        "\tLDR R1, %[gpio]\n"
    // add the fsel value to R1 and store the result into R0
        "\tADD R0, R1, %[fSel]\n"
    // load the value from R0 to R1
        "\tLDR R1, [R0, #0]\n"
    //put the binary value of 7 into R2
        "\tMOV R2, #0b111\n"
    // Shift 111, shift number of times to the left
        "\tLSL R2, %[shift]\n"
    // bitwise clear those bits (not 111)
        "\tBIC R1, R1, R2\n"
    // put the mode into R2
        "\tMOV R2, %[mode]\n"
    // shift 1, shift number of times
        "\tLSL R2, %[shift]\n"
    // or the 2 values found
        "\tORR R1, R2\n"
    //store the value from R1 to R0
        "\tSTR R1, [R0, #0]\n"
    // Store R2 into the result
        "\tMOV %[result], R1\n"
    // output operands
        : [result] "=r" (rslt)
        // : [act] "r" (PIN)
    // Input operands
        : [gpio] "m" (gpio)
        , [fSel] "r" (fSel * 4)
        , [shift] "r" (shift)
        , [mode] "r" (mode)
        :"r0", "r1","r2", "cc"
  );
}

void writeLED(uint32_t *gpio, int led, int value)
{
  int onOff =0;
  int rslt;
  //If the value is HIGH, then turn on the LED
  if(value==HIGH){
    onOff = 7;
  }
  //If not, turn off.
  else{
    onOff = 10;
  }
  asm volatile(
  // Putting the gpio value into R1 register
  "\tLDR R1, %[gpio]\n"
  // add the (register number * 4)onff to R1 and store the result into R0
  "\tADD R0, R1, %[onOff]\n"
  // store 1 into R2
  "\tMOV R2, #1\n"
  //move the pin number into R1
  "\tMOV R1, %[led]\n" 
  //AND the pin value with 31
  "\tAND R1, #31\n"
  // Shift R2, R1 number of times to the left
  "\tLSL R2, R1\n"
  // store the value in R2 inside R0
  "\tSTR R2, [R0, #0]\n"
  // Store R2 into result
  "\tMOV %[result], R2\n"
  // output operands
  : [result] "=r" (rslt)
  // Input operands
  : [led] "r" (led)
  , [gpio] "m" (gpio)
  , [onOff] "r" (onOff*4)
  : "r0", "r1", "r2", "cc");
}

//Function to read whether button is clicked or not
int readButton(uint32_t *gpio, int button)
{
  int rslt = 0;
    int gplev = 13 * 4;
  int btnvalue=LOW;
  asm volatile(

    // load the value if gpio + gplev into R0
        "\tLDR R0, [%[gpio], %[gplev]]\n"
    // move the value of button into R1
        "\tMOV R1, %[button]\n"
    // AND the value from R1 with 31
        "\tAND R1, #31\n"
    // Put 1 into R2
        "\tMOV R2, #1\n"
    // Left shift the value in R2, R1 times 
        "\tLSL R2, R1\n"
    // AND the value of R0 with the value in R2
        "\tAND R0, R2\n"
    // move the value of R0 to the result
        "\tMOV %[result], R0\n"
    // output operands
        :[result] "=r" (rslt)
    // input operands
        :[button] "r" (button)
        ,[gpio] "r" (gpio)
        ,[gplev] "r" (gplev)
        : "r0", "r1", "r2","cc"
  );
    if(rslt != 0){
    btnvalue = HIGH;
  }
  else{
    btnvalue = LOW;
  }
  return btnvalue;
}





void printRound(int num)
{
  fprintf(stderr,"Round %d\n",num); 
}

/* Method to initialise the random secret sequence */
void initSeq() {
      //Dynamically allocated the memory for theSeq variable
    theSeq = (int*)malloc(seqlen * sizeof(int));
  //Generating a new random sequence
  srand(time(NULL));
  //Running a for loop to generate numbers according to seqlen
  for(int i=0;i<seqlen;i++)
  {
    //Generating random number between 1 and seqlen
    int random = rand()%seqlen + 1;
    //Creating the sequence
    theSeq[i] = random;
  }
}

/* Method to display the sequence on the terminal */
void showSeq(int *seq) {

    fprintf(stderr,"The value of sequence was: ");
    //Running the for loop for prinitng the value
    for(int i=0;i<seqlen;i++)
    {
        fprintf(stderr,"%d",seq[i]);
    }
    fprintf(stderr,".\n");
    }

#define NAN1 8
#define NAN2 9

/* counts how many entries in seq2 match entries in seq1 */
/* returns exact and approximate matches, either both encoded in one value, */
/* or as a pointer to a pair of values */
int /* or int* */ *countMatches(int *seq1, int *seq2) {
   //Variables to keep count for exact and approximate
    int exact = 0;
    int approx = 0;
    //Allocating the memory for array 
    int *visited = (int*)malloc(seqlen*sizeof(int));

  for(int i = 0; i<seqlen;i++){
    visited[i] = 0;
  }

    //Running for loop to check sequence
    for (int i = 0; i < seqlen; i++)
    {
        //Checking if the both values are same or not
        if (seq1[i] == seq2[i])
        {
            //If yes then increment exact variable
            exact++;
            //If we have already visited the array then we decrement the approx variable
            if (visited[i])
            {
                //Decrementing the approx
                approx--;
            }  
            //After incrementing we are saying that we have visited this position
            visited[i] = 1;
        }
        //If the values are not same
        else
        {
            //Then we run a loop and check if they are at any other position
            for (int j = 0; j < seqlen; j++)
            {
                //If the value is same and the position has not be visited
                if (seq2[i] == seq1[j] && !visited[j])
                {
                    visited[j] = 1;
                    approx++;
                    break;
                }
            }
        }   
    }
    //Using int pointer to store both the values
    int *val = malloc(2*sizeof(int));
    //Assigning the values
    val[0] = exact;
    val[1] = approx; 
    //Freeing the memory used for the visisted array
    free(visited);
    //Returning the address of val
    return val;   
}

/* show the results from calling countMatches on seq1 and seq1 */
void showMatches(int /* or int* */ *code, /* only for debugging */ int *seq1, int *seq2, /* optional, to control layout */ int lcd_format) {
  //Printing the values
  printf("%d exact match(es)\n",code[0]);
  printf("%d approximate match(es)\n",code[1]);
}

/* parse an integer value as a list of digits, and put them into @seq@ */
/* needed for processing command-line with options -s or -u            */
void readSeq(int *seq, int val) {
  //Running the loop to copy the values
    for(int i=seqlen;i>0;i--)
  {
    //Storing the value of modulus in a variable (Using Reverse Number Logic)
    int value = val%10;
    //Storing the value in sequence variable
    seq[i-1]=value;
    //Dividing it by 10
    val = val/10;
  }
}

/* read a guess sequence fron stdin and store the values in arr */
/* only needed for testing the game logic, without button input */
// Define a function called 'readNum' that takes an integer 'max' as an argument and returns a pointer to an array of integers
int *readNum(int max) {

    // Initialize a variable 'index' to zero
    int index = 0;

    // Dynamically allocate memory for an array of integers 'arr' using the 'malloc' function with size 'seqlen * sizeof(int)'
    int *arr = (int *)malloc(seqlen * sizeof(int));
    
    // Check if memory allocation is successful
    if (!arr)
        // If not, return a null pointer
        return NULL;

    // While 'max' is not zero and 'index' is less than a constant value 'SEQL'
    while (max != 0 && index < SEQL)
    {
        // Assign the remainder of 'max' divided by 10 to the 'arr' array at position 'index'
        arr[index] = max % 10;
        // Increment 'index' by one
        ++index;
        // Divide 'max' by 10 to move on to the next digit
        max /= 10;
    }

    // Return the 'arr' array
    return arr;
}


/* ======================================================= */
/* SECTION: TIMER code                                     */
/* ------------------------------------------------------- */
/* TIMER code */

/* timestamps needed to implement a time-out mechanism */
static uint64_t startT, stopT;

/* ********************************************************** */
/* COMPLETE the code for all of the functions in this SECTION */
/* Implement these as C functions in this file                */
/* ********************************************************** */

/* you may need this function in timer_handler() below  */
/* use the libc fct gettimeofday() to implement it      */
uint64_t timeInMicroseconds(){
    struct timeval tv;
    uint64_t now;
    gettimeofday(&tv, NULL);
    now = (uint64_t)tv.tv_sec * (uint64_t)1000000 + (uint64_t)tv.tv_usec; // in us
    // now  = (uint64_t)tv.tv_sec * (uint64_t)1000 + (uint64_t)(tv.tv_usec / 1000) ; // in ms

    return (uint64_t)now;
}

/* this should be the callback, triggered via an interval timer, */
/* that is set-up through a call to sigaction() in the main fct. */
void timer_handler (int signum) {
    stopT = timeInMicroseconds();
    startT = timeInMicroseconds();
}


/* initialise time-stamps, setup an interval timer, and install the timer_handler callback */
void initITimer(uint64_t timeout){
  /* */
}

/* ======================================================= */
/* SECTION: Aux function                                   */
/* ------------------------------------------------------- */
/* misc aux functions */

int failure (int fatal, const char *message, ...)
{
  va_list argp ;
  char buffer [1024] ;

  if (!fatal) //  && wiringPiReturnCodes)
    return -1 ;

  va_start (argp, message) ;
  vsnprintf (buffer, 1023, message, argp) ;
  va_end (argp) ;

  fprintf (stderr, "%s", buffer) ;
  exit (EXIT_FAILURE) ;

  return 0 ;
}

/*
 * waitForEnter:
 *********************************************************************************
 */

void waitForEnter (void)
{
  printf ("Press ENTER to continue: ") ;
  (void)fgetc (stdin) ;
}

/*
 * delay:
 *  Wait for some number of milliseconds
 *********************************************************************************
 */

void delay (unsigned int howLong)
{
  struct timespec sleeper, dummy ;

  sleeper.tv_sec  = (time_t)(howLong / 1000) ;
  sleeper.tv_nsec = (long)(howLong % 1000) * 1000000 ;

  nanosleep (&sleeper, &dummy) ;
}

void waitForButton(uint32_t *gpio, int button)
{
    for (int j = 0; j < 13; j++)
    {
        if (readButton(gpio, button))
            break;
        delay(DELAY); 
    }
}
/* From wiringPi code; comment by Gordon Henderson
 * delayMicroseconds:
 *  This is somewhat intersting. It seems that on the Pi, a single call
 *  to nanosleep takes some 80 to 130 microseconds anyway, so while
 *  obeying the standards (may take longer), it's not always what we
 *  want!
 *
 *  So what I'll do now is if the delay is less than 100uS we'll do it
 *  in a hard loop, watching a built-in counter on the ARM chip. This is
 *  somewhat sub-optimal in that it uses 100% CPU, something not an issue
 *  in a microcontroller, but under a multi-tasking, multi-user OS, it's
 *  wastefull, however we've no real choice )-:
 *
 *      Plan B: It seems all might not be well with that plan, so changing it
 *      to use gettimeofday () and poll on that instead...
 *********************************************************************************
 */

void delayMicroseconds (unsigned int howLong)
{
  struct timespec sleeper ;
  unsigned int uSecs = howLong % 1000000 ;
  unsigned int wSecs = howLong / 1000000 ;

  /**/ if (howLong ==   0)
    return ;
#if 0
  else if (howLong  < 100)
    delayMicrosecondsHard (howLong) ;
#endif
  else
  {
    sleeper.tv_sec  = wSecs ;
    sleeper.tv_nsec = (long)(uSecs * 1000L) ;
    nanosleep (&sleeper, NULL) ;
  }
}

/* ======================================================= */
/* SECTION: LCD functions                                  */
/* ------------------------------------------------------- */
/* medium-level interface functions (all in C) */

/* from wiringPi:
 * strobe:
 *  Toggle the strobe (Really the "E") pin to the device.
 *  According to the docs, data is latched on the falling edge.
 *********************************************************************************
 */

void strobe (const struct lcdDataStruct *lcd)
{

  // Note timing changes for new version of delayMicroseconds ()
  digitalWrite (gpio, lcd->strbPin, 1) ; delayMicroseconds (50) ;
  digitalWrite (gpio, lcd->strbPin, 0) ; delayMicroseconds (50) ;
}

/*
 * sentDataCmd:
 *  Send an data or command byte to the display.
 *********************************************************************************
 */

void sendDataCmd (const struct lcdDataStruct *lcd, unsigned char data)
{
  register unsigned char myData = data ;
  unsigned char          i, d4 ;

  if (lcd->bits == 4)
  {
    d4 = (myData >> 4) & 0x0F;
    for (i = 0 ; i < 4 ; ++i)
    {
      digitalWrite (gpio, lcd->dataPins [i], (d4 & 1)) ;
      d4 >>= 1 ;
    }
    strobe (lcd) ;

    d4 = myData & 0x0F ;
    for (i = 0 ; i < 4 ; ++i)
    {
      digitalWrite (gpio, lcd->dataPins [i], (d4 & 1)) ;
      d4 >>= 1 ;
    }
  }
  else
  {
    for (i = 0 ; i < 8 ; ++i)
    {
      digitalWrite (gpio, lcd->dataPins [i], (myData & 1)) ;
      myData >>= 1 ;
    }
  }
  strobe (lcd) ;
}

/*
 * lcdPutCommand:
 * Send a command byte to the display
 *********************************************************************************
 */
void lcdPutCommand(const struct lcdDataStruct *lcd, unsigned char command)
{
#ifdef DEBUG
  fprintf(stderr, "lcdPutCommand: digitalWrite(%d,%d) and sendDataCmd(%d,%d)\n", lcd->rsPin, 0, lcd, command);
#endif
  digitalWrite(gpio, lcd->rsPin, 0);
  sendDataCmd(lcd, command);
  delay(2);
}

void lcdPut4Command(const struct lcdDataStruct *lcd, unsigned char command)
{
  register unsigned char myCommand = command;
  register unsigned char i;

  digitalWrite(gpio, lcd->rsPin, 0);

  for (i = 0; i < 4; ++i)
  {
    digitalWrite(gpio, lcd->dataPins[i], (myCommand & 1));
    myCommand >>= 1;
  }
  strobe(lcd);
}

/*
 * lcdHome: lcdClear:
 * Home the cursor or clear the screen.
 *********************************************************************************
 */
void lcdHome(struct lcdDataStruct *lcd)
{
#ifdef DEBUG
  fprintf(stderr, "lcdHome: lcdPutCommand(%d,%d)\n", lcd, LCD_HOME);
#endif
  lcdPutCommand(lcd, LCD_HOME);
  lcd->cx = lcd->cy = 0;
  delay(5);
}

void lcdClear(struct lcdDataStruct *lcd)
{
#ifdef DEBUG
  fprintf(stderr, "lcdClear: lcdPutCommand(%d,%d) and lcdPutCommand(%d,%d)\n", lcd, LCD_CLEAR, lcd, LCD_HOME);
#endif
  lcdPutCommand(lcd, LCD_CLEAR);
  lcdPutCommand(lcd, LCD_HOME);
  lcd->cx = lcd->cy = 0;
  delay(5);
}

/*
 * lcdPosition:
 * Update the position of the cursor on the display.
 * Ignore invalid locations.
 *********************************************************************************
 */
void lcdPosition(struct lcdDataStruct *lcd, int x, int y)
{
  // struct lcdDataStruct *lcd = lcds [fd] ;

  if ((x > lcd->cols) || (x < 0))
    return;
  if ((y > lcd->rows) || (y < 0))
    return;

  lcdPutCommand(lcd, x + (LCD_DGRAM | (y > 0 ? 0x40 : 0x00) /* rowOff [y] */));

  lcd->cx = x;
  lcd->cy = y;
}

/*
 * lcdDisplay: lcdCursor: lcdCursorBlink:
 * Turn the display, cursor, cursor blinking on/off
 *********************************************************************************
 */
void lcdDisplay(struct lcdDataStruct *lcd, int state)
{
  if (state)
    lcdControl |= LCD_DISPLAY_CTRL;
  else
    lcdControl &= ~LCD_DISPLAY_CTRL;

  lcdPutCommand(lcd, LCD_CTRL | lcdControl);
}

void lcdCursor(struct lcdDataStruct *lcd, int state)
{
  if (state)
    lcdControl |= LCD_CURSOR_CTRL;
  else
    lcdControl &= ~LCD_CURSOR_CTRL;

  lcdPutCommand(lcd, LCD_CTRL | lcdControl);
}

void lcdCursorBlink(struct lcdDataStruct *lcd, int state)
{
  if (state)
    lcdControl |= LCD_BLINK_CTRL;
  else
    lcdControl &= ~LCD_BLINK_CTRL;

  lcdPutCommand(lcd, LCD_CTRL | lcdControl);
}

/*
 * lcdPutchar:
 * Send a data byte to be displayed on the display. We implement a very
 * simple terminal here - with line wrapping, but no scrolling. Yet.
 *********************************************************************************
 */
void lcdPutchar(struct lcdDataStruct *lcd, unsigned char data)
{
  digitalWrite(gpio, lcd->rsPin, 1);
  sendDataCmd(lcd, data);

  if (++lcd->cx == lcd->cols)
  {
    lcd->cx = 0;
    if (++lcd->cy == lcd->rows)
      lcd->cy = 0;

    // TODO: inline computation of address and eliminate rowOff
    lcdPutCommand(lcd, lcd->cx + (LCD_DGRAM | (lcd->cy > 0 ? 0x40 : 0x00) /* rowOff [lcd->cy] */));
  }
}

/*
 * lcdPuts:
 * Send a string to be displayed on the display
 *********************************************************************************
 */
void lcdPuts(struct lcdDataStruct *lcd, const char *string)
{
  while (*string)
    lcdPutchar(lcd, *string++);
}

/* ======================================================= */
/* SECTION: aux functions for game logic                   */
/* ------------------------------------------------------- */

/* ********************************************************** */
/* COMPLETE the code for all of the functions in this SECTION */
/* Implement these as C functions in this file                */
/* ********************************************************** */

/* --------------------------------------------------------------------------- */
/* interface on top of the low-level pin I/O code */

/* blink the led on pin @led@, @c@ times */
void blinkN(uint32_t *gpio, int led, int c) { 
  for(int i=0;i<c;i++)
  {
    if ((led & 0xFFFFFFC0) == 0)
      {    
      writeLED(gpio,led,HIGH);
      delay(500);
      writeLED(gpio,led,LOW);
      delay(200);
    } 
  }
}

/* ======================================================= */
/* SECTION: main fct                                       */
/* ------------------------------------------------------- */
int main(int argc, char *argv[])
{
  struct lcdDataStruct *lcd;
  int bits, rows, cols;
  unsigned char func;

  int found = 0, attempts = 0, i, j, code;
  int c, d, buttonPressed, rel, foo;
  int *attSeq;

  int pinLED = LED, pin2LED2 = LED2, pinButton = BUTTON;
  int fSel, shift, pin, clrOff, setOff, off, res;
  int fd;

  int exact, contained;
  char str1[32];
  char str2[32];

  struct timeval t1, t2;
  int t;

  char buf[32];

  // variables for command-line processing
  char str_in[20], str[20] = "some text";
  int verbose = 0, debug = 0, help = 0, opt_m = 0, opt_n = 0, opt_s = 0, unit_test = 0, res_matches = 0;

  // -------------------------------------------------------
  // process command-line arguments
  // see: man 3 getopt for docu and an example of command line parsing
  {
    int opt;
    while ((opt = getopt(argc, argv, "hvdus:")) != -1)
    {
      switch (opt)
      {
      case 'v':
        verbose = 1;
        break;
      case 'h':
        help = 1;
        break;
      case 'd':
        debug = 1;
        break;
      case 'u':
        unit_test = 1;
        break;
      case 's':
        opt_s = atoi(optarg);
        break;
      default: /* '?' */
        fprintf(stderr, "Usage: %s [-h] [-v] [-d] [-u <seq1> <seq2>] [-s <secret seq>]  \n", argv[0]);
        exit(EXIT_FAILURE);
      }
    }
  }

  if (help)
  {
    fprintf(stderr, "MasterMind program, running on a Raspberry Pi, with connected LED, button and LCD display\n");
    fprintf(stderr, "Use the button for input of numbers. The LCD display will show the matches with the secret sequence.\n");
    fprintf(stderr, "For full specification of the program see: https://www.macs.hw.ac.uk/~hwloidl/Courses/F28HS/F28HS_CW2_2022.pdf\n");
    fprintf(stderr, "Usage: %s [-h] [-v] [-d] [-u <seq1> <seq2>] [-s <secret seq>]  \n", argv[0]);
    exit(EXIT_SUCCESS);
  }

  if (unit_test && optind >= argc - 1)
  {
    fprintf(stderr, "Expected 2 arguments after option -u\n");
    exit(EXIT_FAILURE);
  }

  if (verbose && unit_test)
  {
    printf("1st argument = %s\n", argv[optind]);
    printf("2nd argument = %s\n", argv[optind + 1]);
  }

  if (verbose)
  {
    fprintf(stdout, "Settings for running the program\n");
    fprintf(stdout, "Verbose is %s\n", (verbose ? "ON" : "OFF"));
    fprintf(stdout, "Debug is %s\n", (debug ? "ON" : "OFF"));
    fprintf(stdout, "Unittest is %s\n", (unit_test ? "ON" : "OFF"));
    if (opt_s)
      fprintf(stdout, "Secret sequence set to %d\n", opt_s);
  }

  seq1 = (int *)malloc(seqlen * sizeof(int));
  seq2 = (int *)malloc(seqlen * sizeof(int));
  cpy1 = (int *)malloc(seqlen * sizeof(int));
  cpy2 = (int *)malloc(seqlen * sizeof(int));

  // check for -u option, and if so run a unit test on the matching function
  if (unit_test && argc > optind + 1)
  { // more arguments to process; only needed with -u
    strcpy(str_in, argv[optind]);
    opt_m = atoi(str_in);
    strcpy(str_in, argv[optind + 1]);
    opt_n = atoi(str_in);
    // CALL a test-matches function; see testm.c for an example implementation
    readSeq(seq1, opt_m); // turn the integer number into a sequence of numbers
    readSeq(seq2, opt_n); // turn the integer number into a sequence of numbers
    if (verbose)
      fprintf(stdout, "Testing matches function with sequences %d and %d\n", opt_m, opt_n);
    res_matches = countMatches(seq1, seq2);
    showMatches(res_matches, seq1, seq2, 1);
    exit(EXIT_SUCCESS);
  }
  else
  {
    /* nothing to do here; just continue with the rest of the main fct */
  }

  if (opt_s)
  { // if -s option is given, use the sequence as secret sequence
    if (theSeq == NULL)
      theSeq = (int *)malloc(seqlen * sizeof(int));
    readSeq(theSeq, opt_s);
    if (verbose)
    {
      fprintf(stderr, "Running program with secret sequence:\n");
      showSeq(theSeq);
    }
  }

  // -------------------------------------------------------
  // LCD constants, hard-coded: 16x2 display, using a 4-bit connection
  bits = 4;
  cols = 16;
  rows = 2;
  // -------------------------------------------------------

  printf("Raspberry Pi LCD driver, for a %dx%d display (%d-bit wiring) \n", cols, rows, bits);

  if (geteuid() != 0)
    fprintf(stderr, "setup: Must be root. (Did you forget sudo?)\n");

  // init of guess sequence, and copies (for use in countMatches)
  attSeq = (int *)malloc(seqlen * sizeof(int));
  cpy1 = (int *)malloc(seqlen * sizeof(int));
  cpy2 = (int *)malloc(seqlen * sizeof(int));

  // -----------------------------------------------------------------------------
  // constants for RPi3
  gpiobase = 0x3F200000;

  // -----------------------------------------------------------------------------
  // memory mapping
  // Open the master /dev/memory device
  if ((fd = open("/dev/mem", O_RDWR | O_SYNC | O_CLOEXEC)) < 0)
    return failure(FALSE, "setup: Unable to open /dev/mem: %s\n", strerror(errno));

  // GPIO:
  gpio = (uint32_t *)mmap(0, BLOCK_SIZE, PROT_READ | PROT_WRITE, MAP_SHARED, fd, gpiobase);
  if ((int32_t)gpio == -1)
    return failure(FALSE, "setup: mmap (GPIO) failed: %s\n", strerror(errno));

  // -------------------------------------------------------
  // Configuration of LED, BUTTON and LCD pins
  pinMode(gpio, pinLED, OUTPUT);
  pinMode(gpio, pin2LED2, OUTPUT);
  pinMode(gpio, pinButton, INPUT);
  pinMode(gpio, STRB_PIN, OUTPUT);
  pinMode(gpio, RS_PIN, OUTPUT);
  pinMode(gpio, DATA0_PIN, OUTPUT);
  pinMode(gpio, DATA1_PIN, OUTPUT);
  pinMode(gpio, DATA2_PIN, OUTPUT);
  pinMode(gpio, DATA3_PIN, OUTPUT);

  // -------------------------------------------------------
  // INLINED version of lcdInit (can only deal with one LCD attached to the RPi):
  // you can use this code as-is, but you need to implement digitalWrite() and
  // pinMode() which are called from this code
  // Create a new LCD:
  lcd = (struct lcdDataStruct *)malloc(sizeof(struct lcdDataStruct));
  if (lcd == NULL)
    return -1;

  // hard-wired GPIO pins
  lcd->rsPin = RS_PIN;
  lcd->strbPin = STRB_PIN;
  lcd->bits = 4;
  lcd->rows = rows; // # of rows on the display
  lcd->cols = cols; // # of cols on the display
  lcd->cx = 0;      // x-pos of cursor
  lcd->cy = 0;      // y-pos of curosr

  lcd->dataPins[0] = DATA0_PIN;
  lcd->dataPins[1] = DATA1_PIN;
  lcd->dataPins[2] = DATA2_PIN;
  lcd->dataPins[3] = DATA3_PIN;
  // lcd->dataPins [4] = d4 ;
  // lcd->dataPins [5] = d5 ;
  // lcd->dataPins [6] = d6 ;
  // lcd->dataPins [7] = d7 ;

  // lcds [lcdFd] = lcd ;

  digitalWrite(gpio, lcd->rsPin, 0);
  pinMode(gpio, lcd->rsPin, OUTPUT);
  digitalWrite(gpio, lcd->strbPin, 0);
  pinMode(gpio, lcd->strbPin, OUTPUT);

  for (i = 0; i < bits; ++i)
  {
    digitalWrite(gpio, lcd->dataPins[i], 0);
    pinMode(gpio, lcd->dataPins[i], OUTPUT);
  }
  delay(35); // mS

  if (bits == 4)
  {
    func = LCD_FUNC | LCD_FUNC_DL; // Set 8-bit mode 3 times
    lcdPut4Command(lcd, func >> 4);
    delay(35);
    lcdPut4Command(lcd, func >> 4);
    delay(35);
    lcdPut4Command(lcd, func >> 4);
    delay(35);
    func = LCD_FUNC; // 4th set: 4-bit mode
    lcdPut4Command(lcd, func >> 4);
    delay(35);
    lcd->bits = 4;
  }
  else
  {
    failure(TRUE, "setup: only 4-bit connection supported\n");
    func = LCD_FUNC | LCD_FUNC_DL;
    lcdPutCommand(lcd, func);
    delay(35);
    lcdPutCommand(lcd, func);
    delay(35);
    lcdPutCommand(lcd, func);
    delay(35);
  }

  if (lcd->rows > 1)
  {
    func |= LCD_FUNC_N;
    lcdPutCommand(lcd, func);
    delay(35);
  }

  // Rest of the initialisation sequence
  lcdDisplay(lcd, TRUE);
  lcdCursor(lcd, FALSE);
  lcdCursorBlink(lcd, FALSE);
  lcdClear(lcd);

  lcdPutCommand(lcd, LCD_ENTRY | LCD_ENTRY_ID);     // set entry mode to increment address counter after write
  lcdPutCommand(lcd, LCD_CDSHIFT | LCD_CDSHIFT_RL); // set display shift to right-to-left

  // END lcdInit ------ 
  // -----------------------------------------------------------------------------
  // Start of game
  printf("\n");
  fprintf(stderr,"//////////////// GAME STARTING //////////////// \n \n");
  printf("You have 5 attempts, good luck!");
  printf("\n");
  printf("\n");
  
  lcdPuts(lcd, "Welcome to");
  lcdPosition(lcd, 1, 1);
  lcdPuts(lcd, "MasterMind");
  delay(2000);
  lcdClear(lcd);
   
  //If the -s mode is not selected then we are inititalising the secret code
  if (!opt_s)
    {
        initSeq();   
    }

  //If the debug mode is selected
  if (debug)
  {
    showSeq(theSeq);
  }

  printf("Secret Code length: %d.\n",seqlen);
  printf("\n");
  
  // optionally one of these 2 calls:
  lcdPuts(lcd, "Press enter");
  lcdPosition(lcd, 0, 1);
  lcdPuts(lcd, "to start");
  waitForEnter();
  
  // Turn LED off if was ON previous game
  digitalWrite(gpio, pinLED, LOW);
  digitalWrite(gpio, pin2LED2, LOW);

  //Starting the main loop
  while (attempts!=5) 
  {
	  // clear the lcd from previous round
    lcdClear(lcd);
    //Incrementing the attempts
    attempts++;
    //Printing the round number
    printRound(attempts);
    
    // Print the number of attempts on the next line
    lcdPosition(lcd, 0, 1);
    char roundString[32];
    lcdPuts(lcd, "Starting");
    sprintf(roundString, "Round: %d", attempts);
    lcdPuts(lcd, roundString);
    
    delay(2000);

    //Declaring variable to store whether the button is clicked or not 
    int res = 0;
    //Taking the temperory variable
    int current=HIGH;
    
    //Running the for loop till the length of the sequence
    for(int i=0;i<seqlen;i++)
    {
      //Declaring the variable to count how many times button is pressed 
      int count=0;
      
      //Printing the message
      fprintf(stderr,"\nGuess %d\n",(i+1));
      int timer=0;
      lcdClear(lcd);
      
      while(count<seqlen)
      {     
        //Storing whether the button is pressed or not  
        res = readButton(gpio,pinButton);
        
        lcdPuts(lcd, "Press the button");
		lcdPosition(lcd, 0, 1);
		lcdPuts(lcd, "now");

		// Wait for 2 seconds
		delay(1000);

		// Clear the LCD screen
		lcdClear(lcd);
      
      
        //Checking if the button is pressed or not
        if((pinButton & 0xFFFFFFC0) == 0) 
                {
          //If the button is pressed and the value is 1 means button is pressed again
          if(res!=0 && current==HIGH)
          {
            current=LOW;
            //Incrementing the count
            count++;
            //Printing that the button is pressed
            fprintf(stderr,"Button pressed! Value is now %d",count);
            printf("\n");
            lcdPuts(lcd, "Button Pressed");
			delay(300);
			lcdClear(lcd);
			
          }
          //If the button is released then changing the value of current
          else if(res==0 && current==LOW )
          {
            current = HIGH;
          }
        }
                else
        {
                    fprintf(stderr, "only supporting on-board pins\n");
                }
        //Delayig the time
        delay(65);
        //Incrementing the time
        timer++;
        //If the timer is equal to 65
        if(timer==65)
        {
          //Breaking the loop
          break;
        }
      }
      //If the person does'nt press the button then taking the count as 1
      if(count==0)
      {
        count=1;
      }
      //Printing the message
      fprintf(stderr,"\n********** End of guess %d ********** \n",(i+1));
      fprintf(stderr,"\nValue stored: %d \n",count);
      
      //Blinking the red light once as input is accepted
      blinkN(gpio,pin2LED2,1);
      
      //Blinking the green light count times
      blinkN(gpio,pinLED,count);
      
      //Assinging the code to the variable
      attSeq[i] = count;
    }
    //Blinking red LED twice as input of sequence is completed
        blinkN(gpio,pin2LED2,2);
        showSeq(attSeq);
        int * res_match = countMatches(theSeq,attSeq);
        showMatches(res_match, theSeq, attSeq,1);
        
    //Blinking red LED exact times
    blinkN(gpio,pinLED,res_match[0]);
    
    // Compare the sequence with the secret sequence
    int matches = countMatches(attSeq, theSeq);
    int approx = matches % 10;
    int exact = (matches - approx) / 10;

    printf("%d exact \n", exact);
    printf("%d approximate \n", approx);

    delay(500);

    // prints exact on the lcd
    lcdClear(lcd);
    blinkN(gpio, pinLED, exact);
    sprintf(buf, "%d exact", exact);
    lcdPosition(lcd, 1, 0);
    lcdPuts(lcd, buf);
    
    //Blinking red LED once as a separator
        blinkN(gpio,pin2LED2,1);
        
    // prints approximate on the lcd
    blinkN(gpio, pinLED, approx);
    sprintf(buf, "%d approximate", approx);
    lcdPosition(lcd, 1, 1);
    lcdPuts(lcd, buf);

    delay(1000);

    lcdClear(lcd);    
        
    //Blinking red LED approx times
    blinkN(gpio,pinLED,res_match[1]);
    
    //If the eaxct is same as length
        if(res_match[0]==seqlen)
        {
      //Then breaking the loop
            found=1;
            break;
        }
  
    fprintf(stderr,"End of round %d\n",attempts);
    
    
    //Blink Red 3 times as new round starts
    blinkN(gpio,pinLED,3);
  }
  //If the code is found then printing the code found
  if (found) 
  {
    fprintf(stderr,"Code cracked! Codebreaker wins! \n");
    printf("SUCCESS\n");
    lcdPuts(lcd, "SUCCESS");
    
    
  } 
  else 
  {
    fprintf(stdout, "Codekeeper wins! Better luck next time.\n");
    showSeq(theSeq);
    lcdClear(lcd);
    lcdPuts(lcd, "YOU LOSE!");

  }
  return 0;
}
