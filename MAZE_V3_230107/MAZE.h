/*
 * MAZE.h
 *
 * Created: 2020-05-12
 * Author: JEON HAKYEONG (전하경)
 *
 * History
 *
 * 
 */ 

#include <ARDUINO.h>
#include <SPI.h>
#include <SD.h>
//#include "Adafruit_GFX.h"
#include "Adafruit_ILI9341.h"


//
//Sample Numbers for Average of Analog Value. 
//
#define NumberOfSamples 4   //가능한 변경하지 마세요.. 이값을 변경하고자 하면 Header File쪽도 수정해야 함. 4개

//평균을 구하기 위한 배열
//volatile unsigned int ADC_Raw[NumberOfSamples+1][18];


//모터 엔코더
#define ENCODER1_A 62 //PK0  //62 
#define ENCODER1_B 63 //PK1  //63 
#define ENCODER2_A 64 //PK2  //64
#define ENCODER2_B 65 //PK3  //65
#define ENCODER3_A 66 //PK4  //66 
#define ENCODER3_B 67 //PK5  //67
#define ENCODER4_A 68 //PK6  //68 
#define ENCODER4_B 69 //PK7  //69

volatile long Encoder1 = 0; //모터1의 엔코더의 증감값
volatile int Encoder1A;   //모터 엔코더 A 신호 상태값 
volatile int Encoder1B;   //B 상태값

volatile long Encoder2 = 0; //모터2의 엔코더의 증감값
volatile int Encoder2A;   //모터 엔코더 A 신호 상태값 
volatile int Encoder2B;   //B 상태값

volatile long Encoder3 = 0; //모터3의 엔코더의 증감값
volatile int Encoder3A;   //모터 엔코더 A 신호 상태값 
volatile int Encoder3B;   //B 상태값


volatile long Encoder4 = 0; //모터4의 엔코더의 증감값
volatile int Encoder4A;   //모터 엔코더 A 신호 상태값 
volatile int Encoder4B;   //B 상태값

/* RGB 16-bit color table definition (RG565) */
#define COLOR_BLACK          0x0000      /*   0,   0,   0 */
#define COLOR_WHITE          0xFFFF      /* 255, 255, 255 */
#define COLOR_BLUE           0x001F      /*   0,   0, 255 */
#define COLOR_GREEN          0x07E0      /*   0, 255,   0 */
#define COLOR_RED            0xF800      /* 255,   0,   0 */
#define COLOR_NAVY           0x000F      /*   0,   0, 128 */
#define COLOR_DARKBLUE       0x0011      /*   0,   0, 139 */
#define COLOR_DARKGREEN      0x03E0      /*   0, 128,   0 */
#define COLOR_DARKCYAN       0x03EF      /*   0, 128, 128 */
#define COLOR_CYAN           0x07FF      /*   0, 255, 255 */
#define COLOR_TURQUOISE      0x471A      /*  64, 224, 208 */
#define COLOR_INDIGO         0x4810      /*  75,   0, 130 */
#define COLOR_DARKRED        0x8000      /* 128,   0,   0 */
#define COLOR_OLIVE          0x7BE0      /* 128, 128,   0 */
#define COLOR_GRAY           0x8410      /* 128, 128, 128 */
#define COLOR_GREY           0x8410      /* 128, 128, 128 */
#define COLOR_SKYBLUE        0x867D      /* 135, 206, 235 */
#define COLOR_BLUEVIOLET     0x895C      /* 138,  43, 226 */
#define COLOR_LIGHTGREEN     0x9772      /* 144, 238, 144 */
#define COLOR_DARKVIOLET     0x901A      /* 148,   0, 211 */
#define COLOR_YELLOWGREEN    0x9E66      /* 154, 205,  50 */
#define COLOR_BROWN          0xA145      /* 165,  42,  42 */
#define COLOR_DARKGRAY       0x7BEF      /* 128, 128, 128 */
#define COLOR_DARKGREY       0x7BEF      /* 128, 128, 128 */
#define COLOR_SIENNA         0xA285      /* 160,  82,  45 */
#define COLOR_LIGHTBLUE      0xAEDC      /* 172, 216, 230 */
#define COLOR_GREENYELLOW    0xAFE5      /* 173, 255,  47 */
#define COLOR_SILVER         0xC618      /* 192, 192, 192 */
#define COLOR_LIGHTGRAY      0xC618      /* 192, 192, 192 */
#define COLOR_LIGHTGREY      0xC618      /* 192, 192, 192 */
#define COLOR_LIGHTCYAN      0xE7FF      /* 224, 255, 255 */
#define COLOR_VIOLET         0xEC1D      /* 238, 130, 238 */
#define COLOR_AZUR           0xF7FF      /* 240, 255, 255 */
#define COLOR_BEIGE          0xF7BB      /* 245, 245, 220 */
#define COLOR_MAGENTA        0xF81F      /* 255,   0, 255 */
#define COLOR_TOMATO         0xFB08      /* 255,  99,  71 */
#define COLOR_GOLD           0xFEA0      /* 255, 215,   0 */
#define COLOR_ORANGE         0xFD20      /* 255, 165,   0 */
#define COLOR_SNOW           0xFFDF      /* 255, 250, 250 */
#define COLOR_YELLOW         0xFFE0      /* 255, 255,   0 */

void drawText(uint16_t x, uint16_t y, String s, uint16_t color);
int RelativeHeading();

ISR(PCINT2_vect) 
{ 
    // 엔코더 계산 
    int Encoder1ATemp = digitalRead(ENCODER1_A); 
    int Encoder1BTemp = digitalRead(ENCODER1_B); 

    if (Encoder1A != Encoder1ATemp) {    
        Encoder1 += (Encoder1ATemp == Encoder1BTemp)?1:-1;    
        Encoder1A = Encoder1ATemp; 
    } 
    if (Encoder1B != Encoder1BTemp) { 
        Encoder1 += (Encoder1ATemp == Encoder1BTemp)?-1:1; 
        Encoder1B = Encoder1BTemp; 
    } 
    
    int Encoder2ATemp = digitalRead(ENCODER2_A); 
    int Encoder2BTemp = digitalRead(ENCODER2_B); 

    if (Encoder2A != Encoder2ATemp) {    
        Encoder2 += (Encoder2ATemp == Encoder2BTemp)?1:-1;    
        Encoder2A = Encoder2ATemp; 
    } 
    if (Encoder2B != Encoder2BTemp) { 
        Encoder2 += (Encoder2ATemp == Encoder2BTemp)?-1:1; 
        Encoder2B = Encoder2BTemp; 
    } 

    int Encoder3ATemp = digitalRead(ENCODER3_A); 
    int Encoder3BTemp = digitalRead(ENCODER3_B); 

    if (Encoder3A != Encoder3ATemp) {    
        Encoder3 += (Encoder3ATemp == Encoder3BTemp)?-1:1;    
        Encoder3A = Encoder3ATemp; 
    } 
    if (Encoder3B != Encoder3BTemp) { 
        Encoder3 += (Encoder3ATemp == Encoder3BTemp)?1:-1; 
        Encoder3B = Encoder3BTemp; 
    } 

    int Encoder4ATemp = digitalRead(ENCODER4_A); 
    int Encoder4BTemp = digitalRead(ENCODER4_B); 

    if (Encoder4A != Encoder4ATemp) {    
        Encoder4 += (Encoder4ATemp == Encoder4BTemp)?1:-1;    
        Encoder4A = Encoder4ATemp; 
    } 
    if (Encoder4B != Encoder4BTemp) { 
        Encoder4 += (Encoder4ATemp == Encoder4BTemp)?-1:1; 
        Encoder4B = Encoder4BTemp; 
    } 
}

void port_init(void)
{
  DDRA=0xFF;
  PORTA=0xFF;

  DDRB=0xF6;
  PORTB=0xFF;

  DDRC=0xFF;
  PORTC=0xFF;

  DDRD=0xFB;
  PORTD=0xFF;

  DDRE=0xFE;
  PORTE=0x0F;

  DDRF=0x80;
  PORTF=0x80;

  DDRG=0x3F;
  PORTG=0x3F; 

  DDRH=0xFE;
  PORTH=0x0F; 

  DDRJ=0xFF;
  PORTJ=0xF0;

  DDRK=0x00;
  PORTK=0xFF;

  DDRL=0x00;
  PORTL=0xFF;
  
}

void PWM_Init(void)
{
  TCCR3A = ((1 << COM3B1) | (1 << COM3B0) | (1 << COM3C1)  | (1 << COM3C0)  | (1 << WGM30));  // 8 bit Phase correct PWM Mode , Low Active
  TCCR3B = ((1 << CS31));                                                                     // Clock/8    3,921 HZ PWM GENERATION
    
  TCCR4A = ((1 << COM4B1) | (1 << COM4B0) | (1 << COM4C1)  | (1 << COM4C0)  | (1 << WGM40));  // 8 bit Phase correct PWM Mode , Low Active
  TCCR4B = ((1 << CS41));                                                                     // Clock/8    3,921 HZ PWM GENERATION    
}

void init_devices(void)
{
	MCUCR = 0x00;
	EIMSK = 0x00;
  
  PWM_Init();
  port_init();

  XMCRA = 0x8F; //외부메모리 사용 , 0x2200 ~ 0xFFFF
  XMCRB = 0x00; //Not use BUS Keeper, no wate states
  
  noInterrupts(); 
  PCICR |= (1 << PCIE2); 
  PCMSK2 |= (1 << PCINT16 | 1 << PCINT17 | 1 << PCINT18 | 1 << PCINT19 
                          | 1 << PCINT20 | 1 << PCINT21 | 1 << PCINT22 | 1 << PCINT23);  //모터 엔코더 
  interrupts();
}
