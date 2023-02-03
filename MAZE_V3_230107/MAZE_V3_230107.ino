
/*
 * MAZE.c
 *
 * Created: 2020-02-07
 * Author: JEON HAKYEONG (전하경)
 *
 * History
 *
 */ 

#define Voltage ((int)((float)analogRead(PF6)*0.15056 - 0.6704))

#define TICKS_PER_REVOLUTION  1008  //모터가 한바퀴 돌때 발생하는 엔코더 값
#define WHEEL_DIAMETER        70    //바퀴 지름 (mm)

#define RedLED    0x10
#define GreenLED  0x20
#define BlueLED   0x40

#define GAP 250
#define go  282
#define HeatTemp 40

bool ReturnState = false;

volatile char MovingDirection = 0;  // 0:북쪽(0시 방향) 1:동쪽(3시 방향) 2:남쪽(6시 방향) 3:서쪽(9시 방향)
int Distance[8];

int StackPointer = -1;
int dirCheckStackPointer = -1;
int checkPointStackPointer = -1;

uint8_t mazePath[1000];   // 이동 경로를 저장 할 STACK
uint8_t Maze[15][15];         // 각 구역의 정보를 저장할 배열, 출발은 7,7
uint8_t PosX = 7;
uint8_t PosY = 7;
uint8_t dirCheck[1000]; // 000 막다른길 100 왼쪽 010 앞 001 오른쪽
int checkPoint[1000];  //갈림길 인덱스
bool returned = false;

#include "MAZE.h"
#include <Wire.h>
#include <VL53L0X.h>
#include <Adafruit_MLX90614.h>
#include <Adafruit_BNO055.h>
#include <Servo.h>


// For 2.2" TFT LCD With ILI9341.
#define TFT_DC  6   //PH3 //6   
#define TFT_CS  76  //PH2 //76  
#define TFT_RST 4   //PG5 //4   
#define TFT_LED 38  //PD7 //38  

//For SD Card
#define SD_CS 13  //PB7 //13  

#define TFT_WHITE 0xFFFF
#define TFT_BLACK 0x0000
#define TFT_RED   0xF800

#define BNO055_SAMPLERATE_DELAY_MS (10)

Adafruit_ILI9341 tft = Adafruit_ILI9341(TFT_CS, TFT_DC, TFT_RST);
VL53L0X TOF;
Adafruit_MLX90614 mlx = Adafruit_MLX90614();
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x29);
Servo RescueKit;  // create servo object to control a servo

#define KeyLeft  (~PINL & 0x04)
#define KeyRight (~PINL & 0x02)
#define KeyEnter (~PINL & 0x01)
#define KeyRun   (~PINL & 0x08)
#define KeyStop  (~PINL & 0x10)
//#define SwitchIn (~PINL & 0x20)
#define BumperL  (~PINL & 0x40)
#define BumperR  (~PINL & 0x80)

#define MovingF  1
#define TurnL90  2
#define TurnR90  3
#define Turn180  4
#define MovingLF 5
#define MovingRF 6
 
#include "Motor.h"          //모터 관련 함수들

//************************************************************************************

int ReadDistance(int no)  //거리를 반환 함. (단위 mm)
{
  if(no <0 || no > 7) return -1;
  
  PORTJ = (PORTJ & 0xF0) | no;
  return TOF.readRangeContinuousMillimeters();
}

int ReadAmbientTemp(int no) // 주변 온도를 반환 함.
{
  if(no <0 || no > 1) return -9999;
  
  PORTJ = (PORTJ & 0xF0) | (no + 8);
  return mlx.readAmbientTempC();
}

int ReadObjectTemp(int no)  // 대상물의 온도를 반환 함.
{
  if(no <0 || no > 1) return -9999;
  
  PORTJ = (PORTJ & 0xF0) | (no + 8);
  return mlx.readObjectTempC();
}

int ReadHeading()   // 절대 방향값을 반환 함. (0 ~ 359)
{
    sensors_event_t event;

    PORTJ = (PORTJ & 0xF0) | 10;    
        
    bno.getEvent(&event);    
    
    int h = event.orientation.x;    
    delay(BNO055_SAMPLERATE_DELAY_MS);
    
    return h;
}

int RelativeHeading()   // 이동 방향에 따른 상대 방향값을 반환함. ( -179 ~ 0 ~ 180)
{
    int h = ReadHeading();
    
    h -= MovingDirection*90;
    
    if(h < 0) h += 360;
    if(h > 180) h -= 360;
    
    return h;
}

void PutKit(void)   // 구조 키트 한개를 보급 함
{
  for (int pos = 0; pos <= 180; pos += 2) { // goes from 0 degrees to 180 degrees
    RescueKit.write(pos);                   // tell servo to go to position in variable 'pos'
    delay(15);                              // waits 15ms for the servo to reach the position
  }
  for (int pos = 180; pos >= 0; pos -= 2) { // goes from 180 degrees to 0 degrees
    RescueKit.write(pos);                   // tell servo to go to position in variable 'pos'
    delay(15);                              // waits 15ms for the servo to reach the position
  }
}

void drawText(uint16_t x, uint16_t y, String s, uint16_t color)
{
  //TEXT OVERWRITE https://www.avdweb.nl/arduino/hardware-interfacing/ili9341-tft
  tft.setCursor(x, y);
  tft.setTextColor(color,  ILI9341_BLACK);
  tft.print(s);
}

void InfoForm(void)
{
  for (int i = 0; i < 8; i++)
  {          
    drawText(0, (i+1)*16, "TOF" +String(i+1) + ":    mm", COLOR_WHITE);
  }

  drawText(0, 150, "TEMP 1 Ambient = " , COLOR_WHITE);
  drawText(0, 170, "TEMP 1 Object  = " , COLOR_RED);

  drawText(0, 200, "TEMP 2 Ambient = " , COLOR_WHITE);
  drawText(0, 220, "TEMP 2 Object  = " , COLOR_RED);    

  drawText(160, 16, "Voltage=     " , COLOR_WHITE);    

  drawText(160, 32, "HEADING=   " , COLOR_WHITE);    

  drawText(160, 64, "Light 1=    " , COLOR_WHITE);    
  drawText(160, 80, "Light 2=    " , COLOR_WHITE);   
  
  drawText(160, 112, "ENC 2=       " , COLOR_RED);    
  drawText(160, 128, "ENC 3=       " , COLOR_RED);    
}

void InfoDisplay(void)
{
  String s;
  
  for (int i = 0; i < 8; i++)
  {          
    s = "    " + String(ReadDistance(i));

    drawText(36, (i+1)*16, String(i+1) + ":" + s.substring(s.length()-4,s.length()) + "mm", COLOR_WHITE);
    if (TOF.timeoutOccurred())
    {
      drawText(0, (i+1)*16, "TIME OUT TOF " + String(i+1), COLOR_RED);
    }
  }

  drawText(204, 150, String(ReadAmbientTemp(0)) + "C", COLOR_WHITE);
  drawText(204, 170, String(ReadObjectTemp(0)) + "C", COLOR_RED);

  drawText(204, 200, String(ReadAmbientTemp(1)) + "C", COLOR_WHITE);
  drawText(204, 220, String(ReadObjectTemp(1)) + "C", COLOR_RED);    

  s = "    " + String((int)Voltage/10)+"."+String((int)Voltage%10)+"V";
  drawText(256, 16, s.substring(s.length()-5,s.length()) , COLOR_WHITE);    

  s = "    " + String(ReadHeading());
  drawText(256, 32, s.substring(s.length()-4,s.length()) , COLOR_WHITE);    

  s = "    " + String(analogRead(PF0));
  drawText(256, 64, s.substring(s.length()-4,s.length()) , COLOR_WHITE);    
  s = "    " + String(analogRead(PF1));
  drawText(256, 80, s.substring(s.length()-4,s.length()) , COLOR_WHITE);   
  
  s = "       " + String(Encoder2);
  drawText(232, 112, s.substring(s.length()-7,s.length()) , COLOR_RED);    
  s = "       " + String(Encoder3);
  drawText(232, 128, s.substring(s.length()-7,s.length()) , COLOR_RED);     
 
}

void setup(void) 
{

  init_devices();
  PORTG &= 0xEF;
  
  Wire.begin();
  
  RescueKit.attach(12);  // 12(PB6):SERVO1, 10(PB4):SERVO2, 11(PB5):SERVO3
  RescueKit.write(0);
  
  // initialize the GLCD and show a message
  // asking the user to open the serial line
  tft.begin();
  tft.setCursor(0, 0);
  tft.setTextColor(ILI9341_WHITE);
  tft.setTextSize(2);
  tft.setRotation(3);
  tft.fillScreen(ILI9341_BLACK);
    
  drawText(0, 0, "SEXY Younghwan", COLOR_BLUE);

  Wire.begin();

  for(int i = 0 ;i < 8; i++)
  {
    PORTJ = (PORTJ & 0xF0) | i;
    TOF.setTimeout(500);
    if (!TOF.init())
    {
      drawText(0, 30, "FAILED TOF " + String(i+1), COLOR_RED);
      while (1) {}
   }
  // Start continuous back-to-back mode (take readings as
  // fast as possible).  To use continuous timed mode
  // instead, provide a desired inter-measurement period in
  // ms (e.g. sensor.startContinuous(100)).

    TOF.startContinuous();
  }
  
  PORTJ = (PORTJ & 0xF0) | 10;
  if(!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    drawText(0, 30, "FAILED BNO055", COLOR_RED);
    while(1);
  }
   
  bno.setExtCrystalUse(true);

  // 구역 정보 클리어
  for(int i =0; i<7; i++)
    for(int j = 0; j < 7; j++)
      Maze[i][j]=0;
}

void Push(uint8_t data)
{
//  StackPointer++;
  mazePath[++StackPointer] = data;
}

void DirCheckPush(uint8_t data)
{
//  StackPointer++;
  dirCheck[++dirCheckStackPointer] = data;
}

void CheckPointPush(int data)
{
//  StackPointer++;
  checkPoint[++checkPointStackPointer] = data;
}

uint8_t Pop()
{
  uint8_t data = mazePath[StackPointer];
  StackPointer--;
  return data;
}
uint8_t DirCheckPop()
{
  uint8_t data = dirCheck[dirCheckStackPointer];
  dirCheckStackPointer--;
  return data;
}
int CheckPointPop()
{
  int data = checkPoint[checkPointStackPointer];
  checkPointStackPointer--;
  return data;
}

void returning(int data)
{
  switch(data)
  {
    case +1:
      GoBack(70,go);
      break;
    case -1:
      GoAhead(70,go);
      break;
    case +2:
      TurnRight(70,84);
      break;
    case -2:
      TurnLeft(70,84);
      break;
  }
}

void PushToWall(void)
{
  move(50,50);
  while(!BumperL && !BumperR) ;
  move(0,0);
}

void goOneBlock(int power)
{
  int event = 0;
  int frontLargerTof = ReadDistance(0);
  int behindLargerTof = ReadDistance(5);

  if(ReadDistance(0) <  ReadDistance(1))  frontLargerTof = ReadDistance(1);
  else                                    frontLargerTof = ReadDistance(0);

  if(ReadDistance(4) <  ReadDistance(5))  behindLargerTof = ReadDistance(5);
  else                                    behindLargerTof = ReadDistance(4);
  
  double hp;
  double Kp = 5.0;

  Kp = Kp * power / 100;
  
  int frontStartDistance = frontLargerTof;
  int behindStartDistance = behindLargerTof;

  if(ReadDistance(0) < ReadDistance(4) + 30)
  {
    while(abs(frontStartDistance - frontLargerTof) <= 295 && frontLargerTof >= 80 && !event)
    {
      hp = RelativeHeading();
  
      move(power - hp * Kp, power + hp * Kp);
  
      if(BumperL || ReadDistance(7) < 22) ShiftRight();
      if(BumperR || ReadDistance(2) < 22) ShiftLeft();

      if(ReadDistance(0) <  ReadDistance(1))  frontLargerTof = ReadDistance(1);
      else                                    frontLargerTof = ReadDistance(0);
    
      if(ReadDistance(4) <  ReadDistance(5))  behindLargerTof = ReadDistance(5);
      else                                    behindLargerTof = ReadDistance(4);
      /*
      if(ReadObjectTemp(0)> HeatTemp && !Maze[PosX][PosY] && (ReadDistance(2) <= 100 && ReadDistance(3) <= 100))
      {
        move(0,0);
        GoBack(50,35);
        TurnRight90(70);
        PushToWall();
        BlinkLED(5, RedLED);
        PutKit();
        GoBack(50,75);
        TurnLeft90(70);
        Maze[PosX][PosY] = 1;
      }
      if(ReadObjectTemp(1) > HeatTemp && !Maze[PosX][PosY] && (ReadDistance(6) <= 100 && ReadDistance(7) <= 100))
      {
        move(0,0);
        GoBack(50,35);
        TurnLeft90(70);
        PushToWall();
        BlinkLED(5, RedLED);
        PutKit();
        GoBack(50,75);
        TurnRight90(70);
        Maze[PosX][PosY] = 1;
      }
      if((analogRead(PF0) > 800 || analogRead(PF1) > 800) && !ReturnState)   event = 3;  //목적지      */
      Maze[PosX][PosY] = 1;
      if(ReadDistance(0) < 80 || ReadDistance(1) < 80)     event = 1;  //앞쪽 벽에 너무 붙지않게 떨어져서 멈춤.
    }
  }
  else
  {
    while(abs(behindStartDistance - behindLargerTof) <= 295 && frontLargerTof >= 80 && !event)
    {
      hp = RelativeHeading();
  
      move(power - hp * Kp, power + hp * Kp);
  
      if(BumperL || ReadDistance(7) < 22) ShiftRight();
      if(BumperR || ReadDistance(2) < 22) ShiftLeft();
      
      if(ReadDistance(0) <  ReadDistance(1))  frontLargerTof = ReadDistance(1);
      else                                    frontLargerTof = ReadDistance(0);
    
      if(ReadDistance(4) <  ReadDistance(5))  behindLargerTof = ReadDistance(5);
      else                                    behindLargerTof = ReadDistance(4);
  /*
      if(ReadObjectTemp(0)> HeatTemp && !Maze[PosX][PosY] && (ReadDistance(2) <= 100 && ReadDistance(3) <= 100))
      {
        move(0,0);
        GoBack(50,35);
        TurnRight90(70);
        PushToWall();
        BlinkLED(5, RedLED);
        PutKit();
        GoBack(50,75);
        TurnLeft90(70);
        Maze[PosX][PosY] = 1;
      }
      if(ReadObjectTemp(1) > HeatTemp && !Maze[PosX][PosY] && (ReadDistance(6) <= 100 && ReadDistance(7) <= 100))
      {
        move(0,0);
        GoBack(50,35);
        TurnLeft90(70);
        PushToWall();
        BlinkLED(5, RedLED);
        PutKit();
        GoBack(50,75);
        TurnRight90(70);
        Maze[PosX][PosY] = 1;
      }
  //    if(ReadDistance(6) >= 200 && ReadDistance(7)>= 200) event = 1;  //왼쪽이 뚫렸음.
      if((analogRead(PF0) > 800 || analogRead(PF1) > 800) && !ReturnState)   event = 3;  //목적지    */  
      Maze[PosX][PosY] = 1;
      if(ReadDistance(0) < 80 || ReadDistance(1) < 80)     event = 1;  //앞쪽 벽에 너무 붙지않게 떨어져서 멈춤.
    }
  }

 // motor_stop();
   
  
  switch(MovingDirection)
  {
    case 0: PosY++;   //0시 방향
            break;
    case 1: PosX++;   //3시 방향
            break;
    case 2: PosY--;   //6시 방향
            break;
    case 3: PosX--;   //9시 방향
            break;  
  }
}
void goBackOneBlock(int power)
{
  int event = 0;
  int frontLargerTof = ReadDistance(0);
  int behindLargerTof = ReadDistance(5);

  if(ReadDistance(0) <  ReadDistance(1))  frontLargerTof = ReadDistance(1);
  else                                    frontLargerTof = ReadDistance(0);

  if(ReadDistance(4) <  ReadDistance(5))  behindLargerTof = ReadDistance(5);
  else                                    behindLargerTof = ReadDistance(4);
  
  double hp;
  double Kp = 5.0;

  Kp = Kp * power / 100;
  
  int frontStartDistance = frontLargerTof;
  int behindStartDistance = behindLargerTof;

  if(ReadDistance(4) < ReadDistance(0) + 30)
  {
    while(abs(behindStartDistance - behindLargerTof) <= 295 && behindLargerTof >= 80 && !event)
    {
      hp = RelativeHeading();
  
      move((-1 * power) + hp * Kp, (-1 * power) - hp * Kp);
  
      //if(BumperL || ReadDistance(7) < 22) ShiftRight();
      //if(BumperR || ReadDistance(2) < 22) ShiftLeft();

      if(ReadDistance(0) <  ReadDistance(1))  frontLargerTof = ReadDistance(1);
      else                                    frontLargerTof = ReadDistance(0);
    
      if(ReadDistance(4) <  ReadDistance(5))  behindLargerTof = ReadDistance(5);
      else                                    behindLargerTof = ReadDistance(4);
      /*
      if(ReadObjectTemp(0)> HeatTemp && !Maze[PosX][PosY] && (ReadDistance(2) <= 100 && ReadDistance(3) <= 100))
      {
        move(0,0);
        GoBack(50,35);
        TurnRight90(70);
        PushToWall();
        BlinkLED(5, RedLED);
        PutKit();
        GoBack(50,75);
        TurnLeft90(70);
        Maze[PosX][PosY] = 1;
      }
      if(ReadObjectTemp(1) > HeatTemp && !Maze[PosX][PosY] && (ReadDistance(6) <= 100 && ReadDistance(7) <= 100))
      {
        move(0,0);
        GoBack(50,35);
        TurnLeft90(70);
        PushToWall();
        BlinkLED(5, RedLED);
        PutKit();
        GoBack(50,75);
        TurnRight90(70);
        Maze[PosX][PosY] = 1;
      }*/
      //if((analogRead(PF0) > 800 || analogRead(PF1) > 800) && !ReturnState)   event = 3;  //목적지      
      if(ReadDistance(0) < 80 || ReadDistance(1) < 80)     event = 1;  //앞쪽 벽에 너무 붙지않게 떨어져서 멈춤.
    }
  }
  else
  {
    while(abs(frontStartDistance - frontLargerTof) <= 295 && behindLargerTof >= 80 && !event)
    {
      hp = RelativeHeading();
  
      move((-1 * power) + hp * Kp, (-1 * power) - hp * Kp);
  
      if(BumperL || ReadDistance(7) < 22) ShiftRight();
      if(BumperR || ReadDistance(2) < 22) ShiftLeft();
      
      if(ReadDistance(0) <  ReadDistance(1))  frontLargerTof = ReadDistance(1);
      else                                    frontLargerTof = ReadDistance(0);
    
      if(ReadDistance(4) <  ReadDistance(5))  behindLargerTof = ReadDistance(5);
      else                                    behindLargerTof = ReadDistance(4);
  /*
      if(ReadObjectTemp(0)> HeatTemp && !Maze[PosX][PosY] && (ReadDistance(2) <= 100 && ReadDistance(3) <= 100))
      {
        move(0,0);
        GoBack(50,35);
        TurnRight90(70);
        PushToWall();
        BlinkLED(5, RedLED);
        PutKit();
        GoBack(50,75);
        TurnLeft90(70);
        Maze[PosX][PosY] = 1;
      }
      if(ReadObjectTemp(1) > HeatTemp && !Maze[PosX][PosY] && (ReadDistance(6) <= 100 && ReadDistance(7) <= 100))
      {
        move(0,0);
        GoBack(50,35);
        TurnLeft90(70);
        PushToWall();
        BlinkLED(5, RedLED);
        PutKit();
        GoBack(50,75);
        TurnRight90(70);
        Maze[PosX][PosY] = 1;
      }*/
  //    if(ReadDistance(6) >= 200 && ReadDistance(7)>= 200) event = 1;  //왼쪽이 뚫렸음.
    //  if((analogRead(PF0) > 800 || analogRead(PF1) > 800) && !ReturnState)   event = 3;  //목적지      
      if(ReadDistance(4) < 80 || ReadDistance(5) < 80)     event = 1;  //뒤쪽 벽에 너무 붙지않게 떨어져서 멈춤.
    }
  }

 // motor_stop();
   
  
  switch(MovingDirection)
  {
    case 0: PosY--;   //보는 방향 0시 방향
            break;
    case 1: PosX--;   //3시 방향
            break;
    case 2: PosY++;   //6시 방향
            break;
    case 3: PosX++;   //9시 방향
            break;  
  }
}

void BlinkLED(uint8_t Secs, uint8_t LEDColor)
{
  for(int count=0; count < Secs*5; count++)
  {
      PORTJ ^= LEDColor;    //Toggle LED
      delay(200);
  }
  PORTJ |= LEDColor;    //Toggle LED    
}


void loop(void) 
{
  tft.setTextSize(2);
  tft.setTextColor(ILI9341_WHITE, ILI9341_BLACK);

  InfoForm();

  //------------------ 샘플 프로그램 ---------------------------------

  while(1)
  {
    InfoDisplay();
    
   if(KeyLeft)
   {  
    while(analogRead(PF0) < 800 && analogRead(PF1) < 800) //빛센서값이 둘다 800 보다 작을때 (목적지가 아님)
    {
      
      for(int i=0; i<=7; i++) //벽과 로봇 사이 간격은 75mm가 이상적인 값임.
      Distance[i] = ReadDistance(i);
   
      
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
      
      if(Distance[7] >= GAP || Distance[6] >= GAP) //No walls on Left side
      {
        TurnLeft90(80);
        Push(TurnL90);
        //younghwan++;
  
        goOneBlock(70); //한 블럭 전진
        Push(MovingF);
       // younghwan++;
      }  
      else if(Distance[1] >= GAP || Distance[0] >= GAP) //No walls on front side 
      {            
        goOneBlock(70); //한 블럭 전진
        Push(MovingF);
       // younghwan++; 
      }  
      else if(Distance[2] >= GAP || Distance[3] >= GAP) // No Walls on Right Side
      {              
        TurnRight90(80);   
        Push(TurnR90);
       // younghwan++;
    
  
        goOneBlock(70); //한 블럭 전진
        Push(MovingF);
       // younghwan++;
 
      }  
      else // Walls on 3 Sides except Behind  //pop flag
      {              
        TurnRight(80,175);
        MovingDirection += 2;
        
        if(MovingDirection > 3) MovingDirection -=4;
        Push(Turn180);
     
      }   
    }
  
    
    GoAhead(70, 150);
    TurnRight(80,175);
    MovingDirection += 2;
    if(MovingDirection > 3) MovingDirection -=4;
    
    ReturnState = true;
    
    while(StackPointer >= 0)
    {
      switch(Pop())
      {
        case MovingF: goOneBlock(70);
                      break;
        case TurnL90: TurnRight90(80);
                      break;
        case TurnR90: TurnLeft90(80);
                      break; 
        case Turn180: TurnRight(80,175);
                      MovingDirection += 2;
                      if(MovingDirection > 3) MovingDirection -=4;
                      break;     
      }
    }
    move(0,0);
    while(1);
  }

   if(KeyRight)
   {  
    while(analogRead(PF0) < 800 && analogRead(PF1) < 800) //빛센서값이 둘다 800 보다 작을때 (목적지가 아님)
    {
      
      for(int i=0; i<=7; i++) //벽과 로봇 사이 간격은 75mm가 이상적인 값임.
      Distance[i] = ReadDistance(i);
      
      
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
      
      if(Distance[3] >= GAP || Distance[2] >= GAP) //No walls on Right side
      {
        TurnRight90(80);
        Push(TurnR90);
        //younghwan++;
  
        goOneBlock(70); //한 블럭 전진
        Push(MovingF);
       // younghwan++;
      }  
      else if(Distance[1] >= GAP || Distance[0] >= GAP) //No walls on front side 
      {            
        goOneBlock(70); //한 블럭 전진
        Push(MovingF);
      }  
      else if(Distance[6] >= GAP || Distance[7] >= GAP) // No Walls on Left Side
      {              
        TurnLeft90(80);   
        Push(TurnL90);
    
  
        goOneBlock(70); //한 블럭 전진
        Push(MovingF);
 
      }  
      else // Walls on 3 Sides except Behind  //pop flag
      {              
        TurnRight(85,175);
        MovingDirection += 2;
        
        if(MovingDirection > 3) MovingDirection -=4;
        Push(Turn180);
     
      }   
    }
  
    
    GoAhead(70, 150);
    TurnRight(85,175);
    MovingDirection += 2;
    if(MovingDirection > 3) MovingDirection -=4;
    
    ReturnState = true;
    
    while(StackPointer >= 0)
    {
      switch(Pop())
      {
        case MovingF: goOneBlock(70);
                      break;
        case TurnL90: TurnRight90(85);
                      break;
        case TurnR90: TurnLeft90(85);
                      break; 
        case Turn180: TurnRight(85,175);
                      MovingDirection += 2;
                      if(MovingDirection > 3) MovingDirection -=4;
                      break;     
      }
    }
    move(0,0);
    while(1);
  }    
    if(KeyRun)////////////////////////////////////////////////////////////////
    {
      int availableDir = 0;
      Push(0);
      CheckPointPush(StackPointer);
            
      while(returned == false || checkPointStackPointer > 0 || checkPoint[checkPointStackPointer] != 0) //종료 조건, 이후 수정해야함
      {
        for(int i=0; i<=7; i++) //벽과 로봇 사이 간격은 75mm가 이상적인 값임.
        Distance[i] = ReadDistance(i);

        if(returned == false)
        {
          if(Distance[6] >= GAP || Distance[7] >= GAP) // No Walls on Left Side
          {              
            availableDir += 1;
          }
          if(Distance[1] >= GAP || Distance[0] >= GAP) //No walls on front side 
          {            
            availableDir += 2;
          }        
          if(Distance[3] >= GAP || Distance[2] >= GAP) //No walls on Right side
          {
            availableDir += 4;
          }
          DirCheckPush(availableDir);
        }
        else
        {
          availableDir = dirCheck[dirCheckStackPointer];
          returned = false;
        }
        if(availableDir != 0 && availableDir != 1 && availableDir != 2 && availableDir != 4)
        {
          CheckPointPush(StackPointer);
        }
        
        if(dirCheck[dirCheckStackPointer] & 1)
        {
          dirCheck[dirCheckStackPointer] -= 1;
          TurnLeft90(80);
          goOneBlock(70); //한 블럭 전진
          Push(MovingLF);
        }
        else if(dirCheck[dirCheckStackPointer] & 2)
        {
          dirCheck[dirCheckStackPointer] -= 2;
          goOneBlock(70); //한 블럭 전진
          Push(MovingF);
        }
        else if(dirCheck[dirCheckStackPointer] & 4)
        {
          dirCheck[dirCheckStackPointer] -= 4;
          TurnRight90(80);   
          goOneBlock(70); //한 블럭 전진
          Push(MovingRF);
        }
        else if(dirCheck[dirCheckStackPointer] == 0) // Walls on 3 Sides except Behind
        {
          move(0,0);
          delay(10);
          int prevCheckPoint = CheckPointPop();           
          //바로 이전 갈림길까지 역행
          while(StackPointer > prevCheckPoint)// 갈림길 인덱스 정보 버퍼 추가예정,,,
          {
            DirCheckPop();
            switch(Pop()) //수정!
            {
              case MovingF: goBackOneBlock(70);
                            break;
              case MovingLF: goBackOneBlock(70); TurnRight90(80); 
                            break;
              case MovingRF: goBackOneBlock(70); TurnLeft90(80); 
                            break; 
            }
            
          }
          returned = true;
        }
        availableDir = 0;
      }
      move(0,0);
      while(1);
    }
  }
  
  move(0,0);
}
