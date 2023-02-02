#include <Esplora.h>

/*
 * MOTOR.h
 *
 * Created: 2015-08-06 오후 1:59:58
 * Author: JEON HAKYEONG (전하경)
 *
 * History
 *
 * 2019-07-01
 * 
 *  헤더 파일들 정리 작업.
 *  모터 구동 관련 함수들.
 *
 */ 
volatile unsigned int OldDirA =0;
volatile unsigned int OldDirB =0;
volatile unsigned int OldDirC =0;
volatile unsigned int OldDirD =0;

#define madirF (PORTE &= 0xBF)
#define madirB (PORTE |= 0x40)
#define mbdirF (PORTE &= 0x7F)
#define mbdirB (PORTE |= 0x80)
#define mcdirF (PORTH &= 0xBF)
#define mcdirB (PORTH |= 0x40)
#define mddirF (PORTH &= 0x7F)
#define mddirB (PORTH |= 0x80)

int ReadHeading();

void WarningDisplay()
{
  
	OCR3B = 0;
	madirB;
	
	OCR3C = 0;
	mbdirB;
	
	OCR4B = 0;
	mcdirB;

  OCR4C = 0;
  mddirB;
  
	OldDirA=0;
	OldDirB=0;
	OldDirC=0;	
  OldDirD=0;  

  tft.fillScreen(ILI9341_BLACK);
  tft.setTextSize(2);
  drawText(60, 30, "LOW BATTERY!", COLOR_RED);
}

int Sign(int value)
{
	if (value==0) return 0;
	else if (value<0) return -1;
	else return 1;
}

void MOTORA(int ma)
{
	int tmp = abs(ma);
	
	if(Voltage < 95)
	{
		WarningDisplay();
	}
	else
	{
		if(Sign(ma) != OldDirA)
		{
			OCR3B=0;
			madirF;
			delay(20);
		}
		
		tmp=tmp*255/100;
		if(tmp>255)	tmp=255;
		
		if(ma<0)	madirB;
		else		madirF;

		OCR3B = tmp;

		OldDirA=Sign(ma);
	}
}

void MOTORB(int mb)
{
	int tmp = abs(mb);
	
	tmp=tmp*255/100;
	if(tmp>255)	tmp=255;
	if(Voltage < 95)
	{
		WarningDisplay();
	}
	else
	{
		if(Sign(mb) != OldDirB)
		{
			OCR3C=0;
			mbdirF;
			delay(20);
		}
		
		if(mb<0)	mbdirB;
		else		mbdirF;

		OCR3C = tmp;

		OldDirB=Sign(mb);
	}
}

void MOTORC(int mc)
{
	int tmp = abs(mc);

  mc *= -1;
  
	tmp=tmp*255/100;
	if(tmp>255)	tmp=255;
	if(Voltage < 95)
	{
		WarningDisplay();
	}
	else
	{
		if(Sign(mc) != OldDirC)
		{
			OCR4B=0;
			mcdirF;
			delay(20);
		}
		
		if(mc<0)	mcdirB;
		else		mcdirF;

		OCR4B = tmp;

		OldDirC=Sign(mc);
	}
}

void MOTORD(int md)
{
  int tmp = abs(md);
  
  tmp=tmp*255/100;
  if(tmp>255) tmp=255;
  if(Voltage < 95)
  {
    WarningDisplay();
  }
  else
  {
    if(Sign(md) != OldDirD)
    {
      OCR4C=0;
      mddirF;
      delay(20);
    }
    
    if(md<0)  mddirB;
    else    mddirF;

    OCR4C = tmp;

    OldDirD=Sign(md);
  }
}

void motor_stop(void)
{
   MOTORA(0);
   MOTORB(0);
   MOTORC(0);
   MOTORD(0);
}

void move(int left, int right)
{
   MOTORB(right);
   MOTORC(left);
}

int RelatedAngle(int Startpos)
{
    int heading = (ReadHeading()-Startpos);
    
    if(heading < 0)    heading += 360;
//    if(heading >  360) heading -= 360;
    if(heading > 180)  heading -= 360;
    
    return heading;  //-179 ~ 0 ~ 180
}

void GoAhead(int power, double mm)
{
  int TargetEncoder = Encoder2 + (int)(mm * TICKS_PER_REVOLUTION / PI / WHEEL_DIAMETER);

  if(mm < 0)  mm *= -1;
  if(power < 0) power *= -1;
  if(power > 100) power = 100;
  
  move(power,power);
  while(Encoder2 < TargetEncoder) ;
  move(0,0);
  delay(200);
}

void TurnLeft(int power, int angle)
{
  int KP=0.2;
  int dE = 0;
  int StartPos = ReadHeading();  //현재 물리적인 방향값(0~359)

  while(RelatedAngle(StartPos) > -angle+3)      //RelatedAngle(StartPos)함수 20ms 걸림
  {
    dE = (int)((double)(Encoder2 + Encoder3)*KP);
    Encoder2 = 0;
    Encoder3 = 0;
//    move(-(power+KP),power-KP);                   //20ms마다 속도 보정
    move(-power,power);                   //20ms마다 속도 보정
  }
  move(0,0);
  delay(200);
}

void TurnRight(int power, int angle)
{
  int KP=0.2;
  int dE = 0;
  int StartPos = ReadHeading();  //현재 물리적인 방향값(0~359)

  if(angle < 0)  angle *= -1;
  if(power < 0) power *= -1;
  if(power > 100) power = 100;
  
  while(RelatedAngle(StartPos) < angle-3)   //RelatedAngle(StartPos)함수 20ms 걸림
  {
    dE = (int)((double)(Encoder2 + Encoder3)*KP);
    Encoder2 = 0;
    Encoder3 = 0;
//    move(power-KP,-(power+KP));               //20ms마다 속도 보정
    move(power,-power);               //20ms마다 속도 보정
  }
  move(0,0);
  delay(200);
}

void GoBack(int power, double mm)
{
  int TargetEncoder = Encoder2 - (int)(mm * TICKS_PER_REVOLUTION / PI / WHEEL_DIAMETER);

  if(mm < 0)  mm *= -1;
  if(power < 0) power *= -1;
  if(power > 100) power = 100;
  
  move(-power,-power);
  while(Encoder2 > TargetEncoder) ;
  move(0,0);
  delay(200);
}

void ShiftLeft(void)  // 로봇을 좌측으로 약 30mm 이동
{
  GoBack(70,50);
  TurnLeft(70, 28);
  GoAhead(70, 63);
  TurnRight(70, 28);
}

void ShiftRight(void) // 로봇을 우측으로 약 30mm 이동
{
  GoBack(70,50);
  TurnRight(70, 28);
  GoAhead(70, 63);
  TurnLeft(70, 28);
}

void TurnRight90(int power)
{
  int KP=0.2;
  int dE = 0;

  MovingDirection++;
  if(MovingDirection > 3) MovingDirection -= 4;

  while(RelativeHeading() < -5)      //RelatedAngle(StartPos)함수 20ms 걸림
  {
    dE = (int)((double)(Encoder2 + Encoder3)*KP);

    move(power+dE,-(power-dE));                   //20ms마다 속도 보정
//    move(-power,power);                   //20ms마다 속도 보정
    Encoder2 = 0;
    Encoder3 = 0;
  }
  move(0,0);
}

void TurnLeft90(int power)
{
  int KP=0.2;
  int dE = 0;

  MovingDirection--;
  if(MovingDirection < 0) MovingDirection += 4;

  while(RelativeHeading() > 5)      //RelativeHeading(int power)함수 20ms 걸림
  {
    dE = (int)((double)(Encoder2 + Encoder3)*KP);

    move(-(power+dE),power-dE);                   //20ms마다 속도 보정
//    move(-power,power);                   //20ms마다 속도 보정
    Encoder2 = 0;
    Encoder3 = 0;
  }
  move(0,0);
}
