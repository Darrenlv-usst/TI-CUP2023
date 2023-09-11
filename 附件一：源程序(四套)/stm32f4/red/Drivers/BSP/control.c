#include "control.h"
#include "math.h"
#include "FreeRTOS.h"
#include "cmsis_os.h"

//uint16_t angle(int x, int y, Servo_t *servo)
//{
//	//servo->x = atan(x)
//}

float abs(float a)
{
	if(a<0)
		a = -a;
	return a;
}

void Pos2Pwm(float x, float y, Servo_t *servo)
{
	servo->x = atan(x)/PI*1950;
	servo->y = atan(y/sqrt(1+x*x))/PI*1950;
}

//void Reset(Point_t *pos, Servo_t *servo)
//{
//	while(pos->x!=0||pos->y!=0)
//	{
//		if(abs(pos->x)<0.00001)
//			pos->x = 0;
//		else if(pos->x>0)
//			pos->x-=0.005;
//		else if(pos->x<0)
//			pos->x+=0.005;
//		
//		if(abs(pos->y)<0.00001)
//			pos->y = 0;
//		else if(pos->y>0)
//			pos->y-=0.005;
//		else if(pos->y<0)
//			pos->y+=0.005;
//		
//		servo->x = atan(pos->x)/PI*1950;
//		servo->y = atan(pos->y)/PI*1950;
//		//servo->y = atan(pos->y/sqrt(1+pos->x*pos->x))/PI*1950;
//		SERVOX = SERVO_MIDX - servo->x;
//		SERVOY = SERVO_MIDY - servo->y;
//		osDelay(10);
//	}
//}

float Move(float x,float y,int i ,float step)
{
	return x+(y-x)*i/step;
}

//void Move(double targetx, double targety, Point_t *pos, Servo_t *servo)
//{
//	targetx += xoffset;
//	targety += yoffset;
//	double stepx,stepy;
//	stepx = (pos->x - targetx)/25.0;
//	stepy = (pos->y - targety)/25.0;
//	
//	while(pos->x!=targetx||pos->y!=targety)
//	{
//		if(abs(pos->x - targetx)<0.005)
//			pos->x = targetx;
//		else if(pos->x > targetx)
//			pos->x-=stepx;
//		else if(pos->x < targetx)
//			pos->x-=stepx;
//		
//		if(abs(pos->y - targety)<0.005)
//			pos->y = targety;
//		else if(pos->y > targety)
//			pos->y-=stepy;
//		else if(pos->y < targety)
//			pos->y-=stepy;
//		
//		servo->x = atan(pos->x)*650.7/*/PI*1950*/;
//		//servo->y = atan(pos->y/sqrt(1+pos->x*pos->x))/PI*1950;
//		servo->y = atan(pos->y)*655.7;
//		SERVOX = SERVO_MIDX - servo->x;
//		SERVOY = SERVO_MIDY - servo->y;
//		osDelay(5);
//	}
//}
