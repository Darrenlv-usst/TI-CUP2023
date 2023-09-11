#ifndef __ENCODER_H
#define __ENCODER_H

#include "main.h"

typedef struct 
{
	int Left;
	int Right;
}Encoder_t;

void Encoder_Init(Encoder_t* encoder);
void EncoderGet(Encoder_t* encoder);
//int EncoderGet_L(void);
//int EncoderGet_R(void);

#endif

