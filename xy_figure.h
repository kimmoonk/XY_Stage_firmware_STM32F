#ifndef   __XY_FIGURE_H__
#define   __XY_FIGURE_H__

#include "stm32f4xx.h"
#include "_init.h"

extern void Straight(void);
extern void Diagonal(void);
extern void Arc(void);
extern uint8_t Direction(void);
extern void Data_Check(void);
extern void XY_Data(void);
extern void laser_control(void);

//extern void Bezier(void);

#endif




