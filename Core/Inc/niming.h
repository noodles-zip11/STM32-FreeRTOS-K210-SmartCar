//
// Created by miyeon on 2025/11/29.
//

#ifndef CAR_NIMING_H
#define CAR_NIMING_H

#include "main.h"

#define BYTE0(dwTemp) (*(char*)(&dwTemp))
#define BYTE1(dwTemp) (*((char*)(&dwTemp)+1))
#define BYTE2(dwTemp) (*((char*)(&dwTemp)+2))
#define BYTE3(dwTemp) (*((char*)(&dwTemp)+3))



void AND_DT_Send_F1(uint16_t _a, uint16_t _b,uint16_t _c, uint16_t _d);
void AND_DT_Send_F2(int16_t _a, int16_t _b,int16_t _c, int16_t _d);
void AND_DT_Send_F3(int16_t _a, int16_t _b,int32_t _c);



#endif //CAR_NIMING_H