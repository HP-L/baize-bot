/*
 * math3D.h
 *
 *  Created on: Oct 1, 2024
 *      Author: lake
 */

#ifndef INC_MATH3D_H_
#define INC_MATH3D_H_

#include "st7789.h"
#include "math.h"


#define X 0
#define Y 1
#define Z 2//������
#define White 0xFFFF
#define Silver 0xC618
#define Yellow 0xFFE0
#define Gold 0xFEA0
#define Green 0x07E0
#define Pink 0xFE19
#define Orange 0xFD20
#define Red 0xF800
#define Blue 0x001F
#define Brown 0xA145
#define Black 0x0000
#define BackGround 0x0000//��ɫ�����ð�ɫ�ͺ�ɫ

extern int16_t Ox; //ԭ�������
extern int16_t Oy; //ԭ��������
extern float Rect[];//����������

void Cart_Point(int16_t x,int16_t y,uint16_t color);//�ѿ�������ϵ����
void Cart_Coor(uint16_t color,uint8_t per);//���Ƶѿ�������ϵ
void Cart_Line(int16_t x,int16_t y,int16_t x1,int16_t y1,uint16_t cor,uint8_t per);//����
void Cart_DrawRect(int16_t x,int16_t y,int16_t x1,int16_t y1,uint16_t cor,uint8_t per);//�����ľ���
void Cart_DrawRound(int16_t ox,int16_t oy,uint8_t r,uint16_t co);//������Բ
void Cart_DrawFRound(uint8_t ox,uint8_t oy,uint8_t r,uint16_t cor);//��ʵ��Բ
void Se_Rotation(uint16_t Angle,float AD2[],uint8_t len);//��ά��ת���󣬶�ָ�����������������ת�任
void Th_Rotation(uint16_t Angle,float AD3[],uint8_t Rot,uint8_t len);//��ά��ת���󣬶�ָ�����������������ת�任
void Th_Projection(float AD3[],float AD2[],int16_t l,int16_t n,uint16_t len);//ͶӰ
void Cart_Draw3DRect(float AD2[],float lAD2[],uint8_t color,uint8_t per);//
void Cart_SetRect(int16_t x,int16_t y,int16_t z);//
void Cart_DrawFor(int16_t x,int16_t y,int16_t x1,int16_t y1,int16_t x2,int16_t y2,int16_t x3,int16_t y3,uint16_t cor,uint8_t per);//
#endif /* INC_MATH3D_H_ */
