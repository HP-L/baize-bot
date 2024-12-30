/*
 * menu.c
 *
 *  Created on: Oct 2, 2024
 *      Author: lake
 */

#include "main.h"
#include "menu.h"
#include "MathSin.h"



void LintToLine(uint16_t Line0_A_X, uint16_t Line0_A_Y, uint16_t Line0_B_X,
		uint16_t Line0_B_Y, uint16_t color, uint16_t Line1_A_X,
		uint16_t Line1_A_Y, uint16_t Line1_B_X, uint16_t Line1_B_Y) {
	int LineA_X;
	int LineA_Y;
	int LineB_X;
	int LineB_Y;
	int dx_a = (Line1_A_X - Line0_A_X);
	int dy_a = (Line1_A_Y - Line0_A_Y);
	int dx_b = (Line1_B_X - Line0_B_X);
	int dy_b = (Line1_B_Y - Line0_B_Y);
	for (uint16_t g = 180; g < 360; g+=10) {
		LineA_X = Line0_A_X + (COS[g]+1)*0.5 * dx_a;
		LineA_Y = Line0_A_Y + (COS[g]+1)*0.5 * dy_a;
		LineB_X = Line0_B_X + (COS[g]+1)*0.5 * dx_b;
		LineB_Y = Line0_B_Y + (COS[g]+1)*0.5 * dy_b;
		ST7789_DrawLine(LineA_X, LineA_Y, LineB_X, LineB_Y, color);
	}

}

