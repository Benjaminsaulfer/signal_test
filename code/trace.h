#pragma once

#include "zf_common_headfile.h"
#include "math.h"

extern uint8 Binary_map[188][120];//binary map
extern int16_t remenber_point;//��¼������ɨ�������е�

////////////��ʾ������//////////////
void Screen_Add(uint8 threshold);   

////////////��ֵ��ͼ��//////////////
void binarizeImage(uint8 threshold);//Convert to binarizeImage
void binarizeImage_ZIP(uint8 threshold);//ѹ��ͼ��
int otsu_threshold(unsigned char *image);

/////////////ɨ�߷���///////////////
void Trace_middleLine();//Trace_middleLine
void Trace_Eight_fields_new();//�Ż�������
