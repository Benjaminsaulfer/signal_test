#include "trace.h"
////��ͨɨ��//////
uint8 Middle_line[110] = {60};//middle line
uint8 PointX_L[120] = {0};//��¼��ߵ�
uint8 PointX_R[120] = {0};//��¼�ұߵ�
uint8 lengthX_L = 0;//��¼����
uint8 lengthX_R = 0;//��¼����

///////������///////
uint16 Fileds_len = 0;//������¼�������¼�ĵ���
uint8 eight_fields_x[300] = {0};//������X�������
uint8 eight_fields_y[300] = {0};//������Y�������
typedef enum Diraction{
  Right,
  UP,
  Down,
  Left
}Diraction;
///////�е�////////
int16_t remenber_point = 94;

uint8 Binary_map[188][120];//binary map

//����Ļ����ʾ����������
void Screen_Add(uint8 threshold){
  //Middle basic point��׼��
  ips200_draw_big_point(94,80,RGB565_BLUE);//���Ļ�׼��(94,80)
  
  //����ɨ��ɨ�����е�(x,80)
  if(remenber_point>1 && remenber_point<187)
    ips200_draw_big_point(remenber_point,80,RGB565_PURPLE);
  else if(remenber_point<=1)
    ips200_draw_big_point(0,80,RGB565_PURPLE);
  else if(remenber_point>=187)
    ips200_draw_big_point(187,80,RGB565_PURPLE);
  //ips200_draw_line(34,110,154,110,RGB565_YELLOW);//����ɨ���׼��X 34~110
  
  if(mt9v03x_finish_flag){
       mt9v03x_finish_flag= 0;
       ips200_show_gray_image(0, 0, mt9v03x_image[0], MT9V03X_W, MT9V03X_H, 188, 120, threshold);//ѭ��ˢ�³���ֵ��ͼ��
  }
   //////////////////////ɨ����ɨ��/////////////////////////////
  for(uint8_t x = 0,y=110;x<lengthX_L;x++,y--){
    ips200_draw_point(PointX_L[x],y,RGB565_RED);
  }
  for(uint8_t x = 0,y=110;x<lengthX_R;x++,y--){
    ips200_draw_point(PointX_R[x],y,RGB565_RED);
  }
  
   ///////////////////////ɨ��ɨ��////////////////////////////
   
   /*//////////////////////������ɨ��///////////////////////// 
   for(int i = 0;i<Fileds_len;i++)//�����߽���
   {
     ips200_draw_point(eight_fields_x[i],eight_fields_y[i],RGB565_RED);
   }
   for(int i = 0;i<Fileds_len/2;i++)//draw middle line��������
   {
     ips200_draw_point((eight_fields_x[i]+eight_fields_x[(uint8)(Fileds_len/2) + i])/2
                      ,(eight_fields_y[i]+eight_fields_y[(uint8)(Fileds_len/2) + i])/2,RGB565_GREEN); //draw middle line
   }
   *///////////////////////������ɨ��//////////////////////
}
////////////////////////////////////////////��ֵ��ͼ��/////////////////////////////////////////////////
//ֱ����ֵ��ֵ��
void binarizeImage(uint8 threshold) {
  //scan all the gray map then convert it intp binary map
  for (uint8_t y = 0; y < 120; y++) {
    for(uint8_t x =0;x<188;x++){
       //if lager than threshold we will turn this pixel to white else black
       if(mt9v03x_image[y][x] >= threshold)Binary_map[x][y] = 1;   //  white 1
       else Binary_map[x][y] = 0;          //  black 0
    }
  }
}
//ͼ��ѹ��
void binarizeImage_ZIP(uint8 threshold) {
  //scan all the gray map then convert it intp binary map
  uint8_t bin_x = 0,bin_y = 0;;
  for (uint8_t y = 0; y < 120; y+=2) {
       bin_y++;
    for(uint8_t x =0;x<188;x+=2){
       bin_x++;
       //if Pixel lager than threshold we will turn this pixel to white else black
       if(mt9v03x_image[y][x] >= threshold)Binary_map[bin_x][bin_y] = 1;   //  white 1
       else Binary_map[bin_x][bin_y] = 0;                                    //  black 0
    }
  }
}

/*��̬����Ӧ�㷨:���
image:����ͼ��
*/
int otsu_threshold(unsigned char *image) {
    int hist[256] = {0};
    double sum = 0;
    double sumB = 0;
    int wB = 0;
    int wF = 0;
    double varMax = 0;
    int threshold = 0;

    // ����ֱ��ͼ
    for (int i = 0; i < 22560; i++)//22560 = 188*120ͼ���С
        hist[image[i]]++;
    
    for (int i = 0; i < 256; i++) // ��������ֵ�ܺ�
        sum += i * hist[i];

    for (int t = 0; t < 256; t++) {// �������п��ܵ���ֵ
        wB += hist[t];
        if (wB == 0) continue;
        wF = 22560 - wB;
        if (wF == 0) break;

        sumB += t * hist[t];
        double mB = sumB / wB;
        double mF = (sum - sumB) / wF;

        double varBetween = (double)wB * (double)wF * (mB - mF) * (mB - mF);

        if (varBetween > varMax) {
            varMax = varBetween;
            threshold = t;
        }
    }

    return threshold;
}
////////////////////////////////////ѭ������////////////////////////////////////////////////
//��ͨɨ��
void Trace_middleLine(){
    uint8 Right_scan_flag = 1;//�������ɨ��
    uint8 Left_scan_flag = 1;//�����ұ�ɨ��
    uint8_t LeftX = 34;//��ߵ��ʼ��
    uint8_t RightX = 154;//�ұߵ��ʼ��
    lengthX_L = 0;//���³�ʼ��������鳤��
    lengthX_R = 0;//���³�ʼ���ұ����鳤��
    static uint8_t Last_remenber_point=0;//��¼��һ�εõ����е�

    for(uint8_t i = 34;i <= 154;i++){//Ѱ�Ұ��������
      if(Binary_map[i][110] < Binary_map[i+1][110]){//������С���ұ�
        LeftX = i;
        i = i + 6;//�������
      }
      else if(Binary_map[i][110] > Binary_map[i+1][110]){//�����ߴ����ұ�
        RightX = i+1;
        break;
      }     
    }
    uint8_t midpoint = (RightX+ LeftX)/2;//��ʼ�е�
    
    for(uint8 y = 110;y>20;y--){//��Y=110���ϵ�Y=20
      for (uint8_t x = midpoint; x > 20 && Left_scan_flag; x--) {//�е���X=20��
        if(Binary_map[x-1][y] < Binary_map[x][y]){//�ҵ���ߵ�
          if(y == 80)LeftX = x-1;//��¼������е�
          PointX_L[lengthX_L] = x-1;
          lengthX_L++;
          break;
        }
        else if(x == 21){//���Խ��û�ҵ�
          Left_scan_flag = 0;
          break;
        }
      }
      for (int x = midpoint ; x < 168 && Right_scan_flag; x++) {//�е���X=168��
        if(Binary_map[x+1][y] < Binary_map[x][y]){//���ұߵ�
          if(y == 80)RightX = x-1;//��¼�ҵ����е�
          PointX_R[lengthX_R] = x+1;
          lengthX_R++;
          break;
        }
        else if(x == 167){//���Խ��û�ҵ�
          Right_scan_flag = 0;
        }
      }
    }
    remenber_point = (RightX + LeftX) / 2;//�����ҵ����е�
    if(lengthX_L < 30 && (lengthX_R > lengthX_L))  remenber_point -= 40;//��ת
    else if(lengthX_R < 30 && (lengthX_L > lengthX_R))  remenber_point += 40;//��ת
    else if(lengthX_L ==0 && lengthX_R==0)  remenber_point = Last_remenber_point;
    
    Last_remenber_point = remenber_point;
}

/////�Ż�������ɨ��//////
uint8_t passed_pointX[200]={0};
uint8_t passed_pointY[200]={0};
void Trace_Eight_fields_new(){
    /*
    ���м俪ʼ������ɨ(begin with (94,100)),ɨ�赽�ڵ�֮��ֱ�ӽ��������.
    ������ɨ��˳��(����):
     5 4 3              3 4 5
     6 * 2              2 * 6
     7 8 1              1 8 7
    */
    static uint8_t Lx[8] = { 1 ,  1,  1,  0, -1, -1, -1, 0 };//����
    //static uint8_t Rx[8] = {-1 , -1, -1,  0,  1,  1,  1, 0 };//����
    static uint8_t dy[8] = { 1 ,  0, -1, -1, -1,  0,  1, 1 };
    
    //uint8 remenber_Rpoint = 0;
    //uint8 remenber_Lpoint = 0;
    
    uint8_t LeftX = 34;
    uint8_t RightX = 154;
    uint8_t beginY = 110;
    Fileds_len = 0;//��¼����
    uint8_t visited[188][120] = {false};//�߹���X��Y
    
    for(int i = 34;i <= 154;i++){//Ѱ�Ұ��������
      if(Binary_map[i+1][110] > Binary_map[i][110])
         LeftX = i+1;
      else if(Binary_map[i+1][110] < Binary_map[i][110])
         RightX = i+1;
    }
    ips200_draw_big_point(LeftX,110,RGB565_BLUE);
    ips200_draw_big_point(RightX,110,RGB565_BLUE);
    
    for(uint16_t i =0;i<100;i++,Fileds_len++){
        uint8_t j =0;
        for (j = 0; j < 8; j++) {
            //��ʼ����
            uint8_t new_x = LeftX + Lx[j];
            uint8_t new_y = beginY + dy[j];
            //����ҵ��˺ڵ��������㲻����һ����
            if (Binary_map[new_x][new_y] == 0 && visited[new_x][new_y] == false) {
               //���µ㵼��������
               eight_fields_x[i] = new_x;
               eight_fields_y[i] = new_y;
               visited[LeftX][beginY] = true;//����߹��ĵ�
               //��¼���µĵ�
               LeftX = new_x;
               beginY = new_y;
               break;
             }                                       
        }
    }
}