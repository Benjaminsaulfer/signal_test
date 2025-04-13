/*********************************************************************************************************************
* CYT4BB Opensourec Library ���� CYT4BB ��Դ�⣩��һ�����ڹٷ� SDK �ӿڵĵ�������Դ��
* Copyright (c) 2022 SEEKFREE ��ɿƼ�
*
* ���ļ��� CYT4BB ��Դ���һ����
*
* CYT4BB ��Դ�� ��������
* �����Ը��������������ᷢ���� GPL��GNU General Public License���� GNUͨ�ù������֤��������
* �� GPL �ĵ�3�棨�� GPL3.0������ѡ��ģ��κκ����İ汾�����·�����/���޸���
*
* ����Դ��ķ�����ϣ�����ܷ������ã�����δ�������κεı�֤
* ����û�������������Ի��ʺ��ض���;�ı�֤
* ����ϸ����μ� GPL
*
* ��Ӧ�����յ�����Դ���ͬʱ�յ�һ�� GPL �ĸ���
* ���û�У������<https://www.gnu.org/licenses/>
*
* ����ע����
* ����Դ��ʹ�� GPL3.0 ��Դ���֤Э�� �����������Ϊ���İ汾
* �������Ӣ�İ��� libraries/doc �ļ����µ� GPL3_permission_statement.txt �ļ���
* ���֤������ libraries �ļ����� �����ļ����µ� LICENSE �ļ�
* ��ӭ��λʹ�ò����������� ���޸�����ʱ���뱣����ɿƼ��İ�Ȩ����������������
*
* �ļ�����          main_cm7_1
* ��˾����          �ɶ���ɿƼ����޹�˾
* �汾��Ϣ          �鿴 libraries/doc �ļ����� version �ļ� �汾˵��
* ��������          IAR 9.40.1
* ����ƽ̨          CYT4BB
* ��������          https://seekfree.taobao.com/
*
* �޸ļ�¼
* ����              ����                ��ע
* 2024-1-4       pudding            first version
********************************************************************************************************************/

#include "zf_common_headfile.h"

#pragma location = 0x28001000
float m7_1_data[50] = {0};//CPU�ٶ�
//��ַ�䶯50*4=200byte,0x28001000 + 200(C8)

uint16 ArrowPos = 128;//arrow position

void key_even(){
    //������������ҵ���رյ�ʱ�����ͷ�ƶ�
    if(gpio_get_level(P20_2) == 0 && m7_1_data[1] == 0)
    {
        while (gpio_get_level(P20_2) == 0)
        ips200_Printf(80,ArrowPos,(ips200_font_size_enum)0,"   ");
        ArrowPos+=8;
        if(ArrowPos>=176)ArrowPos=128;
    }
    if(m7_1_data[1] == 0){//�������ر�
      switch(ArrowPos){
         case 128://Kp
            if(gpio_get_level(P20_0) == 0)m7_1_data[2]+=0.1;
            if(gpio_get_level(P20_3) == 0)m7_1_data[2]-=0.1;
            break;
         case 136://Ki
            if(gpio_get_level(P20_0) == 0)m7_1_data[3]+=0.1;
            if(gpio_get_level(P20_3) == 0)m7_1_data[3]-=0.1;
            break;
         case 144://Kd
            if(gpio_get_level(P20_0) == 0)m7_1_data[4]+=0.1;
            if(gpio_get_level(P20_3) == 0)m7_1_data[4]-=0.1;
            break;
         case 152://�ع��
            if(gpio_get_level(P20_0) == 0 && m7_1_data[5]<200)m7_1_data[5]++;
            if(gpio_get_level(P20_3) == 0 && m7_1_data[5]>0)m7_1_data[5]--;
            break;
         case 160://����ٶ�
            if(gpio_get_level(P20_0) == 0)m7_1_data[6]+=100;
            if(gpio_get_level(P20_3) == 0)m7_1_data[6]-=100;
            break;
         case 168://���ʹ��
            if(gpio_get_level(P20_0) == 0)m7_1_data[1] = 0;
            if(gpio_get_level(P20_3) == 0){
              m7_1_data[1] = 1;
              ips200_Printf(0,168,(ips200_font_size_enum)0,"motor:  %d",(uint32_t)m7_1_data[1]);// ���ʹ��
            }
            break;
      }
    }else{//��������
      if(gpio_get_level(P20_0) == 0)m7_1_data[1] = 0;
         if(gpio_get_level(P20_3) == 0){
          m7_1_data[1] = 1;
          ips200_Printf(0,168,(ips200_font_size_enum)0,"motor:  %d",(uint32_t)m7_1_data[1]);// ���ʹ��
      }
    }
}

void Baterry_ChecK(){
  if(m7_1_data[1]>= 6 &&   m7_1_data[1] <= 7.2 ){//����2S��ض���(6~7.2����)
    gpio_toggle_level(P19_4);
  }
  else if(m7_1_data[1]>= 9 &&   m7_1_data[1] <= 10.8 ){//����3S��ض���9~10.8����)
    gpio_toggle_level(P19_4);
  }
}

int main(void)
{
    static uint32_t speed;
    clock_init(SYSTEM_CLOCK_250M); 	// ʱ�����ü�ϵͳ��ʼ��<��ر���>
    debug_info_init();                  // ���Դ�����Ϣ��ʼ��
    
    //ips200_init(IPS200_TYPE_PARALLEL8);//��Ļ��ʼ��
    timer_init(TC_TIME2_CH1, TIMER_US);//us
    adc_init(ADC0_CH00_P06_0, ADC_12BIT); //����ص�ѹ
    /*
    ips200_Printf(190,0,(ips200_font_size_enum)0,"Battery");
    ips200_Printf(0,300,(ips200_font_size_enum)0,"speed:");
    ips200_Printf(0,152,(ips200_font_size_enum)0,"threshold:");//����ͷ�ع��
    ips200_Printf(0,160,(ips200_font_size_enum)0,"Moter_V:");//�����ʼ�ٶ�
    ips200_Printf(0,168,(ips200_font_size_enum)0,"motor:");// ���ʹ��
    ips200_Printf(0,216,(ips200_font_size_enum)0,"pid:");//��ʾPIDת��
    ips200_Printf(0,288,(ips200_font_size_enum)0,"EncoderL:");//��߱�����ֵ
    ips200_Printf(120,288,(ips200_font_size_enum)0,"EncoderR:");//�ұ߱�����ֵ
  */
    while(true)
    {
        timer_clear(TC_TIME2_CH1);
        timer_start(TC_TIME2_CH1);
        ////////////////////////////////////////////////////////////////////////////////////
        
        SCB_CleanInvalidateDCache_by_Addr(&m7_1_data, sizeof(m7_1_data));//����RAM����
        //key_even();
        Baterry_ChecK();
        /*
 
        if(m7_1_data[1] == 0){
            ips200_Printf(80,ArrowPos,(ips200_font_size_enum)0,"<<");
            ips200_Printf(190,8,(ips200_font_size_enum)1,"%.2fV ",(float)adc_convert(ADC0_CH00_P06_0)/4096 * 3.3*4.1);//��ʾ��ص�ѹ
            
            ips200_Printf(50,168,(ips200_font_size_enum)0,"%d",(uint32_t)m7_1_data[1]);// ���ʹ��
            ips200_Printf(0,128,(ips200_font_size_enum)0,"Kp:%.1f ",m7_1_data[2]);//kp
            ips200_Printf(0,136,(ips200_font_size_enum)0,"Ki:%.1f ",m7_1_data[3]);//ki
            ips200_Printf(0,144,(ips200_font_size_enum)0,"Kd:%.1f ",m7_1_data[4]);//kd
            ips200_Printf(60,152,(ips200_font_size_enum)0,"%d ",(uint32_t)m7_1_data[5]);//����ͷ�ع��
            ips200_Printf(50,160,(ips200_font_size_enum)0,"%d ",(uint32_t)m7_1_data[6]);//�����ʼ�ٶ�
            ips200_Printf(24,216,(ips200_font_size_enum)0,"%.1f ",m7_1_data[7]);//��ʾPIDת��
            ips200_Printf(0,272,(ips200_font_size_enum)0,"pidL:%d ",(uint32_t)m7_1_data[8]);//��ʾL�ߵ��pwmֵ
            ips200_Printf(120,272,(ips200_font_size_enum)0,"pidR:%d ",(uint32_t)m7_1_data[9]);//��ʾR�ߵ��pwmֵ  
        }

        ////////////////////////////////////////////////////////////////////////////////////
        ips200_Printf(40,300,(ips200_font_size_enum)0,"%d ",(uint32_t)m7_1_data[0]);//��ʾ�����ٶ�
        ips200_Printf(80,300,(ips200_font_size_enum)0,"M2:%d ",speed);//�ڶ������������ٶ���ʾ�����ٶ�
*/
        speed = timer_get(TC_TIME2_CH1);
        timer_stop(TC_TIME2_CH1);
    }
}

// **************************** �������� ****************************
