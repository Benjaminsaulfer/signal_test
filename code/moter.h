#pragma once

#define Max_encoder 450 //����������ٶ�,��ռ�ձ�������ߣ����ɵõ�����������������ٶ�
#define  Encoder_speed(x) (float)(x)*Max_encoder/100  /*�����������ת��/100Ŀ���ǰѱ�����100�ȷ�
                                        ���Ժ������ٶȻ���ʱ��ֱ������xΪ,50��Ϊ�԰ٷ�֮50���ٶ�ǰ��  */
#define MoterL TCPWM_CH25_P09_1
#define MoterR TCPWM_CH24_P09_0

//PID�ṹ��
typedef struct {
  float Kp;
  float Ki;
  float Kd;
  float target;         //Ŀ���ٶȣ�����Ҫ���ٶȣ�
  float LastError;      //��һ�����
  float PrevError;     //����һ�����
  float OUT;
}PID;

typedef enum {
  Right = 0,
  Left = 1
}Moter_WHO;

void   moter_init();//�����ʼ��
///////////////////////�ײ�PID����////////////////////////
void   PID_init(PID* pid,float Kp,float Ki,float Kd,float target);
float  PID_Increse(PID* pid,float current_value);             //������ʽPID
float  PID_location(PID *pid, float current_value);           //λ��ʽPID

////////////////////////Ӧ����PID/////////////////////////
void   Steering_FeedBack(PID * pid);                          //ת��
void   Speed_FeedBack(PID * pid,Moter_WHO moter);             //�ٶȻ�
void   Cascade_FeedBack(PID * SteeringPID,PID * SpeedPID_L,PID * SpeedPID_R);   //����PID (ת�򻷣��ٶȻ�)  

/////////////////////////������/////////////////////////////
void Encoder_Get_Max(int16* Encoder_L,int16* Encoder_R);      //��ȡ���������ֵ
void Encoder_Test();                                          //���Ա��������ֵ������ʵ��ֵ