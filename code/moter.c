#include "zf_common_headfile.h"
#include "moter.h"

extern uint8 remenber_point;
extern int16 EncoderL;
extern int16 EncoderR;
extern uint8 motor_flag;
extern uint16_t motor_base;
extern int16 Max_encoderL;//����������ٶ�
extern int16 Max_encoderR;


//�����ʼ��
void  moter_init(){
    gpio_init(P10_2,GPO,0,GPO_PUSH_PULL);
    gpio_init(P10_3,GPO,0,GPO_PUSH_PULL);
    pwm_init(MoterL, 1000, 0);//Init_PWM 
    pwm_init(MoterR, 1000, 0);//Init_PWM
}

/*PID_init()��ʼ������
PID* pid  ����PID�ṹ��
uint8 Kp 
uint8 Ki
uint8 Kd
float target ����ֵ
*/
void PID_init(PID* pid,float Kp,float Ki,float Kd,float target){
    pid->Kp = Kp;
    pid->Ki = Ki;
    pid->Kd = Kd;
    pid->LastError = 0;
    pid->PrevError = 0;
    pid->target = target;
    pid->OUT=0;
}


/*////////����ʽPID///////////
PID*             ����PID�Ľṹ��
current_value    �������Ĳ⵽��ʵ���ٶ�
*/
float  PID_Increse(PID* pid,float current_value){
  /*
    float output = 0;
    float error = pid->target - current_value;                  // ��ǰ���
    float d_error = error - pid->LastError;                     // ��������
    float prev_d_error = pid->LastError - pid->PrevError;      // ��һ����������
    
    // ������������ = kp*������� + ki*��ǰ��� + kd(�����������+�ϴ��������)
    float increment = pid->Kp * d_error + pid->Ki * error + pid->Kd * (d_error - prev_d_error);

    // �������
    output += increment;

    // ��������¼
    pid->PrevError = pid->LastError;
    pid->LastError = error;
    
    return output;
*/
    float error = pid->target - current_value;
    pid->PrevError += error;
    //�����޷�
    if(pid->target >=225 && pid->PrevError>= 5000)pid->PrevError = 5000;//�����޷�
    else if(pid->target >=30 && pid->target <225 && pid->PrevError>= 12000)pid->PrevError = 12000;//�����޷�
    else if(pid->target <30 && pid->PrevError>= 18000)pid->PrevError = 18000;//�����޷�
    float derivative = error - pid->LastError;
    pid->LastError = error;
    pid->OUT =  pid->Kp * error + pid->Ki * pid->PrevError + pid->Kd * derivative;

    return pid->OUT;
}

/*
PID*             ����PID�Ľṹ��
current_value    ��ǰ��λ��
*/
float PID_location(PID *pid, float current_value) {
    float error = pid->target - current_value;  // ��ǰ���
    static float integral = 0;//������
    integral += error;         // �������ۼ�

    // ��� = Kp * ��ǰ��� + Ki *  ������ + Kd * (������ - ��һ�����)
    pid->OUT = pid->Kp * error + pid->Ki * integral + pid->Kd * (error - pid->LastError);

    // ������һ�ε����
    pid->LastError = error;

    return pid->OUT;
}

//����
void   Steering_FeedBack(PID * pid){
  static int input = 0;
  static int Rinput = 0;
  static int Linput = 0;
  input = (int)PID_location(pid,remenber_point);
  Rinput = motor_base+input;
  Linput = motor_base-input;
  //Protect��λ����
  if(Rinput>=10000)Rinput = 10000;
  else if(Rinput<=0)Rinput=0;
  if(Linput>=10000)Linput = 10000;
  else if(Linput<=0)Linput=0;
  //��PID���ֵ����PWM
  if(motor_flag){
    pwm_set_duty(MoterR, Rinput);
    pwm_set_duty(MoterL, Linput);
  }else{
    pwm_set_duty(MoterR, 0);
    pwm_set_duty(MoterL, 0);
  }

}

//�ٶȻ�
void   Speed_FeedBack(PID * pid,Moter_WHO moter){
  //���ڲ���
  if(moter == Left){//���������
    uint16_t input = (uint16_t)(pid->target / Max_encoder * 10000);
    if(input>=10000)input = 10000;
    else if(input <=0)input = 0;
    pwm_set_duty(MoterL,  (uint32)(input + PID_Increse(pid,EncoderL)));
    ips200_Printf(0,280,(ips200_font_size_enum)0,"set:%.0f ",pid->target);//���õ�ֵ
    ips200_Printf(0,272,(ips200_font_size_enum)0,"pid:%d ",(int)PID_Increse(pid,EncoderL));//pid���ֵ
  }
  else{//������ҵ��
    uint16_t input = (uint16_t)(pid->target / Max_encoder * 10000);
    if(input>=10000)input = 10000;
    else if(input <=0)input = 0;
    pwm_set_duty(MoterR,  (uint32)(input + PID_Increse(pid,EncoderR)));
    ips200_Printf(120,280,(ips200_font_size_enum)0,"set:%.0f ",pid->target);//���õ�ֵ
    ips200_Printf(120,272,(ips200_font_size_enum)0,"pid:%d ",(int)PID_Increse(pid,EncoderR));//pid���ֵ
  }
}

/*����PID �� ת��->�ٶȻ�
PID * SteeringPID ת��pid����
PID * SpeedPID_L ���ٶȻ�pid����
PID * SpeedPID_R ���ٶȻ�pid����
*/
void   Cascade_FeedBack(PID * SteeringPID,PID * SpeedPID_L,PID * SpeedPID_R){
  static float basic_Speed;
  static uint8_t once_flag = 1;
  if(once_flag){//ִֻ��һ�Σ���¼�µ������ٶ�
      basic_Speed = SpeedPID_L->target;
      once_flag = 0;
  }
  
  //ת�������pid�����¼��SteeringPID->OUT
  PID_location(SteeringPID,remenber_point);//ת�򻷽������
  
  //�ٶȻ���������ת�򻷵Ĳ���,�޸ĵ��ٶȻ���Ŀ��ֵ(����ֵ)
  SpeedPID_L->target = basic_Speed - SteeringPID->OUT;//�����ٶ� + SteeringPID->OUT
  SpeedPID_R->target = basic_Speed + SteeringPID->OUT;//�����ٶ� - SteeringPID->OUT
  
  //�ٶȻ����м��㣬pid�����¼��SpeedPID_L->OUT �� SpeedPID_R->OUT
  PID_Increse(SpeedPID_L,EncoderL);
  PID_Increse(SpeedPID_R,EncoderR);
  
  //Protectռ�ձȱ���
  if(SpeedPID_L->OUT >=10000)SpeedPID_L->OUT  = 10000;
  else if(SpeedPID_L <=0)SpeedPID_L = 0;
  if(SpeedPID_R->OUT >=10000)SpeedPID_R->OUT  = 10000;
  else if(SpeedPID_R <=0)SpeedPID_R = 0;
  
  //���ٶȻ�pidOUT�ֱ�����ڵ����
  if(motor_flag){
    pwm_set_duty(MoterR, (uint16_t)SpeedPID_L->OUT);
    pwm_set_duty(MoterL, (uint16_t)SpeedPID_R->OUT);
  }else{
    pwm_set_duty(MoterR,0);
    pwm_set_duty(MoterL,0);
  }
  /////////////////���ڲ���/////////////////
  static uint8_t onec_flag2 = 1;
  if(onec_flag2){
    ips200_Printf(0,280,(ips200_font_size_enum)0,"pidL_Target:");//��ʾL�ߵ��pwmֵ
    ips200_Printf(120,280,(ips200_font_size_enum)0,"pidR_Target:");//��ʾL�ߵ��pwmֵ
    onec_flag2 = 0;
  }
  ips200_Printf(72,280,(ips200_font_size_enum)0,"%.0f ",SpeedPID_L->target);//��ʾL�ߵ��pwmֵ
  ips200_Printf(192,280,(ips200_font_size_enum)0,"%.0f ",SpeedPID_R->target);//��ʾL�ߵ��pwmֵ
  /////////////////���ڲ���/////////////////
}

//Encoder_L �� Encoder_R �ֱ��������ձ����������ֵ(type int16),�ϵ��������������ٶ�����
void Encoder_Get_Max(int16* Encoder_L,int16* Encoder_R){
    static uint8_t onece_flag = 1;
    if(onece_flag){
      pwm_set_duty(MoterR, 10000);
      pwm_set_duty(MoterL, 10000);
      onece_flag = 0;
    }

    if(EncoderL>*Encoder_L)
      *Encoder_L = EncoderL;
    if(EncoderR>*Encoder_R)
      *Encoder_R = EncoderR;
}

void Encoder_Test(){//�����ڲ���
   motor_flag = 1;
   //ips200_Printf(0,166,(ips200_font_size_enum)0,"   SpeedL:%.2f ",speedL);
   //ips200_Printf(120,166,(ips200_font_size_enum)0," SpeedR:%.2f ",speedR);
   ips200_Printf(0,240,(ips200_font_size_enum)0,"Max_L:%d",Max_encoderL);//��ʾ����������ֵ
   ips200_Printf(0,248,(ips200_font_size_enum)0,"Max_R:%d",Max_encoderR);//��ʾ�ұ��������ֵ
   ips200_Printf(58,288,(ips200_font_size_enum)0,"%d ",EncoderL);//��ʾ���������ֵ
   ips200_Printf(178,288,(ips200_font_size_enum)0,"%d ",EncoderR);//��ʾ�ұ���������
   Encoder_Get_Max(&Max_encoderL,&Max_encoderR);
}