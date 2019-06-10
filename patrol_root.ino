
/*
    Name:       patrol_root.ino
    Created:	2019/5/30 14:54:52
    Author:     ��ͥ��/����������
*/
/*�⺯��*/
#include"AccelStepper/src/AccelStepper.h"
#include"AccelStepper/src/MultiStepper.h"
#include"wire/src/Wire.h"
#include"MPU6050/MPU6050.h"
#include"DFRobot_VL53L1X-master/DFRobot_VL53L1X.h"
#include"RTCDue/src/RTCDue.h"
#include"DFRobot_I2C_Multiplexer-master/DFRobot_I2CMultiplexer.h"
//#include"core_cm3.h"�̼��⺯����Ĭ���Ѱ���
/*�궨��*/
#define BODERADE			9600			//������
#define FOREWHEEL_EN_PIN					//ǰ�ֵ��ʹ�ܶ˿�(δָ����
#define BACKWHEEL_EN_PIN					//���ֵ��ʹ�ܶ˿ڣ�δָ����
#define SPEED_SLOW_START	500.0			//�����������ٶ�
#define DIS_SLOW_HAMMER		50.0			//��������ʱ������Ҫ���ٵľ���,��λcm
#define SPEED_SLOW			500.0			//���ɲ���ȼ��ٵ��ٶȣ�������
#define CONT_NUM			200000			//������������������ֵ
/*ʵ�������󣬹�11�������4�����⴫������ͬ��ַ����δ�������1����̬��������2��翪�أ�δ��д�����˿ھ�δָ��*/
AccelStepper base;  //��̨���
AccelStepper forearm_bend;					//ǰ���������
AccelStepper backarm_bend;					//����������
AccelStepper forearm_stretch;				//ǰ���������
AccelStepper backarm_stretch;				//����������
AccelStepper forearm_lock;					//ǰ���������
AccelStepper backarm_lock;					//����������
AccelStepper forearm_deflect;				//ǰ��ƫת���
AccelStepper backarm_deflect;				//���ƫת���
AccelStepper forearm_wheel;					//ǰ���ֵ��
AccelStepper backarm_wheel;					//����ֵ��
MPU6050 IMU;								//��̬������
VL53L0X laser_sensor;						//������봫����
DFRobot_I2CMultiplexer I2CMultiplexer(0x70);//I2C��չ�壬��δʹ�ã�Ӳ��δ����
RTCDue rtc(XTAL);							//RTCʵʱʱ��
/*ö��*/
enum ROBOT_MODE{SELFTEST_MODE = 0,MANUAL_MODE,AUTO_MODE};//�����˹���ģʽ���Լ졢�ֶ����Զ�
enum TYPE_OF_BLOCK{INSULATOR_STRING=0,SHOCKPROOF_HAMMER,JUMP_WIRE};//�ϰ�����𣬾�Ե�Ӵ������𴸣�����
void self_test();//�����Լ�ģʽ
void Man_Operation();//�����ֶ�����ģʽ

/*ȫ�ֱ���*/
static int mileage;//��̣��˴�Ϊ�ϴ󹤳̣�Ϊ����ķ��� 
char cmd;int data;//���������ָ����
int auto_mark = 0;//�������б��λ
float dis_f_u; float dise_f_d; float dise_b_u; float dis_b_d;//���봫��������
int forewheel_mark = 0; int backwheel_mark = 0;
uint32_t cont;//������
/*NVIC���ж�������δ֪��NVIC_SetPriorityGrouping �ж����ȼ������ÿ⺯��*/
void setup()
{
	/*�������ڲ��趨������*/ 
	Serial.begin(BODERADE);			
	/*ǰ������ӵ����ʼ��������ٶȡ���ʼ�ٶȡ�ʹ�ܶ˿�*/
	forearm_wheel.setMaxSpeed(1000); backarm_wheel.setMaxSpeed(1000); 
	forearm_wheel.setSpeed(0); backarm_wheel.setSpeed(0); 
	forearm_wheel.setEnablePin(FOREWHEEL_EN_PIN); backarm_wheel.setEnablePin(BACKWHEEL_EN_PIN);
	forearm_wheel.disableOutputs(); backarm_wheel.disableOutputs();
	/*����rtcʵʱʱ�ӣ���0��ʼ��¼����ʱ�䣬����1Сʱ������,rtc��NVIC�ж���Ӧ���ȼ���Ϊ0*/
	rtc.begin(); rtc.setTime(0, 0, 0); rtc.setAlarmTime(1, 0, 0); rtc.attachAlarm(attachalarm);
	/*����I2C*/
	Wire.begin();
	laser_sensor.setMode(eContinuous, eHigh);
	laser_sensor.start();
	
}

// Add the main program code into the continuous loop() function
void loop()
{
	/*������ѭ����cpu���̣�ͨ�����λ�ж��Ƿ���Ҫ���ϲ��������������*/
	if (forewheel_mark) { forearm_wheel.runSpeed(); } 	if (backwheel_mark) { backarm_wheel.runSpeed(); }
	/*���봫������Ϣ�ɼ�*/
	
	/*��������ָ���жϣ��ַ�+���ֵ�������ʽ*/
	while (Serial.available())
	{
		cmd = Serial.read();
		data = Serial.parseInt();
	}
	switch (cmd)
	{
	case 'm'://ģʽѡ��
		Mode_select(data);
	case '0':
		if (auto_mark = 1)
		{
			obstacle_class(data);
		}
		else
		{
			Serial.println("δ�����Զ�����״̬");
		}
		
	default:
		break;
	}
	/*������������*/
	if (start_up()) { cont++;} 
}
/*ѡ����ģʽ*/
void Mode_select(int cmddata)
{
	ROBOT_MODE mode;
	mode = (ROBOT_MODE)cmddata;
	switch (mode)
	{
	case SELFTEST_MODE:self_test(); break;
	case MANUAL_MODE:Man_Operation(); break;
	case AUTO_MODE:auto_run(); break;
	default: {Serial.println("����"); }//������ʾ���޸�
	}
}
/*�����𲽲�Ѳ��*/
void auto_run()
{
	self_test();//����һ���Լ�
	cruise();
	auto_mark = 1;
};
/*������Ѳ��״̬,һ��ʱ����л�Ϊ����Ѳ���ٶȣ��ٶȴ�����*/
void cruise()
{
	
	start_up();
	if (cont>CONT_NUM)
	{
		forearm_wheel.setSpeed(1000); backarm_wheel.setSpeed(1000);
		cont = 0;
	}
}
//��������ʹ�ܵ��������������ÿ�ε��þ�����Ϊ���٣��ٶȴ�����
boolean start_up()
{
	forearm_wheel.setSpeed(SPEED_SLOW_START); backarm_wheel.setSpeed(SPEED_SLOW_START);		//��Ϊ����
	forearm_wheel.enableOutputs(); backarm_wheel.enableOutputs();							//ʹ��
	forearm_wheel = 1; backarm_wheel = 1;													//���λ��1
	return true;
}
/*�ϰ�������*/
void obstacle_class(int class_name) 
{
	switch (class_name)
	{
	case 0://����
		hammer_over();
	default:
		break;
	}
}
//RTC���ӵ�ʱ���򣬹��ܴ�������ѡ������λ�����棬����͹��ģ�ѡ���������ͣ����
void attachalarm();
/*����Խ��*/
void hammer_over() 
{
	multiWheel_stop();
}
/*˫�ּ���*/
void multiWheel_stop()
{
	if (dise_f_d <DIS_SLOW_HAMMER )
	{
		forearm_wheel.setSpeed(SPEED_SLOW);
	}
}
