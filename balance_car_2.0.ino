/************************使用的头文件********************************/
#include <PinChangeInt.h>    //外部中断
#include <MsTimer2.h>        //定时中断

#include "U8glib.h"    //用于显示
#include "Adafruit_NeoPixel.h" //WS2812库文件,盘灯

#include "I2Cdev.h"  //I2C协议库
#include "Wire.h"    //服务于I2C通信  
#include "MPU6050_6Axis_MotionApps20.h"//MPU6050库文件
#include "KalmanFilter.h"  //卡尔曼滤波库文件
/*******************************************************************/

/************************实例化*********************************************************/
U8GLIB_SSD1306_128X64 u8g(U8G_I2C_OPT_NONE); //实例化一个 U8GLIB 对象，对象名称为 u8g

MPU6050 mpu6050;                             //实例化一个 MPU6050 对象，对象名称为 mpu6050
KalmanFilter KalFilter;                      //实例化一个卡尔曼滤波器对象，对象名称为 KalFilter

/**************************************************************************************/

/***************************全局变量*************************************************/
int16_t ax, ay, az, gx, gy, gz;  //MPU6050的三轴加速度和三轴陀螺仪（角速度）数据

         /**********引脚分配************/
#define AIN1 9   //TB6612FNG驱动模块控制信号 共6个
#define AIN2 8
#define BIN1 12
#define BIN2 13
#define PWMA 6
#define PWMB 10 // 11号脚是废的

#define ENCODER_L_A 2  //编码器采集引脚 每路2个 共4个 ENCODER_L
#define ENCODER_L_B 3  //DIRECTION_L
#define ENCODER_R_A 4  //ENCODER_R
#define ENCODER_R_B 5  //DIRECTION_R 

#define CHAO_TRIG 7 //超声波引脚
#define CHAO_ECHO 11

#define HONGWAI A0 //红外管脚

#define FMQ A1 //蜂鸣器管脚

#define PANDENG A2 //盘灯管脚

        /********************************/
#define BRIGHTNESS 20  //设置盘灯亮度
Adafruit_NeoPixel strip = Adafruit_NeoPixel(7, PANDENG, NEO_GRB + NEO_KHZ800);  // 实例化一个盘灯对象
        
#define DIFFERENCE 2       //DIFFERENCE是一个衡量平衡小车电机和机械安装差异的一个变量。直接作用于输出，让小车具有更好的一致性。
#define ZHONGZHI -8        //小车的机械中值
float Turn_Amplitude = 40; //转向目标幅度值 
        
int Balance_Pwm, Velocity_Pwm, Turn_Pwm;   //直立 速度 转向环的PWM

int Motor1, Motor2;      //电机叠加之后的PWM
volatile long Velocity_L, Velocity_R = 0;   //左右轮编码器数据
int Velocity_Left, Velocity_Right = 0;     //左右轮速度

int Angle ;      //用于显示的 角度 的临时变量
float Distance;  // 超声波距离

/*******标志位********/
int Flag_Qian, Flag_Hou, Flag_Left, Flag_Right; //遥控相关变量,1为使能
int Yaokong_F ; // 使用遥控的标志位
int Once_F = 1; // 确保不满足避障条件时开启蓝牙并使标志位被蓝牙控制
int Lanya_Bizhang_F; // 蓝牙+避障标志位
int Auto_Bizhang_F;  // 自动避障标志位
int Auto_Gensui_F;   // 自动跟随标志位
int FMQ_F ;          // 控制蜂鸣器响一次
int OLED_F;          //打开显示器的开关，奇开偶关
/*******************/


/**** PID 参数 *********/
float Balance_Kp= 15,  Balance_Kd=0.5;      // 直立PD
float Velocity_Kp= -2.4,  Velocity_Ki= -0.012;    // 速控PI，参考 kp = 2, ki = kp / 200; 
float Turn_Kp = 2, Turn_Kd = 0.001;        // 转向PD
/*******************/

/***************下面是卡尔曼滤波相关变量***************/
float K1 = 0.05; // 对加速度计取值的权重
float Q_angle = 0.001, Q_gyro = 0.005;
float R_angle = 0.5 , C_0 = 1;
float dt = 0.005; //注意：dt的取值为滤波器采样时间 5ms5ms

/**********************************************************************************/


/*********************下为功能函数***************************************/
/**************************************************************************
  函数功能：通过蜂鸣器表示小车状态
  入口参数：无
  返回  值：无
**************************************************************************/
void FMQ_Control() {
  int sensor;
  sensor=digitalRead(HONGWAI); // 获取红外信号
  
  if (Flag_Qian == 0 && Flag_Hou == 0 && Flag_Left == 0 && Flag_Right == 0 && sensor == 1) {
    FMQ_F = 1;
  }
  
  if (Flag_Qian == 1 || Flag_Hou == 1 || Flag_Left == 1 || Flag_Right == 1 ) {      // 运动状态 D~D
    if(FMQ_F){
    analogWrite(FMQ, 255 );
    delay(100); 
    analogWrite(FMQ, 0);//停止发声
    delay(100); 
    analogWrite(FMQ, 255 );
    delay(100); 
    analogWrite(FMQ, 0);//停止发声
    delay(100); 
    FMQ_F = 0;
    }
  }

  if(sensor == 0 && (Lanya_Bizhang_F || Auto_Bizhang_F)){
    analogWrite(FMQ, 255 );
    delay(50); 
    analogWrite(FMQ, 0);//停止发声
    delay(50); 
    analogWrite(FMQ, 255 );
    delay(50); 
    analogWrite(FMQ, 0);//停止发声
    delay(50); 
    analogWrite(FMQ, 255 );
    delay(50); 
    analogWrite(FMQ, 0);//停止发声
    delay(50);
  }
}

/**************************************************************************
函数功能：通过超声波测量与目标的距离
入口参数：无
返回  值：车与目标距离（cm）
**************************************************************************/
float Get_Distance(){
  float cm;
  //发一个10us的高脉冲去触发TrigPin 
  digitalWrite(CHAO_TRIG, LOW); //发一个短时间脉冲去TrigPin
  delayMicroseconds(2); //delayMicroseconds在更小的时间内延时准确
  digitalWrite(CHAO_TRIG, HIGH); 
  delayMicroseconds(10); 
  digitalWrite(CHAO_TRIG, LOW);//通过这里控制超声波的发声
 
  cm = pulseIn(CHAO_ECHO, HIGH) / 58.0;//回波时间换算成cm
  cm = (int(cm * 100.0)) / 100.0;//保留两位小数
  return cm;
  
}

/**************************************************************************
函数功能：利用超声波、红外测距实现自动跟随
入口参数：无
返回  值：无
**************************************************************************/
void Gensui_Control(){
  int Hw_length = 12;
  int bias = 1; 
  if(Auto_Gensui_F){
     int sensor;
    sensor=digitalRead(HONGWAI); // 获取红外信号
    if(Distance>=0 && Distance <= 3*Hw_length){
      if (sensor==0 && Distance <(Hw_length-bias)){   // 太近，向后
        Flag_Qian = 0, Flag_Hou = 1, Flag_Left = 0, Flag_Right = 0 ;
      }
      if (sensor == 0 && (Distance>=(Hw_length-bias)&&Distance<=(Hw_length+bias))){ // 差不多，停车
          Flag_Qian = 0, Flag_Hou = 0, Flag_Left = 0, Flag_Right = 0;
      }
      if (sensor==0 && Distance >(Hw_length+bias)){   // 向左
        Flag_Qian = 0, Flag_Hou = 0, Flag_Left = 1, Flag_Right = 0 ;
      }
      if (sensor == 1 && Distance <(Hw_length + bias)){  //向右
          Flag_Qian = 0, Flag_Hou = 0, Flag_Left = 0, Flag_Right = 1;
        }
      if (sensor == 1 && Distance >(Hw_length+bias)){  //不及，向前
          Flag_Qian = 1, Flag_Hou = 0, Flag_Left = 0, Flag_Right = 0;
        }
    }
    else {
      Flag_Qian = 0, Flag_Hou = 0, Flag_Left = 0, Flag_Right = 0;
    }

  }
  
}

/**************************************************************************
函数功能：小车避障控制
入口参数：无
返回  值：无
**************************************************************************/
void Bizhang_Control(){
  int sensor;
  sensor = digitalRead(HONGWAI);
  
  if(Lanya_Bizhang_F){  // 蓝牙 + 避障
      if(sensor==0) {
      Yaokong_F  = 0 ;
      Once_F  = 1;
      Flag_Qian = 0, Flag_Hou = 0, Flag_Left = 0, Flag_Right = 1; delay(200); //最后强制右转
      
      }
      else if(Once_F == 1){
          Flag_Qian = 0, Flag_Hou = 0, Flag_Left = 0, Flag_Right = 0;
          Yaokong_F  = 1;
          Once_F  = 0;
        }
    }

  if(Auto_Bizhang_F){  //自动避障
    if(sensor==0){
      Flag_Qian = 0, Flag_Hou = 0, Flag_Left = 0, Flag_Right = 1; delay(200); //最后强制右转
    }
    else{
      Flag_Qian = 1, Flag_Hou = 0, Flag_Left = 0, Flag_Right = 0; // 没遇到障碍物向前走
    }
  }
  
}


/**************************************************************************
函数功能：通过接收蓝牙数据实现简单的控制
入口参数：无
返回  值：无
**************************************************************************/
void Lanya_Control(){ 
  static unsigned char Receive_Data;
  
    if (Serial.available())   //如果有收到数据
    {
      Receive_Data = Serial.read();
      switch (Receive_Data)// 进行功能选择
      {
        case 0x64: OLED_F++;if(OLED_F == 10) OLED_F = 0;  //OLED显示 
        case 0x61: Lanya_Bizhang_F = 1, Auto_Bizhang_F = 0 , Auto_Gensui_F = 0 , Yaokong_F = 1;break;   //蓝牙+避障控制
        case 0x62: Lanya_Bizhang_F = 0, Auto_Bizhang_F = 1 , Auto_Gensui_F = 0 , Yaokong_F = 0; break;  //自动避障
        case 0x63: Lanya_Bizhang_F = 0, Auto_Bizhang_F = 0 , Auto_Gensui_F = 1 , Yaokong_F = 0;break;   //自动跟随 
        default:break;
      }
      
      if(Yaokong_F){  // 使用蓝牙遥控
        switch (Receive_Data)
        {
          case 0x41: Flag_Qian = 1, Flag_Hou = 0, Flag_Left = 0, Flag_Right = 0;   break;              //前进
          case 0x42: Flag_Qian = 0, Flag_Hou = 0, Flag_Left = 0, Flag_Right = 1;   break;             //右转
          case 0x43: Flag_Qian = 0, Flag_Hou = 0, Flag_Left = 0, Flag_Right = 1;   break;             //右转
          case 0x44: Flag_Qian = 0, Flag_Hou = 0, Flag_Left = 0, Flag_Right = 1;   break;              //右转
          case 0x45:  Flag_Qian = 0, Flag_Hou = 1, Flag_Left = 0, Flag_Right = 0;   break;             //后退
          case 0x46: Flag_Qian = 0, Flag_Hou = 0, Flag_Left = 1, Flag_Right = 0;  break;               //左转
          case 0x47: Flag_Qian = 0, Flag_Hou = 0, Flag_Left = 1, Flag_Right = 0; break;               //左转
          case 0x48: Flag_Qian = 0, Flag_Hou = 0, Flag_Left = 1, Flag_Right = 0;   break;             //左转
          default: Flag_Qian = 0, Flag_Hou = 0, Flag_Left = 0, Flag_Right = 0; break;                //停止
        }
    }
  }


 //蓝牙避障控制 97 = 0x61
 //自动避障 98 = 0x62
 //自动跟随 99 = 0x63
}



/**************************************************************************
函数功能：绘制OLED显示信息
入口参数：无
返回  值：无
**************************************************************************/
void Show_Screen() {
  u8g.firstPage();
  do {
    u8g.setFont(u8g_font_unifont);//设置要显示字符的字体

    u8g.drawStr(0, 15, "Dist：" );//显示距离
    u8g.setPrintPos(70,15);
    u8g.print(Distance);
    
      u8g.drawStr(0, 30, "Angle: "); //显示角度
      u8g.setPrintPos(70,30);
      u8g.print(Angle); 

      u8g.drawStr(0, 45, "Speed_L: ");//显示左轮速度
      u8g.setPrintPos(70,45);
      u8g.print(Velocity_Left);

      u8g.drawStr(0, 60, "Speed_R: ");//显示右轮速度
      u8g.setPrintPos(70,60);
      u8g.print(Velocity_Right);
     
  } while( u8g.nextPage() );
}

/**************************************************************************
函数功能：通过盘灯表示小车状态
入口参数：无
返回  值：无
**************************************************************************/
void Pandeng_control() {
 
  if (Flag_Qian == 1) {                               // 向前
   uint32_t color = strip.Color(0, 255, 0);         // 绿色
   strip.setPixelColor(0, 0);
   strip.setPixelColor(1, 0);
   strip.setPixelColor(2, 0);
   strip.setPixelColor(3, 0);
   strip.setPixelColor(4, color);
   strip.setPixelColor(5, color);
   strip.setPixelColor(6, color); 
    }
  if (Flag_Hou == 1) {                                // 向后
   uint32_t color = strip.Color(255, 215, 0);         // Gold1
   strip.setPixelColor(0, 0);
   strip.setPixelColor(3, 0);
   strip.setPixelColor(4, 0);
   strip.setPixelColor(5, 0);
   strip.setPixelColor(1, color);
   strip.setPixelColor(2, color);
   strip.setPixelColor(6, color); 
    }
  if (Flag_Left == 1) {                                // 向左
    uint32_t color = strip.Color(0, 255, 255);         // Cyan色
   strip.setPixelColor(1, 0);
   strip.setPixelColor(5, 0);
   strip.setPixelColor(0, color);
   strip.setPixelColor(2, color);
   strip.setPixelColor(3, color);
   strip.setPixelColor(4, color);
   strip.setPixelColor(6, color);
    }
  if (Flag_Right == 1) {                              // 向右
   uint32_t color = strip.Color(255, 0, 255);         // Magenta色
   strip.setPixelColor(2, 0);
   strip.setPixelColor(4, 0);
   strip.setPixelColor(0, color);
   strip.setPixelColor(1, color);
   strip.setPixelColor(3, color);
   strip.setPixelColor(5, color);
   strip.setPixelColor(6, color);
    }
  if (Flag_Qian == 0&& Flag_Hou == 0  && Flag_Left == 0 && Flag_Right == 0) {   // 停止
   uint32_t color = strip.Color(255, 0, 0);         //红色
   strip.setPixelColor(0, color);
   strip.setPixelColor(3, color);
   strip.setPixelColor(1, color);
   strip.setPixelColor(2, color);
   strip.setPixelColor(4, color);
   strip.setPixelColor(5, color);
   strip.setPixelColor(6, 0);
   } 
   strip.show();                                //是LED显示所选的颜色;
}

/********************************上方为功能函数****************************/


/********************************下为闭环函数*********************************/
/**************************************************************************
函数功能：直立PD控制  作者：平衡小车之家
入口参数：角度、角速度
返回  值：直立控制PWM
**************************************************************************/

int balance(float Angle, float Gyro)
{
  float Bias;
  int balance;
  Bias = Angle - ZHONGZHI;   //===求出平衡的角度中值 和机械相关
  balance = Balance_Kp * Bias + Gyro * Balance_Kd; //===计算平衡控制的电机PWM  PD控制   kp是P系数 kd是D系数; 15 ,0.4
  return balance;
}

/**************************************************************************
函数功能：速度PI控制 作者：平衡小车之家
入口参数：左轮编码器、右轮编码器
返回  值：速度控制PWM
**************************************************************************/
int velocity(int encoder_left, int encoder_right)
{
  static float Velocity, Encoder_Least, Encoder, Movement;
  static float Encoder_Integral, Target_Velocity;
  if       ( Flag_Qian == 1) Movement = -300;
  else   if ( Flag_Hou == 1) Movement = 300;
  else    //这里是停止的时候反转，让小车尽快停下来
  {
    Movement = 0;
    if (Encoder_Integral > 300)   Encoder_Integral -= 200;
    if (Encoder_Integral < -300)  Encoder_Integral += 200;
  }
  //=============速度PI控制器=======================//
  Encoder_Least = (encoder_left + encoder_right) - 0;               //===获取最新速度偏差==测量速度（左右编码器之和）-目标速度（此处为零）
  Encoder *= 0.7;                                                   //===一阶低通滤波器
  Encoder += Encoder_Least * 0.3;                                   //===一阶低通滤波器
  Encoder_Integral += Encoder;                                      //===积分出位移 积分时间：40ms
  Encoder_Integral = Encoder_Integral - Movement;                   //===接收遥控器数据，控制前进后退
  if (Encoder_Integral > 10000)    Encoder_Integral = 10000;        //===积分限,控制最高速度
  if (Encoder_Integral < -10000) Encoder_Integral = -10000;         //===积分限幅
  
  Velocity = Encoder * Velocity_Kp + Encoder_Integral * Velocity_Ki;                  //===速度PI控制
  // if (Turn_Off(KalFilter.angle, Battery_Voltage) == 1 || Flag_Stop == 1)    Encoder_Integral = 0;//小车停止的时候积分清零
  return Velocity;
}

/**************************************************************************
函数功能：转向控制 作者：平衡小车之家
入口参数：Z轴陀螺仪
返回  值：转向控制PWM
**************************************************************************/
int turn(float gyro)//转向控制
{
  static float Turn_Target, Turn, Turn_Convert = 1; // Turn_Convert可以控制转向的快慢
  
  if (1 == Flag_Left)             Turn_Target += Turn_Convert;  //根据遥控指令改变转向偏差
  else if (1 == Flag_Right)       Turn_Target -= Turn_Convert;//根据遥控指令改变转向偏差
  else Turn_Target = 0;
  if (Turn_Target > Turn_Amplitude)  Turn_Target = Turn_Amplitude; //===转向速度限幅
  if (Turn_Target < -Turn_Amplitude) Turn_Target = -Turn_Amplitude;
  
  Turn = - Turn_Target * Turn_Kp  + gyro * Turn_Kd;         //===结合Z轴陀螺仪进行PD控制
  return Turn;
}

/**************************************************************************
函数功能：赋值给PWM寄存器 作者：平衡小车之家
入口参数：左轮PWM、右轮PWM
返回  值：无
**************************************************************************/
void Set_Pwm(int moto1, int moto2)
{
  if (moto1 > 0)   {  digitalWrite(AIN1, HIGH);      digitalWrite(AIN2, LOW); } //TB6612的电平控制
  else            { digitalWrite(AIN1, LOW);       digitalWrite(AIN2, HIGH); }//TB6612的电平控制
  analogWrite(PWMA, abs(moto1)); //赋值给PWM寄存器
  
  if (moto2 < 0)  { digitalWrite(BIN1, HIGH);     digitalWrite(BIN2, LOW); }//TB6612的电平控制
  else       { digitalWrite(BIN1, LOW);     digitalWrite(BIN2, HIGH); }//TB6612的电平控制
  analogWrite(PWMB, abs(moto2));//赋值给PWM寄存器
}

/**************************************************************************
函数功能：限制PWM赋值  作者：平衡小车之家
入口参数：无
返回  值：无
**************************************************************************/
void Xianfu_Pwm(void)
{
  int Amplitude = 250;  //===PWM满幅是255 限制在250
   if(Flag_Qian==1)  Motor2-=DIFFERENCE;  //DIFFERENCE是一个衡量平衡小车电机和机械安装差异的一个变量。直接作用于输出，让小车具有更好的一致性。
   if(Flag_Hou==1)   Motor2-=DIFFERENCE-2;
  if (Motor1 < -Amplitude) Motor1 = -Amplitude;
  if (Motor1 > Amplitude)  Motor1 = Amplitude;
  if (Motor2 < -Amplitude) Motor2 = -Amplitude;
  if (Motor2 > Amplitude)  Motor2 = Amplitude;
}

/********************************************************************************/


/**************************************************************************
函数功能：5ms控制函数 核心代码 作者：平衡小车之家
入口参数：无
返回  值：无
**************************************************************************/
void control()
{
  static int Velocity_Count , Turn_Count, Display_Count;

  sei();//全局中断开启
  
  mpu6050.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);  //获取MPU6050陀螺仪和加速度计的数据
  KalFilter.Angletest(ax, ay, az, gx, gy, gz, dt, Q_angle, Q_gyro, R_angle, C_0, K1);          //通过卡尔曼滤波获取角度
  Angle = KalFilter.angle;//Angle是一个用于显示的整形变量

  //直立PD控制 控制周期5ms
  Balance_Pwm = balance(KalFilter.angle, KalFilter.Gyro_x); 

  //速度PI控制，控制周期40ms
  if (++Velocity_Count >= 8) 
  {
    Velocity_Left = Velocity_L;    Velocity_L = 0;  //读取左轮编码器数据，并清零，这就是通过M法测速（单位时间内的脉冲数）得到速度。
    Velocity_Right = - Velocity_R;    Velocity_R = 0; //读取右轮编码器数据，并清零
    Velocity_Pwm = velocity(Velocity_Left, Velocity_Right);//速度PI控制，控制周期40ms
    Velocity_Count = 0;
  }
  
  //转向PD控制，控制周期20ms
  if (++Turn_Count >= 4)
  {
    Turn_Pwm = turn(gz);
    Turn_Count = 0;
  }

/************环的叠加***********************************/
//  // 纯直立调节
//  Motor1 = Balance_Pwm ;  //直立环的叠加
//  Motor2 = Balance_Pwm ; //直立环的叠加

//  // 直立速控调节
//  Motor1 = Balance_Pwm - Velocity_Pwm ;  //直立、速度环的叠加
//  Motor2 = Balance_Pwm - Velocity_Pwm ; //直立、速度环的叠加
//
  // 三环融合
  Motor1 = Balance_Pwm - Velocity_Pwm + Turn_Pwm;  //直立速度转向环的叠加
  Motor2 = Balance_Pwm - Velocity_Pwm - Turn_Pwm; //直立速度转向环的叠加

/*****************************************************/ 
  Xianfu_Pwm(); //限幅

  // 角度保护
  if(Angle > 40 || Angle < -50) Motor1=0,Motor2=0;

 
  Set_Pwm(Motor1, Motor2); //如果不存在异常，赋值给PWM寄存器控制电机
  
  //屏幕显示，每1500ms刷新一次
  if(OLED_F % 2 == 1){
    if(++Display_Count >= 300){
      Show_Screen();
      Display_Count = 0;
    }
  }
}

/**************************************************************************
函数功能：初始化，只执行一次
入口参数：无
返回  值：无
**************************************************************************/
void setup() {
  // put your setup code here, to run once:
  pinMode(AIN1, OUTPUT);        //TB6612控制引脚，控制电机1的方向，01为正转，10为反转
  pinMode(AIN2, OUTPUT);          //TB6612控制引脚，
  pinMode(BIN1, OUTPUT);          //TB6612控制引脚，控制电机2的方向，01为正转，10为反转
  pinMode(BIN2, OUTPUT);          //TB6612控制引脚，
  pinMode(PWMA, OUTPUT);         //TB6612控制引脚，电机PWM
  pinMode(PWMB, OUTPUT);         //TB6612控制引脚，电机PWM
  digitalWrite(AIN1, 0);          //TB6612控制引脚拉低
  digitalWrite(AIN2, 0);          //TB6612控制引脚拉低
  digitalWrite(BIN1, 0);          //TB6612控制引脚拉低
  digitalWrite(BIN2, 0);          //TB6612控制引脚拉低
  analogWrite(PWMA, 0);          //TB6612控制引脚拉低
  analogWrite(PWMB, 0);          //TB6612控制引脚拉低

  pinMode(ENCODER_L_A, INPUT);       //编码器引脚
  pinMode(ENCODER_L_B, INPUT);       //编码器引脚
  pinMode(ENCODER_R_A, INPUT);       //编码器引脚
  pinMode(ENCODER_R_B, INPUT);       //编码器引脚

  pinMode(CHAO_TRIG, OUTPUT);       //超声波引脚
  pinMode(CHAO_ECHO, INPUT);       //超声波引脚

  pinMode(HONGWAI, INPUT);         //红外引脚

  pinMode(FMQ, OUTPUT);         //蜂鸣器引脚
  
  pinMode(PANDENG , OUTPUT);    //盘灯引脚

  strip.setBrightness(BRIGHTNESS);  // 设置盘灯亮度
  strip.begin();                    //初始化盘灯
  strip.show();

  Serial.begin(9600);       //开启串口，设置波特率为 9600  
  Wire.begin();             //加入 IIC 总线
  delay(1500);              //延时等待初始化完成
  
  mpu6050.initialize();     //初始化MPU6050
  delay(20); 

  
  MsTimer2::set(5, control);  //使用Timer2设置5ms定时器中断，
  MsTimer2::start();          //使用中断使能，Timer2开始计时，每5ms进入一次中断程序control

   /*中断触发类型 
   LOW: 低电平触发。
   CHANGE:管脚状态改变触发。
   RISING:上升沿触发。
   FALLING:下降沿触发
  */
 
  /*开启外部中断 编码器接口1，0代表中断号；
   ATmega168 / 328上有两个外部中断引脚，称为INT0和INT1。 INT0和INT1分别映射到引脚2和3；
   #define ENCODER_L_A 2 ,因此可以触发左轮测速函数READ_ENCODER_L
   */
  attachInterrupt(0, READ_ENCODER_L, CHANGE);    

  /*开启外部中断（引脚变化中断） 编码器接口2；
   引脚变化中断可以在任何引脚上激活；
   #define ENCODER_R_A 4 ,因此可以触发右轮测速函数READ_ENCODER_R
  */
  attachPinChangeInterrupt(4, READ_ENCODER_R, CHANGE);

  
}

/**************************************************************************
函数功能：主循环程序体
入口参数：无
返回  值：无
**************************************************************************/
void loop() {
  // put your main code here, to run repeatedly:
  
    Lanya_Control() ; //蓝牙控制

    FMQ_Control();  // 蜂鸣器控制

    Distance = Get_Distance();  //获取超声波测得的距离
    
    Bizhang_Control() ; //避障控制

    Gensui_Control() ; // 跟随控制

    Pandeng_control() ; // 盘灯控制

//  Serial.print("KalFilter.Gyro_x: ");Serial.println(KalFilter.Gyro_x);
//  Serial.print("KalFilter.Gyro_y: ");Serial.println(KalFilter.Gyro_y);
//  Serial.print("KalFilter.Gyro_z: ");Serial.println(KalFilter.Gyro_z);
//  Serial.print("\n");
//  delay(500);

//    Serial.print("Angle: ");Serial.println(Angle);
//    Serial.print("Speed_L: ");Serial.println(Velocity_Left);
//    Serial.print("Speed_R: ");Serial.println(Velocity_Right);
//    Serial.print("\n");
//    delay(10);
//    
//    delay(250);

}

/**************************************************************************
函数功能：外部中断读取编码器数据，具有二倍频功能 注意外部中断是跳变沿触发
入口参数：无
返回  值：无
**************************************************************************/
void READ_ENCODER_L() {
//  Serial.println("L");
  if (digitalRead(ENCODER_L_A) == LOW) {     //如果是下降沿触发的中断
    if (digitalRead(ENCODER_L_B) == LOW)      Velocity_L--;  //根据另外一相电平判定方向
    else      Velocity_L++;
  }
  else {     //如果是上升沿触发的中断
    if (digitalRead(ENCODER_L_B) == LOW)      Velocity_L++; //根据另外一相电平判定方向
    else     Velocity_L--;
  }
}
/**************************************************************************
函数功能：外部中断读取编码器数据，具有二倍频功能 注意外部中断是跳变沿触发
入口参数：无
返回  值：无
**************************************************************************/
void READ_ENCODER_R() {
//  while(1) Serial.println("R");
  if (digitalRead(ENCODER_R_A) == LOW) { //如果是下降沿触发的中断
    if (digitalRead(ENCODER_R_B) == LOW)      Velocity_R--;//根据另外一相电平判定方向
    else      Velocity_R++;
  }
  else {   //如果是上升沿触发的中断
    if (digitalRead(ENCODER_R_B) == LOW)      Velocity_R++; //根据另外一相电平判定方向
    else     Velocity_R--;
  }
}
