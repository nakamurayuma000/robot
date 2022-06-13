/*前後２脚ロボット４パターン場合分けプログラム*/

#define M5STACK_MPU9250 
#include <M5Stack.h>
#include <Kalman.h>
#include "utility/myIMU.h"
#define RAD_TO_DEG 57.324
#include <mcp_can_m5.h>
#include <SPI.h>
#include <RMDX8_M5.h>
// #include <RMDX8PROV2_M5.h>

/*SAMD core*/
#ifdef ARDUINO_SAMD_VARIANT_COMPLIANCE
#define SERIAL SERIALUSB
#else
#define SERIAL Serial
#endif

#define BAUDRATE 115200 //シリアル通信がボトルネックにならないよう，速めに設定しておく
#define LOOPTIME 10   //[ms]
// #define ENDTIME 10000   //[ms]
#define TEXTSIZE 2

/*体幹の前後傾*/
#define Z 4     //PITCH角補正係数4が適当
#define AJF 0.9   //ACC JYRO FILTER（kalman姿勢角度ローパス係数）

/*脚部の係数たち*/
#define KP_S 25      //脚部膝関節モータ比例ゲインstance
#define KD_S 0.15    //脚部膝関節モータ微分ゲインstance

#define KP_SW 25      //股関節モータ_脚部スイング
#define KD_SW 0.2      //股関節モータ_脚部スイング

#define KP_F 15       //脚部股関節モータ比例ゲインflight
#define KD_F 0.15      //脚部股関節モータ微分ゲインflight

#define KP_SWF 7
#define KD_SWF 0.15

#define KP_F1 15       //脚部股関節モータ比例ゲインflight 抱え込み時の係数
#define KD_F1 0.15       //脚部股関節モータ微分ゲインflight　抱え込み時の係数

#define FLIGHT_LENGTH 0 //遊脚期_膝関節抱え込み係数
#define FORCE 0       //跳躍係数
#define STANCE_CURRENT 2500  //接地基準電流指令値

/*歩行プログラムパラメータ*/
#define FLIGHT_POS  7    //飛翔期脚部位置
#define SWING_ANGLE 20   //脚部着地時スイング角度


unsigned long timer[3];
uint16_t cnt = 0;

/*加速度、ジャイロの変数の定義*/
float accX, accY, accZ;
float gyroX, gyroY, gyroZ;
float pitch, roll, yaw;
float my_pitch, my_roll, my_yaw , initial_pitch , initial_roll ;
float Temp;
float dt = 0.001;
float KalAngleX , KalAngleY , initial_KalAngleX;
float angleX, angleY;
float filter_angleX;
float filter_angleY;

/*角度制限（最大角度，最低角度の定義）*/
float max_angle = 100; // 最大角度
float min_angle = -100; // 最小角度
bool exit_tf = false;

/*6軸センサインスタンスの定義*/
myIMU MPU6885;   //acc jyro instance
Kalman KalmanX;  //kalman instance作成
Kalman KalmanY;  //kalman instance作成


/*モータアドレスの設定*/
const uint16_t MOTOR_ADDRESS1 = 0x141; //0x140 + ID(1~32)
const uint16_t MOTOR_ADDRESS2 = 0x142; //0x140 + ID(1~32)

const int SPI_CS_PIN = 12;

#define CAN0_INT 15          // Set INT to pin 2
MCP_CAN_M5 CAN0(SPI_CS_PIN); // Set CS to pin 10


/*RMDX8 instance*/
/*脚部*/
RMDX8_M5 myMotor1(CAN0, MOTOR_ADDRESS1);    // knee_1
RMDX8_M5 myMotor2(CAN0, MOTOR_ADDRESS2);    // hip_1

/*角度変数定義（RMD 3つ分）*/
double initial_pos[2], previous_pos[2], present_vel[2];

int32_t target_pos[2], target_cur[3];

/*脚部圧力センサ変数定義*/
double FOOT_SENSOR =0 ;                                  //脚部圧力センサ生データ
double FILTERED_SENSOR =0 ;                              //脚部圧力センサフィルター
#define FSF 0.9                                          //FOOTSENSOR FILTER（圧力センサローパス係数）
#define PRESSURE_THRESHOLD 340                           //圧力センサ閾値
bool leg_stance = false ;

/*プログラム離脱係数*/
bool exitflag;
bool onewayflag_s = false;
bool onewayflag_f = false;
int  phaseflag = 3 ; //1:stance1 , 2:stance2 , 3:flight1 , 4:flight2(basic)

void init_can();

void GET_DATA(void* arg)
 {
   while (1)
   {
     /*圧力センサ読み取り*/
     FOOT_SENSOR = analogRead( 36 );
     FILTERED_SENSOR = FSF * FILTERED_SENSOR + (1-FSF) * FOOT_SENSOR;
     if( FILTERED_SENSOR > PRESSURE_THRESHOLD )
     {
       leg_stance = true;
     }
     else if( FILTERED_SENSOR <= PRESSURE_THRESHOLD )
     {
       leg_stance = false;
     }

    //  if( M5.BtnA.read() )
    //  {
    //    leg_stance = true;
    //  }
    //  else if( M5.BtnC.read() )
    //  {
    //    leg_stance = false;
    //  }
     
      /*姿勢角度読み取り*/
     MPU6885.getGyro(&gyroX,&gyroY,&gyroZ);  //ジャイロの読み取り
     MPU6885.getAccel(&accX,&accY,&accZ);    //加速度の読み取り
   
     KalAngleX = KalmanX.getAngle( accX , gyroX , dt ) ;       //kalman補正（ピッチ角）

     filter_angleX = AJF * filter_angleX + (1 - AJF) * KalAngleX; //ピッチ角度ローパスフィルタ
     delay(1);
   }
  }

void ACTIVE(void* arg)
 {
   while (1)
   {
     while( exit_tf == false )
     {
         /*角度制限，LOOP停止*/
         if( ( myMotor2.present_angle - initial_pos[2] ) >= max_angle ){
          myMotor1.stop();   
          myMotor2.stop();
          myMotor1.clearState();   
          myMotor2.clearState(); 
          exit_tf = true;
          }
         else if( ( myMotor2.present_angle - initial_pos[2] ) <= min_angle ){
          myMotor1.stop();   
          myMotor2.stop();
          myMotor1.clearState();   
          myMotor2.clearState(); 
          exit_tf = true;
         }
         else if( M5.BtnB.read() ){
          myMotor1.stop();   
          myMotor2.stop();
          myMotor1.clearState();   
          myMotor2.clearState(); 
          exit_tf = true;
         }

         /*TIME SHORTAGE*/
         timer[2] = millis() - timer[1];
         if (timer[2] < LOOPTIME)
          {
            delay(LOOPTIME - timer[2]);
          }
         else
          {
            SERIAL.print("TIME SHORTAGE");
            SERIAL.println(LOOPTIME - timer[2]);
          }

         /*main program*/
         switch ( phaseflag )
         {

         /*stance1*/
         case 1 : 
           delay(1);
           M5.update();
           timer[1] = millis();
           M5.Speaker.mute();

           previous_pos[1] = myMotor1.present_angle; 
           previous_pos[2] = myMotor2.present_angle; 

           /*knee joint current*/
           myMotor1.readPosition();
           present_vel[1] = (myMotor1.present_angle - previous_pos[1])*1000/LOOPTIME;
           target_cur[1] = - KP_S*(myMotor1.present_angle - initial_pos[1]) - KD_S * present_vel[1] + FORCE;  
         
           /*hip joint current*/
           myMotor2.readPosition();
           present_vel[2] = (myMotor2.present_angle - previous_pos[2])*1000/LOOPTIME;
           target_pos[2] = (0.5 * (myMotor1.present_angle - initial_pos[1]) + initial_pos[2] + SWING_ANGLE + ( filter_angleX * Z -initial_pitch ) ) * (600) ; 
           target_cur[2] = - KP_SW*(myMotor2.present_angle - ( initial_pos[2] + SWING_ANGLE )) - KD_SW * present_vel[2];   
      
           myMotor1.writeCurrent(target_cur[1]);     // knee motor: current control (spring)
           myMotor2.writeCurrent(target_cur[2]);     // hip motor: current control (spring)->swing mode

           SERIAL.print("TIM: ");
           SERIAL.print(timer[1] - timer[0]);
           SERIAL.print(" POS_knee1: ");
           SERIAL.print(myMotor1.present_angle - initial_pos[1] );
           SERIAL.print(" TGT_hip1: ");
           SERIAL.print( ( target_pos[2] / 600 ) - initial_pos[2] );
           SERIAL.print(" POS_hip1: ");
           SERIAL.print( myMotor2.present_angle - initial_pos[2] );
           SERIAL.print(" CURRENT ");
           SERIAL.print( target_cur[1] );

           /*pressure*/
           SERIAL.print(" STANCE1 ");
           SERIAL.print( FILTERED_SENSOR );

           /*pitch angle*/
           SERIAL.print( " PITCH " );
           SERIAL.print( filter_angleX * Z -initial_pitch );
            
           SERIAL.println("");

           /*stance1->stance2*/
           if( abs( myMotor2.present_angle - ( initial_pos[2] + SWING_ANGLE +  ( filter_angleX * Z -initial_pitch )) ) >= 5  )
           {
            phaseflag = 1;
           }
           else
           {
            phaseflag = 2;
           }
         break;


         /*stance2*/
         case 2 : 
           delay(1);
           M5.update();
           timer[1] = millis();
           M5.Speaker.mute();

           previous_pos[1] = myMotor1.present_angle; 
           previous_pos[2] = myMotor2.present_angle; 

           /*knee joint current*/
           myMotor1.readPosition();
           present_vel[1] = (myMotor1.present_angle - previous_pos[1])*1000/LOOPTIME;
           target_cur[1] = - KP_S*(myMotor1.present_angle - initial_pos[1]) - KD_S * present_vel[1] + FORCE;  
         
           /*hip joint current*/
           myMotor2.readPosition();
           present_vel[2] = (myMotor2.present_angle - previous_pos[2])*1000/LOOPTIME;
           target_pos[2] = (0.5 * (myMotor1.present_angle - initial_pos[1]) + initial_pos[2] + SWING_ANGLE + ( filter_angleX * Z -initial_pitch ) ) * (600) ; 
           target_cur[2] = - KP_SW*(myMotor2.present_angle - ( initial_pos[2] + SWING_ANGLE )) - KD_SW * present_vel[2];   
      
           myMotor1.writeCurrent(target_cur[1]);     // knee motor: current control (spring)
           myMotor2.writePosition(target_pos[2]);    // hip motor: stop

           SERIAL.print("TIM: ");
           SERIAL.print(timer[1] - timer[0]);
           SERIAL.print(" POS_knee1: ");
           SERIAL.print(myMotor1.present_angle - initial_pos[1] );
           SERIAL.print(" TGT_hip1: ");
           SERIAL.print( ( target_pos[2] / 600 ) - initial_pos[2] );
           SERIAL.print(" POS_hip1: ");
           SERIAL.print( myMotor2.present_angle - initial_pos[2] );
           SERIAL.print(" CURRENT ");
           SERIAL.print( target_cur[1] );

           /*pressure*/
           SERIAL.print(" STANCE2 ");
           SERIAL.print( FILTERED_SENSOR );

           /*pitch angle*/
           SERIAL.print( " PITCH " );
           SERIAL.print( filter_angleX * Z -initial_pitch );
            
           SERIAL.println("");

           /*stance2->flight1*/
           if ( leg_stance == true )
           {
             phaseflag = 2;
           }
           else
           {
             phaseflag = 3;
           }
         break;


         case 3 : //flight1
         delay(1);      
         M5.update();
         timer[1] = millis();
         M5.Speaker.tone(440, 100);

         previous_pos[1] = myMotor1.present_angle;
         previous_pos[2] = myMotor2.present_angle;

         /*knee joint*/
         myMotor1.readPosition();
         present_vel[1] = (myMotor1.present_angle - previous_pos[1])*1000/LOOPTIME;
         target_cur[1] = - KP_F*(myMotor1.present_angle - ( FLIGHT_LENGTH + initial_pos[1]) ) - KD_F * present_vel[1] + FORCE;  

         /*hip joint*/
         myMotor2.readPosition();
         present_vel[2] = (myMotor2.present_angle - previous_pos[2])*1000/LOOPTIME;
         target_pos[2] = (0.5 * (myMotor1.present_angle - initial_pos[1]) + initial_pos[2] - FLIGHT_POS + ( filter_angleX * Z -initial_pitch ) ) * (600) ; 
         target_cur[2] = - KP_SWF*(myMotor2.present_angle - ( initial_pos[2] - FLIGHT_POS ) ) - KD_SWF * present_vel[2] ;  

         myMotor1.writeCurrent(target_cur[1]);    // knee motor : current control
         myMotor2.writeCurrent(target_cur[2]);    // hip motor: current control

         SERIAL.print("TIM: ");
         SERIAL.print(timer[1] - timer[0]);
         SERIAL.print(" POS_knee1: ");
         SERIAL.print(myMotor1.present_angle - initial_pos[1] );
         SERIAL.print(" TGT_hip1: ");
         SERIAL.print( ( target_pos[2] / 600 ) - initial_pos[2] );
         SERIAL.print(" POS_hip1: ");
         SERIAL.print( myMotor2.present_angle - initial_pos[2] );
         SERIAL.print(" CURRENT ");
         SERIAL.print( target_cur[1] );

         /*pressure*/
         SERIAL.print(" FLIGHT1 ");
         SERIAL.print( FILTERED_SENSOR );

         /*pitch angle*/
         SERIAL.print( " PITCH " );
         SERIAL.print( filter_angleX * Z -initial_pitch );
            
         SERIAL.println("");

         /*flight1->flight2*/
         if ( abs( myMotor2.present_angle - ( initial_pos[2] - FLIGHT_POS + ( filter_angleX * Z -initial_pitch ) ) ) >= 5 )
         {
           phaseflag = 3;
         }
         else
         {
           phaseflag = 4;
         }
         break;

         /*flight2*/
         case 4 : 
         delay(1);      
         M5.update();
         timer[1] = millis();
         M5.Speaker.tone(440, 100);

         previous_pos[1] = myMotor1.present_angle;
         previous_pos[2] = myMotor2.present_angle;

         /*knee joint*/
         myMotor1.readPosition();
         present_vel[1] = (myMotor1.present_angle - previous_pos[1])*1000/LOOPTIME;
         target_cur[1] = - KP_F*(myMotor1.present_angle -  initial_pos[1] ) - KD_F * present_vel[1] + FORCE;  

         /*hip joint*/
         myMotor2.readPosition();
         present_vel[2] = (myMotor2.present_angle - previous_pos[2])*1000/LOOPTIME;
         target_pos[2] = (0.5 * (myMotor1.present_angle - initial_pos[1]) + initial_pos[2] - FLIGHT_POS + ( filter_angleX * Z -initial_pitch ) ) * (600) ; 
         target_cur[2] = - KP_SWF*(myMotor2.present_angle - ( initial_pos[2] - FLIGHT_POS ) ) - KD_SWF * present_vel[2] ;  

         myMotor1.writeCurrent(target_cur[1]);    // knee motor:spring
         myMotor2.writePosition(target_pos[2]);   // hip motor: stop

         SERIAL.print("TIM: ");
         SERIAL.print(timer[1] - timer[0]);
         SERIAL.print(" POS_knee1: ");
         SERIAL.print(myMotor1.present_angle - initial_pos[1] );
         SERIAL.print(" TGT_hip1: ");
         SERIAL.print( ( target_pos[2] / 600 ) - initial_pos[2] );
         SERIAL.print(" POS_hip1: ");
         SERIAL.print( myMotor2.present_angle - initial_pos[2] );
         SERIAL.print(" CURRENT ");
         SERIAL.print( target_cur[1] );

         /*pressure*/
         SERIAL.print(" FLIGHT2 ");
         SERIAL.print( FILTERED_SENSOR );

         /*pitch angle*/
         SERIAL.print( " PITCH " );
         SERIAL.print( filter_angleX * Z -initial_pitch );
            
         SERIAL.println("");

         /*flight2->stance1*/
         if ( leg_stance == false )
         {
           phaseflag = 4;
         }
         else
         {
           phaseflag = 1;
         }
         break;

         }

     }
    
     // stop command
     myMotor1.clearState();
     myMotor2.clearState();
     M5.Speaker.mute();
    
     delay(500);
     SERIAL.println("Program finish!");
     M5.Lcd.setCursor(0, 160);
     M5.Lcd.printf("Program finish!");
      while (true)
     {
         delay(100);
     }
  }
 }

void setup() {
   /*M5stack start*/
    M5.begin();
    dacWrite(25, 0); 
    M5.Power.begin();
    MPU6885.init();
    SERIAL.begin(BAUDRATE);
    // M5.IMU.Init();
    delay( 5000 );

    /*RMDX8 CAN通信 SET UP*/
    myMotor1.canSetup();
    myMotor2.canSetup();
    delay( 1000 );

    /*M5stack ディスプレイ設定*/
    M5.Lcd.fillScreen(BLACK);
    M5.Lcd.setTextColor(GREEN, BLACK);
    M5.Lcd.setTextSize(TEXTSIZE);

    init_can();
    SERIAL.println("NOW SET UP...");
    delay(1000);

    timer[0] = millis();

    KalmanX.setAngle( 0 );
    KalmanY.setAngle( 0 );
    delay(1);

    M5.Lcd.setCursor(0, 40);
    M5.Lcd.printf("(Exit) Press Center Button >>");

    /*RMDX8 初期化*/
    myMotor1.clearState();   
    myMotor2.clearState();   
    delay( 1 );

    
    /*脚部の初期位置の読み取り*/
  myMotor1.readPosition();
  initial_pos[1] = myMotor1.present_angle;  //膝関節の初期位置設定
  myMotor2.readPosition();
  initial_pos[2] = myMotor2.present_angle;  //股関節の初期位置設定
  delay(1000);

  /*姿勢角度の初期位置読み取り*/
  MPU6885.getGyro(&gyroX,&gyroY,&gyroZ);
  MPU6885.getAccel(&accX,&accY,&accZ);

  initial_KalAngleX = KalmanX.getAngle( accX , gyroX , dt );
  
  initial_pitch = initial_KalAngleX * Z ;
  delay(100);

  /*タスクの生成*/
  xTaskCreatePinnedToCore(GET_DATA, "Task0", 4096, NULL, 0, NULL, 0);
  xTaskCreatePinnedToCore(ACTIVE, "Task1", 4096, NULL, 1, NULL, 1);
  // xTaskCreatePinnedToCore(task2, "Task2", 4096, NULL, 2, NULL, 2);
}

void init_can()
{
    M5.Lcd.setTextSize(TEXTSIZE);
    M5.Lcd.setCursor(0, 10);
    M5.Lcd.printf("CAN Test!\n");

    // Initialize MCP2515 running at 16MHz with a baudrate of 500kb/s and the masks and filters disabled.
    if (CAN0.begin(MCP_ANY, CAN_1000KBPS, MCP_8MHZ) == CAN_OK)
    {
        SERIAL.println("MCP2515 Initialized Successfully!");
    }
    else
    {
        SERIAL.println("Error Initializing MCP2515...");
    }

    CAN0.setMode(MCP_NORMAL); // Change to normal mode to allow messages to be transmitted
}

void loop() {
  delay(1);
}
