#include <M5Stack.h>       //|
#include <mcp_can_m5.h>    //|
#include <SPI.h>           //|モータを動かすのに必要な奴
#include <RMDX8_M5.h>      //|

#ifdef ARDUINO_SAMD_VARIANT_COMPLIANCE
#define SERIAL SERIALUSB
#else
#define SERIAL Serial
#endif

#define BAUDRATE 115200 //シリアル通信がボトルネックにならないよう，速めに設定しておく
#define LOOPTIME 10      //[ms]
//#define LOOPTIME 5     //[ms]
#define ENDTIME 20000   //[ms]
#define TEXTSIZE 2


#define KP 0.1
#define KD 0.10
#define TGT_POS 0

unsigned long timer[3];

const uint16_t MOTOR_ADDRESS1 = 0x141; //0x140 + ID(1~32)
const uint16_t MOTOR_ADDRESS2 = 0x142;
const int SPI_CS_PIN = 12;

#define CAN0_INT 15          // Set INT to pin 2
MCP_CAN_M5 CAN0(SPI_CS_PIN);// Set CS to pin 10

RMDX8_M5 myMotor1(CAN0, MOTOR_ADDRESS1);
RMDX8_M5 myMotor2(CAN0, MOTOR_ADDRESS2);



int A = 30; // [servoHornDeg]*[gearRatio]*[encorderResolution]
double f = 5;       //[Hz]
double omega = 2 * 3.14 * f;
double initial_pos1, initial_pos2, previous_pos1, previous_pos2, present_vel1, present_vel2;
int32_t target_cur1, target_cur2;

void init_can();

void setup()
{
    M5.begin();
    M5.Power.begin();
    SERIAL.begin(BAUDRATE);
    delay(1000);
    M5.Lcd.fillScreen(BLACK);
    M5.Lcd.setTextColor(GREEN, BLACK);
    M5.Lcd.setTextSize(TEXTSIZE);

    init_can();
    SERIAL.println("Test CAN...");
    timer[0] = millis();

    myMotor1.readPosition();
    initial_pos1 = myMotor1.present_angle;
    myMotor2.readPosition();
    initial_pos2 = myMotor1.present_angle;
}

void loop()
{
    while (millis() - timer[0] < ENDTIME)
    {
        timer[1] = millis();

        // read multi turn angle
        previous_pos1 = myMotor1.present_angle;
        myMotor1.readPosition();
        previous_pos2 = myMotor2.present_angle;
        myMotor2.readPosition();
        delay(1);

          present_vel2 = (myMotor2.present_angle - previous_pos2)*1000/LOOPTIME;
//        int32_t target_cur2 = myMotor1.present_angle;
//        int32_t target_cur2 =  - KP*(myMotor2.present_angle - previous_pos2) - KD * present_vel2;
        int32_t target_cur2 =  - KP*(myMotor2.present_angle - initial_pos2) - KD * present_vel2;
//        int32_t target_cur1 =  A * sin (omega * (timer[1] - timer[0])*0.001);
//        int32_t target_cur1 =  A * sin (omega * (timer[1] - timer[0])*0.001);

        
        
        myMotor2.writePosition(target_cur2);
        delay(1);

        M5.update();

        // vel = (myMotor2.present_pos - pos_buf)/(LOOPTIME*0.01);

        // Debug (M5モニタ)(この処理重いので注意)
        // M5.Lcd.setCursor(0, 40);
        // M5.Lcd.printf("TIM:            ");
        // M5.Lcd.setCursor(0, 40);
        // M5.Lcd.printf("TIM: %d", timer[1] - timer[0]);
        // M5.Lcd.setCursor(0, 70);
        // M5.Lcd.printf("POS:            ");
        // M5.Lcd.setCursor(0, 70);
        // M5.Lcd.printf("POS: %d", myMotor2.present_pos);
        // M5.Lcd.setCursor(0, 100);
        // M5.Lcd.printf("HRN:            ");
        // M5.Lcd.setCursor(0, 100);
        // M5.Lcd.printf("HRN: %d", myMotor2.present_angle);
        // M5.Lcd.setCursor(0, 130);
        // M5.Lcd.printf("VEL:            ");
        // M5.Lcd.setCursor(0, 130);
        // M5.Lcd.printf("VEL: %d", vel);

        // Debug (SERIAL)
        SERIAL.print("TIM: ");
        SERIAL.print(timer[1] - timer[0]);
//        SERIAL.print(" TGT1: ");
//        SERIAL.print(initial_pos1);
        SERIAL.print(" TGT2: ");
        SERIAL.print(initial_pos2);
//        SERIAL.print(" POS1: ");
//        SERIAL.print(myMotor1.present_angle);
        SERIAL.print(" POS2: ");
        SERIAL.print(myMotor2.present_angle);
//        SERIAL.print(" VEL1: ");
//        SERIAL.print(present_vel1);
         SERIAL.print(" VEL2: ");
        SERIAL.print(present_vel2);
//        SERIAL.print(" CUR1: ");
//        SERIAL.print(target_cur1);
//        SERIAL.print(" CUR2: ");
//        SERIAL.print(target_cur2);
        SERIAL.println("");

        timer[2] = millis() - timer[1];
        if (timer[2] < LOOPTIME)
        {
            delay(LOOPTIME - timer[2]);
        }
        else
        {   
            delay(100);
            SERIAL.print("TIME SHORTAGE");
            SERIAL.println(LOOPTIME - timer[2]);
            //M5.Lcd.printf("TIME SHORTAGE: %d\n", LOOPTIME - timer[2]);
        }
    }
    // finishing
   
    M5.Lcd.setCursor(0, 160);
    M5.Lcd.printf("Program finish!");
    while (true)
    {
        delay(1);
    }
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
