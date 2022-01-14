/*
    Project: Final Project Code V7
    Author: Mingyan Zhou & Jiacheng Song & Dayong Tong
    Time: 12/11/2021
    Copyright: Derek Zhou - All Rights Reserved
    License: No license yet
    Reference: Mark Yim's Code
*/

//-----------------------------INCLUDE---------------------------------//

#include "RemoterJS.h"  // contains string "body" html code
#include "html510.h"
#include <WiFi.h>
#include <WiFiUdp.h>
#include "vive510.h"


//------------------------------EXTERN---------------------------------//

// NOTE: THIS FUNCTION USES myPos[0] and myPos[1] as true position of the car
void GetAngleDistanceTowardCan(int canx, int cany, double angledist[]);


//-----------------DEFINE AND CONSTANT AND VARIABLE--------------------//

//-----------------------------CONFIG----------------------------------

// 1 Access Point Mode + Set IP
// 2 Station Mode + DHCP + use derek's hotspot
// 3 Station Mode + DHCP + Router
// 4 Station Mode + Set IP + Router
#define WIFI_MODE       4

//------------------------------PIN------------------------------------

const uint8_t Pin_Servo1=25;
const uint8_t Pin_Servo2=26;
const uint8_t Pin_ServoClaw=14;
const uint8_t Pin_Beacon1=33;
const uint8_t Pin_Beacon2=27;
const uint8_t Pin_Vive1=10;
const uint8_t Pin_Vive2=9;
const uint8_t Pin_Dip1=37;
const uint8_t Pin_Dip2=38;
const uint8_t Pin_Dip3=34;
const uint8_t Pin_Dip4=35;
const uint8_t Pin_FrontIR=36;
const uint8_t Pin_RightIR=39;
const uint8_t Pin_GreenLed=4;

//-------------------------CALLING PERIOD------------------------------

#define HTTP_PERIOD     10
#define IR_PERIOD       5  // 200Hz 
// #define VIVE_PERIOD     1000
// #define UDP_PERIOD      500
#define VIVE_PERIOD     5
#define UDP_PERIOD      1

//--------------------------STATE MACHINE------------------------------

//bit definitions for robot motion and state byte
#define TELEOP          0   // Teleoperation 
#define WALL_FOLLOW     1   // Wall following
#define GOTO_CAN        2   // Go to can position
#define BEACON_TRACK    3   // Beacon tracking
#define FORWARD         4   // Going Forward
#define TURNING         5   // Turning
#define HOLDING         6   // Stop and calculate

volatile byte state = 0;

//----------------------------MOBILITY---------------------------------

// define & const & struct for PWM
#define SERVO_LEDC_RES_BITS     12 // resolution bits
#define SERVO_LEDC_FREQ         50 // frequency of servos' PWM
#define SERVO1_LEDC_CH          0
#define SERVO2_LEDC_CH          1

static uint32_t turnOldTime, turnCurrTime;

//--------------------------WALL FOLLOWING-----------------------------

#define BASE_SPEED          15  // base left and right speed for autonomous behaviors
#define MAX_DIST            12  // maximum distance that IR sensor can see (in)
#define MIN_DIST            2.5     // minimum distance that IR sensor can see (in)
#define TRIG_DIST           3   // distance that front IR sensor recognize an obstacle
#define LEFT_90_TURN_TIME   600 // turning time for a 90 degree turn using turning speed

// IR reading adc values and distance
double frontIrAdc, rightIrAdc; // keep catching the data all the time!
volatile float frontIR, rightIR; // distance

// bit definitions for sensor data flag byte [rt_snr left_snr left_ir right_ir rear_ir front_ir]
volatile byte obFlag = 0;
volatile bool obFront = false;   // Front IR trip [used to detect front wall for corner]

// PID Control Gain
const float kpR = 4;
const float kdR = 10;
const float kpL = 4;
const float kdL = 10;

// IR PID Control Goal Distance with Deadband
const float irMax = 5.0;
const float irMin = 5.0;

// define PD control global variables, curr_error = current reading - setpoint, prev_error = curr_error on previous iteration
//store previous error to calculate derror = curr_error-prev_error, side_derror = side front sensor - side back sensor
//store derror = difference between left and right error (used for hall follow to center the robot)
volatile float ri_curr;     // right ir current reading
volatile float ri_cerror;   // right ir current error
volatile float ri_perror;   // right ir previous error
volatile float ri_derror;   // right ir delta error

//-----------------------------BEACON----------------------------------

#define BEACON_FREQ             700     //PERIOD
#define NOISE_TLRNC             35      //allowed offset
#define BEACON_SERVO_FWD        6.7     //6.7% duty cycle ~ 90° please check ServoDutyCycleTest.ino for more details
#define BEACON_SERVO_LEFT       11.5    //180° duty cycle
#define BEACON_SERVO_RIGHT      2.5     //180° duty cycle
#define BEACON_OFFSET_COE       0.01    //coefficient for sensitivity
#define MAF_SAMPLE_SIZE         3       // moving average filter sample size
#define BEACON_TURN_SPEED       2       
#define BEACON_REFRESH_TIME     10000      // microsecond
#define BEACON_FORWARD_TIME     15000000     // microsecond

static bool b1IsSeen;           // phototransistor 1 is seeing the beacon 
static bool b2IsSeen;           // phototransistor 2 is seeing the beacon

static uint32_t beaconOldTime, beaconCurrTime;

static float beaconBalance = 0.0;   // a counter for how many times left and right phototransistor sees the beacon

//--------------------------UDP AND VIVE-------------------------------

#define ROBOT_UDP_PORT  2510    // port for robots is 2510
#define CAN_UDP_PORT    1510    // port for cans is 1510
#define UDP_PACKET_SIZE 14      // RX is 14 bytes, TX is 13 bytes!!!!

// Wifi definition
#if WIFI_MODE == 1 // Access Point Mode + Set IP
    const char* ssid     = "Group 16 Remoter";
#elif WIFI_MODE == 2 // Station Mode + DHCP + use derek's hotspot
    const char* ssid     = "Derek's";
    const char* password = "19971105";
#elif WIFI_MODE == 3 // Station Mode + DHCP + use Router
    const char* ssid     = "TP-Link_05AF";
    const char* password = "47543454";
#elif WIFI_MODE == 4 // Station Mode + Set IP + use Router
    const char* ssid     = "TP-Link_05AF";
    const char* password = "47543454";
#endif

IPAddress ipTarget(192, 168, 1, 185); // 255 is a broadcast address to everyone at 192.168.1.xxx!!!!
IPAddress ipLocal(192, 168, 0, 209);  // replace with your IP address
WiFiUDP myUdpServer;
WiFiUDP canUdpServer;
WiFiUDP robotUdpServer;

Vive510 vive1(Pin_Vive1);
Vive510 vive2(Pin_Vive2);

uint8_t robotId;

//--------------------------LOCALIZATION-------------------------------

double myPos[8]; // vive1x, vive1y, vive2x, vive2y, centerX, centerY, gripperX, gripperY
char myPosStr[UDP_PACKET_SIZE-1];
int canPos[8][2];
int otherRobotPos[3][2];

#define CENTER_TO_GRIPPER_LENGTH    8
#define CAN_FORWARD_STEP_TIME       400     // millis
#define SMALL_TURN_TIME             50     // millis
#define SMALL_HOLDING_TIME          500     // millis
#define POS_MAF_SAMPLE_SIZE         15      // moving average filter for Vive position sensor
#define CANANGLE_FILTER_LENGTH      10      // number of times to calculate angle and distance for averaging filter

const float CAN_REACH_RADIUS = 500;     // vive unit
const double MAX_ANGULAR_DEVIATION = 10.0 * 3.14159265 / 180.0;     // range of angle acceptable to move forward in GoToCan
// How often the getangledist is calculated
const int CANANGLE_FILTER_INTERVAL = (int) ((SMALL_HOLDING_TIME - 50) / (double) CANANGLE_FILTER_LENGTH);

volatile double canAngle;
volatile double canDistance;
float canAngleTime;
float canDistanceTime;
static uint32_t motionOldTime, motionCurrTime;
static uint32_t calcOldTime, calcCurrTime;
int autoCanIndex = 6;       // actual can number subtract 1

//------------------------------HTTP-----------------------------------

#define AUTO_PERIOD                 5000 // autonomous led flag

HTML510Server h(80);
static uint32_t autoOldTime, autoCurrTime;

//------------------------------CLAW-----------------------------------

#define CLAW_LEDC_CH            2   // channel 2 timer 1!
#define CLAW_LEDC_FREQ          50
#define CLAW_LEDC_RES_BITS      12  // resolution bits

//----------------------------FUNCTIONS--------------------------------//

//-------------------------SETUP AND LOOP------------------------------

void setup()
{
    SerialInit();
    ServoInit();
    WifiInit(); 
    UdpInit();
    ViveInit();
    DipInit();
    IRInit();
    BeaconTrackInit();
    ClawInit();
    AutoLedInit();
    StateInit();

    Serial.println("Robot has been initialized!");
}

void loop()
{
    static uint32_t httpOldTime, httpCurrTime;
    static uint32_t irOldTime, irCurrTime;
    static uint32_t viveOldTime, viveCurrTime, udpOldTime, udpCurrTime;
    static uint32_t printOldTime, printCurrTime;

    // update web page reading
    httpCurrTime = millis(); //10ms
    if ((httpCurrTime - httpOldTime) >= HTTP_PERIOD)
    {
        h.serve();
        httpOldTime = httpCurrTime;
    }

    // update Vive and UDP communication
    viveCurrTime=millis();
    udpCurrTime=millis();
    if ((viveCurrTime-viveOldTime)>=VIVE_PERIOD) // 1ms
    {
        ViveReceiveAndUdpSendMyPos();
        viveOldTime=viveCurrTime;
    }
    if ((udpCurrTime-udpOldTime)>=UDP_PERIOD) // 1ms
    {
        UdpReceiveCanMsg();
        UdpReceiveRobotMsg();
        udpOldTime=udpCurrTime;
    }

    // update IR sensor reading
    irCurrTime = millis();
    frontIrAdc = analogRead(Pin_FrontIR); // keep reading ADC value all the time instead of specific Hz
    rightIrAdc = analogRead(Pin_RightIR);
    // only update record and derive distance values at 200Hz
    if ((irCurrTime - irOldTime) >= IR_PERIOD) // 200Hz
    {
        updateIR();
        irOldTime = irCurrTime;
    }

    // PD Control Wall Following
    if (bitRead(state, WALL_FOLLOW))
    {
        // if the robot is turning, finish the turn within given timer
        if (bitRead(state, TURNING))
        {
            turnCurrTime = millis();
            RobotMove(-BASE_SPEED, BASE_SPEED);
            if ((turnCurrTime - turnOldTime) >= LEFT_90_TURN_TIME)
            {
                bitClear(state, TURNING); // become normal wall following state
                wallFollow();
            }
            Serial.println("Turning");
        }
        // if the robot is not turning and there is obstacle in the front
        else if (!bitRead(state, TURNING) && obFront)
        {
            bitSet(state, TURNING); // become turning
            turnOldTime = millis();
            RobotMove(-BASE_SPEED, BASE_SPEED);
        }
        else
        {
            wallFollow();
            Serial.println("Wall Following");
        }
    }

    // Beacon Tracking
    if (bitRead(state, BEACON_TRACK) && !bitRead(state, FORWARD))
    {
        beaconCurrTime = micros();

        // check the frequency sensed by each phototransistor
        CheckFrequencyBeacon1();
        CheckFrequencyBeacon2();

        // if both side senses beacon signal, then robot should move forward
        if (b1IsSeen && b2IsSeen)
        {
            RobotStop();
            bitSet(state, FORWARD);
            RobotMove(BASE_SPEED, BASE_SPEED);
        }
        // if only left phototransistor is lit, turn left
        else if (b1IsSeen)
        {
            RobotMove(-BEACON_TURN_SPEED, 0);////////////////////////////////////
        }
        // if only right phototransistor or none is lit, turn right
        else
        {
            RobotMove(BEACON_TURN_SPEED, 0);////////////////////////////////////
        }
        beaconOldTime = beaconCurrTime;
    }
    if (bitRead(state, BEACON_TRACK) && bitRead(state, FORWARD))
    {
        CheckFrequencyBeacon1();
        CheckFrequencyBeacon2();
        beaconCurrTime = micros();

        // P control to steer the robot so both phototransistor senses the beacon using beaconBalance
        if (beaconBalance > 0){
            RobotMove(BASE_SPEED - 0.00011 * beaconBalance, BASE_SPEED + 0.00011 * beaconBalance);
        }else{
            RobotMove(BASE_SPEED - 0.0001 * beaconBalance, BASE_SPEED + 0.0001 * beaconBalance);
        }
        // terminal condition: if reach maximum time or front IR sense obstacle
        if (((beaconCurrTime - beaconOldTime) >= BEACON_FORWARD_TIME) || ((frontIR < 6.0) && (frontIR > 2.6)))
        {
            bitClear(state, FORWARD);
            bitClear(state, BEACON_TRACK); // go 3 seconds and stop, for debugging
            Serial.println("Robot Reaching Beacon Or Timed Out");
            RobotStop();
        }
    }

    // New Vive Localization 
    if (bitRead(state, GOTO_CAN))
    {
        // initial condition
        if (!bitRead(state, FORWARD) && !bitRead(state, TURNING) && !bitRead(state, HOLDING))
        {
            if ((canPos[autoCanIndex][0] > 0) && (myPos[4] > 0)){ //we get the non-zero can & robot position
                bitSet(state, HOLDING);
                motionOldTime = millis();
                calcOldTime = millis();
            }
        }
        // holding state for calculating robot position using vive reading and average filter. Done when stationary
        else if (bitRead(state, HOLDING))
        {
            motionCurrTime = millis();
            calcCurrTime = millis();
            RobotStop();

            // average filter calculate current robot angle and distance to the target can
            if ((calcCurrTime - calcOldTime) > CANANGLE_FILTER_INTERVAL)
            {
                GetAngleDistanceTowardCan(autoCanIndex);
                canAngle = AverageFilterCanAngle(canAngle);
                calcOldTime = millis();
            }

            if ((motionCurrTime - motionOldTime) > SMALL_HOLDING_TIME)
            {
                // terminal condition: gripper reached within a radius of the target can position
                if (CheckReachCan(autoCanIndex))
                {
                    ResetAutoBehavior();
                    Serial.println("I have arrived!!!");
                }
                // if target can is within a cone in front of the robot, start moving forward
                else if ((canAngle < MAX_ANGULAR_DEVIATION) && (canAngle > -MAX_ANGULAR_DEVIATION))
                {
                    bitClear(state, TURNING);
                    bitClear(state, HOLDING);
                    bitSet(state, FORWARD);
                    Serial.print("canAngle: "); Serial.println(canAngle * 180 / 3.14159265);
                    Serial.print("MAX_ANGL: "); Serial.println(MAX_ANGULAR_DEVIATION);
                    motionOldTime = millis();
                }
                else    // if robot not at the right angle, keep turning to search for it
                {
                    bitClear(state, HOLDING);
                    bitClear(state, FORWARD);
                    bitSet(state, TURNING);
                    motionOldTime = millis();
                }
            }
        }
        // if robot is turning
        else if (!bitRead(state, FORWARD) && bitRead(state, TURNING) && !bitRead(state, HOLDING))
        {
            // if can is on the left of the robot
            // left and right motor isn't balanced so the turning duty cycle is different here
            if (canAngle > 0)
            {
                RobotMove(BEACON_TURN_SPEED+3, -BEACON_TURN_SPEED-3);
            } else if(canAngle < 0) {   // if can is on the right of the robot
                RobotMove(-BEACON_TURN_SPEED-7, BEACON_TURN_SPEED+7);
            }
                        
            motionCurrTime = millis();
            // go into holding after a turn
            if ((motionCurrTime - motionOldTime) > SMALL_TURN_TIME)
            {
                bitSet(state, HOLDING);
                bitClear(state, TURNING);
                motionOldTime = millis();
                calcOldTime = millis();
            }
        }
        // if robot is going forward
        else if (bitRead(state, FORWARD) && !bitRead(state, TURNING) && !bitRead(state, HOLDING))
        {
            motionCurrTime = millis();
            RobotMove(BASE_SPEED-7, BASE_SPEED-5);
            // go into holding after a burst forward
            if ((motionCurrTime - motionOldTime) > CAN_FORWARD_STEP_TIME)
            {
                bitClear(state, FORWARD);
                bitSet(state, HOLDING);
                motionOldTime = millis();
                calcOldTime = millis();
            }
        }
    }

    autoCurrTime=millis();
    if ((autoCurrTime - autoOldTime) >= AUTO_PERIOD)
    {
        AutoLedOn();
    }

    // print loop
    // for debug print line
    printCurrTime=millis();
    if ((printCurrTime - printOldTime) >= 1000)
    {
        // Serial.print("canPosX: "); Serial.println(canPos[autoCanIndex][0]);
        // Serial.print("canPosY: "); Serial.println(canPos[autoCanIndex][1]);
        // Serial.print("myPosX: "); Serial.println(myPos[4]);
        // Serial.print("myPosY: "); Serial.println(myPos[5]);
        // Serial.print("canAngle: "); Serial.println(canAngle * 180/3.14159265);
        printOldTime = printCurrTime;
    }

}

//--------------------------STATE MACHINE------------------------------

void StateInit(void)
{
    state = 0;
    bitSet(state, TELEOP);
    // bitSet(state, WALL_FOLLOW);
    // bitSet(state, BEACON_TRACK);
    // bitSet(state, GOTO_CAN);
}

void ResetAutoBehavior(void)
{
    // reset all the variables and timers for autonomous behavior
    RobotStop();
    state = 0;

    ri_curr = 0;     // right ir current reading
    ri_cerror = 0;   // right ir current error
    ri_perror = 0;   // right ir previous error
    ri_derror = 0;   // right ir delta error

    frontIrAdc = 0; rightIrAdc = 0; // keep catching the data all the time!
    frontIR = 0; rightIR = 0; // distance

    obFlag = 0;
    obFront = false; 

    b1IsSeen = false;
    b2IsSeen = false;
    beaconOldTime = 0; beaconCurrTime = 0;
    turnOldTime = 0; turnCurrTime = 0;
    motionCurrTime = 0; motionOldTime = 0;

}

//-----------------------------BEACON-----------------------------------

void BeaconTrackInit(void)
{
    pinMode(Pin_Beacon1, INPUT);
    pinMode(Pin_Beacon2, INPUT);

    // pinMode(Pin_BeaconServo, OUTPUT);

    // ledcSetup(0, PWM_FREQ_SERVO, PWM_RES_SERVO); //channel 0 for servo
    // ledcAttachPin(Pin_BeaconServo, 0);
    // ledcWrite(0, ((1<<PWM_RES_SERVO)-1)*(BEACON_SERVO_FWD)/100);
}
void CheckFrequencyBeacon1(void)
{
    static int b1OldLevel;
    static uint32_t b1OldTime;
    int freq;

    if ((digitalRead(Pin_Beacon1))!=b1OldLevel)
    {
        freq=1000000/(beaconCurrTime-b1OldTime)/2; //1000000us/us=Hz then /2
        // Serial.print("freq="); Serial.println(freq);
        freq=MovingAverageFilter1(freq);

        if (freq>(BEACON_FREQ-NOISE_TLRNC) && freq<(BEACON_FREQ+NOISE_TLRNC)) //if freq is in the window
        {
            b1IsSeen = true;
            if (bitRead(state, FORWARD) && bitRead(state, BEACON_TRACK)){
                beaconBalance+=1.2;
            }
        }
        else
        {
            b1IsSeen = false;
        }

        b1OldLevel=digitalRead(Pin_Beacon1);
        b1OldTime=beaconCurrTime;
    }
}
void CheckFrequencyBeacon2(void) //ch1
{
    static int b2OldLevel;
    static uint32_t b2OldTime;
    int freq;

    if ((digitalRead(Pin_Beacon2))!=b2OldLevel)
    {
        freq=1000000/(beaconCurrTime-b2OldTime)/2;
        freq=MovingAverageFilter2(freq);

        if (freq>(BEACON_FREQ-NOISE_TLRNC) && freq<(BEACON_FREQ+NOISE_TLRNC)) //if freq is in the window
        {
            b2IsSeen = true;
            if (bitRead(state, FORWARD) && bitRead(state, BEACON_TRACK)){
                beaconBalance--;
            }
        }
        else
        {
            b2IsSeen = false;
        }

        b2OldLevel=digitalRead(Pin_Beacon2);
        b2OldTime=beaconCurrTime;
    }
}
int MovingAverageFilter1(int data)
{
    static int MafCnt1;
    static int arr1[MAF_SAMPLE_SIZE]={0};
    uint8_t k=MafCnt1%MAF_SAMPLE_SIZE;
    int sum=0;
    int avg;

    arr1[k]=data;
    // printf("arr1= ");
    for(int i=0; i<MAF_SAMPLE_SIZE; i++)
    {
        // printf("%d ", arr1[i]);
        sum=sum+arr1[i];
    }
    avg=sum/MAF_SAMPLE_SIZE;

    MafCnt1++;

    return avg;
}
int MovingAverageFilter2(int data)
{
    static int MafCnt2;
    static int arr2[MAF_SAMPLE_SIZE]={0};
    uint8_t k=MafCnt2%MAF_SAMPLE_SIZE;
    int sum=0;
    int avg;

    arr2[k]=data;
    for(int i=0; i<MAF_SAMPLE_SIZE; i++)
    {
        sum=sum+arr2[i];
    }
    avg=sum/MAF_SAMPLE_SIZE;

    MafCnt2++;

    return avg;
}

//--------------------------COMMUNICATION------------------------------

// Serial
void SerialInit(void)
{
    Serial.begin(115200);
}

// UDP
void UdpInit(void)
{
    myUdpServer.begin(ROBOT_UDP_PORT); // strange bug needs to come after WiFi.begin but before connect
    canUdpServer.begin(CAN_UDP_PORT);
    robotUdpServer.begin(ROBOT_UDP_PORT);
}
void UdpSend(char *datastr, int len)
{
    myPosStr[12] = 0;
    myUdpServer.beginPacket(ipTarget, ROBOT_UDP_PORT);
    myUdpServer.write((uint8_t *)datastr, len);
    myUdpServer.endPacket();
}
void UdpReceiveCanMsg(void)
{
    uint8_t packetBuffer[UDP_PACKET_SIZE];
    int cb=canUdpServer.parsePacket();

    if (cb)
    {
        int x, y, num;
        packetBuffer[cb]=0; // null terminate string
        canUdpServer.read(packetBuffer, UDP_PACKET_SIZE);

        num=atoi((char *)packetBuffer+0); //????????
        x=atoi((char *)packetBuffer+2);
        y=atoi((char *)packetBuffer+7);

        Serial.print("From Can ");
        Serial.println((char *)packetBuffer);
        // Serial.println(num); 
        // Serial.println(x); 
        // Serial.println(y);

        canPos[num-1][0] = x;
        canPos[num-1][1] = y;

        // if (num<robotId & num>=1)
        // {
        //     canPos[num-1][0]=x;
        //     canPos[num-1][1]=y;
        // }
        // else if (num>robotId && num<=8)
        // {
        //     canPos[num-2][0]=x;
        //     canPos[num-2][1]=y;
        // }
    }
}
void UdpReceiveRobotMsg(void)
{
    uint8_t packetBuffer[UDP_PACKET_SIZE];
    int cb=robotUdpServer.parsePacket();
    
    if (cb)
    {
        int x, y, num;
        packetBuffer[cb]=0; // null terminate string
        robotUdpServer.read(packetBuffer, UDP_PACKET_SIZE);

        num=atoi((char *)packetBuffer+0); //????????
        x=atoi((char *)packetBuffer+2);
        y=atoi((char *)packetBuffer+7);
        Serial.print("From Robot ");
        Serial.println((char *)packetBuffer);
        // Serial.println(num);
        // Serial.println(x);
        // Serial.println(y);

        if (num<robotId & num>=1)
        {
            otherRobotPos[num-1][0]=x;
            otherRobotPos[num-1][1]=y;
        }
        else if (num>robotId && num<=4)
        {
            otherRobotPos[num-2][0]=x;
            otherRobotPos[num-2][1]=y;
        }
    }
}

// Register robot ID based on physical switch input on the robot
void DipInit(void)
{
    pinMode(Pin_Dip1, INPUT);
    pinMode(Pin_Dip2, INPUT);
    pinMode(Pin_Dip3, INPUT);
    pinMode(Pin_Dip4, INPUT);

    if (digitalRead(Pin_Dip1))
    {
        Serial.println("Robot id is 1!");
        robotId=1;
    }
    else if (digitalRead(Pin_Dip2))
    {
        Serial.println("Robot id is 2!");
        robotId=2;
    }
    else if (digitalRead(Pin_Dip3))
    {
        Serial.println("Robot id is 3!");
        robotId=3;
    }
    else if (digitalRead(Pin_Dip4))
    {
        Serial.println("Robot id is 4!");
        robotId=4;
    }
    else
    {
        Serial.println("Robot id unassigned! Default to 1");
        robotId = 1;
    }
}

//----------------------------MOBILITY---------------------------------

void ServoInit(void)
{
    pinMode(Pin_Servo1, OUTPUT); // servo 1 - left
    pinMode(Pin_Servo2, OUTPUT); // servo 2 - right

    ledcSetup(SERVO1_LEDC_CH, SERVO_LEDC_FREQ, SERVO_LEDC_RES_BITS); // channel 0
    ledcSetup(SERVO2_LEDC_CH, SERVO_LEDC_FREQ, SERVO_LEDC_RES_BITS); // channel 1
    
    // RobotMove(0, 0); //throttle=0, stop
}
void RobotMove(int servo1throt, int servo2throt) // Ledc Setup + Execute
{
    ledcAttachPin(Pin_Servo1, SERVO1_LEDC_CH);
    ledcAttachPin(Pin_Servo2, SERVO2_LEDC_CH);
    ledcWrite(SERVO1_LEDC_CH, ((1<<SERVO_LEDC_RES_BITS)-1)*(7.5+servo1throt*0.05)/100); // servo 1
    ledcWrite(SERVO2_LEDC_CH, ((1<<SERVO_LEDC_RES_BITS)-1)*(7.5-servo2throt*0.05)/100); // servo 2, change the direction!
    // Serial.print("servo1throt="); Serial.println(servo1throt);
    // Serial.print("servo12throt="); Serial.println(servo2throt);

}
void RobotStop(void) // Ledc Setup + Execute
{
    ledcDetachPin(Pin_Servo1);
    ledcDetachPin(Pin_Servo2);
    // Serial.println("Robot STOP!");
}

//--------------------------WIFI AND HTML------------------------------

// WiFi setup & Web Handler function
void WifiInit(void)
{
    Serial.print("Connecting to ");  Serial.println(ssid);

#if WIFI_MODE == 1 // Access Point Mode + Set IP
    WiFi.softAP(ssid);
    WiFi.softAPConfig(IPAddress(192, 168, 1, 209), 
                      IPAddress(192, 168, 1, 1), 
                      IPAddress(255, 255, 255, 0)); // 209, Mingyan Zhou's IP address
    IPAddress myIP=WiFi.softAPIP();
    Serial.print("AP IP address: ");  Serial.println(myIP);
#elif WIFI_MODE == 2 // Station Mode + DHCP + Hotspot
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED)
    {
        delay(500);
        Serial.print(".");
    }
    Serial.print("Use this URL to connect: http://");
    Serial.print(WiFi.localIP()); Serial.println("/");
    Serial.println("WiFi connected!");
#elif WIFI_MODE == 3 // Station Mode + DHCP + Router
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED)
    {
        delay(500);
        Serial.print(".");
    }
    Serial.print("Use this URL to connect: http://");
    Serial.print(WiFi.localIP()); Serial.println("/");
    Serial.println("WiFi connected!");
#elif WIFI_MODE == 4 // Station Mode + Set IP
    WiFi.config(IPAddress(192, 168, 0, 209), 
                IPAddress(192, 168, 1, 1), 
                IPAddress(255, 255, 254, 0)); // 209, Mingyan Zhou's IP address
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED)
    {
        delay(500);
        Serial.print(".");
    }
    Serial.print("\n");
#endif

    Serial.println("WiFi connected!");

    h.begin();
    h.attachHandler("/ ", handleRoot);
    h.attachHandler("/press1", handler_Press1_Forward);
    h.attachHandler("/press2", handler_Press2_Backward);
    h.attachHandler("/press3", handler_Press3_Left);
    h.attachHandler("/press4", handler_Press4_Right);
    h.attachHandler("/release", handler_Press_Release);
    h.attachHandler("/click5", handler_Click5_Grip);
    h.attachHandler("/click6", handler_Click6_Release);
    h.attachHandler("/click7", handler_Click7_Wall);
    h.attachHandler("/click8", handler_Click8_Beacon);
    h.attachHandler("/click9", handler_Click9_Can);
    h.attachHandler("/click10", handler_Click10_Control);
}
void handleRoot(void)
{
     h.sendhtml(body);
}
void handler_Press1_Forward(void)
{
    if (bitRead(state, TELEOP))
    {
        Serial.println("Press Button 1, Moving Forward!");
        RobotMove(+50, +50);
    }
    else
    {
        Serial.println("Press Button 1 But Robot Not In Teleop!");
    }

    AutoLedOff();

    h.sendplain(""); // acknowledge         
}
void handler_Press2_Backward(void)
{
    if (bitRead(state, TELEOP))
    {
        Serial.println("Press Button 2, Moving Backward!");
        RobotMove(-50, -50);
    }
    else 
    {
        Serial.println("Press Button 2 But Robot Not In Teleop!");
    }

    AutoLedOff();
    
    h.sendplain(""); // acknowledge   
}
void handler_Press3_Left(void)
{
    if (bitRead(state, TELEOP))
    {
        Serial.println("Press Button 3, Moving Left!");
        RobotMove(-10, +10);
    }
    else
    {
        Serial.println("Press Button 3 But Robot Not In Teleop!");
    }

    AutoLedOff();

    h.sendplain(""); // acknowledge  
}
void handler_Press4_Right(void)
{
    if (bitRead(state, TELEOP))
    {
        Serial.println("Press Button 4, Moving Right!");
        RobotMove(+10, -10);
    }
    else
    {
        Serial.println("Press Button 4 But Robot Not In Teleop!");
    }

    AutoLedOff();

    h.sendplain(""); // acknowledge 
}
void handler_Press_Release(void)
{
    if (bitRead(state, TELEOP))
    {
        Serial.println("STOP!");
        RobotStop();
    }
    else
    {
        Serial.println("Press STOP But Robot Not In Teleop!");
    }

    AutoLedOff();

    h.sendplain(""); // acknowledge 
}
void handler_Click5_Grip(void)
{
    Serial.println("Claw grip!");

    ClawCatch();

    AutoLedOff();

    h.sendplain(""); // acknowledge 
}
void handler_Click6_Release(void)
{
    Serial.println("Claw release!");

    ClawRelease();

    AutoLedOff();

    h.sendplain(""); // acknowledge 
}
void handler_Click7_Wall(void)
{
    Serial.println("Wall following!");
    
    ResetAutoBehavior();
    bitSet(state, WALL_FOLLOW);

    AutoLedOff();

    h.sendplain(""); // acknowledge 
}
void handler_Click8_Beacon(void)
{
    Serial.println("Beacon tracking!");

    ResetAutoBehavior();
    bitSet(state, BEACON_TRACK);

    AutoLedOff();

    h.sendplain(""); // acknowledge 
}
void handler_Click9_Can(void)
{
    Serial.println("Can catching!");

    ResetAutoBehavior();
    bitSet(state, GOTO_CAN);

    AutoLedOff();

    h.sendplain(""); // acknowledge 
}
void handler_Click10_Control(void)
{
    Serial.println("Going to teleop!");

    ResetAutoBehavior();
    bitSet(state, TELEOP);

    AutoLedOff();

    h.sendplain(""); // acknowledge 
}

// wireless package green LED
void AutoLedInit(void)
{
    pinMode(Pin_GreenLed, OUTPUT);

    autoOldTime=millis();
}
void AutoLedOn(void)
{
    digitalWrite(Pin_GreenLed, HIGH);
}
void AutoLedOff(void)
{
    digitalWrite(Pin_GreenLed, LOW);

    autoOldTime=millis();
}

//--------------------------LOCALIZATION--------------------------------

void ViveInit(void)
{
    // Serial.println("Vive trackers start!");
    vive1.begin();
    vive2.begin();
}

// Onboard Vive receive and send position
void ViveReceiveAndUdpSendMyPos(void)
{
    if (vive1.status() == VIVE_LOCKEDON)
    {
        // Serial.printf("X %d, Y %d\n",vive1.xCoord(),vive1.yCoord());
        myPos[0]=MovingAverageFilterMyPos0(vive1.xCoord());
        myPos[1]=MovingAverageFilterMyPos1(vive1.yCoord());
        // myPos[0]=vive1.xCoord();
        // myPos[1]=vive1.yCoord();
        getRotationalCenter();

        // store into a string with format #:####,####, which is robotid, x, y
        sprintf(myPosStr, "%1d:%4d,%4d", robotId, (int) myPos[0], (int) myPos[1]); 
        UdpSend(myPosStr, UDP_PACKET_SIZE-1);
        // Serial.print("sending data: "); Serial.println(myPosStr);
        // Serial.printf("Vive 1 X %d, Y %d\n",vive1.xCoord(),vive1.yCoord());
    }
    else
    {
        vive1.sync(15); // try to resync (nonblocking);
    }

    if (vive2.status() == VIVE_LOCKEDON)
    {
        // Serial.printf("Vive 2 X %d, Y %d\n",vive2.xCoord(),vive2.yCoord());
        myPos[2]=MovingAverageFilterMyPos2(vive2.xCoord());
        myPos[3]=MovingAverageFilterMyPos3(vive2.yCoord());
        // myPos[2]=vive2.xCoord();
        // myPos[3]=vive2.yCoord();
        getRotationalCenter();
    }
    else
    {
        vive2.sync(15); // try to resync (nonblocking);
    }

}
// Moving Average Filter for my vive 1/2 x/y
double MovingAverageFilterMyPos0(double rawpos)
{
    static int pos0MafCnt;
    int k;
    static double rawpos0[POS_MAF_SAMPLE_SIZE]={0};
    double sum=0;
    static double avgpos=0;

    if (rawpos > 0)
    {
        k=pos0MafCnt%POS_MAF_SAMPLE_SIZE;
        rawpos0[k]=rawpos;

        // printf("arr= ");
        for(int i=0; i<POS_MAF_SAMPLE_SIZE; i++)
        {
            // printf("%.2f ", rawpos0[i]);
            sum=sum+rawpos0[i];
        }
        // printf("\n"); 

        avgpos=sum/POS_MAF_SAMPLE_SIZE;
        pos0MafCnt++;
    }

    return avgpos;
}
double MovingAverageFilterMyPos1(double rawpos)
{
    static int pos1MafCnt;
    int k;
    static double rawpos1[POS_MAF_SAMPLE_SIZE]={0};
    double sum=0;
    static double avgpos=0;

    if (rawpos > 0)
    {
        k=pos1MafCnt%POS_MAF_SAMPLE_SIZE;
        rawpos1[k]=rawpos;

        // printf("arr= ");
        for(int i=0; i<POS_MAF_SAMPLE_SIZE; i++)
        {
            // printf("%.2f ", rawpos1[i]);
            sum=sum+rawpos1[i];
        }
        // printf("\n"); 

        avgpos=sum/POS_MAF_SAMPLE_SIZE;
        pos1MafCnt++;
    }

    return avgpos;
}
double MovingAverageFilterMyPos2(double rawpos)
{
    static int pos2MafCnt;
    int k;
    static double rawpos2[POS_MAF_SAMPLE_SIZE]={0};
    double sum=0;
    static double avgpos=0;
    
    if (rawpos > 0)
    {
        k=pos2MafCnt%POS_MAF_SAMPLE_SIZE;
        rawpos2[k]=rawpos;
        // printf("arr= ");

        for(int i=0; i<POS_MAF_SAMPLE_SIZE; i++)
        {
            // printf("%.2f ", rawpos2[i]);
            sum=sum+rawpos2[i];
        }

        // printf("\n"); 
        avgpos=sum/POS_MAF_SAMPLE_SIZE;
        pos2MafCnt++;
    }

    return avgpos;
}
double MovingAverageFilterMyPos3(double rawpos)
{
    static int pos3MafCnt;
    int k;
    static double rawpos3[POS_MAF_SAMPLE_SIZE]={0};
    double sum=0;
    static double avgpos=0;

    if (rawpos > 0)
    {
        k=pos3MafCnt%POS_MAF_SAMPLE_SIZE;
        rawpos3[k]=rawpos;

        // printf("arr= ");
        for(int i=0; i<POS_MAF_SAMPLE_SIZE; i++)
        {
            // printf("%.2f ", rawpos3[i]);
            sum=sum+rawpos3[i];
        }
        // printf("\n"); 

        avgpos=sum/POS_MAF_SAMPLE_SIZE;
        pos3MafCnt++;
    }

    return avgpos;
}

// compute robot rotational center using vive positions
void getRotationalCenter(void)
{
    myPos[4] = (myPos[0] + myPos[2]) / 2.0;
    myPos[5] = (myPos[1] + myPos[3]) / 2.0;
}

// void GoToCanAngleDist(int canIdx)
// {
//     double angledist[] = {0.0, 0.0};
//     GetAngleDistanceTowardCan(canPos[canIdx][0], canPos[canIdx][1], angledist);
//     canAngle = angledist[0];
//     canDistance = angledist[1];
//     // // robot turn angle 
//     // canAngleTime = AngleToTime(canAngle);
//     // canDistanceTime = DistanceToTime(canDistance);
// }

// float DistanceToTime(float distance)
// {
//   // TODO: Calculate timer in millis for robot to reach certain distance
// }
// float AngleToTime(float angle)
// {
//   // TODO: Calculate timer in millis for robot to turn certain angle
// }

// NOTE: THIS FUNCTION USES myPos[0] and myPos[1] as true position of the car
void GetAngleDistanceTowardCan(int canIdx)
{
    int canx = canPos[canIdx][0];
    int cany = canPos[canIdx][1];
    double vx1 = myPos[2] - myPos[0]; // car vector x
    double vy1 = myPos[3] - myPos[1]; // car vector y
    double vx2 = canx - myPos[4]; // car to can vector x
    double vy2 = cany - myPos[5]; // car to can vector y
    double dot = vx1*vx2 + vy1*vy2; // dot product
    double det = vx1*vy2 - vy1*vx2; // determinant
    canAngle = atan2(det, dot); // from vector to angle in 2D case
    canDistance = sqrt(pow(vx2, 2) + pow(vy2, 2));

    // unit vector of car direction
    double unitDirX = vx1 / sqrt(pow(vx1, 2) + pow(vy1, 2));
    double unitDirY = vy1 / sqrt(pow(vx1, 2) + pow(vy1, 2));
    // update center of gripper position
    myPos[6] = myPos[4] + CENTER_TO_GRIPPER_LENGTH * unitDirX;
    myPos[7] = myPos[5] + CENTER_TO_GRIPPER_LENGTH * unitDirY;
}
double AverageFilterCanAngle(double rawangle)
{
    static double anglearr[CANANGLE_FILTER_LENGTH]={0};
    static int anglecnt=0;
    double sum=0;
    static double avgangle=0;

    anglearr[anglecnt]=rawangle;
    
    if (anglecnt == CANANGLE_FILTER_LENGTH-1)
    {
        for (int i=0; i<CANANGLE_FILTER_LENGTH; i++)
        {
            sum+=anglearr[i];
        }
        avgangle=sum/10.0;
        anglecnt=0;
    }
    else
    {
        anglecnt++;
    }
    return avgangle;
}

bool CheckReachCan(int canIdx)
{
    double distanceToGoal = sqrt(pow(canPos[canIdx][0] - myPos[6], 2) + pow(canPos[canIdx][1] - myPos[7], 2));
    if (distanceToGoal < CAN_REACH_RADIUS)
    {
        return true;
    } else {
        return false;
    }
}

//--------------------------WALL FOLLOWING-----------------------------

void IRInit(void)
{
    pinMode(Pin_FrontIR, INPUT);
    pinMode(Pin_RightIR, INPUT);
}

void wallFollow(void)
{
    
    if (ri_cerror < 0)
    {
        // turn left, right wheel has pd control
        RobotMove(BASE_SPEED, BASE_SPEED + abs(kpR * ri_cerror) - abs(kdR * ri_derror));
    }
    else if (ri_cerror > 0)
    {
        // turn right, left wheel has pd control
        RobotMove(BASE_SPEED + abs(kpL * ri_cerror) - abs(kdL * ri_derror), BASE_SPEED);
    }
    else
    {
        RobotMove(BASE_SPEED, BASE_SPEED);
    }
}
void updateIR(void) // Sensor reading interrupt function
{
    frontIR = frontIRConvert(frontIrAdc);
    rightIR = rightIRConvert(rightIrAdc);

    if (frontIR > MAX_DIST)
    {
        frontIR = MAX_DIST;
    }
    if (rightIR > MAX_DIST)
    {
        rightIR = MAX_DIST;
    }

    if (frontIR < MIN_DIST)
    {
        frontIR = MIN_DIST;
    }
    if (rightIR < MIN_DIST)
    {
        rightIR = MIN_DIST;
    }

    if (frontIR <= TRIG_DIST)
    {
        obFront = true;
    }
    else
    {
        obFront = false;
    }

    // update error
    ri_curr = rightIR;
    if (ri_curr > irMax)
    {
        ri_cerror = -irMax + ri_curr;
    }
    else if ((ri_curr < irMin))
    {
        ri_cerror = -irMin + ri_curr;
    }
    else
    {
        ri_cerror = 0;
    }

    // derivative error
    ri_derror = ri_cerror - ri_perror;
    ri_perror = ri_cerror;

    // Serial.print("obFront: "); Serial.println(obFront);
}

// convert front and right IR reading to real distance
double rightIRConvert(double num)
{
    double output = 17.355 * exp(-0.0005 * num);
    return output;
}
double frontIRConvert(double num)
{
    double output = 18.199 * exp(-0.0005 * num);
    return output;
}

//-------------------------------CLAW----------------------------------

void ClawInit(void)
{
    pinMode(Pin_ServoClaw, OUTPUT);

    ledcSetup(CLAW_LEDC_CH, CLAW_LEDC_FREQ, CLAW_LEDC_RES_BITS); // channel 2
    ledcAttachPin(Pin_ServoClaw, CLAW_LEDC_CH);
}
void ClawCatch(void)
{
    ledcWrite(CLAW_LEDC_CH, ((1<<SERVO_LEDC_RES_BITS)-1)*0.05); //duty cycle of catching: 5%
}
void ClawRelease(void)
{
    ledcWrite(CLAW_LEDC_CH, ((1<<SERVO_LEDC_RES_BITS)-1)*0.092); //duty cycle of catching: 5%
}

// TODO: drive forward given a distance in inches, need to calculate delay time (or timer) to distance ratio

// TODO: turn a small degree left, calculate delay time to angle function
