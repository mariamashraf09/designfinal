#include <ESP32Servo.h>
#include <WiFi.h>
#include <WiFiUdp.h>

#define limitUP 4
#define limitBASE 18

const char* ssid = "AndroidAP999F";
const char* password = "111111111";

WiFiUDP udp;
unsigned int localUdpPort = 5005;
char incomingPacket[255];

bool homing1 = 0;
bool homing2 = 0;
bool qrConfirmed = 0;

int currentTarget = 0;
const int numTargets = 10;

Servo servo1; //end effector
Servo servo2; //joint 3

//target positions
//towards motor -ve 
long targetPositions1[10] = {((2 * 3828 * 75) / 360) , ((2 * 3828 * 75) / 360) , ((2 * 3828 * 75) / 360),((2 * 3828 * 75) / 360),((2 * 3828 * 75) / 360),((2 * 3828 * 80) / 360),((2 * 3828 * 62) / 360),((2 * 3828 * 20) / 360),((2 * 3828 * 10) / 360),((2 * 3828 * 20) / 360)};
//cw +ve , ccw -ve
long targetPositions2[10] = {-1095 , -1235 , -1300 , -1325 , -1215 , -1225 , -1560 , -350,-350,-400}; //-1097 , -1230 , 
String targetQRs[10] = {"Target1","Target2","Target3","Target4", "Target5" , "Target6" , "Target7","Target8","Target9","Target10"};
int servoAngles1[10] = {140,65,64,140,170,100,65,30, 160,30};
int servoAngles2[10] = {67,70,70,80,60,60,10,0,110,0};

enum State { MOVE_DC_MOTORS, MOVE_SERVOS, IDLE };
State state = MOVE_DC_MOTORS;

//MOTOR upper
float Kp1 = 0.056, Ki1 = 0.02, Kd1 = 0.0;
float previousTime1 = 0, previousError1 = 0, errorIntegral1 = 0;
float currentTime1 = 0, deltaTime1 = 0, errorValue1 = 0, edot1 = 0, controlSignal1 = 0;
float Vmax1 = 12.0, V1 = 0;
const int pwmChannel1 = 4;
const int pwmFreq = 5000;
const int pwmResolution = 8;
const byte EnaA1 = 14, IN1_1 = 12, IN2_1 = 13;
const byte interruptPinA_1 = 23, interruptPinB_1 = 5;
volatile long motorPosition1 = 0;
long previousMotorPosition1 = 0; long targetPosition1 = 0.0; long previousTargetPosition1 = 100;

//MOTOR base
float Kp2 = 1.2 , Ki2 = 0.0 , Kd2 = 0.0; //aly zabt kp=0.84 and kd=0.0005
float previousTime2 = 0, previousError2 = 0, errorIntegral2 = 0;
float currentTime2 = 0, deltaTime2 = 0, errorValue2 = 0, edot2 = 0, controlSignal2 = 0;
float Vmax2 = 12.0, V2 = 0;
const int pwmChannel2 = 5;
const byte EnaA2 = 25, IN1_2 = 26, IN2_2 = 27;
const byte interruptPinA_2 = 32, interruptPinB_2 = 33; 
volatile long motorPosition2 = 0;
long previousMotorPosition2 = 0, previousTargetPosition2 = 100, targetPosition2 = 0.0; 

//TIMER
hw_timer_t* timer = NULL;
volatile bool pidFlag = false;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;

//ENCODERS
void IRAM_ATTR encoderISR1() 
{
  bool A = digitalRead(interruptPinA_1);
  bool B = digitalRead(interruptPinB_1);

  if (A == B) {
    motorPosition1++;
  } else {
    motorPosition1--;
  }
}

void IRAM_ATTR encoderISR2() 
{
  bool A = digitalRead(interruptPinA_2);
  bool B = digitalRead(interruptPinB_2);

  if (A == B) {
    motorPosition2++;
  } else {
    motorPosition2--;
  }
}

//PID CALCULATIONS
void calculatePID1(long targetPosition1) 
{
    currentTime1 = millis();
    deltaTime1 = (currentTime1 - previousTime1) / 1000.0;
    previousTime1 = currentTime1;
    errorValue1 = motorPosition1 - targetPosition1;
    edot1 = (errorValue1 - previousError1) / deltaTime1;
    errorIntegral1 += errorValue1 * deltaTime1;
    previousError1 = errorValue1;
    controlSignal1 = (Kp1 * errorValue1) + (Ki1 * errorIntegral1) + (Kd1 * edot1);
    float absSignal1 = constrain(fabs(controlSignal1), 0, Vmax1);
    int pwmValue1 = map((int)absSignal1, 0, (int)Vmax1, 0, 255);
    pwmValue1 = constrain(pwmValue1, 0, 255);
    long currError1 = motorPosition1 - targetPosition1;
    long prevError1 = previousMotorPosition1 - previousTargetPosition1;
    if (abs(currError1) < 5) {
      digitalWrite(IN1_1, HIGH); digitalWrite(IN2_1, HIGH);
      ledcWrite(pwmChannel1, 0);
    } else {
      if (currError1 > 0) {
        digitalWrite(IN1_1, LOW); digitalWrite(IN2_1, HIGH);
      } else {
        digitalWrite(IN1_1, HIGH); digitalWrite(IN2_1, LOW);
      }
      ledcWrite(pwmChannel1, pwmValue1);
    }
    V1 = (pwmValue1 * Vmax1) / 255.0;
    previousMotorPosition1 = motorPosition1;
    previousTargetPosition1 = targetPosition1;
    Serial.print("M1: ");
    Serial.println(((motorPosition1*360)/(2 * 3828))); 
}

void calculatePID2(long targetPosition2) 
{
    currentTime2 = millis();
    deltaTime2 = (currentTime2 - previousTime2) / 1000.0;
    previousTime2 = currentTime2;
    errorValue2 = motorPosition2 - targetPosition2;
    edot2 = (errorValue2 - previousError2) / deltaTime2;
    errorIntegral2 += errorValue2 * deltaTime2;
    previousError2 = errorValue2;
    controlSignal2 = (Kp2 * errorValue2) + (Ki2 * errorIntegral2) + (Kd2 * edot2);
    float absSignal2 = constrain(fabs(controlSignal2), 0, Vmax2);
    int pwmValue2 = map((int)absSignal2, 0, (int)Vmax2, 0, 175);
    pwmValue2 = constrain(pwmValue2, 0, 175);
    Serial.println(pwmValue2);
    long currError2 = motorPosition2 - targetPosition2;
    long prevError2 = previousMotorPosition2 - previousTargetPosition2;
    if (abs(currError2) < 7) {
      digitalWrite(IN1_2, HIGH); digitalWrite(IN2_2, HIGH);
      ledcWrite(pwmChannel2, 0);
    } else {
      if (currError2 > 0) {
        digitalWrite(IN1_2, LOW); digitalWrite(IN2_2, HIGH);
      } else {
        digitalWrite(IN1_2, HIGH); digitalWrite(IN2_2, LOW);
      }
      ledcWrite(pwmChannel2, pwmValue2);
    }
    V2 = (pwmValue2 * Vmax2) / 200.0;
    previousMotorPosition2 = motorPosition2;
    previousTargetPosition2 = targetPosition2;
    Serial.print("M2: ");
    Serial.println(motorPosition2);
}

void homingupper()
{
  //m3 el motor
  digitalWrite(IN1_1, 0); 
  digitalWrite(IN2_1, 1);
  ledcWrite(pwmChannel1, 220);
  delay(300);
}
void homingbase()
{
  //CW
  digitalWrite(IN1_2, 1); 
  digitalWrite(IN2_2, 0);
  ledcWrite(pwmChannel2, 180);
  delay(300);
}
void stopmotorupper()
{
  digitalWrite(IN1_1, LOW); 
  digitalWrite(IN2_1, LOW);
  ledcWrite(pwmChannel1, 0);
}
void stopmotorbase()
{
  digitalWrite(IN1_2, LOW); 
  digitalWrite(IN2_2, LOW);
  ledcWrite(pwmChannel2, 0);
}


//TIMER ISR
void IRAM_ATTR onTimer() {
  portENTER_CRITICAL_ISR(&timerMux);
  pidFlag = true;
  portEXIT_CRITICAL_ISR(&timerMux);
}

//SETUP
void setup() {
  Serial.begin(115200);
 
  // Connect to WiFi
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) 
  {
    delay(1000);
    Serial.println("Connecting to WiFi...");
  }
  Serial.println("Connected to WiFi");
  // Start UDP
  udp.begin(localUdpPort);

  //switches setup
  pinMode(limitUP , INPUT_PULLUP);
  pinMode(limitBASE , INPUT_PULLUP);

  // Attach servos
  servo1.attach(16); 
  servo2.attach(17); //joint3

  // Motor 1 setup
  pinMode(IN1_1, OUTPUT); pinMode(IN2_1, OUTPUT); pinMode(EnaA1, OUTPUT);
  pinMode(interruptPinA_1, INPUT_PULLUP); pinMode(interruptPinB_1, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(interruptPinA_1), encoderISR1, CHANGE);
  ledcSetup(pwmChannel1, pwmFreq, pwmResolution);
  ledcAttachPin(EnaA1, pwmChannel1);

  // Motor 2 setup
  pinMode(IN1_2, OUTPUT); pinMode(IN2_2, OUTPUT); pinMode(EnaA2, OUTPUT);
  pinMode(interruptPinA_2, INPUT_PULLUP); pinMode(interruptPinB_2, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(interruptPinA_2), encoderISR2, CHANGE);
  ledcSetup(pwmChannel2, pwmFreq, pwmResolution);
  ledcAttachPin(EnaA2, pwmChannel2);

  previousTime1 = previousTime2 = millis();

  // Timer setup
  timer = timerBegin(0, 80, true);
  timerAttachInterrupt(timer, &onTimer, true);
  timerAlarmWrite(timer, 10000, true);
  timerAlarmEnable(timer);
  servo1.write(0);
  servo2.write(0);
}

//LOOP
void loop() 
{

    int packetSize = udp.parsePacket();
    if (packetSize) 
    {
      int len = udp.read(incomingPacket, 255);
      if (len > 0) 
      {
        incomingPacket[len] = '\0';  // Null-terminate the string
      }
      String qrData = String(incomingPacket);
      qrData.trim();  // Remove any whitespace or newline characters
      Serial.print("Received QR Code Data: \"");
      Serial.print(qrData);
      Serial.println("\"");
      if (qrData == targetQRs[currentTarget])
      {
        qrConfirmed = true;
        Serial.println("QR code confirmed for current target.");
        Serial.println(qrConfirmed);
      }
    }



    if (homing1 == 0) 
    {
        if (digitalRead(limitUP) == LOW) 
        {
          homing1 = 1;
          Serial.print("H1: ");
          Serial.println(homing1);
          
          motorPosition1 = 0;
          
          stopmotorupper();
        }
        else 
        {
          homingupper(); 
        }
    }

        if (homing2 == 0) 
    {
      if (digitalRead(limitBASE) == LOW) 
      {
        homing2 = 1;
        Serial.print("H2: ");
        Serial.println(homing2);
        motorPosition2 = 0;
        stopmotorbase();
      } 
      else 
      {
        homingbase();
      }
    }
    

    if (pidFlag) {
      portENTER_CRITICAL(&timerMux);
      pidFlag = false;
      portEXIT_CRITICAL(&timerMux);
      if (homing1 == 1 && homing2 == 1) 
      {
        switch (state) 
        {
          case MOVE_DC_MOTORS:
            // Set targets from arrays
            targetPosition1 = targetPositions1[currentTarget];
            targetPosition2 = targetPositions2[currentTarget];

            // Call PID once per loop
            calculatePID1(targetPosition1);
            calculatePID2(targetPosition2);

            if (abs(motorPosition1 - targetPosition1) < 8 && abs(motorPosition2 - targetPosition2) < 8) 
            {
              state = MOVE_SERVOS;
              //delay(500);
            }
          break;

          case MOVE_SERVOS:
            servo1.write(servoAngles1[currentTarget]);
            delay(2000);
            servo2.write(servoAngles2[currentTarget]);
            delay(2000);
          
            if (qrConfirmed)
           {
            currentTarget++;
            qrConfirmed = 0;

            if (currentTarget >= numTargets) 
            {
              state = IDLE;
            } 
            else 
            {
              state = MOVE_DC_MOTORS;
            }
           }
            break;

          case IDLE:
            // Do nothing
            break;
        }
      }
  }
}