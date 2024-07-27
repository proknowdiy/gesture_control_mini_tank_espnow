#include <esp_now.h>
#include <WiFi.h>

//Right motor
int rightMotorPin1=16;
int rightMotorPin2=17;
//Left motor
int leftMotorPin1=18;
int leftMotorPin2=19;

// setting pwm properties
const int PWMFreq = 1000; /* 1 KHz */
const int PWMResolution = 8;
const int rightMotorPWMSpeedChannel_1 = 4;
const int rightMotorPWMSpeedChannel_2 = 5;
const int leftMotorPWMSpeedChannel_1 = 6;
const int leftMotorPWMSpeedChannel_2 = 7;

#define SIGNAL_TIMEOUT 1000  // This is signal timeout in milli seconds. We will reset the data if no signal
unsigned long lastRecvTime = 0;

typedef struct PacketData
{
  byte xAxisValue;
  byte yAxisValue;
  //byte switchPressed;
}PacketData;
PacketData receiverData;


// callback function that will be executed when data is received
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) 
{
  if (len == 0)
  {
    return;
  }
  memcpy(&receiverData, incomingData, sizeof(receiverData));
  String inputData ;
  inputData = inputData + "values " + receiverData.xAxisValue + "  " + receiverData.yAxisValue;
  Serial.println(inputData);
  
  int throttle = map( receiverData.yAxisValue, 254, 0, -255, 255);
  int steering = map( receiverData.xAxisValue, 0, 254, 255, -255); 

  //forward
  if(throttle > 30){                      //forward
    ledcWrite(rightMotorPWMSpeedChannel_1, throttle );          //throttle + steering
    ledcWrite(rightMotorPWMSpeedChannel_2, 0);
    ledcWrite(leftMotorPWMSpeedChannel_1, throttle );           //throttle - steering
    ledcWrite(leftMotorPWMSpeedChannel_2, 0);
  }
  else if(throttle < -30){                //backward
    ledcWrite(rightMotorPWMSpeedChannel_1, 0);
    ledcWrite(rightMotorPWMSpeedChannel_2, abs(throttle) );         //abs(throttle) - steering
    ledcWrite(leftMotorPWMSpeedChannel_1, 0);
    ledcWrite(leftMotorPWMSpeedChannel_2, abs(throttle) );         //abs(throttle) + steering
  }
  else if(steering > 30){                 //Right
    ledcWrite(rightMotorPWMSpeedChannel_1, steering);
    ledcWrite(rightMotorPWMSpeedChannel_2, 0);
    ledcWrite(leftMotorPWMSpeedChannel_1, 0);
    ledcWrite(leftMotorPWMSpeedChannel_2, steering);
  }
  else if(steering < -30){                //Left
    ledcWrite(rightMotorPWMSpeedChannel_1, 0);
    ledcWrite(rightMotorPWMSpeedChannel_2, abs(steering));
    ledcWrite(leftMotorPWMSpeedChannel_1, abs(steering));
    ledcWrite(leftMotorPWMSpeedChannel_2, 0);
  }
  else  {
    ledcWrite(rightMotorPWMSpeedChannel_1, 0);
    ledcWrite(rightMotorPWMSpeedChannel_2, 0);
    ledcWrite(leftMotorPWMSpeedChannel_1, 0);
    ledcWrite(leftMotorPWMSpeedChannel_2, 0);
  }
  lastRecvTime = millis();   
}

void setUpPinModes()
{
  //set Right motor as Output
  pinMode(rightMotorPin1,OUTPUT);
  pinMode(rightMotorPin2,OUTPUT);
  //set Left motor as Output
  pinMode(leftMotorPin1,OUTPUT);
  pinMode(leftMotorPin2,OUTPUT);

  //Set up PWM for motor speed control
  ledcSetup(rightMotorPWMSpeedChannel_1, PWMFreq, PWMResolution);
  ledcSetup(rightMotorPWMSpeedChannel_2, PWMFreq, PWMResolution);
  ledcSetup(leftMotorPWMSpeedChannel_1, PWMFreq, PWMResolution);  
  ledcSetup(leftMotorPWMSpeedChannel_2, PWMFreq, PWMResolution);

  ledcAttachPin(rightMotorPin1, rightMotorPWMSpeedChannel_1);
  ledcAttachPin(rightMotorPin2, rightMotorPWMSpeedChannel_2);
  ledcAttachPin(leftMotorPin1, leftMotorPWMSpeedChannel_1); 
  ledcAttachPin(leftMotorPin2, leftMotorPWMSpeedChannel_2); 
  
  //default motor speed
  ledcWrite(rightMotorPWMSpeedChannel_1, 0);
  ledcWrite(rightMotorPWMSpeedChannel_2, 0);
  ledcWrite(leftMotorPWMSpeedChannel_1, 0);
  ledcWrite(leftMotorPWMSpeedChannel_2, 0);
}


void setup() 
{
  setUpPinModes();
  
  Serial.begin(115200);
  WiFi.mode(WIFI_STA);

  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) 
  {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  esp_now_register_recv_cb(OnDataRecv);
}
 
void loop() 
{
  //Check Signal lost.
  unsigned long now = millis();
  if ( now - lastRecvTime > SIGNAL_TIMEOUT ) 
  {
    //rotateMotor(0, 0);
    ledcWrite(rightMotorPWMSpeedChannel_1, 0);
    ledcWrite(rightMotorPWMSpeedChannel_2, 0);
    ledcWrite(leftMotorPWMSpeedChannel_1, 0);
    ledcWrite(leftMotorPWMSpeedChannel_2, 0);
  }
}