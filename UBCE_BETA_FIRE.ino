//IP Address: 192.168.1.105

#include <WiFi.h>
#include <WebServer.h>
#include <ArduinoJson.h>
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <analogWrite.h>

const int potPin = 15;
int potValue = 0;

//LEDS
#define red_led 35
#define blue_led 34 

//sensor
#define sensor 4
int sensorValue = 0;

//OLED DISPLAY
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels

// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
#define OLED_RESET     -1 // Reset pin # (or -1 if sharing Arduino reset pin)
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

//MOTOR 1
#define MOTOR_A1_PIN 25  //Driver Board IN1
#define MOTOR_B1_PIN 26  //Driver Board IN2
#define PWM_MOTOR_1 33   //Driver Board PWM
//MOTOR 2
#define MOTOR_A2_PIN 12  //Driver Board IN1
#define MOTOR_B2_PIN 14  //Driver Board IN2
#define PWM_MOTOR_2 13   //Driver Board PWM

#define STBY 27

//ENCODER
int hits = 0;
int hits2 = 0;
float wheel_radius = 0.044;
volatile unsigned int current_time;
long time_interval = 100; //how often do you want to know velocity (milliseconds)
float velocity_r = 0; //this is the velocity in length units / time_interval
float velocity_l = 0;

//PID
int clockwise_r = 0;
int clockwise_l = 0;
float e_speed_r = 0;
float e_speed_l = 0;
float velocity_set_right = 0;
float velocity_set_left = 0;
int pwm_pulse_r = 0;
int pwm_pulse_l = 0;
float e_speed_sum_r = 0;
float e_speed_sum_l = 0;
float e_speed_pre_r = 0;
float e_speed_pre_l = 0;
int usSpeed_r = 0;
int usSpeed_l = 0;
float kp = 5;
float ki = 0.5;
float kd = 0.5;

WebServer server;
//uint8_t pin_led = 2;
char* ssid = "Epic_Robotics";
char* password = "elmarlon888";
int value1 = 0;
int value2 = 0;

void setup()
{
  pinMode(red_led, OUTPUT);
  pinMode(blue_led, OUTPUT);
  pinMode(MOTOR_A1_PIN, OUTPUT);
  pinMode(MOTOR_B1_PIN, OUTPUT);
  pinMode(PWM_MOTOR_1, OUTPUT);
  pinMode(MOTOR_A2_PIN, OUTPUT);
  pinMode(MOTOR_B2_PIN, OUTPUT);
  pinMode(PWM_MOTOR_2, OUTPUT);
  pinMode(STBY, OUTPUT);
  pinMode(sensor,INPUT);

  digitalWrite(red_led, HIGH);
  digitalWrite(blue_led, LOW);

  attachInterrupt(18, count, CHANGE);
  attachInterrupt(4, count2, CHANGE);
  current_time = millis();
  
  WiFi.begin(ssid,password);
  Serial.begin(115200);
  while(WiFi.status()!=WL_CONNECTED)
  {
    delay(500);
  }
  if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) { // Address 0x3D for 128x64
    Serial.println(F("SSD1306 allocation failed"));
    for(;;);
  }
  Serial.println("");
  Serial.print("IP Address: ");
  Serial.println(WiFi.localIP());

  server.on("/jblink",jBlinkLED);
  server.on("/kblink", HTTP_POST, postBlink);
  server.onNotFound(handleNotFound);
  server.begin();

}

void loop()
{
  server.handleClient();
  if ( millis() >= current_time + time_interval)
    ang_vel();

  linear_control();
  potValue = analogRead(potPin);
  Serial.println(potValue);
  
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(8,5);
  display.print("UBCE");
  display.print("  -  ");
  display.println("BETA");
  display.println("   ");
  display.print("Estado:");
  display.print("   ");
  display.println("ACTIVO");
  display.println("   ");
  display.print("V.A. Der:");
  display.print("   ");
  display.println(velocity_set_right);
  display.print("V.A. Izq:");
  display.print("   ");
  display.println(velocity_set_left);
  display.print("IP:");
  display.print("   ");
  display.println(WiFi.localIP());
  display.display();
  
  Forward();
}

void jBlinkLED()
{
  String data = server.arg("plain");
  StaticJsonBuffer<200> jBuffer;
  JsonObject& jObject = jBuffer.parseObject(data);
  String del = jObject["pause"];
  String n = jObject["times"];
  value1=n.toInt();
  value2=del.toInt();
  velocity_set_right=value1;
  velocity_set_left=value2;
  server.send(200); 
}

void handleNotFound(){
  server.send(200);
}

void count()
{
 hits++;
}

void count2()
{
 hits2++;
}

void Forward()
{
  digitalWrite(STBY, HIGH);
  if(clockwise_r == 0){
    digitalWrite(MOTOR_A1_PIN, HIGH);
    digitalWrite(MOTOR_B1_PIN, LOW);
    analogWrite(PWM_MOTOR_1, usSpeed_r);
   }else{
    digitalWrite(MOTOR_A1_PIN, LOW);
    digitalWrite(MOTOR_B1_PIN, HIGH);
    analogWrite(PWM_MOTOR_1, usSpeed_r);
   }
  if(clockwise_l == 0){
    digitalWrite(MOTOR_A2_PIN, HIGH);
    digitalWrite(MOTOR_B2_PIN, LOW);
    analogWrite(PWM_MOTOR_2, usSpeed_l);
   }else{
    digitalWrite(MOTOR_A2_PIN, LOW);
    digitalWrite(MOTOR_B2_PIN, HIGH);
    analogWrite(PWM_MOTOR_2, usSpeed_l);
   }
}

void ang_vel()
{
  velocity_r = ((hits*(wheel_radius*((2*PI*100)/12)))/time_interval)/2;//the constant is 2*pi*1000/16
  velocity_l = (hits2*(wheel_radius*((2*PI*100)/12)))/time_interval;//the constant is 2*pi*1000/16
  hits = 0;
  hits2 = 0;
  current_time = millis();
}

void linear_control()
{
  usSpeed_r = abs(round(velocity_set_right));
  if(velocity_set_right < 0){
    clockwise_r = 0;
  }
  else{
    clockwise_r = 1;
  }
  usSpeed_l = abs(round(velocity_set_left));
  if(velocity_set_left < 0){
    clockwise_l = 0;
  }
  else{
    clockwise_l = 1;
  }
}
