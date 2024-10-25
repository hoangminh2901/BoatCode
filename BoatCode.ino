#include <WiFi.h>
#include <WebServer.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <WebPage.h>
#define I2C_SDA 21
#define I2C_SCL 22

const char *ssid = "Meng's Boat";
const char *password = "1234567890";

IPAddress local_ip(192, 168, 43, 120);
IPAddress gateway(192, 168, 43, 120);
IPAddress subnet(255, 255, 255, 0);

WebServer server(80);

int M1_Left = 26;
int M1_Right = 25;
int M2_Left = 12;
int M2_Right = 13;

unsigned long motor1StartTime = 0;
unsigned long motor2StartTime = 0;
const unsigned long motorDuration = 10000;

bool isMotor1Running = false;
bool isMotor2Running = false;

TwoWire I2CBNO = TwoWire(0);
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x29, &I2CBNO);

void setup()
{
  Serial.begin(115200);
  I2CBNO.begin(I2C_SDA, I2C_SCL);
  while (!bno.begin())
  {
    Serial.println("IMU not found!");
    delay(1000);
  }

  pinMode(M1_Left, OUTPUT);
  pinMode(M1_Right, OUTPUT);
  pinMode(M2_Left, OUTPUT);
  pinMode(M2_Right, OUTPUT);

  WiFi.softAP(ssid, password);
  WiFi.softAPConfig(local_ip, gateway, subnet);
  Serial.println("WiFi AP started");
  Serial.print("IP Address: ");
  Serial.println(WiFi.softAPIP());

  server.on("/", handle_OnConnect);
  server.on("/stop", handle_stop);
  server.on("/getHeading", handle_getHeading);
  server.on("/motor1On", handle_motor1On);
  server.on("/motor2On", handle_motor2On);
  server.on("/bothMotorsOn", handle_bothMotorsOn);
  server.onNotFound(handle_NotFound);

  server.begin();
  Serial.println("HTTP server started");
}

void loop()
{
  server.handleClient();
  unsigned long currentTime = millis();
  if (isMotor1Running && (currentTime - motor1StartTime >= motorDuration))
  {
    stopMotor1();
    isMotor1Running = false;
  }
  if (isMotor2Running && (currentTime - motor2StartTime >= motorDuration))
  {
    stopMotor2();
    isMotor2Running = false;
  }
}

void handle_OnConnect()
{
  server.send(200, "text/html", PAGE_MAIN);
}

void handle_stop()
{
  stopMotor1();
  stopMotor2();
  server.send(200, "text/plain", "The submarine stopped running");
}

void handle_NotFound()
{
  server.send(404, "text/plain", "Not found");
}

void handle_motor1On()
{
  moveMotor1(1);
  motor1StartTime = millis();
  isMotor1Running = true;
  server.send(200, "text/plain", "Motor 1 is ON");
}

void handle_motor2On()
{
  moveMotor2(1);
  motor2StartTime = millis();
  isMotor2Running = true;
  server.send(200, "text/plain", "Motor 2 is ON");
}

void handle_bothMotorsOn()
{
  moveMotor1(1);
  moveMotor2(1);
  motor1StartTime = millis();
  motor2StartTime = millis();
  isMotor1Running = true;
  isMotor2Running = true;
  server.send(200, "text/plain", "Both Motors are ON");
}

void moveMotor1(int direction)
{
  if (direction == 1)
  {
    digitalWrite(M1_Left, HIGH);
    digitalWrite(M1_Right, LOW);
  }
  else
  {
    digitalWrite(M1_Left, LOW);
    digitalWrite(M1_Right, HIGH);
  }
}

void moveMotor2(int direction)
{
  if (direction == 1)
  {
    digitalWrite(M2_Left, HIGH);
    digitalWrite(M2_Right, LOW);
  }
  else
  {
    digitalWrite(M2_Left, LOW);
    digitalWrite(M2_Right, HIGH);
  }
}

void stopMotor1()
{
  digitalWrite(M1_Left, LOW);
  digitalWrite(M1_Right, LOW);
}

void stopMotor2()
{
  digitalWrite(M2_Left, LOW);
  digitalWrite(M2_Right, LOW);
}

void handle_getHeading()
{
  sensors_event_t event;
  bno.getEvent(&event);
  int heading = (int)event.orientation.x;
  String jsonResponse = "{\"heading\": " + String(heading) + "}";
  server.send(200, "application/json", jsonResponse);
}
