#include <WiFi.h>
#include <WebServer.h>
#include <ArduinoJson.h>
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
  server.on("/getHeading", handle_getHeading);
  server.on("/speed", handle_setThrusters);
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

void handle_setThrusters()
{
  if (server.hasArg("plain")) {
    StaticJsonDocument<200> doc;
    DeserializationError error = deserializeJson(doc, server.arg("plain"));

    if (!error) {
      int leftSpeed = doc["left"];
      int rightSpeed = doc["right"];

      // Control motors based on trigger values
      setMotorSpeed(M1_Left, M1_Right, leftSpeed);
      setMotorSpeed(M2_Left, M2_Right, rightSpeed);

      server.send(200, "text/plain", "OK");
    } else {
      server.send(400, "text/plain", "Invalid JSON");
    }
  } else {
    server.send(400, "text/plain", "No data received");
  }
}

void setMotorSpeed(int fwdPin, int bwdPin, int speed) {
  if (speed >= 0) {
    analogWrite(fwdPin, speed);  // Forward
    analogWrite(bwdPin, 0);      // No backward
  } else {
    analogWrite(fwdPin, 0);          // No forward
    analogWrite(bwdPin, -speed);     // Backward (convert negative to positive)
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
