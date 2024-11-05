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

TwoWire I2CBNO = TwoWire(0);
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x29, &I2CBNO);

const float Kp = 0.5;
const float Ki = 0.001;
const float Kd = 0.01;

float previousError = 0;
float integral = 0;
unsigned long lastTimePID = 0;
bool isPIDActive = false;

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
	server.on("/stop", handle_stop);
	server.on("/goStraightWithPID", handle_goStraightWithPID);
	server.onNotFound(handle_NotFound);

	server.begin();
	Serial.println("HTTP server started");
}

void loop()
{
	server.handleClient();
}

void handle_OnConnect()
{
	server.send(200, "text/html", PAGE_MAIN);
}

void handle_stop()
{
	stopMotor1();
	stopMotor2();
	isPIDActive = false;
	server.send(200, "text/plain", "The boat stopped running");
}

void stopMotor1()
{
	analogWrite(M1_Left, 0);
	analogWrite(M1_Right, 0);
}

void stopMotor2()
{
	analogWrite(M2_Left, 0);
	analogWrite(M2_Right, 0);
}

void handle_NotFound()
{
	server.send(404, "text/plain", "Not found");
}

void setMotorSpeed(int fwdPin, int bwdPin, int speed)
{
	if (speed >= 0)
	{
		analogWrite(fwdPin, speed); // Forward
		analogWrite(bwdPin, 0);		// No backward
	}
	else
	{
		analogWrite(fwdPin, 0);		 // No forward
		analogWrite(bwdPin, -speed); // Backward (convert negative to positive)
	}
}

void setThrusters(int leftSpeed, int rightSpeed)
{
	setMotorSpeed(M1_Left, M1_Right, leftSpeed);
	setMotorSpeed(M2_Left, M2_Right, rightSpeed);
}

void handle_setThrusters()
{
	if (server.hasArg("plain"))
	{
		StaticJsonDocument<200> doc;
		DeserializationError error = deserializeJson(doc, server.arg("plain"));

		if (!error)
		{
			int leftSpeed = doc["left"];
			int rightSpeed = doc["right"];

			// Control motors based on trigger values
			setThrusters(leftSpeed, rightSpeed);
			server.send(200, "text/plain", "OK");
		}
		else
		{
			server.send(400, "text/plain", "Invalid JSON");
		}
	}
	else
	{
		server.send(400, "text/plain", "No data received");
	}
}

float getHeading()
{
	sensors_event_t event;
	bno.getEvent(&event);
	float heading = event.orientation.x;
	return heading;
}

void handle_getHeading()
{
	int heading = (int)getHeading();
	String jsonResponse = "{\"heading\": " + String(heading) + "}";
	server.send(200, "application/json", jsonResponse);
}

void handlePIDControl(float targetHeading)
{
	isPIDActive = true;
	while (isPIDActive)
	{
		// Get the current heading
		float currentHeading = getHeading();

		// Desired heading (initial heading when the command is given)
		// static float targetHeading = currentHeading; // Keep the initial heading for reference

		// PID calculations
		float error = targetHeading - currentHeading;

		// Normalize the error to the range [-180, 180] to handle wrap-around
		if (error > 180)
		{
			error -= 360;
		}
		else if (error < -180)
		{
			error += 360;
		}

		// Compute PID output
		unsigned long currentTime = millis();
		float deltaTime = (currentTime - lastTimePID) / 1000.0; // in seconds
		lastTimePID = currentTime;

		integral += error * deltaTime;
		float derivative = (error - previousError) / deltaTime;
		previousError = error;

		float output = Kp * error + Ki * integral + Kd * derivative;

		// Adjust motor speeds based on the PID output
		int baseSpeed = 200;									 // base motor speed
		int speedMotor1 = constrain(baseSpeed + output, 0, 255); // left motor speed
		int speedMotor2 = constrain(baseSpeed - output, 0, 255); // right motor speed

		// Move both motors forward with adjusted speeds
		setThrusters(speedMotor1, speedMotor2);

		Serial.print("Heading: ");
		Serial.print(currentHeading);
		Serial.print("| Target Heading: ");
		Serial.print(targetHeading);
		Serial.print(" | Error: ");
		Serial.print(error);
		Serial.print(" | Motor1 Speed: ");
		Serial.print(speedMotor1);
		Serial.print(" | Motor2 Speed: ");
		Serial.println(speedMotor2);
		server.handleClient();

		// If a stop request has been processed, break out of the loop
		if (!isPIDActive)
		{
			break; // Exit the PID loop when stop is called
		}
	}
}

void handle_goStraightWithPID()
{
	float targetHeading = getHeading();
	lastTimePID = millis();
	handlePIDControl(targetHeading);
	server.send(200, "text/plain", "PID Control started");
	Serial.println("CHECK");
	previousError = 0;
}
