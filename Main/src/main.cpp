// INCLUDES
#include <Arduino.h>
#include <Servo.h>
#include "WiFiS3.h"
#include "ArduinoJson.h"
#include "html.h"
#include <TinyGPS++.h>
#include <SoftwareSerial.h>
// END INCLUDES

// SETTINGS
const bool GPSEnabled = false;
const bool UltrasonicEnabled = false;
const bool MotorEnabled = true;
const bool ServoEnabled = true;
const bool WiFiEnabled = true;
const bool SerialEnabled = true;
#define BAUD_RATE 9600

#define SECRET_SSID "DennisNet"
#define SECRET_PASS "dennis_is_a_menace"
#define PORT 80
IPAddress IP_Address(192, 48, 56, 2);
// END SETTINGS

// PIN DEFINITIONS
#define GPS_RX_PIN 10
#define GPS_TX_PIN 11
#define SERVO_PIN 9
#define MOTOR_PIN 6
#define TRIG_PIN 2
#define ECHO_PIN 3
#define GPS_BAUD 9600
// END PIN DEFINITIONS

// INITIALIZE
Servo servoMotor;
SoftwareSerial gpsSerial(GPS_RX_PIN, GPS_TX_PIN);
TinyGPSPlus gps;
WiFiServer server(PORT);
// END INITIALIZE

// VARIABLES
int servoAngle = 0;
int motorSpeed = 0;
long distance = 0;
long duration = 0;
int status = WL_IDLE_STATUS;
// END VARIABLES

// HTTP HANDLERS
class HttpResponse
{
public:
  int statusCode;
  String statusMessage;
  String contentType;
  String body;

  HttpResponse(int code = 200, const String &message = "OK", const String &type = "application/json", const String &content = "")
      : statusCode(code), statusMessage(message), contentType(type), body(content) {}

  String toString() const
  {
    String response = "HTTP/1.1 " + String(statusCode) + " " + statusMessage + "\r\n";
    response += "Content-Type: " + contentType + "\r\n";
    response += "Content-Length: " + String(body.length()) + "\r\n";
    response += "Connection: close\r\n\r\n";
    response += body;
    return response;
  }
};

class HttpRequest
{
public:
  String rawData;
  String method;
  String path;
  String query;
  JsonDocument body;
  String contentType;
  String userAgent;
  String host;

  HttpRequest(const String &rawData)
  {
    parse(rawData);
  }
  void parse(const String &rawData)
  {
    this->rawData = rawData;
    int methodEnd = rawData.indexOf(' ');
    method = rawData.substring(0, methodEnd);
    int pathEnd = rawData.indexOf(' ', methodEnd + 1);
    path = rawData.substring(methodEnd + 1, pathEnd);
    query = path.indexOf('?') != -1 ? path.substring(path.indexOf('?') + 1) : "";
    path = path.indexOf('?') != -1 ? path.substring(0, path.indexOf('?')) : path;

    int contentTypeStart = rawData.indexOf("Content-Type: ") + 15;
    int contentTypeEnd = rawData.indexOf('\r', contentTypeStart);
    contentType = rawData.substring(contentTypeStart, contentTypeEnd);

    int userAgentStart = rawData.indexOf("User-Agent: ") + 12;
    int userAgentEnd = rawData.indexOf('\r', userAgentStart);
    userAgent = rawData.substring(userAgentStart, userAgentEnd);

    int hostStart = rawData.indexOf("Host: ") + 6;
    int hostEnd = rawData.indexOf('\r', hostStart);
    host = rawData.substring(hostStart, hostEnd);

    // Parse JSON body if present
    int bodyStart = rawData.indexOf("\r\n\r\n") + 4;
    if (bodyStart < rawData.length())
    {
      String jsonBody = rawData.substring(bodyStart);
      deserializeJson(body, jsonBody);
    }
  }
};
// END HTTP HANDLERS

// FUNCTION DECLARATIONS
void initializeSerial();
void initializeMotors();
void initializeUltrasonic();
void initializeWiFi();
void initializeGPS();

void updateDistance();
void setServoAngle(int angle);
void setMotorSpeed(int speed);
void checkFirmwareVersion();

void respondToClient(WiFiClient &client, const HttpRequest &request);
void checkStatus();
void checkForClient();
// END FUNCTION DECLARATIONS

// MAIN FUNCTIONS
void setup()
{
  if (SerialEnabled)
    initializeSerial();
  if (WiFiEnabled)
    initializeWiFi();
  if (MotorEnabled)
    initializeMotors();
  if (ServoEnabled)
    servoMotor.attach(SERVO_PIN);
  if (UltrasonicEnabled)
    initializeUltrasonic();
  if (GPSEnabled)
    initializeGPS();

  Serial.println("Setup complete.");
  // delay(1000); // Allow time for setup to complete
}

void loop()
{
  if (WiFiEnabled)
  {
    checkStatus();
    checkForClient();
  }
  if (GPSEnabled && gpsSerial.available())
  {
    while (gpsSerial.available())
      gps.encode(gpsSerial.read());
    if (gps.location.isUpdated())
    {
      Serial.print("Latitude: ");
      Serial.print(gps.location.lat(), 6);
      Serial.print(", Longitude: ");
      Serial.println(gps.location.lng(), 6);
    }
  }
}
// END MAIN FUNCTIONS

// FUNCTION DEFINITIONS
void initializeSerial()
{
  Serial.begin(BAUD_RATE);
  while (!Serial)
    ; // Wait for serial port to connect. Needed for native USB port only
  Serial.println("Serial initialized.");
}

void initializeMotors()
{
  servoMotor.attach(SERVO_PIN);
  setServoAngle(0); // Initialize servo to 0 degrees
  pinMode(MOTOR_PIN, OUTPUT);
  Serial.println("Motors initialized.");
}

void initializeUltrasonic()
{
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  Serial.println("Ultrasonic sensor initialized.");
}

void initializeWiFi()
{
  checkFirmwareVersion();
  WiFi.config(IP_Address);

  Serial.print("Creating access point named: ");
  Serial.println(SECRET_SSID);

  status = WiFi.beginAP(SECRET_SSID, SECRET_PASS);
  if (status != WL_AP_LISTENING)
  {
    Serial.println("Failed to create access point.");
    while (true)
      ;
  }
  Serial.println("Access point created.");
  server.begin();
  Serial.println("Server started.");
}

void initializeGPS()
{
  gpsSerial.begin(GPS_BAUD);
  Serial.println("GPS initialized.");
}

void updateDistance()
{
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  duration = pulseIn(ECHO_PIN, HIGH);
  distance = (duration * 0.034) / 2; // Calculate distance in cm
}

void setServoAngle(int angle)
{
  servoMotor.write(angle);
  servoAngle = angle;
  /*Serial.print("Servo angle set to: ");
  Serial.println(servoAngle);*/
}

void setMotorSpeed(int speed)
{
  analogWrite(MOTOR_PIN, speed);
  motorSpeed = speed;
  /*Serial.print("Motor speed set to: ");
  Serial.println(motorSpeed);*/
}

void checkFirmwareVersion()
{
  String fv = WiFi.firmwareVersion();
  if (fv < WIFI_FIRMWARE_LATEST_VERSION)
  {
    Serial.println("Please upgrade the firmware");
  }
}

void respondToClient(WiFiClient &client, const HttpRequest &request)
{
  Serial.print("Received request: ");
  Serial.print(request.method);
  Serial.print(" at ");
  Serial.println(request.path);
  bool sendResponse = true;
  HttpResponse response;
  if (request.method == "GET")
  {
    if (request.path == "/panel/")
    {
      String panelContent = String(htmlContent.c_str());
      response = HttpResponse(200, "OK", "text/html", panelContent);
    }
  }
  if (sendResponse) {
    char * responseString = (char *)malloc(response.toString().length() + 1);
    strcpy(responseString, response.toString().c_str());
    Serial.println(responseString);
    client.print(responseString);
  }
  
  /*Serial.println("Request Sent");
  if (sendResponse)
  {
    client.print(responseString);
    client.flush();
  }
  Serial.println("Response Received");*/
}

void checkStatus()
{
  if (status != WiFi.status())
  {
    status = WiFi.status();
    if (status == WL_AP_CONNECTED)
    {
      Serial.println("Device connected to AP");
    }
    else
    {
      Serial.println("Device disconnected from AP");
    }
  }
}

void checkForClient()
{
  WiFiClient client = server.available();
  if (client)
  {
    Serial.println(" - CLIENT ADDED - "); // print a message out the serial port
    String data = "";                     // make a String to hold incoming data from the client
    while (client.connected())
    {
      if (client.available())
      {
        char c = client.read();
        data += c;

        // Detect the end of the HTTP headers (double CRLF)
        if (data.indexOf("\r\n\r\n") != -1)
        {
          // Check if there's a Content-Length header to read the body
          String contentLengthHeader = "Content-Length: ";
          int contentLengthIndex = data.indexOf(contentLengthHeader);
          if (contentLengthIndex != -1)
          {
            int valueStart = contentLengthIndex + contentLengthHeader.length();
            int valueEnd = data.indexOf("\r\n", valueStart);
            int contentLength = data.substring(valueStart, valueEnd).toInt();

            // Read the body based on Content-Length
            while (data.length() < data.indexOf("\r\n\r\n") + 4 + contentLength)
            {
              if (client.available())
              {
                data += (char)client.read();
              }
            }
          }
          break;
        }
      }
    }
    HttpRequest httpRequest(data);
    // client.setTimeout(1000);
    respondToClient(client, httpRequest);
    client.stop(); // Close the connection
    Serial.println(" - CLIENT REMOVED - ");
  }
}
