// INCLUDES
#include <Arduino.h>
#include <Servo.h>
#include "WiFiS3.h"
#include "ArduinoJson.h"
// #include "html.h"
#include <TinyGPS++.h>
#include <SoftwareSerial.h>
// END INCLUDES

// SETTINGS
const bool GPSEnabled = true;
const bool UltrasonicEnabled = false;
const bool MotorEnabled = true;
const bool ServoEnabled = true;
const bool WiFiEnabled = true;
const bool SerialEnabled = true;
const bool RadarEnabled = false;
#define BAUD_RATE 9600

#define SECRET_SSID "DennisNet"
#define SECRET_PASS "dennis_is_a_menace"
#define PORT 80
IPAddress IP_Address(192, 48, 56, 2);
// END SETTINGS

// PIN DEFINITIONS
#define SERVO_PIN 9
#define RADAR_PIN 10
#define MOTOR_PIN 6
#define TRIG_PIN 2
#define ECHO_PIN 3
#define GPS_BAUD 9600
#define radarWaitTime 60
#define radarMoveAngle 3
#define radarPowerPin 12
// END PIN DEFINITIONS

// INITIALIZE
Servo servoMotor;
Servo radarMotor;
TinyGPSPlus gps;
WiFiServer server(PORT);
// END INITIALIZE

// VARIABLES
int servoAngle = 0;
int motorSpeed = 0;
long distance = 0;
int radarAngle = 0;
long radarLastMove = 0;
long duration = 0;
int status = WL_IDLE_STATUS;
float latitude = 0;
float longitude = 0;
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
    // response += "Access-Control-Allow-Origin: *\r\n\r\n";
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
void initializeRadar();
void initializeWiFi();
void initializeGPS();

void updateGPS();
void updateDistance();
void updateRadar();
void setServoAngle(int angle);
void setRadarAngle(int angle);
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
  
  if (GPSEnabled)
    initializeGPS();
  if (RadarEnabled){
    initializeRadar();
    if (UltrasonicEnabled)
      initializeUltrasonic();
  }

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
  if (RadarEnabled) {
    updateRadar();
  }
  /*if (GPSEnabled)
  {
    updateGPS();
  }*/
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
  Serial1.begin(GPS_BAUD);
  Serial.println("GPS initialized.");
}

void initializeRadar()
{
  pinMode(radarPowerPin, OUTPUT);
  digitalWrite(radarPowerPin, HIGH); // Power on the radar
  radarMotor.attach(RADAR_PIN);
  setRadarAngle(0); // Initialize radar to 0 degrees
  Serial.println("Radar initialized.");
}

void updateGPS()
{
  if (Serial1.available() == 0)
  {
    latitude = -1;
    longitude = -1;
    return;
  }
  while (Serial1.available() > 0)
  {
    gps.encode(Serial1.read());
    if (gps.location.isValid())
    {
      latitude = gps.location.lat();
      longitude = gps.location.lng();
      /*Serial.print("Latitude: ");
      Serial.print(latitude, 6);
      Serial.print(", Longitude: ");
      Serial.println(longitude, 6);*/
      break;
    } /*else
    {
      latitude = -1;
      longitude = -1;
      break;
    }*/
  }
  
}

void updateRadar() {
  long currentMillis = millis();
  if (currentMillis - radarLastMove >= radarWaitTime) {
    radarLastMove = currentMillis;
    radarAngle += radarMoveAngle;
    if (radarAngle > 180) {
      radarAngle = 0;
      radarLastMove += radarWaitTime*3; // Adjust the last move time to avoid immediate next move
    }
    setRadarAngle(radarAngle);
  }
  if (UltrasonicEnabled) {
    updateDistance();
    Serial.print("Distance: ");
    Serial.print(distance);
    
    Serial.print(" cm at angle: ");
    Serial.println(radarAngle);
  }
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
  if (angle < 0) {
    angle = 0;
  }
  else if (angle > 180) {
    angle = 180;
  }
  servoMotor.write(angle);
  servoAngle = angle;
  /*Serial.print("Servo angle set to: ");
  Serial.println(servoAngle);*/
}

void setRadarAngle(int angle)
{
  if (angle < 0) {
    angle = 0;
  }
  else if (angle > 180) {
    angle = 180;
  }
  radarMotor.write(angle);
  radarAngle = angle;
  /*Serial.print("Radar angle set to: ");
  Serial.println(radarAngle);*/
}

void setMotorSpeed(int speed)
{
  if (speed < 0) {
    speed = 0;
  }
  else if (speed > 255) {
    speed = 255;
  }
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
  Serial.println(request.rawData);
  bool sendResponse = true;
  HttpResponse response;
  if (request.method == "GET")
  {
    /*if (request.path == "/panel/")
    {
      String panelContent = String(htmlContent.c_str());
      response = HttpResponse(200, "OK", "text/html", panelContent);
    }*/
    if (request.path == "/control/distance")
    {
      if (!UltrasonicEnabled)
      {
        JsonDocument jsonResponse;
        jsonResponse["distance"] = -1;
        String jsonResponseStr;
        serializeJson(jsonResponse, jsonResponseStr);
        response = HttpResponse(200, "OK", "application/json", jsonResponseStr);
      }
      else
      {
        updateDistance();
        JsonDocument jsonResponse;
        jsonResponse["distance"] = distance;
        String jsonResponseStr;
        serializeJson(jsonResponse, jsonResponseStr);
        response = HttpResponse(200, "OK", "application/json", jsonResponseStr);
      }
    }
    else if (request.path == "/control/location") {
      if (!GPSEnabled)
      {
        JsonDocument jsonResponse;
        jsonResponse["latitude"] = -1;
        jsonResponse["longitude"] = -1;
        String jsonResponseStr;
        serializeJson(jsonResponse, jsonResponseStr);
        response = HttpResponse(200, "OK", "application/json", jsonResponseStr);
      }
      else
      {
        // Serial.println("Updating GPS...");
        updateGPS();
        JsonDocument jsonResponse;
        jsonResponse["latitude"] = latitude;
        jsonResponse["longitude"] = longitude;
        String jsonResponseStr;
        serializeJson(jsonResponse, jsonResponseStr);
        response = HttpResponse(200, "OK", "application/json", jsonResponseStr);
        response.body = jsonResponseStr; // Ensure body matches Content-Length
        Serial.println("GPS updated.");
      }
    }
  }
  else if (request.method == "POST")
  {
    if (request.path == "/control/motor")
    {
      int motorValue = request.body["motor"];
      setMotorSpeed(motorValue);
      response = HttpResponse(200, "OK", "application/json", "{\"status\":\"success\"}");
    }
    else if (request.path == "/control/servo")
    {
      int servoValue = request.body["servo"];
      setServoAngle(servoValue);
      response = HttpResponse(200, "OK", "application/json", "{\"status\":\"success\"}");
    }
  }
  else
  {
    response = HttpResponse(404, "Not Found", "text/plain", "404 Not Found");
  }

  if (sendResponse)
  {
    String responseString = response.toString();
    client.print(responseString);
    /*Serial.println("Response sent:");
    Serial.println(responseString);*/
  }
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
