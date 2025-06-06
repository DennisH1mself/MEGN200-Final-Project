/* 

 -- Dash n' Dine Robot Final Code --
          Dennis Porter
          James Keller

*/

/*
 -- PLEASE NOTE --

 We removed the ultrasonic sensor and radar functionality; settings and functions still exist for them however they are not enabled.

*/

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
const bool GPSEnabled = true; // Enable GPS functionality
const bool UltrasonicEnabled = false; // Enable Ultrasonic sensor functionality
const bool MotorEnabled = true; // Enable motor control
const bool ServoEnabled = true; // Enable servo motor control
const bool WiFiEnabled = true; // Enable WiFi functionality
const bool SerialEnabled = true; // Enable Serial communication
const bool RadarEnabled = false; // Enable radar functionality
#define BAUD_RATE 9600 // Serial communication baud rate

#define SECRET_SSID "DennisNet" // WiFi SSID
#define SECRET_PASS "dennis_is_a_menace" // WiFi password
#define PORT 80 // Server port
IPAddress IP_Address(192, 48, 56, 2); // Static IP address for the device
// END SETTINGS

// PIN DEFINITIONS AND SETTINGS
#define SERVO_PIN 9
#define RADAR_PIN 10
#define MOTOR_PIN 6
#define TRIG_PIN 2
#define ECHO_PIN 3
#define GPS_BAUD 9600
#define radarWaitTime 60 // Time in milliseconds between radar movements (disabled)
#define radarMoveAngle 3 // Angle increment for each radar movement (disabled)
#define radarPowerPin 12 // Pin to control radar power (disabled)
// END PIN DEFINITIONS

// INITIALIZE
Servo servoMotor; // Servo motor for steering
Servo radarMotor; // Servo motor for radar movement
TinyGPSPlus gps; // GPS library instance for parsing GPS data
WiFiServer server(PORT); // WiFi server instance on the specified port
// END INITIALIZE

// VARIABLES
int servoAngle = 0; // Current angle of the servo motor
int motorSpeed = 0; // Current speed of the motor (0-255)
long distance = 0; // Measured distance from the ultrasonic sensor
int radarAngle = 0; // Current angle of the radar servo motor
long radarLastMove = 0; // Timestamp of the last radar movement
long duration = 0; // Duration of the ultrasonic pulse
int status = WL_IDLE_STATUS; // WiFi connection status
float latitude = 0; // Current latitude from GPS
float longitude = 0; // Current longitude from GPS
// END VARIABLES

// HTTP HANDLERS
class HttpResponse // Class to represent an HTTP response
{
public:
  int statusCode; // HTTP status code (e.g., 200 for OK, 404 for Not Found)
  String statusMessage; // HTTP status message corresponding to the status code
  String contentType; // Content-Type header indicating the type of the response body (e.g., application/json)
  String body; // The body of the HTTP response, containing the actual data

  // Constructor to initialize the HTTP response with default or provided values
  HttpResponse(int code = 200, const String &message = "OK", const String &type = "application/json", const String &content = "")
      : statusCode(code), statusMessage(message), contentType(type), body(content) {}

  // Method to convert the HTTP response into a properly formatted string
  String toString() const
  {
    String response = "HTTP/1.1 " + String(statusCode) + " " + statusMessage + "\r\n"; // Start with the HTTP status line
    response += "Content-Type: " + contentType + "\r\n"; // Add the Content-Type header
    response += "Content-Length: " + String(body.length()) + "\r\n"; // Add the Content-Length header
    response += "Connection: close\r\n\r\n"; // Add the Connection header and end the headers section
    // response += "Access-Control-Allow-Origin: *\r\n\r\n"; // Uncomment to allow cross-origin requests
    response += body; // Append the body of the response
    return response; // Return the complete HTTP response as a string
  }
};

class HttpRequest // Incoming HTTP request class/handler
{
public:
  String rawData; // Raw HTTP request data
  String method; // HTTP method (e.g., GET, POST)
  String path; // Request path (e.g., /control/motor)
  String query; // Query string (e.g., ?key=value)
  JsonDocument body; // Parsed JSON body of the request
  String contentType; // Content-Type header value
  String userAgent; // User-Agent header value
  String host; // Host header value

  HttpRequest(const String &rawData) { parse(rawData); } // Constructor to parse raw HTTP request data

  void parse(const String &rawData)
  {
    this->rawData = rawData; // Store raw data
    int methodEnd = rawData.indexOf(' '); // Find end of method
    method = rawData.substring(0, methodEnd); // Extract method
    int pathEnd = rawData.indexOf(' ', methodEnd + 1); // Find end of path
    path = rawData.substring(methodEnd + 1, pathEnd); // Extract path
    query = path.indexOf('?') != -1 ? path.substring(path.indexOf('?') + 1) : ""; // Extract query string if present
    path = path.indexOf('?') != -1 ? path.substring(0, path.indexOf('?')) : path; // Remove query from path

    int contentTypeStart = rawData.indexOf("Content-Type: ") + 15; // Locate Content-Type header
    int contentTypeEnd = rawData.indexOf('\r', contentTypeStart); // Find end of Content-Type value
    contentType = rawData.substring(contentTypeStart, contentTypeEnd); // Extract Content-Type value

    int userAgentStart = rawData.indexOf("User-Agent: ") + 12; // Locate User-Agent header
    int userAgentEnd = rawData.indexOf('\r', userAgentStart); // Find end of User-Agent value
    userAgent = rawData.substring(userAgentStart, userAgentEnd); // Extract User-Agent value

    int hostStart = rawData.indexOf("Host: ") + 6; // Locate Host header
    int hostEnd = rawData.indexOf('\r', hostStart); // Find end of Host value
    host = rawData.substring(hostStart, hostEnd); // Extract Host value

    int bodyStart = rawData.indexOf("\r\n\r\n") + 4; // Locate start of body
    if (bodyStart < rawData.length()) // Check if body exists
    {
      String jsonBody = rawData.substring(bodyStart); // Extract JSON body
      deserializeJson(body, jsonBody); // Parse JSON body
    }
  }
};
// END HTTP HANDLERS

// FUNCTION DECLARATIONS
// Functions and their purpose are explained in the definitions; these are just declarations 
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
  if (SerialEnabled) // Initialize Serial communication if enabled
    initializeSerial();
  if (WiFiEnabled) // Initialize WiFi functionality if enabled
    initializeWiFi();
  if (MotorEnabled) // Initialize motor control if enabled
    initializeMotors();
  if (ServoEnabled) // Attach servo motor for steering if enabled
    servoMotor.attach(SERVO_PIN);
  if (GPSEnabled) // Initialize GPS functionality if enabled
    initializeGPS();
  if (RadarEnabled) { // Initialize radar functionality if enabled
    initializeRadar();
    if (UltrasonicEnabled) // Initialize ultrasonic sensor if enabled
      initializeUltrasonic();
  }
  Serial.println("Setup complete."); // Indicate setup completion
  // delay(1000); // Allow time for setup to complete
}

void loop()
{
  if (WiFiEnabled) // Check WiFi status and handle client requests if WiFi is enabled
  {
    checkStatus(); // Monitor WiFi connection status
    checkForClient(); // Handle incoming client requests
  }
  if (RadarEnabled) { // Update radar functionality if enabled
    updateRadar(); // Move radar and optionally update distance
  }
}
// END MAIN FUNCTIONS

// FUNCTION DEFINITIONS
void initializeSerial()
{
  Serial.begin(BAUD_RATE); // Start serial communication at the defined baud rate
  while (!Serial)
    ; // Wait for the serial port to connect (required for native USB ports)
  Serial.println("Serial initialized."); // Indicate that serial communication is ready
}

void initializeMotors()
{
  servoMotor.attach(SERVO_PIN); // Attach servo motor to the defined pin
  setServoAngle(90); // Set servo to initial angle of 90 degrees (drive straight)
  pinMode(MOTOR_PIN, OUTPUT); // Set motor pin as output
  Serial.println("Motors initialized."); // Indicate motor initialization
}

void initializeUltrasonic()
{
  pinMode(TRIG_PIN, OUTPUT); // Set the trigger pin as an output to send ultrasonic pulses
  pinMode(ECHO_PIN, INPUT); // Set the echo pin as an input to receive the reflected pulse
  Serial.println("Ultrasonic sensor initialized."); // Indicate that the ultrasonic sensor is ready
}

void initializeWiFi()
{
  checkFirmwareVersion(); // Check if the WiFi firmware is up-to-date
  WiFi.config(IP_Address); // Configure the device with a static IP address

  Serial.print("Creating access point named: "); // Indicate the start of access point creation
  Serial.println(SECRET_SSID); // Print the SSID of the access point

  status = WiFi.beginAP(SECRET_SSID, SECRET_PASS); // Start the access point with the given SSID and password
  if (status != WL_AP_LISTENING) // Check if the access point failed to start
  {
    Serial.println("Failed to create access point."); // Print failure message
    while (true); // Halt execution if access point creation fails
  }
  Serial.println("Access point created."); // Indicate successful access point creation
  server.begin(); // Start the WiFi server
  Serial.println("Server started."); // Indicate that the server is running
}

void initializeGPS()
{
  Serial1.begin(GPS_BAUD); // Start serial communication with the GPS module at the defined baud rate
  Serial.println("GPS initialized."); // Indicate that the GPS module is ready
}

void initializeRadar()
{
  pinMode(radarPowerPin, OUTPUT); // Set radar power pin as output
  digitalWrite(radarPowerPin, HIGH); // Turn on radar power
  radarMotor.attach(RADAR_PIN); // Attach radar servo motor to the defined pin
  setRadarAngle(0); // Set radar servo to initial angle of 0 degrees
  Serial.println("Radar initialized."); // Indicate radar initialization
}

void updateGPS()
{
  if (Serial1.available() == 0) // Check if GPS data is available
  {
    latitude = -1; // Set latitude to -1 if no data
    longitude = -1; // Set longitude to -1 if no data
    return; // Exit the function
  }
  int runs = 0; // Counter for the number of iterations
  while (Serial1.available() > 0) // Process available GPS data
  {
    runs++; // Increment the counter
    gps.encode(Serial1.read()); // Parse incoming GPS data
    if (gps.location.isValid()) // Check if the GPS location is valid
    {
      latitude = gps.location.lat(); // Update latitude with valid data
      longitude = gps.location.lng(); // Update longitude with valid data
      /*time_t time = gps.time.value();
      int year = gps.date.year();
      int month = gps.date.month();
      int day = gps.date.day();
      int hour = gps.time.hour();
      int minute = gps.time.minute();
      int second = gps.time.second();
      Serial.print("Date: ");
      Serial.print(year);
      Serial.print("-");
      Serial.print(month);
      Serial.print("-");
      Serial.print(day);
      Serial.print(" Time: ");
      Serial.print(hour);
      Serial.print(":");
      Serial.print(minute);
      Serial.print(":");
      Serial.println(second);
      Serial.print(" Latitude: ");
      Serial.print(latitude, 6);
      Serial.print(", Longitude: ");
      Serial.println(longitude, 6);*
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
   Serial.println("GPS data not available. No signal.");
  if (runs > 10) { // Break the loop if there is no GPS data after 10 iterations
   break;
  }
  }
  
}

void updateRadar() { // Function to update radar movement and optionally ultrasonic distance
  long currentMillis = millis(); // Get the current time in milliseconds
  if (currentMillis - radarLastMove >= radarWaitTime) { // Check if it's time to move the radar
    radarLastMove = currentMillis; // Update the last move timestamp
    radarAngle += radarMoveAngle; // Increment the radar angle
    if (radarAngle > 180) { // Reset radar angle if it exceeds 180 degrees
      radarAngle = 0; // Reset to 0 degrees
      radarLastMove += radarWaitTime*3; // Delay next move to avoid immediate reset
    }
    setRadarAngle(radarAngle); // Set the radar servo to the new angle
  }
  if (UltrasonicEnabled) { // If ultrasonic sensor is enabled
    updateDistance(); // Update the distance measurement
    Serial.print("Distance: "); // Print the measured distance
    Serial.print(distance); // Print the distance value
    Serial.print(" cm at angle: "); // Print the radar angle
    Serial.println(radarAngle); // Print the angle value
  }
}

void updateDistance()
{
  digitalWrite(TRIG_PIN, LOW); // Set the trigger pin to LOW to ensure a clean pulse
  delayMicroseconds(2); // Wait for 2 microseconds
  digitalWrite(TRIG_PIN, HIGH); // Set the trigger pin to HIGH to send a pulse
  delayMicroseconds(10); // Keep the pulse HIGH for 10 microseconds
  digitalWrite(TRIG_PIN, LOW); // Set the trigger pin back to LOW to end the pulse
  duration = pulseIn(ECHO_PIN, HIGH); // Measure the time it takes for the echo to return
  distance = (duration * 0.034) / 2; // Calculate the distance in cm based on the duration
}

void setServoAngle(int angle) // Function to set the angle of the servo motor
{
  if (angle < 45) { // If the angle is less than 45 degrees
    angle = 30; // Limit the angle to a minimum of 30 degrees
  }
  else if (angle > 135) { // If the angle is greater than 135 degrees
    angle = 150; // Limit the angle to a maximum of 150 degrees
  }  
  servoMotor.write(angle); // Write the adjusted angle to the servo motor
  delay(15); // Allow time (15ms) for the servo to reach the specified position
  servoAngle = angle; // Update the global variable to reflect the current servo angle
}

void setRadarAngle(int angle) // Function to set the angle of the radar servo motor
{
  if (angle < 0) { // If the angle is less than 0 degrees
    angle = 0; // Limit the angle to a minimum of 0 degrees
  }
  else if (angle > 180) { // If the angle is greater than 180 degrees
    angle = 180; // Limit the angle to a maximum of 180 degrees
  }
  radarMotor.write(angle); // Write the adjusted angle to the radar servo motor
  radarAngle = angle; // Update the global variable to reflect the current radar angle
}

void setMotorSpeed(int speed) // Function to set the speed of the motor
{
  if (speed < 0) { // If the speed is less than 0
    speed = 0; // Limit the speed to a minimum of 0
  }
  else if (speed > 255) { // If the speed is greater than 255
    speed = 255; // Limit the speed to a maximum of 255
  }
  analogWrite(MOTOR_PIN, speed); // Write the speed value to the motor pin using PWM
  motorSpeed = speed; // Update the global variable to reflect the current motor speed
  /*Serial.print("Motor speed set to: "); 
  Serial.println(motorSpeed); */
}

void checkFirmwareVersion() // Function to check the WiFi firmware version
{
  String fv = WiFi.firmwareVersion(); // Get the current firmware version of the WiFi module
  if (fv < WIFI_FIRMWARE_LATEST_VERSION) // Compare the current firmware version with the latest version
  {
    Serial.println("Please upgrade the firmware"); // Print a message if the firmware is outdated
  }
}

void respondToClient(WiFiClient &client, const HttpRequest &request) // Function to handle client requests and send appropriate responses
{
  Serial.println(request.rawData); // Print the raw HTTP request data for debugging
  bool sendResponse = true; // Flag to determine if a response should be sent
  HttpResponse response; // Create an HTTP response object
  if (request.method == "GET") // Handle GET requests
  {
    /*if (request.path == "/panel/") // Serve the control panel HTML page
    {
      String panelContent = String(htmlContent.c_str()); // Convert HTML content to a string
      response = HttpResponse(200, "OK", "text/html", panelContent); // Create an HTTP response with the HTML content
    }*/
    if (request.path == "/control/distance") // Handle requests for distance data -- DISABLED --
    {
      if (!UltrasonicEnabled) // Check if the ultrasonic sensor is disabled
      {
        JsonDocument jsonResponse; // Create a JSON response object
        jsonResponse["distance"] = -1; // Set distance to -1 to indicate no data
        String jsonResponseStr; // String to hold serialized JSON
        serializeJson(jsonResponse, jsonResponseStr); // Serialize JSON response
        response = HttpResponse(200, "OK", "application/json", jsonResponseStr); // Create an HTTP response with JSON
      }
      else // If the ultrasonic sensor is enabled
      {
        updateDistance(); // Update the distance measurement
        JsonDocument jsonResponse; // Create a JSON response object
        jsonResponse["distance"] = distance; // Add the measured distance to the response
        String jsonResponseStr; // String to hold serialized JSON
        serializeJson(jsonResponse, jsonResponseStr); // Serialize JSON response
        response = HttpResponse(200, "OK", "application/json", jsonResponseStr); // Create an HTTP response with JSON
      }
    }
    else if (request.path == "/control/location") { // Handle requests for GPS location data
      if (!GPSEnabled) // Check if GPS is disabled
      {
        JsonDocument jsonResponse; // Create a JSON response object
        jsonResponse["latitude"] = -1; // Set latitude to -1 to indicate no data
        jsonResponse["longitude"] = -1; // Set longitude to -1 to indicate no data
        String jsonResponseStr; // String to hold serialized JSON
        serializeJson(jsonResponse, jsonResponseStr); // Serialize JSON response
        response = HttpResponse(200, "OK", "application/json", jsonResponseStr); // Create an HTTP response with JSON
      }
      else // If GPS is enabled
      {
        // Serial.println("Updating GPS..."); // Debug message for GPS update
        updateGPS(); // Update GPS data
        JsonDocument jsonResponse; // Create a JSON response object
        jsonResponse["latitude"] = latitude; // Add latitude to the response
        jsonResponse["longitude"] = longitude; // Add longitude to the response
        String jsonResponseStr; // String to hold serialized JSON
        serializeJson(jsonResponse, jsonResponseStr); // Serialize JSON response
        response = HttpResponse(200, "OK", "application/json", jsonResponseStr); // Create an HTTP response with JSON
        response.body = jsonResponseStr; // Ensure body matches Content-Length
        Serial.print("Latitude: "); // Debug message for latitude
        Serial.print(latitude, 6); // Print latitude with 6 decimal places
        Serial.print(", Longitude: "); // Debug message for longitude
        Serial.println(longitude, 6); // Print longitude with 6 decimal places

        Serial.println("GPS updated."); // Debug message for GPS update completion
      }
    }
  }
  else if (request.method == "POST") // Handle POST requests
  {
    if (request.path == "/control/motor") // Handle motor control requests
    {
      int motorValue = request.body["motor"]; // Extract motor speed value from the request body
      setMotorSpeed(motorValue); // Set the motor speed
      response = HttpResponse(200, "OK", "application/json", "{\"status\":\"success\"}"); // Respond with success
    }
    else if (request.path == "/control/servo") // Handle servo control requests
    {
      int servoValue = request.body["servo"]; // Extract servo angle value from the request body
      setServoAngle(servoValue); // Set the servo angle
      response = HttpResponse(200, "OK", "application/json", "{\"status\":\"success\"}"); // Respond with success
    }
  }
  else // Handle unsupported HTTP methods
  {
    response = HttpResponse(404, "Not Found", "text/plain", "404 Not Found"); // Respond with 404 Not Found
  }

  if (sendResponse) // Check if a response should be sent to the client
  {
    String responseString = response.toString(); // Convert the HttpResponse object to a formatted HTTP response string
    client.print(responseString); // Send the HTTP response string to the client
    /*Serial.println("Response sent:");
    Serial.println(responseString);*/
  }
}

void checkStatus() // Function to monitor and handle changes in WiFi connection status
{
  if (status != WiFi.status()) // Check if the current WiFi status has changed
  {
    status = WiFi.status(); // Update the stored WiFi status
    if (status == WL_AP_CONNECTED) // If a device has connected to the access point
    {
      Serial.println("Device connected to AP"); // Print a message indicating a device is connected
    }
    else // If no device is connected to the access point
    {
      Serial.println("Device disconnected from AP"); // Print a message indicating a device is disconnected
    }
  }
}

void checkForClient() // Function to check for and handle incoming client connections
{
  WiFiClient client = server.available(); // Check if a client is available on the server
  if (client) // If a client is connected
  {
    Serial.println(" - CLIENT ADDED - "); // Print a message indicating a client has connected
    String data = ""; // Initialize a string to hold incoming data from the client
    while (client.connected()) // While the client is connected
    {
      if (client.available()) // If data is available from the client
      {
        char c = client.read(); // Read a character from the client
        data += c; // Append the character to the data string

        // Detect the end of the HTTP headers (double CRLF)
        if (data.indexOf("\r\n\r\n") != -1) // Check if the end of the headers is reached
        {
          // Check if there's a Content-Length header to read the body
          String contentLengthHeader = "Content-Length: "; // Define the Content-Length header string
          int contentLengthIndex = data.indexOf(contentLengthHeader); // Find the index of the Content-Length header
          if (contentLengthIndex != -1) // If the Content-Length header is found
          {
            int valueStart = contentLengthIndex + contentLengthHeader.length(); // Calculate the start index of the header value
            int valueEnd = data.indexOf("\r\n", valueStart); // Find the end index of the header value
            int contentLength = data.substring(valueStart, valueEnd).toInt(); // Extract and convert the Content-Length value to an integer

            // Read the body based on Content-Length
            while (data.length() < data.indexOf("\r\n\r\n") + 4 + contentLength) // While the full body has not been read
            {
              if (client.available()) // If more data is available from the client
              {
                data += (char)client.read(); // Read and append the next character to the data string
              }
            }
          }
          break; // Exit the loop once the headers and body are fully read
        }
      }
    }
    HttpRequest httpRequest(data); // Parse the raw HTTP request data into an HttpRequest object
    respondToClient(client, httpRequest); // Handle the client request and send an appropriate response
    client.stop(); // Close the connection with the client
    Serial.println(" - CLIENT REMOVED - "); // Print a message indicating the client has disconnected
  }
}
