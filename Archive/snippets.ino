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
