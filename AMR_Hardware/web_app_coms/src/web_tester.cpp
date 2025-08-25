#include <WiFi.h>
#include <WebServer.h>
#include <ArduinoJson.h>

const char* ssid = "inflab";
const char* password = "infinity@123";

WebServer server(3000);

float Kp = 0.0;
float Ki = 0.0;
float Kd = 0.0;

void handleCORS() {
  server.sendHeader("Access-Control-Allow-Origin", "*");
  server.sendHeader("Access-Control-Allow-Methods", "POST, GET, OPTIONS");
  server.sendHeader("Access-Control-Allow-Headers", "Content-Type");
  server.send(204);
}

void handleUpdatePID() {
  handleCORS();
  
  String response;
  
  if (server.hasArg("plain")) {
    String body = server.arg("plain");
    DynamicJsonDocument doc(1024);
    
    DeserializationError error = deserializeJson(doc, body);
    
    if (error) {
      response = "{\"status\":\"error\",\"message\":\"Invalid JSON\"}";
    } else {
      Kp = doc["kp"];
      Ki = doc["ki"];
      Kd = doc["kd"];
      
      Serial.println("Received PID values:");
      Serial.print("Kp: "); Serial.println(Kp);
      Serial.print("Ki: "); Serial.println(Ki);
      Serial.print("Kd: "); Serial.println(Kd);
      
      response = "{\"status\":\"success\",\"message\":\"PID values updated\"}";
    }
  } else {
    response = "{\"status\":\"error\",\"message\":\"No data received\"}";
  }
  
  server.send(200, "application/json", response);
}

void setup() {
  Serial.begin(115200);
  
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to WiFi...");
  }
  Serial.println("Connected to WiFi");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());

  server.on("/update-pid", HTTP_OPTIONS, handleCORS);
  server.on("/update-pid", HTTP_POST, handleUpdatePID);
  
  server.begin();
  Serial.println("HTTP server started");
}

void loop() {
  server.handleClient();
}

