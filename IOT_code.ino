#include <ESP8266WiFi.h>

// Wi-Fi credentials
const char* ssid = "DC_5G";
const char* password = "Since2015";
WiFiServer server(80); // Start a web server on port 80

void setup() {
  Serial.begin(115200);

  // Connect to Wi-Fi
  WiFi.begin(ssid, password);
  Serial.print("Connecting to Wi-Fi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.print(".");
  }
  Serial.println("\nWi-Fi Connected");
  Serial.print("IP Address: ");
  Serial.println(WiFi.localIP());

  server.begin(); // Start the server
}

void loop() {
  WiFiClient client = server.available();
  if (client) {
    Serial.println("New Client Connected");
    String request = client.readStringUntil('\r');
    Serial.println(request);
    client.flush();

    // Parse the HTTP request for angle value
    if (request.indexOf("/angle?value=") != -1) {
      int start = request.indexOf("value=") + 6;
      int end = request.indexOf(" ", start);
      String angleStr = request.substring(start, end);
      int angle = angleStr.toInt();
      Serial.println("Received Angle: " + String(angle));
    }

    // HTML response
    String response = R"====(
      <!DOCTYPE html>
      <html>
      <head>
        <title>ESP Angle Input</title>
        <style>
          body { font-family: Arial; text-align: center; margin-top: 20px; }
          input[type="number"], button { margin: 10px; padding: 10px; font-size: 18px; }
        </style>
      </head>
      <body>
        <h1>Enter Angle Value</h1>
        <form onsubmit="sendAngle(); return false;">
          <input type="number" id="angle" min="0" max="360" placeholder="Enter angle (0-360)">
          <button type="submit">Submit</button>
        </form>
        <script>
          function sendAngle() {
            const angle = document.getElementById('angle').value;
            if (angle) {
              fetch('/angle?value=' + angle);
              alert('Angle submitted: ' + angle);
            } else {
              alert('Please enter a valid angle!');
            }
          }
        </script>
      </body>
      </html>
    )====";

    client.print("HTTP/1.1 200 OK\r\nContent-Type: text/html\r\n\r\n");
    client.print(response);
    client.stop();
    Serial.println("Client Disconnected");
  }
}