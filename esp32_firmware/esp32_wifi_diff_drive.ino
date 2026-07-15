#include <WiFi.h>
#include <ESPmDNS.h>

const char* ssid = "Redmi 13C 5G";
const char* password = "24242424";

WiFiServer server(23);
WiFiClient client;

#define L_IN1 4
#define L_IN2 5
#define L_ENA 18
#define L_IN3 19
#define L_IN4 21
#define L_ENB 22

#define R_IN1 23
#define R_IN2 25
#define R_ENA 26
#define R_IN3 27
#define R_IN4 32
#define R_ENB 33

void setLeftSpeed(int spd) {
  spd = constrain(spd, -100, 100);
  int val = map(abs(spd), 0, 100, 0, 255);

  if (spd > 0) {
    digitalWrite(L_IN1, HIGH); digitalWrite(L_IN2, LOW);
    digitalWrite(L_IN3, HIGH); digitalWrite(L_IN4, LOW);
  } else if (spd < 0) {
    digitalWrite(L_IN1, LOW); digitalWrite(L_IN2, HIGH);
    digitalWrite(L_IN3, LOW); digitalWrite(L_IN4, HIGH);
  } else {
    digitalWrite(L_IN1, LOW); digitalWrite(L_IN2, LOW);
    digitalWrite(L_IN3, LOW); digitalWrite(L_IN4, LOW);
  }

  analogWrite(L_ENA, val);
  analogWrite(L_ENB, val);
}

void setRightSpeed(int spd) {
  spd = constrain(spd, -100, 100);
  int val = map(abs(spd), 0, 100, 0, 255);

  if (spd > 0) {
    digitalWrite(R_IN1, HIGH); digitalWrite(R_IN2, LOW);
    digitalWrite(R_IN3, HIGH); digitalWrite(R_IN4, LOW);
  } else if (spd < 0) {
    digitalWrite(R_IN1, LOW); digitalWrite(R_IN2, HIGH);
    digitalWrite(R_IN3, LOW); digitalWrite(R_IN4, HIGH);
  } else {
    digitalWrite(R_IN1, LOW); digitalWrite(R_IN2, LOW);
    digitalWrite(R_IN3, LOW); digitalWrite(R_IN4, LOW);
  }

  analogWrite(R_ENA, val);
  analogWrite(R_ENB, val);
}

void stopAll() {
  setLeftSpeed(0);
  setRightSpeed(0);
  Serial.println("[STOP]");
  if (client && client.connected()) client.println("[STOP]");
}

void printHelp() {
  String help = "\nCommands:\n"
    "  m <left> <right>   Set wheel speeds, -100 to 100\n"
    "  x                  Stop all\n"
    "  ?                  Help\n"
    "\nExamples:\n"
    "  m 20 20    forward (slow)\n"
    "  m -20 -20  backward (slow)\n"
    "  m -20 20   spin left\n"
    "  m 20 -20   spin right\n";
  Serial.println(help);
  if (client && client.connected()) client.println(help);
}

void handleCommand(String line) {
  line.trim();
  if (line.length() == 0) return;

  if (line.charAt(0) == 'x') {
    stopAll();
    return;
  }
  if (line.charAt(0) == '?') {
    printHelp();
    return;
  }
  if (line.charAt(0) == 'm') {
    int firstSpace = line.indexOf(' ');
    int secondSpace = line.indexOf(' ', firstSpace + 1);

    if (firstSpace == -1 || secondSpace == -1) {
      Serial.println("Bad command. Use: m <left> <right>");
      if (client && client.connected()) client.println("Bad command. Use: m <left> <right>");
      return;
    }

    int leftSpd  = line.substring(firstSpace + 1, secondSpace).toInt();
    int rightSpd = line.substring(secondSpace + 1).toInt();

    setLeftSpeed(leftSpd);
    setRightSpeed(rightSpd);

    String msg = "[DRIVE] L=" + String(leftSpd) + " R=" + String(rightSpd);
    Serial.println(msg);
    if (client && client.connected()) client.println(msg);
    return;
  }

  Serial.println("Unknown command. Type ? for help.");
  if (client && client.connected()) client.println("Unknown command. Type ? for help.");
}

void setup() {
  Serial.begin(115200);

  pinMode(L_IN1, OUTPUT); pinMode(L_IN2, OUTPUT);
  pinMode(L_IN3, OUTPUT); pinMode(L_IN4, OUTPUT);
  pinMode(R_IN1, OUTPUT); pinMode(R_IN2, OUTPUT);
  pinMode(R_IN3, OUTPUT); pinMode(R_IN4, OUTPUT);
  pinMode(L_ENA, OUTPUT); pinMode(L_ENB, OUTPUT);
  pinMode(R_ENA, OUTPUT); pinMode(R_ENB, OUTPUT);
  stopAll();

  Serial.println("\n=== ESP32 DIFFERENTIAL DRIVE (WiFi) ===");
  Serial.print("Connecting to WiFi: ");
  Serial.println(ssid);

  WiFi.begin(ssid, password);
  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 30) {
    delay(500);
    Serial.print(".");
    attempts++;
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\nWiFi connected!");
    Serial.print("IP address: ");
    Serial.println(WiFi.localIP());
    Serial.println("Connect via: telnet <IP> 23");

    if (MDNS.begin("esp32rover")) {
      Serial.println("mDNS responder started: esp32rover.local");
    } else {
      Serial.println("mDNS setup failed");
    }

    server.begin();
  } else {
    Serial.println("\nWiFi connection FAILED. Falling back to USB serial only.");
  }

  printHelp();
}

unsigned long lastStatusPrint = 0;

void loop() {
  if (millis() - lastStatusPrint > 5000) {
    lastStatusPrint = millis();
    if (WiFi.status() == WL_CONNECTED) {
      Serial.print("[STATUS] WiFi OK, IP=");
      Serial.print(WiFi.localIP());
      Serial.println(" (esp32rover.local)");
    } else {
      Serial.println("[STATUS] WiFi NOT connected");
    }
  }

  if (WiFi.status() == WL_CONNECTED) {
    if (server.hasClient()) {
      if (!client || !client.connected()) {
        if (client) client.stop();
        client = server.available();
        Serial.println("New WiFi client connected.");
        client.println("Connected to ESP32 Rover. Type ? for help.");
      } else {
        WiFiClient newClient = server.available();
        newClient.stop();
      }
    }

    if (client && client.connected() && client.available()) {
      String line = client.readStringUntil('\n');
      handleCommand(line);
    }
  }

  if (Serial.available()) {
    String line = Serial.readStringUntil('\n');
    handleCommand(line);
  }
}
