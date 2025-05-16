#include <ESP8266WiFi.h>
#include <WiFiUdp.h>

// — Wi‑Fi credentials
const char* ssid     = "signals";
const char* password = "144466ab";

// — UDP settings
WiFiUDP udp;
const unsigned int localUdpPort = 8888;
char incomingPacket[255];

// no SoftwareSerial: use hardware Serial1 (TX only) for Arduino
// wiring: NodeMCU GPIO2 (Serial1 TX) → Arduino RX (pin 0)

void setup() {
  Serial.begin(9600);       // debug over USB
  Serial1.begin(9600);        // to Arduino
  Serial1.println("send from nodeMCU");

  WiFi.begin(ssid, password);
  Serial.print("Connecting Wi‑Fi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWi‑Fi connected, IP: " + WiFi.localIP().toString());

  // disable power‑save for lowest latency
  WiFi.setSleepMode(WIFI_NONE_SLEEP);

  udp.begin(localUdpPort);
  Serial.printf("UDP on port %u\n", localUdpPort);
}

void loop() {
  int packetSize = udp.parsePacket();
  if (packetSize) {
    int len = udp.read(incomingPacket, sizeof(incomingPacket) - 1);
    if (len > 0) {
      incomingPacket[len] = '\0';
      String msg = String(incomingPacket);
      msg.trim();

      Serial.println("Received UDP: " + msg);
      // echo back confirmation
      udp.beginPacket(udp.remoteIP(), udp.remotePort());
      udp.write("receive");
      udp.endPacket();

      // forward raw payload to Arduino
      Serial1.println(msg);
    }
  }
}
