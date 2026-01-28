// ===== Arduino Control + Streaming Sensor (matches your ROS node) =====
#include <DHT.h>
#include <DHT_U.h>

#define ENABLE_PIN 10
#define SENSOR_PIN A0
#define DHTPIN 2
#define DHTTYPE DHT11

DHT_Unified dht(DHTPIN, DHTTYPE);

// Match ROS sense_rate ~20Hz
const unsigned long SAMPLE_INTERVAL_MS = 50;  // 20 Hz

bool streamOn = true;
bool txOn = false;

unsigned long lastSample = 0;

// cached DHT (read at 1Hz)
static unsigned long lastDHTRead = 0;
static float temperature = NAN;
static float humidity = NAN;

void setup() {
  pinMode(ENABLE_PIN, OUTPUT);
  digitalWrite(ENABLE_PIN, LOW);

  Serial.begin(9600);
  Serial.setTimeout(30);
  dht.begin();

  delay(200);
  Serial.println("READY");
}

void loop() {
  handleSerial();

  digitalWrite(ENABLE_PIN, txOn ? HIGH : LOW);

  if (streamOn) {
    sampleAndPrint();
  }
}

void handleSerial() {
  while (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();
    cmd.toUpperCase();

    if (cmd == "S1") {
      txOn = true;
      Serial.println("ACK S1");
    } else if (cmd == "S0") {
      txOn = false;
      Serial.println("ACK S0");
    } else if (cmd == "STREAM1" || cmd == "START") {
      streamOn = true;
      Serial.println("ACK STREAM1");
    } else if (cmd == "STREAM0" || cmd == "STOP") {
      streamOn = false;
      Serial.println("ACK STREAM0");
    } else if (cmd == "PING") {
      Serial.println("PONG");
    } else {
      Serial.print("ERR ");
      Serial.println(cmd);
    }
  }
}

void sampleAndPrint() {
  unsigned long now = millis();
  if (now - lastSample < SAMPLE_INTERVAL_MS) return;
  lastSample = now;

  int sensorValue = analogRead(SENSOR_PIN);
  float voltage = (sensorValue * 5.0f) / 1023.0f;

  // avoid division by zero
  if (voltage < 0.001f) return;

  float resistance = (5.0f / voltage - 1.0f) * 10000.0f;

  // update DHT once per second (optional; not printed in R-line)
  if (now - lastDHTRead >= 1000) {
    lastDHTRead = now;
    sensors_event_t event;
    dht.temperature().getEvent(&event);
    if (!isnan(event.temperature)) temperature = event.temperature;
    dht.humidity().getEvent(&event);
    if (!isnan(event.relative_humidity)) humidity = event.relative_humidity;
  }

  // IMPORTANT: match your ROS parser
  Serial.print("R ");
  Serial.println(resistance, 1);
}
