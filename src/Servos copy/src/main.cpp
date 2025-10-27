#include <WiFi.h>
#include <WiFiUdp.h>
#include <Wire.h>
#include <ArduinoJson.h> // Compatible amb versió 7.4.2
#include <ESP32Servo.h>

// Device ID
const char *deviceId = "G4_Servos";

// Wi-Fi credentials
const char *ssid = "Robotics_UB";
const char *password = "rUBot_xx";

// UDP settings
IPAddress receiverESP32IP(192, 168, 1, 41); // Gripper
IPAddress receiverComputerIP(192, 168, 1, 45);
const int udpPort = 12345;
WiFiUDP udp;

// Servo settings
Servo servo_yaw;
Servo servo_pitch;
Servo servo_roll1;
Servo servo_roll2;

// Pins
const int PIN_ANALOG_YAW = 36;
const int PIN_SIGNAL_YAW = 32;
const int PIN_ANALOG_PITCH = 39;
const int PIN_SIGNAL_PITCH = 33;
const int PIN_ANALOG_ROLL1 = 34;
const int PIN_SIGNAL_ROLL1 = 25;
const int PIN_ANALOG_ROLL2 = 35;
const int PIN_SIGNAL_ROLL2 = 27;

const float Rshunt = 1.6;

// Variables
float Gri_roll = 0.0, Gri_pitch = 0.0, Gri_yaw = 0.0;
float Torque_roll1 = 0.0, Torque_roll2 = 0.0, Torque_pitch = 0.0, Torque_yaw = 0.0;
float prevRoll1 = 0, prevRoll2 = 0, prevPitch = 0, prevYaw = 0;
float sumRoll1 = 0, sumRoll2 = 0, sumPitch = 0, sumYaw = 0;
float OldValueRoll = 0, OldValuePitch = 0, OldValueYaw = 0;
float roll = 0, pitch = 0, yaw = 0;
int s1 = 1, s2 = 1;
// Reference flags & values for initial servo mapping (used by moveServos)
bool servosRefInitialized = false;
float refGri_roll = 0.0;
float refGri_pitch = 0.0;
float refGri_yaw = 0.0;

void connectToWiFi() {
  Serial.print("Connecting to Wi-Fi");
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.print(".");
  }
  Serial.println("\nWi-Fi connected!");
  Serial.println("IP Address: " + WiFi.localIP().toString());
  Serial.print("ESP32 MAC Address: ");
  Serial.println(WiFi.macAddress());
}

void receiveOrientationUDP() {
  int packetSize = udp.parsePacket();
  if (packetSize) {
    byte packetBuffer[512];
    int len = udp.read(packetBuffer, 512);
    if (len > 0) {
      packetBuffer[len] = '\0';
      Serial.print("Received packet size: ");
      Serial.println(packetSize);
      Serial.print("Received: ");
      Serial.println((char*)packetBuffer);

      JsonDocument doc;  // ✅ Versió 7
      DeserializationError error = deserializeJson(doc, packetBuffer);
      if (error) {
        Serial.print(F("deserializeJson() failed: "));
        Serial.println(error.f_str());
        return;
      }

      const char* device = doc["device"];
      if (strcmp(device, "G4_Gri") == 0) {
        Gri_roll = round(doc["roll"].as<float>());
        Gri_pitch = round(doc["pitch"].as<float>());
        Gri_yaw = round(doc["yaw"].as<float>());
        s1 = doc["s1"];
        s2 = doc["s2"];
        Serial.print("Gri_Roll: "); Serial.print(Gri_roll);
        Serial.print(" Gri_Pitch: "); Serial.print(Gri_pitch);
        Serial.print(" Gri_Yaw: "); Serial.println(Gri_yaw);
        Serial.print("S1: "); Serial.print(s1);
        Serial.print(" S2: "); Serial.println(s2);
      } else {
        Serial.println("Unknown device.");
      }
    }
  }
}

float getCurrent(uint32_t integrationTimeMs, int pin) {
  uint32_t startTime = millis();
  float integratedCurrent = 0;
  while (millis() < startTime + integrationTimeMs) {
    uint16_t adcValue = analogRead(pin);
    integratedCurrent += ((float)adcValue / 4095.0 * 3.3) / Rshunt;
  }
  return integratedCurrent;
}

float getTorque(float& sum, int analogPin, float& previous) {
  float current = getCurrent(20, analogPin);
  sum += current;
  float diff = abs(sum - previous);
  previous = sum;
  return diff;
}

void moveServos() {
  // Assignem les darreres orientacions rebudes
  float currentRoll = Gri_roll;
  float currentPitch = Gri_pitch;
  float currentYaw = Gri_yaw;

  // Inicialitzem la referència la primera vegada que tenim dades reals
  if (!servosRefInitialized) {
    // Prenem la primera mesura com a referència (zero relatiu)
    refGri_roll = currentRoll;
    refGri_pitch = currentPitch;
    refGri_yaw = currentYaw;
    servosRefInitialized = true;
    Serial.println("Servo references initialized:");
    Serial.print(" refRoll="); Serial.print(refGri_roll);
    Serial.print(" refPitch="); Serial.print(refGri_pitch);
    Serial.print(" refYaw="); Serial.println(refGri_yaw);
  }

  // Helper per trobar la diferència angular mínima entre a i b (en graus)
  auto angleDiff = [](float a, float b) -> float {
    float d = fmod((a - b + 540.0f), 360.0f) - 180.0f; // retorna en rang [-180,180)
    return d;
  };

  // Calculem variacions relatives respecte la referència
  float relRoll  = angleDiff(currentRoll,  refGri_roll);   // +/- graus
  float relPitch = angleDiff(currentPitch, refGri_pitch);  // +/- graus
  float relYaw   = angleDiff(currentYaw,   refGri_yaw);    // +/- graus

  // Delta addicional quan S1 premut (obrir)
  float delta = 0.0f;
  if (s1 == 0) {
    delta = 40.0f;
    Serial.println("S1 premut → Obrint");
  }

  // MAPPEIG A SERVOS:
  // -> Partim de la posició inicial 90° i apliquem la variació rel
  // Roll: 
  //   servo_roll1 = 90 + relRoll + delta   
  //   servo_roll2 = 90 - relRoll - delta
  float servoRoll1Pos = 90.0f + relRoll + delta;
  float servoRoll2Pos = 90.0f - relRoll - delta;
  // Pitch: moviment senzill respecte 90°
  float servoPitchPos = 90.0f + relPitch;
  // Yaw: variació relativa respecte la referència (independent del nord)
  // Aplicada des de 90°: servo_yaw = 90 + relYaw
  float servoYawPos = 90.0f + relYaw;

  // Clamp a [0, 180]
  auto clamp = [](float v)->int {
    if (v < 0.0f) return 0;
    if (v > 180.0f) return 180;
    return (int)round(v);
  };

  int posRoll1  = clamp(servoRoll1Pos);
  int posRoll2  = clamp(servoRoll2Pos);
  int posPitch  = clamp(servoPitchPos);
  int posYaw    = clamp(servoYawPos);

  // Escriu als servos
  servo_roll1.write(posRoll1);
  servo_roll2.write(posRoll2);
  servo_pitch.write(posPitch);
  servo_yaw.write(posYaw);

  // Guarda valors "antics" si encara els uses en altres càlculs
  OldValueRoll = currentRoll;
  OldValuePitch = currentPitch;
  OldValueYaw = currentYaw;

}

void setup() {
  Serial.begin(115200);
  Wire.begin();
  delay(2000);

  connectToWiFi();
  udp.begin(udpPort);
  Serial.println("UDP initialized");

  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);

  servo_yaw.setPeriodHertz(50);
  servo_pitch.setPeriodHertz(50);
  servo_roll1.setPeriodHertz(50);
  servo_roll2.setPeriodHertz(50);

  servo_yaw.attach(PIN_SIGNAL_YAW);
  servo_pitch.attach(PIN_SIGNAL_PITCH);
  servo_roll1.attach(PIN_SIGNAL_ROLL1);
  servo_roll2.attach(PIN_SIGNAL_ROLL2);

  pinMode(PIN_ANALOG_YAW, INPUT);
  pinMode(PIN_ANALOG_PITCH, INPUT);
  pinMode(PIN_ANALOG_ROLL1, INPUT);
  pinMode(PIN_ANALOG_ROLL2, INPUT);

  servo_yaw.write(90);
  servo_pitch.write(90);
  servo_roll1.write(90);
  servo_roll2.write(90);
}

void sendTorquesUDP() {
  // Calcular els torques utilitzant la funció getTorque ja definida
  Torque_roll1  = getTorque(sumRoll1, PIN_ANALOG_ROLL1, prevRoll1);
  Torque_roll2  = getTorque(sumRoll2, PIN_ANALOG_ROLL2, prevRoll2);
  Torque_pitch  = getTorque(sumPitch, PIN_ANALOG_PITCH, prevPitch);
  Torque_yaw    = getTorque(sumYaw, PIN_ANALOG_YAW, prevYaw);

  // Crear el JSON amb capacitat estàtica
  StaticJsonDocument<256> doc;
  doc["device"] = deviceId;
  doc["Torque_roll1"] = Torque_roll1;
  doc["Torque_roll2"] = Torque_roll2;
  doc["Torque_pitch"] = Torque_pitch;
  doc["Torque_yaw"] = Torque_yaw;

  // Serialitzar i obtenir la llargada real
  char jsonBuffer[256];
  size_t len = serializeJson(doc, jsonBuffer, sizeof(jsonBuffer));

  // Send to ESP32 Gripper
  udp.beginPacket(receiverESP32IP, udpPort);
  udp.write((const uint8_t*)jsonBuffer, len);
  udp.endPacket();

  // Send to Computer
  udp.beginPacket(receiverComputerIP, udpPort);
  udp.write((const uint8_t*)jsonBuffer, len);
  udp.endPacket();

}

void loop() {
  receiveOrientationUDP();
  moveServos();
  sendTorquesUDP();
  delay(10);
}
