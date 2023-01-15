#include <ESP8266WiFi.h>

// Dependency: https://github.com/morrissinger/ESP8266-Websocket
#include <WebSocketClient.h>

// I like wire
#include "Wire.h"  // This library allows you to communicate with I2C devices.

#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"

MPU6050 mpu;

// MPU control/status vars
bool dmpReady = false;   // set true if DMP init was successful
uint8_t devStatus;       // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;     // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;      // count of all bytes currently in FIFO
uint8_t fifoBuffer[64];  // FIFO storage buffer

// orientation/motion vars
Quaternion q;         // [w, x, y, z]         quaternion container
VectorInt16 aa;       // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;   // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;  // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;  // [x, y, z]            gravity vector
float euler[3];       // [psi, theta, phi]    Euler angle container
float ypr[3];         // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

// Wifi connection vars
const char* ssid = "trans rights";
const char* password = "arduinotime";
char path[] = "/";
char host[] = "192.168.137.1";
int port = 1989;

WebSocketClient webSocketClient;

// Use WiFiClient class to create TCP connections
WiFiClient client;

void setup() {

  // Set up serial bus
  Wire.begin();
  Wire.setClock(400000);

  Serial.begin(9600);
  delay(10);

  // Initialize MPU
  mpu.initialize();

  devStatus = mpu.dmpInitialize();

  // Gyro Offsets
  mpu.setXGyroOffset(220);
  mpu.setYGyroOffset(76);
  mpu.setZGyroOffset(-85);

  // make sure it worked (returns 0 if so)
  if (devStatus == 0) {
    // Calibration Time: generate offsets and calibrate our MPU6050
    mpu.CalibrateAccel(6);
    mpu.CalibrateGyro(6);
    mpu.PrintActiveOffsets();
    // turn on the DMP, now that it's ready
    Serial.println(F("Enabling DMP..."));
    mpu.setDMPEnabled(true);

    dmpReady = true;

    // get expected DMP packet size for later comparison
    packetSize = mpu.dmpGetFIFOPacketSize();
  } else {
    // ERROR!
    // 1 = initial memory load failed
    // 2 = DMP configuration updates failed
    // (if it's going to break, usually the code will be 1)
    Serial.print(F("DMP Initialization failed (code "));
    Serial.print(devStatus);
    Serial.println(F(")"));
  }

  // We start by connecting to a WiFi network
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  delay(2500);

  // Connect to the websocket server
  if (client.connect(host, port)) {
    Serial.println("Connected");
  } else {
    Serial.println("Connection failed.");
    while (1) {
      // Hang on failure
    }
  }

  // Handshake with the server
  webSocketClient.path = path;
  webSocketClient.host = host;
  if (webSocketClient.handshake(client)) {
    Serial.println("Handshake successful");
  } else {
    Serial.println("Handshake failed.");
    while (1) {
      // Hang on failure
    }
  }
}

int maxAccel = 0;
unsigned long lastSend = 0;

void loop() {
  String data;

  if (client.connected()) {
    if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {  // Get the Latest packet
      // display initial world-frame acceleration, adjusted to remove gravity
      // and rotated based on known orientation from quaternion
      mpu.dmpGetQuaternion(&q, fifoBuffer);
      mpu.dmpGetAccel(&aa, fifoBuffer);
      mpu.dmpGetGravity(&gravity, &q);
      mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
      mpu.dmpGetLinearAccelInWorld(&aaWorld, &aaReal, &q);

      // Find maximum magnitude of acceleration
      int maxAccelCurrent = max(abs(aaWorld.x), max(abs(aaWorld.y), abs(aaWorld.z)));
      if (maxAccelCurrent > maxAccel) {
        maxAccel = maxAccelCurrent;
      }

      unsigned long nowTime = millis();
      if ((nowTime - lastSend) > 1000) {
        webSocketClient.sendData("Max accel value " + String(maxAccel));
        if (maxAccel > 10000) {
          webSocketClient.sendData("BB happy thank u daddy kojima");
        } else {
          webSocketClient.sendData("BB crying norman reedus please shake");
        }

        maxAccel = 0;
        lastSend = nowTime;
      }
      
      // data = "";
      // data += "aworld\t";
      // data += String(aaWorld.x);
      // data += "\t";
      // data += String(aaWorld.y);
      // data += "\t";
      // data += String(aaWorld.z);
      // webSocketClient.sendData(data);
    }


  } else {
    Serial.println("Client disconnected.");
    while (1) {
      // Hang on disconnect.
    }
  }

  // wait to fully let the client disconnect
  delay(10);
}