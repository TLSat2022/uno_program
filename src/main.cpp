#include <Arduino.h>
#include <Adafruit_Sensor.h>
#include <DHT_U.h>
#include <LSM6.h>
#include <Wire.h>
#include <Adafruit_BMP280.h>
#include <PID_v1.h>
#include <SD.h>

File myFile;

#define MOTOR_PIN 3

DHT_Unified dht(4, DHT22);
Adafruit_BMP280 bmp;

double Setpoint, Input, Output;
double Kp = 2, Ki = 5, Kd = 1;

PID motorPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

bool x = false;
float y = 0.0f;

LSM6 gyro;

double convert_gyro(double val) {
    return val * 8.75 / 1000;
}

void setup() {

    // Initializations
    Serial.begin(9600);
    Wire.begin();

    while(!gyro.init()){
        Serial.println("Failed to detect and initialize magnetometer");
        delay(500);
    }
    while(!bmp.begin(0x76)){
        Serial.println("Failed to detect BMP 280");
        delay(500);
    }
    while(!SD.begin(8)){
        Serial.println("SD card initialization failed!");
        delay(500);
    }
    dht.begin();

    // Settings
    gyro.enableDefault();
    bmp.setSampling(Adafruit_BMP280::MODE_FORCED,
                    Adafruit_BMP280::SAMPLING_X2,
                    Adafruit_BMP280::SAMPLING_X16,
                    Adafruit_BMP280::FILTER_X16,
                    Adafruit_BMP280::STANDBY_MS_500);

    // Pin Inits
    pinMode(MOTOR_PIN, OUTPUT);
    pinMode(LED_BUILTIN, OUTPUT);

    // Variable Init
    Input = 0;
    Setpoint = 0;

    // PID init
    motorPID.SetMode(AUTOMATIC);

    // Open file
    myFile = SD.open("test.txt", FILE_WRITE);
    if(myFile){
        myFile.println("I am testing you nanana 1, 2, 3.");
        myFile.close();
    }
}

int threshold(int speed) {
    return abs(speed) > 5 ? abs(speed) : 0;
}

void loop() {
    sensors_event_t sensor;
    dht.temperature().getEvent(&sensor);
    gyro.read();
    bmp.takeForcedMeasurement();
    int speed = (int)convert_gyro(gyro.g.z);
    Input = threshold(speed);
    motorPID.Compute();

    analogWrite(MOTOR_PIN, abs(Output));
    String to_send = String(y) + ';' + String(bmp.readTemperature()) + ';'
                     + String(convert_gyro(gyro.g.x)) + ";" + String(convert_gyro(gyro.g.y)) + ";" + String(
                             Output) + ";" + String(bmp.readPressure()) + ";";

    Serial.println(to_send);
    digitalWrite(LED_BUILTIN, x = !x);
    y++;
    delay(100);
}

