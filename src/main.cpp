#include "Wire.h" //Arduino Wire Library

//Sensor Libraries 
#include "BMP085.h" //Pressure, Temperature
#include "ADXL345.h" //Accelerometer
#include "HMC5883L.h" //Magnetometer
#include "L3G4200D.h" //Gyroscope

#define G_SCALAR 0.0078

BMP085 barometer;
ADXL345 accelerometer;
HMC5883L magnetometer;
L3G4200D gyroscope;

float pressure;
float temperature;
int32_t altitude;
int16_t ax, ay, az;
int16_t mx, my, mz;
int16_t avx, avy, avz;

void setup() {
    // join I2C bus (I2Cdev library doesn't do this automatically)
    Wire.begin();
    Serial.begin(115200);

    // Initialize devices
    Serial.println("Initializing I2C devices...");
    barometer.initialize();
    accelerometer.initialize();
    magnetometer.initialize();
    gyroscope.initialize();

    // Verify connections
    Serial.println("Testing device connections...");
    Serial.println(barometer.testConnection() ? "BMP085 connection successful." : "BMP085 connection failed.");
    Serial.println(accelerometer.testConnection() ? "ADXL345 connection successful." : "ADXL345 connection failed.");
    Serial.println(magnetometer.testConnection() ? "HMC5883L connection successful." : "HMC5883L connection failed.");
    Serial.println(gyroscope.testConnection() ? "L3G4200D connection successful." : "L3G4200D connection failed.");

    // Data seems to be best when full scale is 2000
    gyroscope.setFullScale(2000);
}

void loop() {

    /*BAROMETER CODE*/
    // Request temperature
    barometer.setControl(BMP085_MODE_TEMPERATURE);
    // Read calibrated temperature value in degrees Celsius
    temperature = barometer.getTemperatureC();
    // Request pressure (3x oversampling mode, high detail, 23.5ms delay)
    barometer.setControl(BMP085_MODE_PRESSURE_3);
    // Read calibrated pressure value in Pascals (Pa)
    pressure = barometer.getPressure();
    // Calculate absolute altitude in meters based on known pressure
    // (may pass a second "sea level pressure" parameter here,
    // otherwise uses the standard value of 101325 Pa)
    altitude = barometer.getAltitude(pressure);
    // display measured values if appropriate
    Serial.print("T: ");
    Serial.print(temperature); 
    Serial.print(" P: ");
    Serial.print(pressure);
    Serial.print(" Alt: ");
    Serial.print(altitude);

    /*ACCELEROMETER CODE*/
    // read raw accelerometer measurements from device
    accelerometer.getAcceleration(&ax, &ay, &az);
    // display tab-separated accelerometer x/y/z values
    Serial.print(" Acc: ");
    Serial.print(ax * G_SCALAR); Serial.print(" ");
    Serial.print(ay * G_SCALAR); Serial.print(" ");
    Serial.print(az * G_SCALAR); Serial.print(" ");

    /*MAGNETOMETER CODE*/
    // read raw heading measurements from device
    magnetometer.getHeading(&mx, &my, &mz);
    // display tab-separated gyroscope x/y/z values
    Serial.print("Mag: ");
    Serial.print(mx); Serial.print(" ");
    Serial.print(my); Serial.print(" ");
    Serial.print(mz); Serial.print(" ");
    /*GYROSCOPE CODE*/
    // read raw angular velocity measurements from device
    gyroscope.getAngularVelocity(&avx, &avy, &avz);
    Serial.print("Ang Vel: ");
    Serial.print(avx); Serial.print(" ");
    Serial.print(avy); Serial.print(" ");
    Serial.println(avz);
    
    // delay 100 msec to allow visually parsing blink and any serial output
    delay(100);
}

