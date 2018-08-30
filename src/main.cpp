
#include "Wire.h"   // Arduino Wire Library
#include "I2Cdev.h" // High level I2C Library
#include "EEPROM.h" // Internal Arduino memory

// Sensor Libraries
#include "BMP085.h" // Barometer, Temperature
// #include "ADXL345.h" // Accelerometer
// #include "HMC5883L.h" // Magnetometer
// #include "L3G4200D.h" // Gyroscope

//Kalman Filter
#include <SimpleKalmanFilter.h>

#define SQUIB_PIN 10

#define SAFETY_CLEAREANSE_TIME 10000 // time waited for safe startup in milliseconds
#define ALTITUDE_IS_LOWER_THRESHOLD 10 // Number of consecutive lower values for altitude to detect apogee
#define MIN_ALT_FOR_APOGEE_DETECTION 0
#define ALTITUDE_DATA_POINTS 30
#define TELEMETRY_DATA_POINTS 4

#define SET_DATA_WRITE_ALLOW false // sets eeprom write state to true
#define REFERENCE_ALTITUDE_ADRESS 8
#define APOGEE_ADRESS 4
#define EEPROM_DATA_TYPE float // data type to be writen to eeprom 
#define EEPROM_ALTITUDE_WRITE_STEP 2 // size of step in meters between eeprom  altitude data writes
#define EEPROM_ADDR_SAFETY_MARGIN 24

BMP085 barometer;
// ADXL345 accel;
// HMC5883L mag;
// L3G4200D gyro;

/* SimpleKalmanFilter(e_mea, e_est, q);
 e_mea: Measurement Uncertainty -- How much do we expect to our measurement vary 
 e_est: Estimation Uncertainty -- It Can be initilized with the same value as e_mea 
 since the kalman filter will adjust its value.
 q: Process Noise -- Usually a small number between 0.001 and 1 - How fast your 
 measurement moves. Recommended 0.01. Should be tunned to your needs.
*/
SimpleKalmanFilter pressureKalmanFilter(1, 1, 0.01);

void updateBarometer(void);
void updateMatrix(float *vetor);

float pressure, filteredPressure;
float temperature;
float referenceAltitude, altitude, apogee = 0;
float altitudeData[ALTITUDE_DATA_POINTS + 1];
float parachuteReleaseTime = 0;
float eepromWriteAltitudeThreshold = MIN_ALT_FOR_APOGEE_DETECTION;

int altitudeIsLowerCounter = 0, eepromCurrentAddr = REFERENCE_ALTITUDE_ADRESS + sizeof(EEPROM_DATA_TYPE);

bool eepromWritePermission;

bool parachuteReleased = false;

void setup()
{
    float lastApogee, lastReferenceAltitude;

    pinMode(SQUIB_PIN, OUTPUT);

    // Join I2C bus (I2Cdev library doesn't do this automatically)
    Wire.begin();
    // 38400 default because it works as well at 8MHz as it does at 16MHz
    Serial.begin(115200);

    // Initialize devices
    Serial.println("Initializing I2C devices...");
    barometer.initialize();
    // accel.initialize();
    // mag.initialize();
    // gyro.initialize();

    // Verify connection
    Serial.println("Testing device connections...");
    Serial.println(barometer.testConnection() ? "BMP085 connection successful" : "BMP085 connection failed");
    // Serial.println(accel.testConnection() ? "ADXL345 connection successful" : "ADXL345 connection failed");
    // Serial.println(mag.testConnection() ? "HMC5883L connection successful" : "HMC5883L connection failed");
    // Serial.println(gyro.testConnection() ? "L3G4200D connection successful" : "L3G4200D connection failed");

    
    delay(SAFETY_CLEAREANSE_TIME);

    for (int i = 0; i <= ALTITUDE_DATA_POINTS; i++) // Initialize altitude data array
    {
        updateBarometer();
        updateMatrix(altitudeData);
    }
    referenceAltitude = altitude; // Altitude correction factor
    apogee = referenceAltitude;

    if(SET_DATA_WRITE_ALLOW)
    {
        EEPROM.put(0, true);
    }

    EEPROM.get(0, eepromWritePermission); // Gets EEPROM write permission

    EEPROM.get(REFERENCE_ALTITUDE_ADRESS, lastReferenceAltitude);
    EEPROM.get(APOGEE_ADRESS, lastApogee);

    Serial.print("Last recorded uncorrected Apogee: ");
    Serial.println(lastApogee);
    Serial.print("Last recorded reference Altitude: ");
    Serial.println(lastReferenceAltitude);
    Serial.print("Last recorded corrected Apogee: ");
    Serial.println(lastApogee - lastReferenceAltitude);
}

void loop()
{
    updateBarometer();          // Updates barometer information
    updateMatrix(altitudeData); // Updates the altitude data matrix



    /*EEPROM WRITE CODE*/
    if(eepromWritePermission && eepromCurrentAddr < (EEPROM.length() - EEPROM_ADDR_SAFETY_MARGIN))  
    {
        if(!parachuteReleased)
        {
            if(altitude - referenceAltitude > eepromWriteAltitudeThreshold) //writes data every EEPROM_ALTITUDE_WRITE_STEP meters on the way up
            {
                EEPROM.put(eepromCurrentAddr, pressure);
                eepromCurrentAddr += sizeof(EEPROM_DATA_TYPE);
                eepromWriteAltitudeThreshold += EEPROM_ALTITUDE_WRITE_STEP;
            }
        }
        else
        {
            if(altitude - referenceAltitude < eepromWriteAltitudeThreshold) //writes data every EEPROM_ALTITUDE_WRITE_STEP meters on the way down
            {
                EEPROM.put(eepromCurrentAddr, pressure);
                eepromCurrentAddr += sizeof(EEPROM_DATA_TYPE);
                eepromWriteAltitudeThreshold -= EEPROM_ALTITUDE_WRITE_STEP;
            }
        } 
    }

    

    if (apogee < altitude) // Checks if the most recent altitude data is higher than the current apogee value
    {
        apogee = altitude; // If it is, updates apogee
    }

    // Display measured values if appropriate
    if (!parachuteReleased)
    {
        Serial.print("T/P/A/Apogee\t");
        Serial.print(temperature);
        Serial.print("\t");
        Serial.print(pressure);
        Serial.print("\t");
        Serial.print(apogee);
        Serial.print("\t");
        Serial.print("Barometer altitude: "); // Raw altitude
        Serial.println(altitude - referenceAltitude);
    }

    /* RECOVERY CODE */
    //    Serial.println(parachuteReleased); // Debugging

    if (parachuteReleased == false)
    {
        // Checks if the oldest data point is higher than the newest
        if (altitudeData[0] > altitudeData[ALTITUDE_DATA_POINTS])
        {
            altitudeIsLowerCounter++;
        }
        else
        {
            // Resets counter to ensure that cumulative errors don't cause false trigger
            altitudeIsLowerCounter = 0;
        }

        // If ALTITUDE_IS_LOWER_THRESHOLD is reached, deploy parachute
        if (altitudeIsLowerCounter >= ALTITUDE_IS_LOWER_THRESHOLD && altitude - referenceAltitude > MIN_ALT_FOR_APOGEE_DETECTION)
        {
            digitalWrite(SQUIB_PIN, HIGH);
            parachuteReleased = true;
            parachuteReleaseTime = millis() / 1000.0;
            
            Serial.println("Parachute released! Apogee/RefAlt");
            Serial.println(apogee);
            Serial.println(referenceAltitude);
            
            if(eepromWritePermission)
            {
                EEPROM.put(REFERENCE_ALTITUDE_ADRESS, referenceAltitude);
                EEPROM.put(APOGEE_ADRESS, apogee);

                EEPROM.put(eepromCurrentAddr, apogee); // adds apogee to next eeprom write
                eepromCurrentAddr += sizeof(EEPROM_DATA_TYPE);

                EEPROM.put(eepromCurrentAddr, altitude); // adds current altitude to next eeprom write
                eepromCurrentAddr += sizeof(EEPROM_DATA_TYPE);

                EEPROM.put(0, false); // sets data write allow to false
            }
        }
    }

    /*TELEMETRY CODE*/
    // float telemetryVector[TELEMETRY_DATA_POINTS]; // Array to be sent by APC
    // telemetryVector[0] = temperature;
    // telemetryVector[1] = pressure;
    // telemetryVector[2] = filteredPressure;
    // telemetryVector[3] = parachuteReleaseTime;
}

void updateMatrix(float *vetor)
{
    // Altitude matrix updates to have the newest data in the last index
    for (int i = 0; i < ALTITUDE_DATA_POINTS; i++)
    {
        altitudeData[i] = altitudeData[i + 1];
    }
    altitudeData[ALTITUDE_DATA_POINTS] = altitude;
}

void updateBarometer(void)
{
    // Request temperature
    barometer.setControl(BMP085_MODE_TEMPERATURE);
    // Read calibrated temperature value in degrees Celsius
    temperature = barometer.getTemperatureC();
    // Request pressure (3x oversampling mode, high detail, 23.5ms delay)
    barometer.setControl(BMP085_MODE_PRESSURE_3);
    // Read calibrated pressure value in Pascals (Pa)
    pressure = barometer.getPressure();
    // Runs raw data trough Kalman Filter
    filteredPressure = pressureKalmanFilter.updateEstimate(pressure);
    // Calculate absolute altitude in meters based on known pressure
    // (may pass a second "sea level pressure" parameter here,
    // otherwise uses the standard value of 101325 Pa)
    altitude = barometer.getAltitude(filteredPressure);
}
