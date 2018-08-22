#include "Wire.h"   //Arduino Wire Library
#include "I2Cdev.h" //High level I2C Library
#include "EEPROM.h" //Internal Arduino memory

//Sensor Libraries 
#include "BMP085.h" //Barometer, Temperature
//#include "ADXL345.h" //Accelerometer
//#include "HMC5883L.h" //Magnetometer
//#include "L3G4200D.h" //Gyroscope

//Kalman Filter
#include <SimpleKalmanFilter.h> 

#define DEBUG_SERIAL_TIMEOUT_MILLIS 10000
#define SQUIB 10 // squib digital pin
#define ALTITUDE_IS_LOWER_THRESHOLD 10 //Recovery System 
#define ALTITUDE_DATA_POINTS 30
#define TELEMETRY_DATA_POINTS 4
#define EEPROM_BYTES_NUMBER 8
#define MIN_ALT_FOR_APOGEE_DETECTION 10

BMP085 barometer;
//not using, just here to check sensors health
ADXL345 accel;
HMC5883L mag;
L3G4200D gyro;


/* SimpleKalmanFilter(e_mea, e_est, q);
 e_mea: Measurement Uncertainty -- How much do we expect to our measurement vary 

 e_est: Estimation Uncertainty -- It Can be initilized with the same value as e_mea 
 since the kalman filter will adjust its value.

 q: Process Noise -- Usually a small number between 0.001 and 1 - How fast your 
 measurement moves. Recommended 0.01. Should be tunned to your needs.
*/
SimpleKalmanFilter pressureKalmanFilter(1, 1, 0.01);

void updateBarometer(void);
void updateMatrix(float* vetor);

float pressure, filteredPressure, temperature, referenceAltitude;
float altitudeData[ALTITUDE_DATA_POINTS + 1];
float parachuteReleaseTime = 0;

int altitudeIsLowerCounter = 0;
int apogee = 0;
unsigned int eepromMemLocation = 0;

bool parachuteReleased = false;

int32_t altitude;

void setup() {

    float lastApogee, lastReferenceAltitude;
    // join I2C bus (I2Cdev library doesn't do this automatically)
    Wire.begin();

    // initialize serial communication
    // (38400 chosen because it works as well at 8MHz as it does at 16MHz, but
    // it's really up to you depending on your project)
    Serial.begin(115200);

    //waits for pc connection for DEBUG_SERIAL_TIMEOUT_MILLIS
    unsigned long serialDebugTimeout = millis();
    while(!Serial && (millis() - serialDebugTimeout < DEBUG_SERIAL_TIMEOUT_MILLIS))

    // initialize devices
    Serial.println("Initializing I2C devices...");
    barometer.initialize();
    accel.initialize();
    mag.initialize();
    gyro.initialize();

    // verify connection
    Serial.println("Testing device connections...");
    Serial.println(barometer.testConnection() ? "BMP085 connection successful" : "BMP085 connection failed");
    Serial.println(accel.testConnection() ? "ADXL345 connection successful" : "ADXL345 connection failed");
    Serial.println(mag.testConnection() ? "HMC5883L connection successful" : "HMC5883L connection failed");
    Serial.println(gyro.testConnection() ? "L3G4200D connection successful" : "L3G4200D connection failed");


    for (int i = 0; i <= ALTITUDE_DATA_POINTS; i++) //initialize altitude data array
    {
        updateBarometer();
        updateMatrix(altitudeData);
    }
    referenceAltitude = altitude; //altitude correction factor

    EEPROM.get(1, eepromMemLocation); //gets current EEPROM memory adress to be used
    
    if(!Serial) // data writing code, only for when pc is not conected 
    {
        eepromMemLocation += EEPROM_BYTES_NUMBER; //changes to new write location

        //checks if at the end of memory restart count
        if(eepromMemLocation + EEPROM_BYTES_NUMBER > EEPROM.length())
        {
            eepromMemLocation = 0;
        }
        EEPROM.put(1, eepromMemLocation); //updates memory location tracker
    }
    else
    {
        EEPROM.get(eepromMemLocation, lastApogee);
        EEPROM.get(eepromMemLocation + 4, lastReferenceAltitude);
    }
    Serial.print("Last recorded uncorrected Apogee: ");
    Serial.println(lastApogee);
    Serial.print("Last recorded reference Altitude: ");
    Serial.println (lastReferenceAltitude);
    Serial.print("Last recorded corrected Apogee: ");
    Serial.println(lastApogee - lastReferenceAltitude);
    
    pinMode(SQUIB, OUTPUT);
 
}

void loop() {

    updateBarometer(); //updates barometer information
    if (apogee < altitude) //checks if the most recent altitude data is higher than the current apogee value
    {
        apogee = altitude; //if it is, updates apogee
    }
    
    //display measured values if appropriate
    Serial.print("T/P/A\t");
    Serial.print(temperature); Serial.print("\t");
    Serial.print(pressure); Serial.print("\t");
    Serial.print("Barometer altitude: "); //raw altitude 
    Serial.println(altitude - referenceAltitude);


    
    /*RECOVERY CODE*/
    updateMatrix(altitudeData); //updates the altitude data matrix
    
    Serial.println(parachuteReleased); //debugging
    
    if(parachuteReleased == false)
    {
        // checks if the oldest data point is higher than the newest
        if (altitudeData[0] > altitudeData[ALTITUDE_DATA_POINTS])
        {
            altitudeIsLowerCounter++;
        }
        else
        {
            // resets counter to ensure that cumulative errors don't cause false trigger
            altitudeIsLowerCounter = 0;
        }

        // if ALTITUDE_IS_LOWER_THRESHOLD is reached write to pin
        if (altitudeIsLowerCounter >= ALTITUDE_IS_LOWER_THRESHOLD && altitude - referenceAltitude > MIN_ALT_FOR_APOGEE_DETECTION)
        {
            digitalWrite(SQUIB, HIGH);
            parachuteReleaseTime = millis() / 1000.0;
            Serial.println("Parachute Release!");
            parachuteReleased = true;

            if(!Serial) //if pc is not connected write info
            {
                EEPROM.put(eepromMemLocation, apogee);
                EEPROM.put(eepromMemLocation + 4, referenceAltitude);
            }
        }
    }

    for(int k = 0; k <= ALTITUDE_DATA_POINTS; k++) //debugging
    {
        Serial.println(altitudeData[k]);
    }



    /*TELEMETRY CODE*/
    float telemetryVector[TELEMETRY_DATA_POINTS]; //array to be sent by APC
    telemetryVector[0] = temperature;
    telemetryVector[1] = pressure;
    telemetryVector[2] = filteredPressure;
    telemetryVector[3] = parachuteReleaseTime;

    // delay 100 msec to allow visually parsing blink and any serial output
    delay(500);
}

void updateMatrix(float* vetor) 
{
    // altitude matrix updates to have the newest data in the last index
    for (int i = 0; i <= ALTITUDE_DATA_POINTS; i++)
    {
        altitudeData[i] = altitudeData[i + 1];
    }
    altitudeData[ALTITUDE_DATA_POINTS] = altitude;
}

void updateBarometer(void)
{
    /*BAROMETER CODE*/
   
    // request temperature
    barometer.setControl(BMP085_MODE_TEMPERATURE);
    
    // read calibrated temperature value in degrees Celsius
    temperature = barometer.getTemperatureC();

    // request pressure (3x oversampling mode, high detail, 23.5ms delay)
    barometer.setControl(BMP085_MODE_PRESSURE_3);

    // read calibrated pressure value in Pascals (Pa)
    pressure = barometer.getPressure();
    
    // runs raw data trough Kalman Filter
    filteredPressure = pressureKalmanFilter.updateEstimate(pressure);

    // calculate absolute altitude in meters based on known pressure
    // (may pass a second "sea level pressure" parameter here,
    // otherwise uses the standard value of 101325 Pa)
    altitude = barometer.getAltitude(filteredPressure);
}
