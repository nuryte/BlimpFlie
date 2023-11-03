#include "baro280.h"

baro280::baro280(){
    
    //pass
    
}
void baro280::init(){
    /* Initialise the sensor */
    int countTries = 0;
    baroOn = bme.begin(BMP280_ADDRESS_ALT, BMP280_CHIPID);
    while (!baroOn) {
        delay(100);
        if (countTries > 10) {
            Serial.println(F("Could not find a valid BMP280 sensor, check wiring or "
                            "try a different address!"));
            Serial.print("SensorID was: 0x"); Serial.println(bme.sensorID(),16);
            Serial.print("        ID of 0xFF probably means a bad address, a BMP 180 or BMP 085\n");
            Serial.print("   ID of 0x56-0x58 represents a BMP 280,\n");
            Serial.print("        ID of 0x60 represents a BME 280.\n");
            Serial.print("        ID of 0x61 represents a BME 680.\n");
            break;
        }
        countTries += 1;
        baroOn = bme.begin(BMP280_ADDRESS_ALT, BMP280_CHIPID);
    }
    bme.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                Adafruit_BMP280::STANDBY_MS_63); /* Standby time. */
    if (baroOn){
        estimatedZ = bme.readAltitude(1013.25);
        oldZ = estimatedZ;
    } else {
        estimatedZ = 0;
        oldZ = 0;
    }
    
    
}
bool baro280::updateBarometer(){
    if (baroOn){
        if ((micros() - tStart) > (BMP280_SAMPLERATE_DELAY_MS * 1000)) {
            oldZ = estimatedZ;
            estimatedZ = bme.readAltitude(1013.25);
            dtBaro = micros() - tStart;
            tStart = micros();
            return true;
        }
    }
    return false;
}

float baro280::getEstimatedZ(){
    return estimatedZ;


}
float baro280::getVelocityZ(){
    return (estimatedZ - oldZ)/((float)dtBaro / 1000000.0f);
}