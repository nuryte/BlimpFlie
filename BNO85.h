#include <Wire.h>
#include "SparkFun_BNO08x_Arduino_Library.h"
#include <data_types.h>

class BNO85
{
private:
    bool bnoOn = false;
    BNO08x myIMU;
    void setReports(void);
    
public:
    BNO85();
    void init();
    void updateSensors(sensors_t *sensors, sensor_weights_t *weights, RollPitchAdjustments *rollPitchAdjust);
};