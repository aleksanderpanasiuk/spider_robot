#include <MPU9255.h>

MPU9255 mpu;

double real_gyro_values[3] = {0, 0, 0};
// gyro offsets for x, y and z correspondingly
int gyro_offsets[3] = {41, -72, 8};

double process_angular_velocity(int16_t input)
{
    return input / 16.4;
}

void setup()
{
    Serial.begin(9600);

    if(mpu.init())
        Serial.println("initialization failed");
    else
        Serial.println("initialization successful!");
    
    // set gyro bandwidth
    Serial.println("Gyro bandwidth : 5Hz");
    mpu.set_gyro_bandwidth(gyro_5Hz); 
    
    // set gyro scale to +- 2000 dps
    Serial.println("Gyro scale : +- 2000 dps");
    mpu.set_gyro_scale(scale_2000dps);

    // set gyro offsets
    mpu.set_gyro_offset(X_axis, gyro_offsets[0]);
    mpu.set_gyro_offset(Y_axis, gyro_offsets[1]);
    mpu.set_gyro_offset(Z_axis, gyro_offsets[2]);
}

void loop()
{
    mpu.read_gyro();
    Serial.print("GX:  ");
    double calculated_gyro_x = process_angular_velocity(mpu.gx);
    double calculated_gyro_y = process_angular_velocity(mpu.gy);
    double calculated_gyro_z = process_angular_velocity(mpu.gz);
    double gyro_error = 0.01;
    double time = 0.005;

    if ((calculated_gyro_x / 10) >= gyro_error || (calculated_gyro_x / 10) <= -gyro_error)
        real_gyro_values[0] += (calculated_gyro_x * time);
    
    // Serial.print(mpu.gx);
    // Serial.print("\n");
    Serial.print(real_gyro_values[0]);
    Serial.print("\n");
    delay(time*1000);
}

