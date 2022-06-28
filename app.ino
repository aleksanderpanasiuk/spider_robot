#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <MPU9255.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
MPU9255 mpu;

#define SERVOMIN  120 // This is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX  460 // This is the 'maximum' pulse length count (out of 4096)
#define SERVO_FREQ 50 // Analog servos run at ~50 Hz updates

const double ARM_A = 55, ARM_B = 85, ARM_C = 55; // real length of every arm in mm
int move_length = 80; // length of a step
unsigned long czas_poczatek = millis();

double real_gyro_values[3] = {0, 0, 0};
// gyro offsets for x, y and z correspondingly
int gyro_offsets[3] = {41, -68, 8};

// starting position for all legs
double val_leg[4][3] = 
	{{-60, -50, 100},
	{30, -50, 100},
	{-20, -50, 140},
	{-20, -50, 140}};

// servo offsets
double offset[12] = 
	{-5, 0, 0,
	-10, 0, 0,
	5, 0, 0,
	0, 0, 0};

// changing raw data into degrees/second^2
double process_angular_velocity(int16_t input)
{
    return input / 16.4;
}

// setting servo position using drivers function
void set_pwm_dg(uint16_t serwo = 0, double degree = 0)
{
	pwm.setPWM(serwo, 0, (340 * degree) / 180 + 120);
}

// calculating servos in one leg position using reverse kinematics and gyro data
void leg_pos(int leg = 0, double x = 0,  double y = -50, double z = 140)
{
	double a = 0, b = y, c = 0, d = x, e = z;
	double alpha = 0, beta = 0, gamma = 0, delta = 0, epsilon = 0;

	// gyroscpe calculations
	mpu.read_gyro();
    double calculated_gyro_x = process_angular_velocity(mpu.gx);
    double calculated_gyro_y = process_angular_velocity(mpu.gy);
    double calculated_gyro_z = process_angular_velocity(mpu.gz);
    double gyro_error = 0.02;
	double time = millis() - czas_poczatek;

    if ((calculated_gyro_x / 10) >= gyro_error || (calculated_gyro_x / 10) <= -gyro_error)
        real_gyro_values[0] += (calculated_gyro_x * (time/1000.0));
	if ((calculated_gyro_y / 10) >= gyro_error || (calculated_gyro_y / 10) <= -gyro_error)
        real_gyro_values[1] += (calculated_gyro_y * (time/1000.0));
	if ((calculated_gyro_z / 10) >= gyro_error || (calculated_gyro_z / 10) <= -gyro_error)
        real_gyro_values[2] += (calculated_gyro_z * (time/1000.0));
    
	czas_poczatek = millis();

	double x_angle_in_degrees = (a+40) * tan(real_gyro_values[0]*PI/180);
	double y_angle_in_degrees = (d+38) * tan(real_gyro_values[1]*PI/180);

	if (leg == 1 || leg == 2)
		b += x_angle_in_degrees;
	else
		b -= x_angle_in_degrees;

	if (leg == 2 || leg == 3)
		b -= y_angle_in_degrees;
	else
		b += y_angle_in_degrees;

	// inverse kinematics calculations
	a = sqrt((e*e) + (d*d)) - ARM_C;
	c = sqrt((a*a) + (b*b));

	alpha = asin(b/c) * 180/PI;

	beta = ((ARM_A*ARM_A) + (c*c) - (ARM_B*ARM_B)) / (2.0 * ARM_A * c);
	beta = acos(beta) * 180.0 / PI;

	gamma = ((c*c) + (ARM_B*ARM_B) - (ARM_A*ARM_A)) / (2.0 * c * ARM_B);
	gamma = acos(gamma) * 180.0 / PI;

	delta = 180 - beta - gamma;

	epsilon = asin(d/a) * 180.0 / PI;
	
	// servo driver connection
	// leg 0: 0, 1, 2
	// leg 1: 4, 5, 6
	// leg 2: 8, 9, 10
	// leg 3: 12, 13, 14

	switch(leg)
	{
		case 0:
			set_pwm_dg(0, 90 - epsilon + offset[0]);
			set_pwm_dg(1, beta + alpha + 90 + offset[1]);
			set_pwm_dg(2, 180 - (180-delta) + offset[2]);
			break;

		case 1:
			set_pwm_dg(4, 90 - epsilon + offset[3]);
			set_pwm_dg(5, beta + alpha + 90 + offset[4]);
			set_pwm_dg(6, 180 - (180-delta) + offset[5]);
			break;

		case 2:
			set_pwm_dg(8, 90 + epsilon + offset[6]);
			set_pwm_dg(9, 90 -(beta + alpha) + offset[7]);
			set_pwm_dg(10, 180 - delta + offset[8]);
			break;
		
		case 3:
			set_pwm_dg(12, 90 + epsilon + offset[9]);
			set_pwm_dg(13, 90 -(beta + alpha) + offset[10]);
			set_pwm_dg(14, 180 - delta + offset[11]);
			break;

		default:
			Serial.println("ERROR: in leg_pos(): zly numer serwa");
			break;
	}
}

void leg_mvmnt(int leg, int speed)
{
	for (int i = 1; i <= move_length; i++)
	{
		val_leg[leg][1] += i;
		leg_pos(leg, val_leg[leg][0], val_leg[leg][1], val_leg[leg][2]);
		delay(speed);
		if (i != move_length)
			val_leg[leg][1] -= i;
	}

	for (int i = 1; i <= move_length; i++)
	{
		val_leg[leg][0] += i;
		leg_pos(leg, val_leg[leg][0], val_leg[leg][1], val_leg[leg][2]);
		delay(speed);
		if (i != move_length)
			val_leg[leg][0] -= i;
	}	
	
	val_leg[leg][1] -= move_length;

	for (int i = move_length; i > 0; i--)
	{
		val_leg[leg][1] += i;
		leg_pos(leg, val_leg[leg][0], val_leg[leg][1], val_leg[leg][2]);
		delay(speed);
		val_leg[leg][1] -= i;
	}
}

void all_legs_back(int next_leg, int speed)
{
	int side = -1;

	if (next_leg <= 1)
		side = 1;
	
	for (int i = 1; i <= move_length/4; i++)
	{
		val_leg[0][0] -= i;
		val_leg[1][0] -= i;
		val_leg[2][0] -= i;
		val_leg[3][0] -= i;

		val_leg[0][2] += (i*side);
		val_leg[1][2] += (i*side);
		val_leg[2][2] -= (i*side);
		val_leg[3][2] -= (i*side);

		leg_pos(0, val_leg[0][0], val_leg[0][1], val_leg[0][2]);
		leg_pos(1, val_leg[1][0], val_leg[1][1], val_leg[1][2]);
		leg_pos(2, val_leg[2][0], val_leg[2][1], val_leg[2][2]);
		leg_pos(3, val_leg[3][0], val_leg[3][1], val_leg[3][2]);

		if (i != move_length/4)
		{
			val_leg[0][0] += i;
			val_leg[1][0] += i;
			val_leg[2][0] += i;
			val_leg[3][0] += i;

			val_leg[0][2] -= (i*side);
			val_leg[1][2] -= (i*side);
			val_leg[2][2] += (i*side);
			val_leg[3][2] += (i*side);
		}
		
		delay(speed);
	}
}

void move_forward(int speed)
{
	leg_mvmnt(2, speed);
	all_legs_back(0, speed);

	leg_mvmnt(0, speed);
	all_legs_back(1, speed);

	leg_mvmnt(1, speed);
	all_legs_back(3, speed);

	leg_mvmnt(3, speed);
	all_legs_back(2, speed);

}

void setup()
{
	Serial.begin(9600);
	pwm.begin();

	pwm.setOscillatorFrequency(27000000);
  	pwm.setPWMFreq(SERVO_FREQ);  // Analog servos run at ~50 Hz updates

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

    delay(1000);
	leg_pos(0, val_leg[0][0], val_leg[0][1], val_leg[0][2]);
	leg_pos(1, val_leg[1][0], val_leg[1][1], val_leg[1][2]);
	leg_pos(2, val_leg[2][0], val_leg[2][1], val_leg[2][2]);
	leg_pos(3, val_leg[3][0], val_leg[3][1], val_leg[3][2]);

	Serial.println("5 sec to start loop()");
	delay(5000);
	Serial.println("start");
	
}

void loop()
{ 
	// move_forward(2);
	
	leg_pos(0, val_leg[0][0], val_leg[0][1], val_leg[0][2]);
	leg_pos(1, val_leg[1][0], val_leg[1][1], val_leg[1][2]);
	leg_pos(2, val_leg[2][0], val_leg[2][1], val_leg[2][2]);
	leg_pos(3, val_leg[3][0], val_leg[3][1], val_leg[3][2]);
	
}
