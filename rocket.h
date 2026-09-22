#ifndef ROCKET_H
#define ROCKET_H
//All needed libraries
#include "LSM6DS3.h"
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_BMP280.h>
#include <ArduinoEigenDense.h>
#include <Arduino.h>
#include <lis3mdl.h>

using namespace Eigen;


//constants
#define deg_to_rad  std::numbers::pi/180
class rocket
{
  public:

    rocket(double local_pressure= 1010, double local_g = 9.8); //initializer with optional local gravity and local sea level pressure.

    
    Matrix<double, 4,1> attitude_vector; //attitude vector in quaternions. Easy to convert to euler angles, see calculation of forwardness in update function

    Matrix<double, 2,1> yz_state; //State vector in off-direction

    Matrix<double, 3,1> state_vector; //access to the filtered state vector, formated as (altitude, velocity and acceleration), all in metric

    Matrix<double,3,3> state_covariance; //covariance matrix of state vector

    double forwardness; //cosine of deviation from vertical flight

    double altitude; //raw barometer altitude reading (m)

    Matrix<double,3,3> Mag_correction;

    Vector3d mag_offset; //zero gauss reading of magnetometers

    Vector3d mag_raw; //raw magnetometer reading(gauss)

    Vector3d acc_raw; //raw accelerometer reading (m/s^2)

    Vector3d gyro_raw; //raw gyroscope readings (rad/s)

    Vector3d euler_attitude;  //attitude in more readable euler angles.


    bool detect_apogee();

    bool detect_liftoff();

    void begin(); //just initializes sensors.


    void update(double time_interval = 0.001); /*reads sensors and updates state and attitude. must pass interval since last reading. must always be run
    in any active flight state, as this reads all data, filters it and makes it available as the above defined parameters*/

    double barometric_velocity(double time_interval = 0.001); /*uses stored barometer
    reading and calculates current barometric velocity. mainly for landing detection, where filter is ureliable. must run update first.*/
 
    void calibrate(int N = 100); //zeroes sensors, must be as close to vertical as possible. must be run after begin in setup.


  private:
  //passed local parameters
    double start_pressure;
    double g;
    
};


#endif