#include <rocket.h>




using namespace Eigen;


rocket::rocket(double local_pressure, double local_g):
start_pressure(local_pressure), g(local_g){

}

//sensor objects
LIS3MDL magnetometer(7,4);
LSM6 sensor(10, 2,500);
Adafruit_BMP280 bmp(6);



double B_total;
double y_adj;
double z_adj;
//sensor offsets
double offset_x;
double offset_y;
double offset_z;
double offset_gx;
double offset_gy;
double offset_gz;


//barometric altitude
double alt_buffer=0;
double alt_v_lpf = 0;
double alt_v_buffer = 0;

//acceleration stuff
double vert_accel;
double yz_raw;

// low pass filter
double prevY = 0;
double prevZ = 0;
double alpha = 0.6;
double ylpf;
double zlpf;

//orientation


double w1;
double w2;
double w3;
double sin_phi;
double cos_phi;
double sin_theta;
double cos_theta;
double sin_psi;
double cos_psi;
double cos_incl;


double pitch;
double roll;
double yaw;




//timekeeping

int runs;
double half_time;


//attitude filter matrices
Matrix<double, 4, 4> Ag{
  {0, 0, 0, 0},
  {0, 0, 0, 0},
  {0, 0, 0, 0},
  {0, 0, 0, 0}
};
Matrix<double, 4, 4> ID{
  {1, 0, 0, 0},
  {0, 1, 0, 0},
  {0, 0, 1, 0},
  {0, 0, 0, 1}
};
Matrix<double, 4, 4> Hg = ID;
Matrix<double, 4,4> Htg = Hg.transpose();
/*Q and R are a total mistery for me. the gyros are not treated as "measurements",
instead they are used in the A matrix, so theyre in a weird grey zone between model and measurement.
 The measurements come from the accelerometer and 
magnetometer. I just copied these values from the tutorial I saw.
*/
Matrix<double,4, 4> Qg = 0.01*ID;
Matrix<double,4, 4> Rg = 10*ID;
Matrix<double,4, 4> Pg;
Matrix<double,4, 4> Kg;

Matrix<double, 4,1> vg{
  {1},
  {0},
  {0},
  {0}
};
Matrix<double, 4,1> zg{
  {1},
  {0},
  {0},
  {0}
};
Matrix<double,4, 1> vpg;
Matrix<double,4, 4> Ppg;
// gyro filter initial conditions
Matrix<double, 4,1> v0g{
  {1},
  {0},
  {0},
  {0}
};

Matrix<double,4, 4> P0g = ID;

//pos filter definitions

Matrix<double, 3, 3> A{
  {1, 0, 0}, 
  {0, 1, 0}, 
  {0, 0, 1},
};
Matrix<double, 2, 3> H{
  {1, 0, 0},
  {0, 0, 1}
};
Matrix<double, 3,2> Ht = H.transpose();
Matrix<double,3, 3> Q{ //need to acc callibrate properly, works fine on the ground.
  {0, 0, 0},
  {0, 0, 0},
  {0, 0, .1}

};
Matrix<double,2, 2> R{
{1,0},
{0,0.025}
};
Matrix<double,3, 2> K;
Matrix<double,3, 3> P;

Matrix<double,2, 1> z;
Matrix<double,3, 1> vp;
Matrix<double,3, 1> v;
Matrix<double,3, 3> Pp;

// position filter initial conditions


Matrix<double,3, 3> P0 = A;


void rocket::begin(){
  
  magnetometer.begin();

  unsigned status;

  //bmp startup
  status = bmp.begin();

  if (!status) {
    while (1) {
      Serial.println("Barometer failed to start");
      delay(10000);
      }
  }

  /* Default settings from datasheet. */
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                  Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                  Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */

                  Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                  Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */

  sensor.begin();
  //LSM6DS3 startup

  mag_offset(0) = 0;
  mag_offset(1) = 0;
  mag_offset(2) = 0;
  Mag_correction(0,0) = 1;
  Mag_correction(1,1) = 1;
  Mag_correction(2,2) = 1;

}

void rocket::calibrate(int N){
  runs = N;
  
  while (runs>0){
    offset_gx += sensor.g_x();
    offset_gy += sensor.g_y();
    offset_gz += sensor.g_z();
    offset_x += sensor.xl_x();
    offset_y += sensor.xl_y();
    offset_z += sensor.xl_z();
    altitude += bmp.readAltitude(start_pressure);
    runs-=1;
  }
  offset_gx = (offset_gx/N);
  offset_gy = offset_gy/N;
  offset_gz = offset_gz/N;

  offset_x = ((offset_x/N)-g); //remove g from this one as the rocket is upright
  offset_y = offset_y/N;
  offset_z = offset_z/N;

  altitude = altitude/N; //sets starting ASL altitude

  Matrix<double,3, 1> v0(altitude, 0, 0);
  v = v0; 
  alt_buffer = altitude;
  Serial.println("Calibrated!"); 
}

void rocket::update(double time_interval){

  mag_raw(0) = magnetometer.Bx(mag_offset(0));  
  mag_raw(1) = magnetometer.By(mag_offset(1));  
  mag_raw(2) = magnetometer.Bz(mag_offset(2));  

  mag_raw = Mag_correction*mag_raw;

  mag_raw(0) = -mag_raw(0);
  mag_raw(1) = -mag_raw(1); //adjust to same coords as rest of vehicle
  

  acc_raw(1) = sensor.xl_y(offset_y);
  acc_raw(2) = sensor.xl_z(offset_z);


  //low pass filter, y and z acceleration to minimize vibration

  ylpf = alpha*prevY + (1-alpha)*acc_raw(1);
  prevY = ylpf;
  zlpf = alpha*prevZ + (1-alpha)*acc_raw(2);
  prevZ = zlpf;



  // ANGULAR KALMAN FILTER

  //gyros readout
  w3 = deg_to_rad*sensor.g_x(offset_gx);
  w2 = deg_to_rad*sensor.g_y(offset_gy);
  w1 = deg_to_rad*sensor.g_z(offset_gz);

  gyro_raw(0) = w3;  //raw gyro data
  gyro_raw(1) = w2;
  gyro_raw(2) = w1;

  half_time = time_interval/2;
  Ag(0,0) = 0;
  Ag(0,1) = -w1;
  Ag(0,2) = -w2;
  Ag(0,3) = -w3;
  Ag(1,0) = w1;
  Ag(1,1) = 0;
  Ag(1,2) = w3;
  Ag(1,3) = -w2;
  Ag(2,0) = w2;
  Ag(2,1) = -w3;
  Ag(2,2) = 0;
  Ag(2,3) = w1;
  Ag(3,0) = w3;
  Ag(3,1) = w2;
  Ag(3,2) = -w1;
  Ag(3,3) = 0;

  Ag = Ag*(half_time);
  Ag += ID;

  vpg = Ag*vg; //predictions
  Ppg = Ag*Pg*Ag.transpose() + Qg;
  
  Kg = Ppg*Htg*((Hg*Ppg*Htg + Rg).inverse()); //gain calc.

  //note yaw is not updated yet, will include when magnetometer is available.
  pitch = (zlpf/g);
  roll = (-ylpf/(g*cos(pitch))); //intermediate step of calculation, allows us to check if acceleration value is valid

  if ((sq(pitch)<1) && (sq(roll)<1)) { //stops computer from trying to take asin of numbers greater than 1
    pitch = asin(pitch);
    roll = asin(roll);
    y_adj = ((mag_raw(1))*cos(roll)) - (mag_raw(0)*sin(roll));
    z_adj = ((mag_raw(2))*cos(pitch)) - ((mag_raw(1))*sin(roll)*sin(pitch)) + ((mag_raw(0))*cos(roll)*sin(pitch));
    yaw = atan2(y_adj,z_adj);
    sin_phi = sin(roll/2);
    cos_phi = cos(roll/2);
    sin_theta = sin(pitch/2);
    cos_theta = cos(pitch/2);
    sin_psi = sin(yaw/2);
    cos_psi = cos(yaw/2);
    zg(0) = sin_phi*sin_theta*sin_psi + cos_phi*cos_theta*cos_psi; //weird quarternon stuff i dont understand
    zg(1) = sin_phi*cos_theta*cos_psi - cos_phi*sin_theta*sin_psi;
    zg(2) = cos_phi*sin_theta*cos_psi + sin_phi*cos_theta*sin_psi;
    zg(3) = cos_phi*cos_theta*sin_psi - sin_phi*sin_theta*cos_psi;
    // update vector normally here
  }
  //update estimate. uses gyros only if the cond. above is not met
  vg = vpg + Kg*(zg - Hg*vpg);
  Pg = Ppg - Kg*Hg*Ppg;

  attitude_vector = vg; //update attitude vector

  /*these arent really named right because most sources i found used airplane convention,
   the rockets real "roll" is reversed with yaw
  */
  yaw = atan2(2*(vg(1)*vg(2)+vg(0)*vg(3)),(sq(vg(0))+sq(vg(1))-sq(vg(2))-sq(vg(3)))); //back to euler angles
  pitch = asin(-2*((vg(1)*vg(3)) - vg(0)*vg(2)));
  roll = atan2(2*(vg(2)*vg(3)+vg(0)*vg(1)),(sq(vg(0))-sq(vg(1))-sq(vg(2))+sq(vg(3))));
  cos_incl = cos(roll)*cos(pitch);

  forwardness = cos_incl; //cosine of angle from vertical

  euler_attitude(0) = (yaw*180)/std::numbers::pi; //saving stuff to euler vector in deg/sec
  euler_attitude(1) = (pitch*180)/std::numbers::pi;
  euler_attitude(2) = (roll*180)/std::numbers::pi;


  // POSITION KALMAN FILTER

  // first, get the vertical accel.
  acc_raw(0) = sensor.xl_x(offset_x); // raw x acceleration
  if ((!isnan(cos_incl)) && (cos_incl != 0)){ //if the attitude filter breaks, we ignore it and assume vertical flight
    vert_accel = cos_incl*(acc_raw(0) - (g*cos_incl)); // corrects x-axis acceleration to vertical

    /* ***EXPERIMENTAL*** this uses the kalman velocity and acceleration, projects it to vertical then to y and z to 
    get estimates of 3d position.
    */
    yz_raw = (((state_vector(2))*sq(time_interval) * 0.5) + (state_vector(1) * time_interval))/cos_incl; //prelim step to save time
    yz_state(0) += yz_raw*sin(roll); //y postion
    yz_state(1) += yz_raw*sin(pitch); //z position
   
  }
  else{
    vert_accel = (acc_raw(0) - g);
    //if vertical, y and z dont change

  }

  altitude = bmp.readAltitude(start_pressure); //raw altitude.

  // prediction step
  A(0,1) = time_interval;
  A(0,2) = (time_interval*time_interval*0.5);
  A(1,2) = time_interval;

  vp = A*v;
  Pp = A*P*A.transpose() + Q;
  K = Pp*H.transpose()*((H*Pp*H.transpose() + R).inverse());
  
 
  z(0) =  altitude;
  z(1) = vert_accel; //Measurement vector
  
  
  
  //update step
  v = vp + K*(z - H*vp);
  P = Pp - K*H*Pp;


  state_vector = v; //update state vector
  state_covariance = P; //update state covariance

}

bool rocket::detect_apogee(){
  if (state_vector(1) < (-2*sqrt(P(1,1)) - 1)){ // 95% confident speed is negative
    return true;
  }
  else{
    return false;
  }
}

bool rocket::detect_liftoff(){
  if (state_vector(1) > ((2*sqrt(P(1,1)))) && (state_vector(2) > 1)){ /* 95% confident speed is bigger than 10
   and acceleration is also gereater than 10 for safety*/
    return true;
  }
  else{
    return false;
  }
}

double rocket::barometric_velocity(double time_interval){

  //low pass filter to get rid of discrete nature of barometer

  alt_v_lpf = alt_v_buffer*0.95 + (0.05)*((altitude - alt_buffer)/time_interval); /*crazy agressive lpf, delayed but doesnt matter
   since this is just for landing detection*/
  alt_v_buffer = alt_v_lpf;
  alt_buffer = altitude;
  return alt_v_lpf;
}

