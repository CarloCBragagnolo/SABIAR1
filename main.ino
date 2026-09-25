#include <rocket.h>
#include <FS.h>
#include <SPIFFS.h>
#include <ArduinoEigenDense.h>

using namespace Eigen;


File file; //internal file

Matrix<double, 3, 3> Soft_iron{
  {1.465, 0.039, 0.05},
  {0.014, 1.531, -0.129},
  {-0.056, 0.07, 1.611}
};
//timekeeping
double t0 = 0;
double t = 0;
double t1;
double deltat = 0;
double t_launch;

//flight logic and file system
String line = "";
int sys_state = 0;
int velocity_counter = 0;
int runs_til_save = 30;
int down_counter =0;

rocket vehicle(1004, 9.815);


void setup() {
  Serial.begin(9600);


  //TEMPORARY, gets magnetometer to shut up
  pinMode(7, OUTPUT);
  digitalWrite(7,HIGH);

  pinMode(17,OUTPUT);
  digitalWrite(17, HIGH);
 

  vehicle.begin();
    
  delay(2000);

  vehicle.calibrate(200);
  
  if (!SPIFFS.begin(true)) {

  Serial.println("Error mounting SPIFFS");
  return;
  }
  delay(1000);
  
  pinMode(21, OUTPUT);
  vehicle.mag_offset(0) = -0.058;
  vehicle.mag_offset(1) = -0.581;
  vehicle.mag_offset(2) = -0.207;
  vehicle.Mag_correction = Soft_iron;
  t = micros(); //need to do this so first loop has valid deltat.
}

void loop() {
  
  while (sys_state < 3){
    deltat = (t - t0)/1000000;
    t0 = micros(); //millis not acc enough
    vehicle.update(deltat);
    Serial.println(String(vehicle.state_vector(0),4) + "," + String(vehicle.state_vector(1),4) + "," + String(vehicle.state_vector(2),4) + ","  + String(vehicle.forwardness, 4) + ", " + String(sqrt(vehicle.state_covariance(0,0))));
    //Serial.println(String(vehicle.barometric_velocity(deltat),4) + "," + String((deltat),6) + "," + String(vehicle.altitude));
    //Serial.println(String(vehicle.euler_attitude(0), 4) + "," + String(vehicle.euler_attitude(1), 4) + "," + String(vehicle.euler_attitude(2), 4) + ","  + String(vehicle.forwardness, 4));
    //Serial.println(String(vehicle.yz_state(0), 4) + "," + String(vehicle.yz_state(1), 4));
    //Serial.println(vehicle.orientation);
    //delay(5);
    

    if (sys_state == 0){ //ground
      if ((vehicle.detect_liftoff() && ((t/1000000) > 5))){
        sys_state += 1;
        t_launch = t;
      }
    }
    if (sys_state == 1){//pre apogee, detect and log data
      t1 = (t-t_launch)/1000000;
      line += String(t1) + "," + String(vehicle.state_vector(0),4) + "," + String(vehicle.state_vector(1),4) + "," + String(vehicle.state_vector(2),4) + ","
      + String(vehicle.altitude,4) + "," + String(vehicle.acc_raw(0),4) + "," +  String(vehicle.forwardness, 4) + "," + String(sys_state) + "\n"; 

      if (runs_til_save ==0){
      runs_til_save = 30;
      //file = SPIFFS.open("/file.txt", FILE_APPEND); //logs data every 30 reads. oppening and closing files takes too long to do it every time (10x sensor rate improvement this way)
      //file.print(line);
      //file.close();
      line = "";
      }

      if(vehicle.detect_apogee()){
        sys_state +=1;
      }
      runs_til_save -=1; 
    }
    if(sys_state == 2){ //waiting for landing
      t1 = (t-t_launch)/1000000;
        line += String(t1) + "," + String(vehicle.state_vector(0),4) + "," + String(vehicle.state_vector(1),4) + "," + String(vehicle.state_vector(2),4) + ","
        + String(vehicle.altitude,4) + "," + String(vehicle.acc_raw(0),4) + "," +  String(vehicle.forwardness, 4) + "," + String(sys_state) + "\n"; 

      if (runs_til_save ==0){
        runs_til_save = 30;
        //file = SPIFFS.open("/file.txt", FILE_APPEND); //logs data every 30 reads. oppening and closing files takes too long to do it every time (10x sensor rate improvement this way)
        //file.print(line);
        //file.close();
        line = "";
      }
      if (sq(vehicle.barometric_velocity(deltat)) < 0.25){
        down_counter +=1;
      }
      else{
        down_counter =0;
      }
      if (down_counter == 1000){
        //sys_state = 3; // landed
        velocity_counter = 0;
        //file = SPIFFS.open("/file.txt", FILE_APPEND); //log last lines of buffered data
        //file.print(line);
        //file.close();
      }
      runs_til_save -=1;
    }
    //Serial.println(String(sys_state) + "," + String(down_counter) + "," + String(vehicle.state_vector(1), 4));
    t = micros();
    }
    
    digitalWrite(21, HIGH);
  }
  