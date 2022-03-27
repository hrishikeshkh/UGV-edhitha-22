#include <math.h>
#include <Servo.h>
#include <TinyGPS++.h>
#include <SoftwareSerial.h>
#include <MPU9250.h>

Servo s1;
Servo s2;
SoftwareSerial ss(4, 3);
TinyGPSPlus gps;  
int RXPin = 4;
int TXPin = 3;

int GPSBaud = 9600;
SoftwareSerial gpsSerial(RXPin, TXPin);

// an MPU9250 object with the MPU-9250 sensor on I2C bus 0 with address 0x68
MPU9250 IMU(Wire,0x68);
int status;

struct Coordinate_pair{
  double latitude,longitude;
};
//these are pins for motors
const int m1p1=8;
const int m1p2=12;
const int m2p1=10;
const int m2p2=11;

// SUPER IMP keep all units in SI

//get destination coordinates
double destination_latitude = 0;
double destination_longitude = 0;

//keep these 2 in same units keep in m 
double ugv_breadth = 10;
double wheel_radius = 10;

//distance between each latitude in m 
double lat_gap = 110567;

//tolerance for distance in m 
long double tolerance = 0.00000904428;

//tolerance for angle in degrees
double ang_tolerance = 5;

//this is a delay parameter, sets gap between corrective movements, unit : seconds
double n = 3;

//rpm estimate, update this value with real world as regularly as possible
double rpm = 100;

//old motor functions for servo, find the new ones below this

// void stop()
// {
//   s1.writeMicroseconds(1500);
//   s2.writeMicroseconds(1500);
// }

// void backward(double duration){
//   s1.writeMicroseconds(1000);
//   s2.writeMicroseconds(2000);
//   if(duration!=-1) delay(duration);
// }

// void forward(double duration){
//   s1.writeMicroseconds(2000);
//   s2.writeMicroseconds(1000);
//   if(duration!=-1) delay(duration);
// }

// void right(double duration){
//   s1.writeMicroseconds(1500);
//   s2.writeMicroseconds(1000);
//   if(duration!=-1) delay(duration);
// }

// void left(double duration){
//   s1.writeMicroseconds(2000);
//   s2.writeMicroseconds(1500);
//   if(duration!=-1) delay(duration);
// }

void forward(int duration)
{
  digitalWrite(m1p1,HIGH);
  digitalWrite(m1p2,LOW);
  digitalWrite(m2p1,LOW);
  digitalWrite(m2p2,HIGH);
  if (duration !=-1){delay(duration);}
}
void backward(int duration)
{
  digitalWrite(m1p1,LOW);
  digitalWrite(m1p2,HIGH);
  digitalWrite(m2p1,HIGH);
  digitalWrite(m2p2,LOW);
  if (duration !=-1){delay(duration);}
};

void right(int duration)
{
  digitalWrite(m1p1,LOW);
  digitalWrite(m1p2,LOW);
  digitalWrite(m2p1,LOW);
  digitalWrite(m2p2,HIGH);
  if (duration !=-1){delay(duration);}
}
void left(int duration)
{
  digitalWrite(m1p1,HIGH);
  digitalWrite(m1p2,LOW);
  digitalWrite(m2p1,LOW);
  digitalWrite(m2p2,LOW);
  if (duration !=-1){delay(duration);}
}
void stop_motor()
{
  digitalWrite(m1p1,LOW);
  digitalWrite(m1p2,LOW);
  digitalWrite(m2p1,LOW);
  digitalWrite(m2p2,LOW);
}

//function to get the current heading with IMU values
double get_current_heading()
{
  //read sensor of IMU
   IMU.readSensor();
  
  //these used to calc angle
  float heading;
  float magX ;
  float magY;
  float magZ;

  magX= (IMU.getMagX_uT());
  magY= (IMU.getMagY_uT());
  magZ= (IMU.getMagZ_uT());

  heading = atan2(magY, magZ);
  
  //adjusts declination
  float declinationAngle = -0.0214;
  heading += declinationAngle;

  //Correct for heading < 0deg and heading > 360deg
  if (heading < 0){
    heading += 2 * PI;
  }
  if (heading > 2 * PI){
    heading -= 2 * PI;
  }

  // Convert to degrees
  double headingDegrees = (((heading * 180/M_PI) - 127)*2); 
  
  //you may need these
  //Serial.print("MagX: ");  
  //Serial.print(magX, 6);
  //Serial.print("\t");  
  //Serial.print("MagY: ");
  //Serial.print(magY, 6);
  //Serial.print("\t");
  //Serial.print("MagZ: ");  
  //Serial.println(IMU.getMagZ_uT(),6);

  //final headings 

  //Serial.print("Heading: ");
  //Serial.print(heading);
  Serial.print("HeadingDegrees: " );
  Serial.println((headingDegrees-127)*2);

  //return heading degrees
  return (headingDegrees);
}

Coordinate_pair get_coordinate()
{
   if (gps.encode(gpsSerial.read())){
      if (gps.location.isValid()){
    Serial.print("Latitude: ");
    Serial.println(gps.location.lat(), 6);
    Serial.print("Longitude: ");
    Serial.println(gps.location.lng(), 6);
  //Coordinate_pair coordinates;
  Coordinate_pair coor;
  
  coor.latitude = gps.location.lat();
  coor.longitude= gps.location.lng();
  
  return coor;
}
}
}


void turn_till_facing(double dest_angle_func, double current_heading_func_ang)
{
  //turn till we face, 0 when not aligned and 1 when facing
  int facing = 0;

  //turn until facing
  while (facing == 0)
  {
    //turn by a small amount 
    if (dest_angle_func - current_heading_func_ang < 0)
  {
    left(20);
  }
    else
  {
    right(20);
  }
  
   if (fabs(dest_angle_func - current_heading_func_ang) < ang_tolerance)
    {
    facing = 1;
    break;
    }
  }
}

/* double get_rpm()
{
  //get current coordinates
  Coordinate_pair coord = get_coordinate();
  double current_latitude = coord.latitude;
  double current_longitude = coord.longitude;

  forward(2000);
  coord = get_coordinate();
  //get coordinates again 
  //double new_latitude = coord.latitude;
  //double new_longitude = coord.longitude;

  //calculate heading
//  double current_slope = (new_longitude - current_longitude)/ (new_latitude - current_latitude);

  backward(2000);

  right(2000);

  //get current coordinates
  current_latitude = get_coordinate().latitude;
  current_longitude = get_coordinate().longitude;

  forward(2000);

  //get coordinates again 
//  new_latitude = get_coordinate().latitude;
  //new_longitude = get_coordinate().longitude;

  //calculate heading
  //double slope_upon_turning = (new_longitude - current_longitude)/ (new_latitude - current_latitude);

  backward(2000);

  double angle_turned_by = atan ((slope_upon_turning - current_slope)/(1 + slope_upon_turning * current_slope));  

  double distance_turned_by = angle_turned_by * ugv_breadth * 3.1415 / 180;

  //speed of ugv in m/min
  double speed_of_turning = distance_turned_by / (2 * 60);

  double rpm = speed_of_turning/ (2 * 3.1415 * wheel_radius);

  return rpm;
}
*/

//boundary points
double left_bottom_lat = 0;
double left_bottom_long = 0;

double right_bottom_lat = 0;
double right_bottom_long = 0;

double left_top_lat = 0;
double left_top_long = 0;

double right_top_lat = 0;
double right_top_long = 0;



void corrective_measures()
{ 
  //double req_ang;

  //get current coordinates
  double current_latitude = get_coordinate().latitude;
  double current_longitude = get_coordinate().longitude;

  //wait to get new coordinates
  delay(2000);

  // get coordinates again 
  // double new_latitude = get_coordinate().latitude;
  // double new_longitude = get_coordinate().longitude;
  
  // double current_slope = (new_longitude - current_longitude)/ (new_latitude - current_latitude);

  double dest_slope = (destination_longitude - current_longitude)/ (destination_latitude - current_latitude);

  double dest_angle = (atan(dest_slope) * (180/M_PI)) ;

  // if (fabs(current_slope - dest_slope) > tolerance)
  // {
  //   req_ang = (atan((dest_slope - current_slope)/(1 + dest_slope * current_slope)) * 180 / M_PI) ;
    
  // }
  // else
  // {
  //   req_ang = 0;
  // }

  // turn_by_degrees(req_ang);

  double current_heading = get_current_heading();

  if (fabs(current_heading - dest_angle) > ang_tolerance)
  {
    turn_till_facing(dest_angle, current_heading);
  }

  if (!(is_in_boundary()))
  {
    //add tolarance to this, MUST DO
    stop_motor();
  }
}



void turn_by_degrees(double angle){

  double turn_duration = (angle * ugv_breadth * 60) / (2 * 180 * wheel_radius * rpm);
  if (angle > 0)
  {
    right(turn_duration);
  }
  else
  {
    left(turn_duration);
  }
}

boolean is_in_boundary()
{
  double current_latitude = get_coordinate().latitude;
  double current_longitude = get_coordinate().longitude;

  if (current_latitude > left_bottom_lat && current_latitude < right_bottom_lat && current_longitude > left_bottom_long && current_longitude < right_bottom_long)
  {
    return true;
  }
  else
  {
    return false;
  }
}

void setup()
{
  Serial.begin(9600);
  ss.begin(9600);
  gpsSerial.begin(GPSBaud);
  //double rpm = get_rpm();

  //activate motor pins
  //these are pin modes found earlier, subject to change
  //m1p1=forward, m1p2= backward m1 is the right motor
  //m2p1=backward, m2p2=forward  m2 is the left motor

  pinMode(m1p1, OUTPUT);
  pinMode(m1p2, OUTPUT);
  pinMode(m2p1, OUTPUT);
  pinMode(m2p2, OUTPUT); 

  // start communication with IMU 
  status = IMU.begin();
  
  //gives imu error if any
  if (status < 0) {
    Serial.println("IMU initialization unsuccessful");
    Serial.println("Check IMU wiring or try cycling power");
    Serial.print("Status: ");
    Serial.println(status);
    while(1) {}
  }
  //attach motors
  // s2.attach(9);
  // s1.attach(10);

  pinMode(LED_BUILTIN, OUTPUT);
  
  //wait for coordinates to be updated
  
  Coordinate_pair c1;
  c1 = get_coordinate();
  double current_latitude = c1.latitude;
  double current_longitude = c1.longitude;

  /*//move forward func here with distance param
  // forward(2000);


  // //get coordinates again 
  // Coordinate_pair c2;
  // c2 = get_coordinate();
  // double new_latitude = c2.latitude;
  // double new_longitude = c2.longitude;*/

  //double current_slope = (new_longitude - current_longitude)/ (new_latitude - current_latitude);

  //turn till horizontal
  while (true)
  { 
    //get current coordinates
    Coordinate_pair c; 

    //check if left or right of destination
    c = get_coordinate();
    double current_latitude = c.latitude;
    double current_longitude = c.longitude;
    if (current_latitude > destination_latitude)
    {
      //turn left
      left(20);
      delay(200);
      double current_header = get_current_heading();
      if (fabs(current_header + 90) < ang_tolerance)
      {
        break;
      }
    }
    else
    {
      //turn right
      right(20);
      delay(200);
      double current_header = get_current_heading();
      if (fabs(current_header - 90) < ang_tolerance)
      {
        break;
      }
    }
  }

  //move till reached destination
  while(true)
  {
    forward(2000);
    Coordinate_pair current_coor = get_coordinate();
    if (fabs(current_coor.longitude - destination_longitude) < tolerance)
    {
      break;
    }
  }

  //turn till vertical
  while (true)
  {
    //get current coordinates
    Coordinate_pair c;
    c = get_coordinate();
    double current_latitude = c.latitude;
    double current_longitude = c.longitude;

    //check if above or below of destination
    if (current_latitude > destination_latitude)
    {
      //turn left
      left(20);
      delay(200);
      double current_header = get_current_heading();
      if (fabs(current_header - 180) < ang_tolerance)
      {
        break;
      }
    }
    else
    {
      //turn right
      right(20);
      delay(200);
      double current_header = get_current_heading();
      if (fabs(current_header - 0) < ang_tolerance)
      {
        break;
      }
    }

    left(20);
    delay(200);
    double current_header = get_current_heading();
    if (fabs(current_header - 0) < ang_tolerance)
    {
      break;
    }
  }

  //move till reached destination
  while(true)
  {
    forward(2000);
    Coordinate_pair current_coor = get_coordinate();
    if (fabs(current_coor.latitude - destination_longitude) < tolerance)
    {
      break;
    }
  }
  //double req_ang = (atan((dest_slope - current_slope)/(1 + dest_slope * current_slope))* 180/ M_PI) + 180;

  // turn_by_degrees(req_ang);
}

void loop()
{
  //get coordinates
}
