#include <math.h>
#include <Servo.h>
#include <TinyGPSPlus.h>
#include <SoftwareSerial.h>

Servo s1;
Servo s2;
SoftwareSerial ss(4, 3);

//these are pins for motors
const int m1p1 = 11;
const int m1p2=10;
const int m2p1=8;
const int m2p2=12;

// SUPER IMP keep all units in SI

//keep these 2 in same units keep in m 
double ugv_breadth = 10;
double wheel_radius = 10;

//distance between each latitude in m 
double lat_gap = 110567;

//tolerance for distance in m 
double tolerance = 0.00000904428;

//this is a delay parameter, sets gap between corrective movements, unit : seconds
double n = 3;

//rpm estimate, update this value with real world as regularly as possible
double rpm = 100;

typedef struct Coordinate_pair{
  double latitude,longitude;
}Coordinate_pair;

//motor functions

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

double get_rpm()
{
  //get current coordinates
  double current_latitude = get_coordinate().latitude;
  double current_longitude = get_coordinate().longitude;

  forward(2000);

  //get coordinates again 
  double new_latitude = get_coordinate().latitude;
  double new_longitude = get_coordinate().longitude;

  //calculate heading
  double current_slope = (new_latitude - current_latitude)/(new_longitude - current_longitude);

  backward(2000);

  right(2000);

  //get current coordinates
  current_latitude = get_coordinate().latitude;
  current_longitude = get_coordinate().longitude;

  forward(2000);

  //get coordinates again 
  new_latitude = get_coordinate().latitude;
  new_longitude = get_coordinate().longitude;

  //calculate heading
  double slope_upon_turning = (new_latitude - current_latitude)/(new_longitude - current_longitude);

  backward(2000);

  double angle_turned_by = atan ((slope_upon_turning - current_slope)/(1 + slope_upon_turning * current_slope));  

  double distance_turned_by = angle_turned_by * ugv_breadth * 3.1415 / 180;

  //speed of ugv in m/min
  double speed_of_turning = distance_turned_by / (2 * 60);

  double rpm = speed_of_turning/ (2 * 3.1415 * wheel_radius);

  return rpm;
}

//boundary points
double left_bottom_lat = 0;
double left_bottom_long = 0;

double right_bottom_lat = 0;
double right_bottom_long = 0;

double left_top_lat = 0;
double left_top_long = 0;

double right_top_lat = 0;
double right_top_long = 0;

//defining a new variable type with latitude and longitude

/*struct Coordinate_pair{
  double latitude,longitude;
}  Coordinate_pair;*/


/*struct Coordinate_pair{
  double latitude;
  double longitude;
};*/

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

void corrective_measures()
{ 
  //get destination coordinates
  double dest_latitude = 0;
  double dest_longitude = 0; 

  double req_ang;
  //get current coordinates
  double current_latitude = get_coordinate().latitude;
  double current_longitude = get_coordinate().longitude;

  //wait to get new coordinates
  delay(2000);

  //get coordinates again 
  double new_latitude = get_coordinate().latitude;
  double new_longitude = get_coordinate().longitude;
  
  double current_slope = (new_latitude - current_latitude)/(new_longitude - current_longitude);

  double dest_slope = (dest_latitude - new_latitude)/(dest_longitude - new_longitude);

  if (fabs(current_slope - dest_slope) > tolerance)
  {
    req_ang = atan ((dest_slope - current_slope)/(1 + dest_slope * current_slope));
  }
  else
  {
    req_ang = 0;
  }

  turn_by_degrees(req_ang);

  if (not(is_in_boundary()))
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
  double rpm = get_rpm();

  //activate motor pins
  //these are pin modes found earlier, subject to change
  //m1p1=forward, m1p2= backward m1 is the right motor
  //m2p1=backward, m2p2=forward  m2 is the left motor

  pinMode(m1p1, OUTPUT);
  pinMode(m1p2, OUTPUT);
  pinMode(m2p1, OUTPUT);
  pinMode(m2p2, OUTPUT);

  //attach motors
  // s2.attach(9);
  // s1.attach(10);

  //get destination coordinates
  double dest_latitude = 0;
  double dest_longitude = 0;

  //get current coordinates
  double cur_arr[2];
  cur_arr = get_coordinate();
  double current_latitude = cur_arr[0];
  double current_longitude = cur_arr[1];

  //move forward func here with distance param
  forward(2000);


  //get coordinates again 
  double cur_arr[2] = get_coordinate();
  double new_latitude = cur_arr[0];
  double new_longitude = cur_arr[1];

  double current_slope = (new_latitude - current_latitude)/(new_longitude - current_longitude)/;

  double dest_slope = (dest_latitude - new_longitude)/(dest_longitude - new_longitude)/ ;

  double req_ang = atan ((dest_slope - current_slope)/(1 + dest_slope * current_slope));

  Serial.println("req_ang: ");
  Serial.println(req_ang);

  turn_by_degrees(req_ang);
}

void loop()
{
  //get coordinates
  double current_latitude = get_coordinate().latitude;
  double current_longitude = get_coordinate().longitude;

  double destination_latitude = 0;
  double destination_longitude = 0;
  
  if (fabs(current_latitude - destination_latitude) < tolerance && fabs(current_longitude - destination_longitude) < tolerance)
  {
    stop_motor();
  }
  else
  {
    forward(-1);
  }

  corrective_measures();
}
