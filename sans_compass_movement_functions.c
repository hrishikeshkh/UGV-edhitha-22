#include <math.h>
#include <Servo.h>
#include <TinyGPS++.h>
#include <SoftwareSerial.h>

Servo s1;
Servo s2;

int RXPin = 4;
int TXPin = 3;
TinyGPSPlus gps;
SoftwareSerial gpsSerial(RXPin, TXPin);

//angle tolerence
double ang_tolerance = 5;

//check if reached 
bool reached = false;

//destination coordinates
double destination_latitude = 13.031297;
double destination_longitude = 77.565239;

//these are pins for motors
const int m1p1 = 11;
const int m1p2= 10;
const int m2p1= 8;
const int m2p2= 12;

// SUPER IMP keep all units in SI

//keep these 2 in same units keep in m 
double ugv_breadth = 0.185;
double wheel_radius = 0.0529;

//distance between each latitude in m 
double lat_gap = 110567;

//tolerance for distance in m 
double tolerance = 0.0104428;

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
  //Serial.println("f");
  digitalWrite(m1p1,HIGH);
  digitalWrite(m1p2,LOW);
  digitalWrite(m2p1,LOW);
  digitalWrite(m2p2,HIGH);
  if (duration !=-1){delay(duration);}
}
void backward(int duration)
{
  //Serial.println("b");
  digitalWrite(m1p1,LOW);
  digitalWrite(m1p2,HIGH);
  digitalWrite(m2p1,HIGH);
  digitalWrite(m2p2,LOW);
  if (duration !=-1){delay(duration);}
};

void right(int duration)
{
  //Serial.println("r");
  digitalWrite(m1p1,LOW);
  digitalWrite(m1p2,LOW);
  digitalWrite(m2p1,LOW);
  digitalWrite(m2p2,HIGH);
  if (duration !=-1){delay(duration);}
}
void left(int duration)
{
  //Serial.println("l");
  digitalWrite(m1p1,HIGH);
  digitalWrite(m1p2,LOW);
  digitalWrite(m2p1,LOW);
  digitalWrite(m2p2,LOW);
  if (duration !=-1){delay(duration);}
}
void stop_motor()
{ 
  //Serial.println("stopped");
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
  delay(1000);
  //get coordinates again 
  double new_latitude = get_coordinate().latitude;
  double new_longitude = get_coordinate().longitude;

  //calculate heading
  double current_ang = atan2((new_latitude - current_latitude), (new_longitude - current_longitude));
  delay(1000);

  backward(2000);

  right(2000);

  delay(1000);

  //get current coordinates
  current_latitude = get_coordinate().latitude;
  current_longitude = get_coordinate().longitude;

  forward(2000);
  delay(1000);

  //get coordinates again 
  new_latitude = get_coordinate().latitude;
  new_longitude = get_coordinate().longitude;

  //calculate heading
  double ang_upon_turning = atan2((new_latitude - current_latitude), (new_longitude - current_longitude));

  backward(2000);
  delay(1000);

  double angle_turned_by = fabs(ang_upon_turning - current_ang);  

  double distance_turned_by = angle_turned_by * ugv_breadth;

  //speed of ugv in m/min
  double speed_of_turning = (distance_turned_by / 2) * 60;

  //rpm of ugv
  double rpm = speed_of_turning/ (2 * 3.1415 * wheel_radius);
  
  Serial.println("rpm");
  Serial.println(rpm);
  return rpm;
}

// double true_angle(double current_latitude, double current_longitude, double new_latitude, double new_longitude)
// {
//   double current_slope = (new_latitude - current_latitude)/(new_longitude - current_longitude);
//   double destination_slope = (destination_latitude - current_latitude)/(destination_longitude - current_longitude);
//   double angle_turned_by = atan((destination_slope - current_slope)/(1 + (destination_slope * current_slope)));

//   if (new_latitude - current_latitude > 0 && new_longitude - current_longitude > 0)
//   {
//     return angle_turned_by;
//   }
//   else if (new_latitude - current_latitude > 0 && new_longitude - current_longitude < 0)
//   {
//     return angle_turned_by + 3.1415;
//   }
//   else if (new_latitude - current_latitude < 0 && new_longitude - current_longitude < 0)
//   {
//     return angle_turned_by + 3.1415;
//   }
//   else if (new_latitude - current_latitude < 0 && new_longitude - current_longitude > 0)
//   {
//     return angle_turned_by + 3.1415;
//   }
// }

//boundary points
double left_bottom_lat = -1000;
double left_bottom_long = -1000;

double right_bottom_lat = -1000;
double right_bottom_long = 1000;

double left_top_lat = 1000;
double left_top_long = -1000;

double right_top_lat = 1000;
double right_top_long = 1000;

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
  if (!reached){
   if (gpsSerial.available() > 0){
    //Serial.println("cond1");
    if (gps.encode(gpsSerial.read())){
    Serial.println("cond2");
      if (gps.location.isValid()){
        Serial.println("cond3");
    Serial.print("Latitude: ");
    Serial.println(gps.location.lat(), 6);
    Serial.print("Longitude: ");
    Serial.println(gps.location.lng(), 6);
    //Serial.println(gps.course.deg());

    double curren_lat = gps.location.lat();
    double curren_lon = gps.location.lng();

    Coordinate_pair coor;
    coor.latitude = curren_lat;
    coor.longitude = curren_lon;

    return coor;
    }
  /*double dest_lat=13.031297;
  double dest_long=77.565239;
  double distancem=TinyGPSPlus::distanceBetween(gps.location.lat(),gps.location.lng(),dest_lat,dest_long);
  Serial.print(distancem);*/
    }
  }  
  }

 else
 {
  Serial.println("invalid");
 }

}

void corrective_measures()
{ 

  double req_ang;
  //get current coordinates
  double current_latitude = get_coordinate().latitude;
  double current_longitude = get_coordinate().longitude;

  //wait to get new coordinates
  delay(2000);

  //get coordinates again 
  double new_latitude = get_coordinate().latitude;
  double new_longitude = get_coordinate().longitude;
  
  double current_ang = atan2((new_latitude - current_latitude), (new_longitude - current_longitude));

  double dest_ang = atan2((destination_latitude - new_latitude), (destination_longitude - new_longitude));

  if (fabs(current_ang - dest_ang) > ang_tolerance)
  {
    req_ang = dest_ang - current_ang;
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

  double turn_duration = (fabs(angle) * ugv_breadth * 60) / (2 * 3.1415 * wheel_radius * rpm);
  Serial.println("turn_duration");
  Serial.println(turn_duration);

  if (angle < 0)
  {
    right(turn_duration * 1000);
  }
  else
  {
    left(turn_duration * 1000);
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
    Serial.println("is_not_in_boundary");
  }
}

void setup()
{
  gpsSerial.begin(9600);
  delay(5000);
  Serial.begin(9600);
//  ss.begin(9600);

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
  double dest_latitude = 13.0312559;
  double dest_longitude = 77.5655982;
  delay(1000);
  //get current coordinates
  //double cur_arr[2];
  //cur_arr = get_coordinate();
  Serial.println("coordinates go here");
  double current_latitude = get_coordinate().latitude;
  double current_longitude = get_coordinate().longitude;

  //move forward func here with distance param
  forward(2000);

  delay(1000);
  //get coordinates again 
  //double cur_arr[2] = get_coordinate();
  double new_latitude = get_coordinate().latitude;
  double new_longitude = get_coordinate().longitude;

  double current_ang = atan2((new_latitude - current_latitude), (new_longitude - current_longitude));

  double dest_ang = atan2((destination_latitude - new_latitude), (destination_longitude - new_longitude));

  double req_ang = dest_ang - current_ang;

  Serial.println("req_ang: ");
  Serial.println(req_ang);

  turn_by_degrees(req_ang);
  delay(2000);
}

void loop()
{
  //get coordinates
  double current_latitude = get_coordinate().latitude;
  double current_longitude = get_coordinate().longitude;

  Serial.println("curr lat:");
  Serial.println(current_latitude);

  Serial.println("curr lon");
  Serial.println(current_longitude);
  
  Serial.println("lat del:");
  //Serial.println(fabs(current_latitude - destination_latitude));

  Serial.println("long del:");
  //Serial.println(fabs(current_longitude - destination_longitude));
  if (fabs(current_latitude - destination_latitude) < tolerance && fabs(current_longitude - destination_longitude) < tolerance)
  {
    stop_motor();
    Serial.println("reached");
    digitalWrite(LED_BUILTIN, HIGH);
    reached = true;
    exit(0);
  }
  else
  {
    forward(-1);
  }

  //corrective_measures();
}
