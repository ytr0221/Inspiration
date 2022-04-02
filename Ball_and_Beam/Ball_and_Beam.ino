#include <Servo.h>
#define Umax 66 // maximum angle of the servomotr in degrees
#define Umin -66 // minimun angle
#define Umax_rad 1.151  // maximum angle of the servomotr in radiants
#define Umin_rad -1.151 // minimun angle
#define T 0.09 // sampling time

const int echoPin2= 4; 
const int trigPin2= 3;
const int echoPin1= 6; 
const int trigPin1= 7; 

Servo servo;

double setpoint, setpoint_prec;  // In meters : 30cm --> 0.3m
double y, y_prec;
double error;
double P, I, D, U;
double I_prec=0, U_prec=0, D_prec=0;        
boolean Saturation = false;

double Kp = 8.6; 
double Ki = 1.1; 
double Kd = 6.3;  

float measure_1 (void);
float measure_2 (void);
void move_servo(int);

void setup() {

   Serial.begin(9600);
   
   pinMode(trigPin2, OUTPUT);
   pinMode(echoPin2, INPUT);
   pinMode(trigPin1, OUTPUT);
   pinMode(echoPin1, INPUT);
   servo.attach(9);   
  
   delay(1000); 
   move_servo(90);
   delay(2000);
   setpoint_prec = measure_2();  // cube
   delay(1000);
   y_prec = measure_1();  // cart
   delay(1000);
   
}

void loop() {
   
   setpoint = measure_2();  // distance of the cube from the sensor ( meters )
   setpoint = 0.53*setpoint + 0.47*setpoint_prec  ;
   
   delay(3);
   
   y = measure_1();  // distance of the cart from the sensor ( meters )   
   y =  0.53*y + 0.47*y_prec  ;   // (  alfa*y :   if alfa increases, y less attenuated and similar to the measured y  --> so the measurement is noisy but fast )
  
   delay (3);
   
   error = round( 100*(y - setpoint) )*0.01;     // meters            
   
   P = Kp*error;
   
   if ( ! Saturation )  I = I_prec + T*Ki*error;

   D = (Kd/T)*(y - y_prec);
   
   D = 0.56*D + 0.44*D_prec;    // filtering D    
   
   U = P + I + round(100*D)*0.01 ;  // U in radiants
   
   if ( U < Umin_rad)  {
                        U=Umin_rad; 
                        Saturation = true;
                       }
                   
   else if ( U > Umax_rad)  {
                             U=Umax_rad; 
                             Saturation = true;
                            }

   else     Saturation = false;                   
   
   U=round(U*180/M_PI);     // Transform U in degrees. Now I have :   -63° < U < 63°   
          
   U=map(U, Umin, Umax, 24, 156); // I map the computed value of U to the corresponding value of the servomotor
   
   if (U < 83 || U > 95 || abs(error) > 0.02 ) move_servo( round(U) );   // I continue until I have error and the control action U is greater than a threshold.
   
   delay (24);  

   //Serial.print(setpoint*100);
   //Serial.print(" ");
   //Serial.print(y*100);
   //Serial.print(" ");
   //Serial.print(U);
   //Serial.println();
   
   I_prec = I;
   y_prec = y;
   D_prec = D;
   setpoint_prec = setpoint;     
}


float measure_1 (void) {

long durata=0;
float distanza=0; 

digitalWrite(trigPin1, LOW); 
delayMicroseconds(10); 

digitalWrite(trigPin1, HIGH);
delayMicroseconds(10);
 
digitalWrite(trigPin1, LOW);

durata = pulseIn(echoPin1, HIGH);
distanza = (float)durata/58.2;

delay(30);

if (distanza > 42) distanza=43;
else if (distanza < 0) distanza=0;

return 0.01*(distanza-1.5+0.5);   // meters   

}

float measure_2 (void) {

long durata=0;
float distanza=0; 

digitalWrite(trigPin2, LOW); 
delayMicroseconds(10); 

digitalWrite(trigPin2, HIGH);
delayMicroseconds(10);
 
digitalWrite(trigPin2, LOW);

durata = pulseIn(echoPin2, HIGH);
distanza = (float)durata/58.2;

delay(30);

if (distanza > 42) distanza=43;
else if (distanza < 0) distanza=0;

return 0.01*(distanza+2);   // meters     ( +2 to measure the center of the cube )

} 

void move_servo(int u) {
  
servo.write(u-map(u, 30, 150, 14, 3));
   
}
