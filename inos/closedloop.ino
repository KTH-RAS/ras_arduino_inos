/*
 *
 * closeloop.ino
 * --------------------
 * Copyright : (c) 2013, Germain Haessig <germain.haessig@ens-cachan.fr>
 * Licence   : BSD3
 * Device target : Arduino Mega 2560 Rev3
 * Shields : | Official ArduinoMotorShield
 *           | Home made KTHDD2425 board
 * 
 * Main embedded project for KTH DD2425
 *
 * Speed regulation is done each "/motion/Speed" callback.
 * Don't forget to initialize the controller values, and your robot's size
 * by using the "/human/Parameters" topic 
 *
 * Subscribes on topics :
 *      | "/arduino/wheel_angular_velocities"   (msg type: ras_arduino_msgs/WheelAngularVelocities)   to receive speed instructions (rad/s)
 *      | "/arduino/controller_parameters"      (msg type: ras_arduino_msgs/ControllerParams)         to receive parameters
 *      | "/arduino/servo_motors"               (msg type: ras_arduino_msgs/ServoMotors)              to receive servo desired position
 *
 * Publishes on topics :
 *      | "/arduino/encoders"         (msg type: ras_arduino_msgs/Encoders):         sends Encoder values
 *      | "/arduino/adc"              (msg type: ras_arduino_msgs/ADC):              sends ADC values
 *      | "/arduino/battery_status"   (msg type: ras_arduino_msgs/BatteryStatus):    sends battery voltage
 *      | "/arduino/odometry"         (msg type: ras_arduino_msgs/Odometry):         sends odometry
 */


#include <ros.h>
#include <ras_arduino_msgs/WheelAngularVelocities.h>
#include <ras_arduino_msgs/Odometry.h>
#include <ras_arduino_msgs/Encoders.h>
#include <ras_arduino_msgs/ADC.h>
#include <ras_arduino_msgs/BatteryStatus.h>
#include <ras_arduino_msgs/ControllerParams.h>
#include <ras_arduino_msgs/ServoMotors.h>
#include <std_msgs/Header.h>

#include <stdint.h>

#include <Motors.h>
#include <math.h>

#include <Servo.h> 

#include <music.h> 

/* PinOut definition */
#define ChA_Dir 12
#define ChA_Pwm 3
#define ChA_Brk 9
#define ChA_CFb A0

#define ChB_Dir 13
#define ChB_Pwm 11
#define ChB_Brk 8
#define ChB_CFb A1

#define ChA_Encoder1 20
#define ChA_Encoder2 21

#define ChB_Encoder1 18
#define ChB_Encoder2 19

#define N 10 // Averaging of measured speed with the N last values

/* Battery monitoring const */
#define seuil_cell 3.6
#define seuil_batt 10.8

int led_pin[6] = {38,40,42,44,46,48};

/* Motor objects creation */
/* Please refer to "Motors.h" for further informations */
Motors MotorA(ChA_Dir,ChA_Pwm,ChA_Brk,ChA_CFb);  // schotch jaune
Motors MotorB(ChB_Dir,ChB_Pwm,ChB_Brk,ChB_CFb);

/* ServoMotors definition */
Servo servo[8];
char servo_pin[] = {22,24,26,28,30,32,34,36};

/* Lot of global variables (nasty) */
float W1_cons = 0;
float W2_cons = 0;

float r = 38.9E-3 ;
float B = 0.1867 ;
float r_r = 0.5*0.0777;
float r_l = 0.5*0.0779;

float Te = 10*1E-3 ;

float V_lin = 0;
float V_rot = 0;

float x=0;
float y=0;
float theta=0;

int encoder1, encoder1_old ;
int encoder1_loc,encoder2_loc ;
int encoder2, encoder2_old ;

unsigned long time,time_old;
unsigned long t_enc, t_enc_old;
unsigned long wdtime ;

int cpt = 0;

/* Specific parameters for NL control */ 
//#define NLCTRL // comment to use a classic Proportional-Integral *speed* control
#ifdef NLCTRL
float x_ghost=0;
float y_ghost=0;
float theta_ghost=0;
float z1,z2,z3;
float V_lin_ghost,V_rot_ghost;
float u1,u2;
int k1=10;k2=150;k3=80;
#endif


/* ROS Use */
ros::NodeHandle  nh;

ras_arduino_msgs::Encoders encoders_msg ;
ras_arduino_msgs::Odometry odom_msg ;
ras_arduino_msgs::ADC adc_msg ;
ras_arduino_msgs::BatteryStatus battery_status_msg;

ros::Publisher encoders_publisher("/arduino/encoders", &encoders_msg);  // Create a publisher to "/arduino/encoders" topic
ros::Publisher odometry_publisher("/arduino/odometry", &odom_msg);  // Create a publisher to "/arduino/odometry" topic
ros::Publisher adc_publisher("/arduino/adc", &adc_msg);  // Create a publisher to "/arduino/adc" topic
ros::Publisher battery_status_publisher("/arduino/battery_status", &battery_status_msg);  // Create a publisher to "/arduino/battery_status" topic

/* Subscriber Callback */
void wheelAngularVelocitiesCallback( const ras_arduino_msgs::WheelAngularVelocities& cmd_msg){
  static int i = 0;
  static float array1[N],array2[N];
  static float loc1=0,loc2=0;
  wdtime = millis() ;  // store the time for Watchdog
  /* get the time from message header */
  time_old = time ;  
  time = cmd_msg.header.stamp.nsec ;
  if(time>time_old)	{
    Te = (time - time_old)*1E-9 ;
  }
  else	{
    Te = (1E9 + time - time_old)*1E-9;
  }
  
  /* get the speed from message */
  W1_cons = cmd_msg.W1 ;
  W2_cons = cmd_msg.W2 ;
  
  /* Control loop */
  encoder1_loc = encoder1 ;
  encoder2_loc = encoder2 ;

  encoders_msg.encoder1 = encoder1;
  encoders_msg.encoder2 = encoder2;
  encoders_msg.delta_encoder1 = encoder1_loc-encoder1_old ;
  encoders_msg.delta_encoder2 = encoder2_loc-encoder2_old ;
  
  /* get the time since last call */
  t_enc_old = t_enc ;  
  t_enc = millis() ;

  encoders_msg.timestamp = t_enc - t_enc_old ;
  encoders_publisher.publish(&encoders_msg);
  
  MotorA.Read_speed(encoder1_loc,encoder1_old,Te);
  MotorB.Read_speed(encoder2_loc,encoder2_old,Te);
  
  /* Compute odometry */
  V_lin = 1.0/2*(r_r*MotorA._speed-r_l*MotorB._speed);
  V_rot = 1.0/B*(r_r*MotorA._speed+r_l*MotorB._speed);
  
  // Speed averaging
  array1[i]=V_lin;
  array2[i]=V_rot;
  
  if(i<N-1)  {i++;}
  else  {i=0;}
  
  for(int k=0;k<N;k++)  {
    loc1+=array1[k];
    loc2+=array2[k];
  }
  
  V_lin = loc1/N;
  V_rot = loc2/N;
  
  loc1=0;
  loc2=0;
  
  theta+= V_rot*Te;
  x+= V_lin*Te*cos(theta);
  y+= V_lin*Te*sin(theta);
  
  
  
  #ifdef NLCTRL 
  /* Compute ghost position */
  V_lin_ghost = r/2*(W1_cons+W2_cons);
  V_rot_ghost = r/B*(W2_cons-W1_cons);
  
  theta_ghost+= V_rot_ghost*Te;
  x_ghost+= V_lin_ghost*Te*cos(theta_ghost);
  y_ghost+= V_lin_ghost*Te*sin(theta_ghost);
  
  /* Will try to follow ghost */
  z1 = (x-x_ghost)*cos(theta_ghost)+(y-y_ghost)*sin(theta_ghost);
  z2 = -(x-x_ghost)*sin(theta_ghost)+(y-y_ghost)*cos(theta_ghost);
  z3 = tan(theta-theta_ghost);
  
  u1 = -k1*abs(V_lin)*(z1+z2+z3);
  u2 = -k2*V_lin*z2-k3*abs(V_lin)*z3;
  
  V_lin_ghost = (u1+V_lin)/cos(theta-theta_ghost);
  V_rot_ghost = u2*(cos(theta-theta_ghost))^2+V_rot;
  
  W1_cons = 1/r*V_lin_ghost+B/(2*r)*V_rot_ghost;
  W2_cons = 1/r*V_lin_ghost-B/(2*r)*V_rot_ghost;
  
  /*
  W1_cons = 1/r*V_lin_ghost+B/(2*r)*V_rot_ghost;
  W2_cons = 1/r*V_lin_ghost-B/(2*r)*V_rot_ghost;
  */
  #endif
  
  MotorA.Speed_regulation(W1_cons,Te,encoder1_loc,encoder1_old);
  MotorB.Speed_regulation(-W2_cons,Te,encoder2_loc,encoder2_old);
  
  if(cpt<10)  {cpt++;}
  else  {
    cpt = 0;
    
    /* Publish Odometry */
    odom_msg.x = x ;
    odom_msg.y = y;
    odom_msg.theta = theta ;       
    
    /* Publish Odometry and sensors value */
    odometry_publisher.publish(&odom_msg);
  }
  
  /* Store encoders value */
  encoder1_old = encoder1_loc ;
  encoder2_old = encoder2_loc ; 
}

void controllerParamsCallback(const ras_arduino_msgs::ControllerParams& params)  {
  MotorA.Set_control_parameters(params.K,params.KI,params.INT_MAX,params.ticks);
  MotorB.Set_control_parameters(params.K,params.KI,params.INT_MAX,params.ticks);
  r = params.r;
  r_r = params.r_r;
  r_l = params.r_l;
  B = params.B;
}

void servoMotorsCallback(const ras_arduino_msgs::ServoMotors& params)  {
  for(int i=0;i<8;i++) {
           servo[i].write(params.servoangle[i]);
  }
}

/* Create Subscriber to "/arduino/wheel_angular_velocities" topic. Callback function is wheelAngularVelocitiesCallback */
ros::Subscriber<ras_arduino_msgs::WheelAngularVelocities> wheel_angular_velocities_subscriber("/arduino/wheel_angular_velocities", &wheelAngularVelocitiesCallback);

/* Create Subscriber to "/arduino/servo_motors" topic. Callback function is servoMotorsCallback */
ros::Subscriber<ras_arduino_msgs::ServoMotors> servo_motors_subscriber("/arduino/servo_motors", &servoMotorsCallback);

/* Create Subscriber to "/arduino/controller_parameters" topic. Callback function is controllerParametersCallback */
ros::Subscriber<ras_arduino_msgs::ControllerParams> controller_params_subscriber("/arduino/controller_params", &controllerParamsCallback);

void hello_world()  {
  for(int m=0;m<6;m++)  {
      digitalWrite(led_pin[m],HIGH);
      delay(50);
      digitalWrite(led_pin[m],LOW);
  }
  for(int m=4;m>=0;m--)  {
      digitalWrite(led_pin[m],HIGH);
      delay(50);
      digitalWrite(led_pin[m],LOW);
  }
  tone(7,700,50);
  //play_starwars();
  //play_tetris();
  
}
  
void setup()  {  
            
         /* Set the motors in stby */  
         MotorA.Set_speed(0);
         MotorB.Set_speed(0);
         
         /* Set the good parameters */
         MotorA.Set_control_parameters(5, 0, 3, 1000);
         MotorB.Set_control_parameters(5, 0, 3, 1000);
         
         /* Define interruptions, on changing edge */
         attachInterrupt(3,interrupt1,CHANGE);  // A
         attachInterrupt(2,interrupt2,CHANGE);  // A
         attachInterrupt(5,interrupt4,CHANGE);  // B
         attachInterrupt(4,interrupt3,CHANGE);  // B
         
         /* define the outputs pins */
         for(int i=0;i<6;i++) {
           pinMode(led_pin[i],OUTPUT);
         }
         pinMode(7,OUTPUT);
         
         
         /* Configure servomotors pins */
         for(int i=0;i<8;i++) {
           servo[i].attach(servo_pin[i]);
           
         }
         
         /* Initialize ROS stuff */
         nh.initNode();  // initialize node
         
         nh.advertise(encoders_publisher);  // advertise on /arduino/encoders
         nh.advertise(odometry_publisher);  // advertise on /arduino/odometry         
         nh.advertise(adc_publisher);  // advertise on /arduino/adc
         
         nh.subscribe(wheel_angular_velocities_subscriber);  // Subscribe to /arduino/wheel_angular_velocities
         nh.subscribe(servo_motors_subscriber);  // Subscribe to /arduino/servo_motors
         nh.subscribe(controller_params_subscriber);  // Subscribe to /arduino/controller_params
         
         /* Advertise booting */
         hello_world();
         
         servo[0].write(0);
         delay(1000);
         servo[0].write(180);
}



/*****************************
* Main Loop 
*
* Watchdog timer : if no message recieved during 2s, set the motors in stby,
* computer may have crashed.
******************************/
void loop()  {
  static unsigned long t;
  static unsigned long t_ADC;
  static boolean low_batt = false ;
  nh.spinOnce();
  /* Watchdog timer */
  if(millis()-wdtime > 2000)  { 
    MotorA.Set_speed(0);
    MotorB.Set_speed(0);
    wdtime = millis();
  }
  

  /* Read IR sensors value every 100ms */
  if(millis()-t_ADC>100)
  { 
    adc_msg.ch1 = analogRead(A8);
    adc_msg.ch2 = analogRead(A9);
    adc_msg.ch3 = analogRead(A10);
    adc_msg.ch4 = analogRead(A11);
    adc_msg.ch5 = analogRead(A12);
    adc_msg.ch6 = analogRead(A13);
    adc_msg.ch7 = analogRead(A14);
    adc_msg.ch8 = analogRead(A15);  
    
    /* Publish sensor value */
    adc_publisher.publish(&adc_msg);
    t_ADC = millis();
  }

    if(millis()-t > 1000)  {
    float v1 = analogRead(A7)*0.0049*1.5106;
    float v2 = analogRead(A6)*0.0049*2.9583;
    float v3 = analogRead(A5)*0.0049*2.9583;
    
    battery_status_msg.cell1 = v1;
    battery_status_msg.cell2 = v2 - v1; 
    battery_status_msg.cell3 = v3 - v2;
    battery_status_msg.on_batt = digitalRead(10);
    
    for(int m=0;m<floor((v3-seuil_batt)*4);m++)  {
      digitalWrite(led_pin[m],HIGH);
    }
    for(int m=floor((v3-seuil_batt)*4);m<6;m++)  {
      digitalWrite(led_pin[m],LOW);
    }
    
    if((battery_status_msg.cell1<seuil_cell || battery_status_msg.cell2<seuil_cell || battery_status_msg.cell3<seuil_cell) && v1>2)  {
      low_batt = true ;
    }
    
    if(low_batt && ((battery_status_msg.cell1>seuil_cell+0.2 || battery_status_msg.cell2>seuil_cell+0.2 || battery_status_msg.cell3>seuil_cell+0.2) || v1<2))  {
      low_batt = false;
    }
    
    if(low_batt)  {
      tone(7,440,500); 
    }
    
    battery_status_publisher.publish(&battery_status_msg);
    t = millis();
  }
    
}

/*********************************************
 *
 *  Interruption subroutines
 *  Called each changing edge of an encoder
 *
 *********************************************/

void interrupt2() {
  if(digitalRead(ChA_Encoder2))
  {
    if (digitalRead(ChA_Encoder1))  encoder1--;
    else                  encoder1++;
  }
   else
  {
    if (!digitalRead(ChA_Encoder1)) encoder1--;
    else                  encoder1++;
  }
}





void interrupt1() {
  if(digitalRead(ChA_Encoder1))
  {
    if (!digitalRead(ChA_Encoder2)) encoder1--;
    else                  encoder1++;
  }
  else
  {
    if (digitalRead(ChA_Encoder2))  encoder1--;
    else                  encoder1++;
  }
}

void interrupt4() {
  if(digitalRead(ChB_Encoder2))
  {
    if (digitalRead(ChB_Encoder1))  encoder2--;
    else                  encoder2++;
  }
   else
  {
    if (!digitalRead(ChB_Encoder1)) encoder2--;
    else                  encoder2++;
  }
}





void interrupt3() {
  if(digitalRead(ChB_Encoder1))
  {
    if (!digitalRead(ChB_Encoder2)) encoder2--;
    else                  encoder2++;
  }
  else
  {
    if (digitalRead(ChB_Encoder2))  encoder2--;
    else                  encoder2++;
  }
}


// -----------------------------------------------------

/* END OF FILE */


