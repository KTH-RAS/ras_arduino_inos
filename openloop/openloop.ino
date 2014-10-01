/*
 *
 * openloop.ino
 * --------------------
 * Copyright : (c) 2013, Germain Haessig <germain.haessig@ens-cachan.fr>
 * Licence   : BSD3
 * Device target : Arduino Mega 2560 Rev3
 * Shields : | Official ArduinoMotorShield
 *           | Home made KTHDD2425 board
 * 
 * Main embedded project for KTH DD2425
 *   No control
 *   Silly application of what you ask
 *
 * Send back the encoders change when receive a post on "/motion/Speed"
 *
 * Subscribes on topics :
 *      | "/arduino/pwm" (msg type: ras_arduino_msgs/PWM):                  receives PWM motor commands
 *      | "/arduino/servo_motors" (msg type: ras_arduino_msgs/ServoMotors):   receives servo position commands
 *
 * Publishes on topics :
 *      | "/arduino/encoders"         (msg type: ras_arduino_msgs/Encoders):         sends Encoder values
 *      | "/arduino/adc"              (msg type: ras_arduino_msgs/ADConverter):              sends ADC values
 *      | "/arduino/battery_status"   (msg type: ras_arduino_msgs/BatteryStatus):    sends battery voltage
 */


#include <ros.h>
#include <ras_arduino_msgs/PWM.h>
#include <ras_arduino_msgs/ServoMotors.h>
#include <ras_arduino_msgs/Encoders.h>
#include <ras_arduino_msgs/ADConverter.h>
#include <ras_arduino_msgs/BatteryStatus.h>

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

/* Battery monitoring const */
#define seuil_cell 3.6
#define seuil_batt 10.8

int led_pin[6] = {38,40,42,44,46,48};

int red_pin = led_pin[3];
int green_pin = led_pin[4];
int blue_pin = led_pin[5];

/* Motor objects creation */
/* Please refer to "Motors.h" for further informations */
Motors MotorA(ChA_Dir,ChA_Pwm,ChA_Brk,ChA_CFb);  // schotch jaune
Motors MotorB(ChB_Dir,ChB_Pwm,ChB_Brk,ChB_CFb);

/* Servomotors definition */
Servo servo[8];
char servo_pin[] = {22,24,26,28,30,32,34,36};


/* Lot of global variables (nasty) */
int encoder1, encoder1_old ;
int encoder1_loc,encoder2_loc ;
int encoder2, encoder2_old ;

unsigned long time,time_old;
unsigned long t_enc, t_enc_old;
unsigned long wdtime ;

int cpt = 0;

/* ROS Use */
ros::NodeHandle  nh;

ras_arduino_msgs::Encoders encoders_msg ;
ras_arduino_msgs::ADConverter adc_msg ;
ras_arduino_msgs::BatteryStatus battery_status_msg;

ros::Publisher encoders_publisher("/arduino/encoders", &encoders_msg);  // Create a publisher to "/arduino/encoders" topic
ros::Publisher adc_publisher("/arduino/adc", &adc_msg);  // Create a publisher to "/arduino/adc" topic
ros::Publisher battery_status_publisher("/arduino/battery_status", &battery_status_msg);  // Create a publisher to "/arduino/battery_status" topic

/* Subscriber Callback */
void pwmCallback( const ras_arduino_msgs::PWM &cmd_msg){
  /* store the time for Watchdog */
  wdtime = millis() ;  
  
  /* get the time since last call */
  time_old = time ;  
  time = millis() ;
  
  /* Store encoders value and publish */
  encoder1_loc = encoder1 ;
  encoder2_loc = encoder2 ;
  
  encoders_msg.encoder1 = encoder1;
  encoders_msg.encoder2 = encoder2;
  encoders_msg.delta_encoder1 = encoder1_loc-encoder1_old ;
  encoders_msg.delta_encoder2 = encoder2_loc-encoder2_old ;
  
  encoders_msg.timestamp = time - time_old ;
  encoders_publisher.publish(&encoders_msg);
  
  /* get the speed from message and apply it */
  MotorA.Set_speed(cmd_msg.PWM1);
  MotorB.Set_speed(cmd_msg.PWM2);
  
  if(cpt<10)  {cpt++;}
  else  {
    cpt = 0;   
  }
  
  /* Store encoders value */
  encoder1_old = encoder1_loc ;
  encoder2_old = encoder2_loc ; 
}

void servoCallback(const ras_arduino_msgs::ServoMotors& params)  {
  for(int i=0;i<8;i++) {
           servo[i].write(params.servoangle[i]);
  }
}

/* Create Subscriber to "/arduino/pwm" topic. Callback function is messageSpeed */
ros::Subscriber<ras_arduino_msgs::PWM> pwm_subscriber("/arduino/pwm", &pwmCallback);

/* Create Subscriber to "/arduino/servo_motors" topic. Callback function is messageServo */
ros::Subscriber<ras_arduino_msgs::ServoMotors> servo_motors_subscriber("/arduino/servo_motors", &servoCallback);

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
         MotorA.Set_control_parameters(5, 150, 3, 1000);
         MotorB.Set_control_parameters(5, 150, 3, 1000);
         
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
         pinMode(10,INPUT);
         
         
         /* Configure servomotors pins */
         for(int i=0;i<8;i++) {
           servo[i].attach(servo_pin[i]);
           
         }
         
         /* Initialize ROS stuff */
         nh.initNode();  // initialize node
         
         nh.advertise(encoders_publisher);  // advertise on /arduino/encoders
         nh.advertise(adc_publisher);  // advertise on /arduino/adc
         nh.advertise(battery_status_publisher);  // advertise on /arduino/battery_status
         
         nh.subscribe(pwm_subscriber);  // Subscribe to /arduino/pwm
         nh.subscribe(servo_motors_subscriber);  // Subscribe to /arduino/servo_motors
         
         /* Advertise booting */
         hello_world();
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
    wdtime = millis() ;
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
   
    float v_a7 = ((float) analogRead(A7))*0.0049;   
    float v_a6 = ((float) analogRead(A6))*0.0049;
    float v_a5 = ((float) analogRead(A5))*0.0049;
    
    float v1 = v_a7*1.5106;
    float v2 = v_a6*2.9583;
    float v3 = v_a5*2.9583;
    
    battery_status_msg.cell1 = v1;
    battery_status_msg.cell2 = v2 - v1; 
    battery_status_msg.cell3 = v3 - v2;
    battery_status_msg.on_batt = digitalRead(10);
    
    float led_level = floor((v3-seuil_batt)*4);
    
    if(led_level>=5.0)
    {
      digitalWrite(red_pin, LOW);
      digitalWrite(green_pin, HIGH);
      digitalWrite(blue_pin, LOW);
    }
    
    else if(led_level>=3.0)
    {
      digitalWrite(red_pin, HIGH);
      digitalWrite(green_pin, HIGH);
      digitalWrite(blue_pin, LOW);      
    }
    
    else
    {
      digitalWrite(red_pin, HIGH);
      digitalWrite(green_pin, LOW);
      digitalWrite(blue_pin, LOW);   
    }
    
    if((battery_status_msg.cell1<seuil_cell || battery_status_msg.cell2<seuil_cell || battery_status_msg.cell3<seuil_cell) && v1>2)  {
      low_batt = true ;
    }
    
    if(low_batt && ((battery_status_msg.cell1>seuil_cell+0.2 || battery_status_msg.cell2>seuil_cell+0.2 || battery_status_msg.cell3>seuil_cell+0.2) || v1<2))  {
      low_batt = false;
    }
    
    // in this case the balance connector is disconnected, put RED LED but don't beep all the time...
    if(v_a6<0.2 && v_a5<0.2)
    {
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
    if (digitalRead(ChB_Encoder1))  encoder2++;
    else                  encoder2--;
  }
   else
  {
    if (!digitalRead(ChB_Encoder1)) encoder2++;
    else                  encoder2--;
  }
}





void interrupt3() {
  if(digitalRead(ChB_Encoder1))
  {
    if (!digitalRead(ChB_Encoder2)) encoder2++;
    else                  encoder2--;
  }
  else
  {
    if (digitalRead(ChB_Encoder2))  encoder2++;
    else                  encoder2--;
  }
}


// -----------------------------------------------------

/* END OF FILE */


