#include <ros.h>
#include<geometry_msgs/Vector3.h>
#include<geometry_msgs/Twist.h>

/*velocity of 2 wheel*/
#define pwm_left 10
#define pwm_right 11

/*direction of wheel*/
#define DCleft_A 9
#define DCleft_B 8
#define DCright_A 7
#define DCright_B 13

/*2 chanel each encode*/
int encoderL_PinA = 2;
int encoderL_PinB = 5;
int encoderR_PinA = 3;
int encoderR_PinB = 4;

/*encoder pulse*/
volatile int encoderLeft=0;
volatile int encoderRight=0; 

float li_x=0, an_z=0;
float vel_right=0, vel_left =0;
/*
 
ros::NodeHandle  nh;


void messageCb( const geometry_msgs::Twist & cmd_vel){

    li_x=cmd_vel.linear.x;
    an_z=cmd_vel.angular.z;
 
} 

ros::Subscriber<geometry_msgs::Twist> sub("cmd_vel", messageCb );


 
geometry_msgs::Vector3 encoder_msg;
ros::Publisher encoder("encoders", &encoder_msg);


*/
void CountL(void);
void CountR(void); 

void setup()
{
    Serial.begin(9600); // opens serial port, sets data rate to 9600 bps
    //nh.initNode();
    //nh.advertise(encoder);
    //nh.subscribe(sub);
    pinMode(pwm_left,OUTPUT);
    pinMode(pwm_right,OUTPUT);
    
    pinMode(DCleft_A,OUTPUT);
    pinMode(DCleft_B,OUTPUT);
    pinMode(DCright_A,OUTPUT);
    pinMode(DCright_B,OUTPUT);
  
    pinMode (encoderL_PinA,INPUT_PULLUP); 
    pinMode (encoderL_PinB,INPUT);
    pinMode (encoderR_PinA,INPUT_PULLUP);
    pinMode (encoderR_PinB,INPUT);
  
    attachInterrupt(0, CountL, FALLING);
    attachInterrupt(1, CountR, FALLING);  
    
    digitalWrite(DCleft_A,LOW);
    digitalWrite(DCleft_B,LOW);
    digitalWrite(DCright_A,LOW);
    digitalWrite(DCright_B,LOW);

    vel_left=30;
    /*tien*/
    /*digitalWrite(DCleft_A,LOW);
    digitalWrite(DCleft_B,HIGH);
    digitalWrite(DCright_A,LOW);
    digitalWrite(DCright_B,HIGH);*/
    /*lui*/
    digitalWrite(DCleft_A,HIGH);
    digitalWrite(DCleft_B,LOW);
    digitalWrite(DCright_A,HIGH);
    digitalWrite(DCright_B,LOW);
    
    analogWrite(pwm_left,(int)vel_left );
    analogWrite(pwm_right,(int)vel_left );
  
}

void loop()
{
    //encoder_msg.x = 12;
    //encoder_msg.y = 15;

     
    

    Serial.print(encoderLeft);
    Serial.print(" ");
    Serial.print(encoderRight);
    Serial.println("");
    Serial.println(vel_left);

    
/*
    digitalWrite(DCright_A,LOW);
    digitalWrite(DCright,HIGH);
    analogWrite(pwm_right,(int)vel_right );
    serial.println(encoderRight);
*/    
  /* 
    encoder.publish( &encoder_msg );
  
    vel_right=li_x + (an_z * 0.25)/2;
    vel_left = li_x *2  - vel_right;
  
    if(vel_left >0 ){
        digitalWrite(DCleft_A,LOW);
        digitalWrite(DCleft_B,HIGH);    
        analogWrite(pwm_left,vel_left );

    }
    else if(vel_left <0){
        digitalWrite(DCleft_A,HIGH);
        digitalWrite(DCleft_B,LOW);    
        analogWrite(pwm_left,-vel_left );
    }
    else{
        digitalWrite(DCleft_A,LOW);
        digitalWrite(DCleft_B,LOW); 
    }

  
    if(vel_right>0){
        digitalWrite(DCright_A,LOW);
        digitalWrite(DCright_B,HIGH);
        analogWrite(pwm_right,vel_left );
    }
    else if(vel_right<0){
        digitalWrite(DCright_A,HIGH);
        digitalWrite(DCright_B,LOW);
        analogWrite(pwm_right,vel_left );
    }
    else{
        digitalWrite(DCright_A,LOW);
        digitalWrite(DCright_B,LOW);
    }

    nh.spinOnce();
    */
    delay(500);
}


void CountL(void)
{
    if(digitalRead(encoderL_PinB) == 1){
        encoderLeft ++;
        if(encoderLeft == -30000) encoderLeft == 0;
    }
    else{
        encoderLeft --;
        if(encoderLeft == 30000) encoderLeft == 0;
    }  
}
void CountR(void)
{
    if(digitalRead(encoderR_PinB) == 1){
      encoderRight --;
      if(encoderRight == -30000) encoderRight = 0;
    }
    else{
      encoderRight ++;
      if(encoderRight == 30000) encoderRight = 0;
    }
}
