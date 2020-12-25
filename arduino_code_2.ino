#include <ros.h>
#include<geometry_msgs/Vector3.h>
#include<geometry_msgs/Twist.h>

#define testMotor 0
#define debugBySerial 0

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

 
ros::NodeHandle  nh;


void messageCb( const geometry_msgs::Twist & cmd_vel){

    li_x=cmd_vel.linear.x;
    an_z=cmd_vel.angular.z; 
} 

/*khoi tao mot subcribe name sub toi topic cmd_vel (goi ham messageCb)*/
ros::Subscriber<geometry_msgs::Twist> sub("cmd_vel", messageCb );

/*tao message kieu vector de truyen xung encoder*/ 
geometry_msgs::Vector3 encoder_msg;
ros::Publisher encoder("encoders", &encoder_msg);

/*cau hinh pin cho dong co*/
void configParaMotor(void);
/*Dem xung encoder trai*/ 
void CountL(void);
/*Dem xung encoder phai*/
void CountR(void); 
/*Dieu khien dong co*/
void controlMotor(void);

void setup()
{   
    #ifdef debugBySerial == 1
    Serial.begin(9600); // opens serial port, sets data rate to 9600 bps
    #else
    Serial.begin(57600); // opens serial port, sets data rate to 57600 bps
    #endif

    nh.initNode();
    nh.advertise(encoder);
    nh.subscribe(sub);
    
    configParaMotor();

    #ifdef testMotor==1
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
    #endif
}

void loop()
{
    encoder_msg.x = encoderLeft + 1;
    encoder_msg.y = encoderRight;     
    
    /*debug on arduino*/
    #ifdef debugBySerial == 1

    Serial.print(encoderLeft);
    Serial.print(" ");
    Serial.print(encoderRight);
    Serial.println("");
    Serial.println(vel_left);    

    digitalWrite(DCright_A,LOW);
    digitalWrite(DCright_B,HIGH);
    analogWrite(pwm_right,(int)vel_right );
    Serial.println(encoderRight);
    #endif 

    /*publish encoders to topic encoder */
    encoder.publish( &encoder_msg );
    encoderLeft = 0;
    encoderRight = 0;

    /*convert linear velocity and angle velocity to velocity of right/left wheel*/
    vel_right=li_x + (an_z * 0.25)/2;
    vel_left = li_x *2  - vel_right;

    /*dieu khien dong co */
    controlMotor();

    

    nh.spinOnce();
}
/***********************************************************
       Ham: void configParaMotor(void)
       chuc nang: cau hinh pin cho dong co
***********************************************************/
void configParaMotor(void)
{
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
}
/***********************************************************
       Ham: CountL(void)
       chuc nang: doc encoder cua banh trai
***********************************************************/
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

/***********************************************************
       Ham: CountR(void)
       chuc nang: doc encoder cua banh phai
***********************************************************/
void CountR(void)
{
    if(digitalRead(encoderR_PinB) == 1){
      encoderRight --; 
    }
    else{
      encoderRight ++; 
    }
}
/***********************************************************
       Ham: void controlMotor(void)
       chuc nang: Dieu khien 2 dong co
***********************************************************/
void controlMotor(void)
{
    /*Banh trai */
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

    /*Banh phai*/
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
}