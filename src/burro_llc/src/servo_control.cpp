#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "i2cpwm_board/Servo.h"
#include "i2cpwm_board/ServoArray.h"
#include <vector>

int CENTER_VALUE = 333;
int SERVO_RANGE = 90;
int DIRECTION = 1;
double CMD_TIMEOUT = 5.0;

class ServoConverter {

  private:
  
    int id;
    int center_value;
    int half_range;
    int direction;
    int value_out;
    
  public:
  
    ServoConverter(int id, int center_value, int servo_range, int direction) 
    {
      this->id = id;
      this->center_value = center_value;
      this->half_range = servo_range/2;
      this->direction = direction;
      this->value_out = center_value;
    }
    
    int convertValue(double value_in)
    {
      this->value_out = this->direction*value_in*this->half_range + this->center_value;
      return this->value_out;
    }
    
    int getValue()
    {
      return this->value_out;
    }
    
    int getId()
    {
      return this->id;
    }
    
};

class ServoControl {

  private:
  
    ros::NodeHandle nh;
    ServoConverter throttle;
    ServoConverter steering;
    ros::Publisher pub;
    ros::Subscriber twist_sub;
    i2cpwm_board::ServoArray servo_msg;
    double last_time_cmd_rcv;
    double cmd_timeout;
    
  public:
  
    ServoControl() :
      nh(ros::NodeHandle()),
      pub(nh.advertise<i2cpwm_board::ServoArray>("/servos_absolute", ServoArray, 10),
      twist_sub(nh.subscribe("/cmd_vel", 10, &ServoControl::callback, this)
    {
      // Initialize output message
      for(int i{0}; i < 2; i++)
      {
        this->servo_msg.servos.push_back(i2cpwm_board::Servo());
      }
      // Set up servo and throttle converters;
      // these are kept separate so that the servo
      // values can be tuned individually as needed
      this->throttle = ServoConverter(1, CENTER_VALUE, SERVO_RANGE, DIRECTION);
      this->steering = ServoConverter(2, CENTER_VALUE, SERVO_RANGE, DIRECTION);
      
      // Record current ROS time and timeout duration
      this->last_time_cmd_rcv = ros::Time::now().toSec();
      this->cmd_timeout = CMD_TIMEOUT;
    }
    
    void callback(const geometry_msgs::Twist::ConstPtr& msg)
    {
      // Save when this message was received
      this->last_time_cmd_rcv = ros::Time::now.toSec();
      
      // Convert Twist to servo values
      this->throttle.convertValue(msg->linear.x);
      this->steering.convertValue(msg->angular.z);
      
      // Publish the message
      this->sendServoMsg();
    }
    
    void sendServoMsg()
    {
      // Save current data to servo message
      this->servo_msg.servos[0].servo = this->throttle.getId();
      this->servo_msg.servos[0].value = this->throttle.getValue();
      this->servo_msg.servos[1].servo = this->steering.getId();
      this->servo_msg.servos[1].value = this->steering.getValue();
      
      this->pub.publish(this->servo_msg);
    }
    
    bool isControllerConnected()
    {
      return (ros::Time::now.toSec() - this->last_time_cmd_rcv) < this->cmd_timeout;
    }
    
    void setActuatorsIdle()
    {
      this->throttle.convertValue(0);
      this->steering.convertValue(0);
      ROS_INFO("Setting actuators idle");
      
      // Publish the idle message
      this->sendServoMsg();
    }
    
    void run()
    {
      // Define update rate
      ros::Rate r(10);
      
      while(ros::ok())
      {
        if( !(this->isControllerConnected()) )
          this->setActuatorsIdle();
          
        ros::spinOnce();
        r.sleep();
      }
    }
};

int main(int argc, char **argv)
{
  // Start a node
  ros::init(argc, argv, "servo_control");
  servoControl control;
  control.run();

  return 0;
}
