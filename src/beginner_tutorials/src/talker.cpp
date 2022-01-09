#include <sstream>

#include "beginner_tutorials/PID.h"
#include "beginner_tutorials/fuzzy_ctrl.h"

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "beginner_tutorials/Monitor.h"


/*##################################################################################*/
int count = 0;
float ref = 1000;
double y = 0;
double u = 0;
double temp;
double error = 0, error_dot = 0, pre_error = 0;
/*##################################################################################*/
float set_ref(float duration, int &temp)
{
	int ref = 0;
	if(temp < (duration/2)) ref = 20;
	else if(temp < duration) ref = 5;
	else temp = 0;

	return ref;
}
/*##################################################################################*/
static double uk1 = 0;
static double uk2 = 0;
static double yk1 = 0;
static double yk2 = 0;

double a = 8.594603E-05;
double b = 8.271147E-05;
double c = -1.88986;
double d = 0.8912866;

double plant_motor(double uk)
{
	double yk = a*uk1 + b*uk2 - c*yk1 - d*yk2;
	uk1 = uk;
	uk2 = uk1;
	yk1 = yk;
	yk2 = yk1;
	
	return yk;
}
/*##################################################################################*/

int main(int argc, char **argv)
{
  	ros::init(argc, argv, "talker");
  	ros::NodeHandle n;

  	ros::Publisher chatter_pub = n.advertise<beginner_tutorials::Monitor>("thanhduong", 1000);

  	ros::Rate loop_rate(50);

  	PID *pid;
	pid = new PID(8, 4, 5, 0.01, "auto");

  	FUZZY *fuzzy;
	fuzzy = new FUZZY();

  	fuzzy->rule_fuzzy();

  	int count = 0;

  	while (ros::ok())
  	{
    	count++;
    	ref = set_ref(1000, count);
		
	  	//fuzzy compute
	  	error = ref - y;
	  	error_dot = (error - pre_error)/0.01;
	  	pre_error = error;
		
	  	fuzzy->value_hlt_e(error);
	  	fuzzy->value_hlt_e_dot(error_dot);
	  	fuzzy->find_beta();	
			
	  	pid->set_Kp(fuzzy->defuzzy_kp());
	  	pid->set_Ki(fuzzy->defuzzy_ki());
	  	pid->set_Kd(fuzzy->defuzzy_kd());

	  	u = pid->compute(ref, y);	
	  	y = plant_motor(u);

    	/*##################################################################################*/
    	// std_msgs::String msg;
    	// std::stringstream ss;
    	// ss << "ref: " << ref << "  y: " << y;
		// msg.data = ss.str();
    	// ROS_INFO("%s", msg.data.c_str());

    	std::cout<< "  ref: " << ref << "  y: " << y << "  ";
		pid->show_info();
		std::cout<<std::endl;

		beginner_tutorials::Monitor Monitor;
		Monitor.ref = ref;
		Monitor.y = y;

    	chatter_pub.publish(Monitor);
    	ros::spinOnce();
    	loop_rate.sleep();
  }
  return 0;
}