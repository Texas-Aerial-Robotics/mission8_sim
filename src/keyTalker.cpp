#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include <string>
#include <iostream>
#include <vector>
#include <map>
#include <array>
#include <termios.h>
#include <unistd.h>
#include <GL/glew.h>
#include <GLFW/glfw3.h>

using namespace std;

string msg = "Reading from the keyboard  and Publishing to Twist!\n"
		"---------------------------\n"
		"Moving around:\n"
		"   q    w    e\n"
		"   a    s    d\n"
/*
		"   m    ,    .\n"
		"For Holonomic mode (strafing), hold down the shift key:\n"
		"---------------------------\n"
		"   U    I    O\n"
		"   J    K    L\n"
		"   M    <    >\n"
		"t : up (+z)\n"
		"b : down (-z)\n"
		"anything else : stop\n"
*/
		"u/m : increase/decrease max speeds by 10%\n"
		"i/, : increase/decrease only linear speed by 10%\n"
		"o/. : increase/decrease only angular speed by 10%\n"
		"ESC to quit\n";

double speed, turn, x, y, z, th;
int status;

std::map<int, array<double, 4>> moveBindings;

std::map<int, array<double, 2>> speedBindings;

ros::Publisher pub;

string vels(double speed, double turn);

static void key_callback(GLFWwindow* window, int key, int scancode, int action, int mods)
{
    if (key == GLFW_KEY_ESCAPE && action == GLFW_PRESS)
        glfwSetWindowShouldClose(window, GLFW_TRUE);
    else if (action == GLFW_PRESS) {

    	// checks if key is movement key
    	if (moveBindings.find(key) != moveBindings.end()) {
			
			auto elem = moveBindings.find(key);
			x = elem->second [0];
			y = elem->second [1];
			z = elem->second [2];
			th = elem->second [3];
			
			}
		// checks if key adjusts speed
		else if (speedBindings.find(key) != speedBindings.end()) { 
				
			auto svalue = speedBindings.find(key);
			speed = speed * (float)svalue->second [0];
			turn = turn * (float)svalue->second [1];
			
			cout<<vels(speed,turn);
			if (status == 14) {
				cout << msg;;

			}
			status = (status + 1) % 15;
			}
		}
	else if (action == GLFW_RELEASE) {
		x = 0;
		y = 0;
		z = 0;
		th = 0;
		}
	
	// publishes twist
	geometry_msgs::Twist msg;
	msg.linear.x = x*speed;
	msg.linear.y = y*speed;
	msg.linear.z = z*speed;
	msg.angular.x = 0;
	msg.angular.y = 0;
	msg.angular.z = th*turn;
	pub.publish(msg);

}

GLFWwindow* window;

void init_glfw(){
	    if (!glfwInit())
        exit(EXIT_FAILURE);
        window = glfwCreateWindow(640, 480, "Complicated example", NULL, NULL);
    if (!window)
    {
        glfwTerminate();
        exit(EXIT_FAILURE);
    }
    glfwSetKeyCallback(window, key_callback);
    glfwMakeContextCurrent(window);
    glewInit();
    glfwSwapInterval(1);

}




string vels(double speed, double turn)
{
	return "Currently:\tspeed " + std::to_string(speed) + "\tturn " + std::to_string(turn) + "\n";
}



char getKey(void)
{
	struct termios term;
    tcgetattr(STDIN_FILENO, &term);
    term.c_lflag &= ~ICANON;
    tcsetattr(STDIN_FILENO, TCSANOW, &term);

    int ch;
    struct termios t_old, t_new;

    tcgetattr(STDIN_FILENO, &t_old);
    t_new = t_old;
    t_new.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &t_new);

    ch = getchar();

    tcsetattr(STDIN_FILENO, TCSANOW, &t_old);

	return (char)ch;
}


int main(int argc, char **argv)
{
	init_glfw();

	    
        moveBindings[69] = {0,1,0,1};		//'e'
        moveBindings[81] = {0,1,0,-1};		//'q'
        moveBindings[87] = {0,1,0,0};		//'w'
        moveBindings[65] = {-1,0,0,0};		//'a'
        moveBindings[68] = {1,0,0,0};		//'d'
        moveBindings[83] = {0,-1,0,0};		//'s'
        moveBindings[74] = {0,0,0,-1};		//'j'
        moveBindings[76] = {0,0,0,1};		//'l'



        
        speedBindings[85] = {1.1,1.1};	 	//'u'
        speedBindings[77] = {.9,.9};		//'m'
        speedBindings[73] = {1.1,1};		//'i'
        speedBindings[44] = {.9,1};			//','
        speedBindings[79] = {1,1.1};		//'o'
        speedBindings[46] = {1,.9};			//'.'

	ros::init(argc, argv, "keyTalker");
	
	ros::NodeHandle nh;
	
	pub = nh.advertise<geometry_msgs::Twist>("set_model_state", 100);

	nh.param("speed", speed, 0.5);
	nh.param("turn", turn, 1.0);


	x = 0;
	y = 0;
	z = 0;
	th = 0;
	status = 0;

	try {
		cout<<msg;
		cout<<vels;
		while (!glfwWindowShouldClose(window))
    	{
    		glfwSwapBuffers(window);
        	glfwPollEvents();
		}
		/*
		while(1) {
			key = getKey();
			if (moveBindings.find(key) != moveBindings.end()) {
				for(auto elem : moveBindings) 
					{
					x = elem.second [0];
					y = elem.second [1];
					z = elem.second [2];
					th = elem.second [3];
					}
			}
			else if (speedBindings.find(key) != speedBindings.end()) { 
				
				auto svalue = speedBindings.find(key);
				speed = speed * (float)svalue->second [0];
				turn = turn * (float)svalue->second [1];
				
				cout<<vels(speed,turn);
				if (status == 14) {
					cout << msg;;
				}
				status = (status + 1) % 15;
			}
			else {
				x = 0;
				y = 0;
				z = 0;
				th = 0;
				if (key == '1')
					break;
			}
		geometry_msgs::Twist msg;
		msg.linear.x = x*speed;
		msg.linear.y = y*speed;
		msg.linear.z = z*speed;
		msg.angular.x = 0;
		msg.angular.y = 0;
		msg.angular.z = th*turn;
		pub.publish(msg);
		}
	*/

	}
	catch (int e) {
		cout<<e;
	}
	geometry_msgs::Twist msg;
	msg.linear.x = x*speed;
	msg.linear.y = y*speed;
	msg.linear.z = z*speed;
	msg.angular.x = 0;
	msg.angular.y = 0;
	msg.angular.z = 0;
	pub.publish(msg);

	return 0;
}