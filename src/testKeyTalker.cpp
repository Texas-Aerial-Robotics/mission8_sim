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

std::string msg = "Reading from the keyboard  and Publishing to Twist!\n"
		"Hello friend. Do you want to play a game?"
		"---------------------------\n"
		"Moving around:\n"
		"   q    w    e\n"
		"   a    s    d\n"
		"u/m : increase/decrease max speeds by 10%\n"
		"ESC to quit\n";

// initializes list of boolean states of keyboard keys
const int KEYS = 349;
bool pressed[KEYS];

// input class to store all input received from glfw
class Input {
	int key;
	int scancode;
	int action;
	int mods;
public:
	void keys(GLFWwindow *win, int key, int scancode, int action, int mods);
};
// method to pass received input into boolean states
void Input::keys(GLFWwindow *win, int key, int scancode, int action, int mods){
	if (key == GLFW_KEY_UNKNOWN) return;
	if (action == GLFW_PRESS)
		pressed[key] = true;
	else if (action == GLFW_RELEASE)
		pressed[key] = false;
	switch(key){
	case GLFW_KEY_ESCAPE:
		if (action == GLFW_PRESS)
			glfwSetWindowShouldClose(win, true);
	}
}
double speed, turn, x, y, z, th;
int status;
ros::Publisher pub;

string vels(double speed)
{
	return "Currently:\tspeed " + std::to_string(speed) + '\n';
}
// iterates through list and changes stored velocity depending on boolean state
// also supports changing speed size
void inputHandle(void){
	for(int i = 0; i < KEYS; i++){
		if(!pressed[i]) continue;
		switch(i){
		case GLFW_KEY_SPACE:{
			z = z + 1;
			break;
		}
		case GLFW_KEY_LEFT_CONTROL:{
			z = z - 1;
			break;
		}
		case GLFW_KEY_W:{
			y = y + 1;
			break;
		}
		case GLFW_KEY_A:{
			x = x - 1;
			break;
		}
		case GLFW_KEY_S:{
			y = y - 1;
			break;
		}
		case GLFW_KEY_D:{
			x = x + 1;
			break;
		}
		case GLFW_KEY_Q:{
			y = y + 1;
			th = th - 1;
			break;
		}
		case GLFW_KEY_E:{
			y = y + 1;
			th = th + 1;
			break;
		}
		case GLFW_KEY_U:{
			speed = speed * 1.1;
			cout<<vels(speed);
			break;
		}
		case GLFW_KEY_M:{
			speed = speed * 0.9;
			cout<<vels(speed);
			break;
		}
		}

		geometry_msgs::Twist msg;
		msg.linear.x = x*speed;
		msg.linear.y = y*speed;
		msg.linear.z = z;
		msg.angular.x = 0;
		msg.angular.y = 0;
		msg.angular.z = 0;
		pub.publish(msg);

	}
}

Input input;

// glfw function that gets input and passes to Input class
static void key_callback(GLFWwindow* window, int key, int scancode, int action, int mods)
{
	input.keys(window, key,scancode,action,mods);
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

int main(int argc, char **argv)
{
	init_glfw();

	ros::init(argc, argv, "Inner Voice");
	
	ros::NodeHandle nh;
	
	pub = nh.advertise<geometry_msgs::Twist>("set_model_state", 100);

	nh.param("speed", speed, 0.5);
	nh.param("turn", turn, 1.0);


	x = 0;
	y = 0;
	z = 0;
	status = 0;
	int counter = 0;

	try {
		cout<<msg;
		cout<<vels;
		ros::Rate r(50); // 10 hz
		while (!glfwWindowShouldClose(window))
    	{
    		glfwSwapBuffers(window);
        	glfwPollEvents();
        	if (counter % 10 == 0){
        		inputHandle();
        		counter = 0;
        	}
        	counter++;
        	r.sleep();
		}
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
