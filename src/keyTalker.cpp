#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include <string>
#include <iostream>
#include <vector>
#include <map>
#include <array>
#include <termios.h>
#include <unistd.h>

using namespace std;

string msg = "Reading from the keyboard  and Publishing to Twist!\n"
		"---------------------------\n"
		"Moving around:\n"
		"   u    i    o\n"
		"   j    k    l\n"
		"   m    ,    .\n"
		"For Holonomic mode (strafing), hold down the shift key:\n"
		"---------------------------\n"
		"   U    I    O\n"
		"   J    K    L\n"
		"   M    <    >\n"
		"t : up (+z)\n"
		"b : down (-z)\n"
		"anything else : stop\n"
		"q/z : increase/decrease max speeds by 10%\n"
		"w/x : increase/decrease only linear speed by 10%\n"
		"e/c : increase/decrease only angular speed by 10%\n"
		"1 to quit\n";







string vels(double speed, double turn)
{
	return "Currently:\tspeed " + std::to_string(speed) + "\tturn " + std::to_string(turn) + "\n";
}

int getch() {
    int ch;
    struct termios t_old, t_new;

    tcgetattr(STDIN_FILENO, &t_old);
    t_new = t_old;
    t_new.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &t_new);

    ch = getchar();

    tcsetattr(STDIN_FILENO, TCSANOW, &t_old);
    return ch;
}

int getKey(void)
{
	struct termios term;
    tcgetattr(STDIN_FILENO, &term);
    term.c_lflag &= ~ICANON;
    tcsetattr(STDIN_FILENO, TCSANOW, &term);
	return (char)getch();
}


int main(int argc, char **argv)
{
	double speed, turn, x, y, z, th;
	int status;
	char key;

	std::map<char, array<double, 4>> moveBindings;
	    
	    moveBindings['i'] = {1,0,0,0};
        moveBindings['o'] = {1,0,0,-1};
        moveBindings['j'] = {0,0,0,1};
        moveBindings['l'] = {0,0,0,-1};
        moveBindings['u'] = {1,0,0,1};
        moveBindings[','] = {-1,0,0,0};
        moveBindings['.'] = {-1,0,0,1};
        moveBindings['m'] = {-1,0,0,-1};
        moveBindings['O'] = {1,-1,0,0};
        moveBindings['I'] = {1,0,0,0};
        moveBindings['J'] = {0,1,0,0};
        moveBindings['L'] = {0,-1,0,0};
        moveBindings['U'] = {1,1,0,0};
        moveBindings['<'] = {-1,0,0,0};
        moveBindings['>'] = {-1,-1,0,0};
        moveBindings['M'] = {-1,1,0,0};
        moveBindings['t'] = {0,0,1,0};
        moveBindings['b'] = {0,0,-1,0};

    std::map<char, array<double, 2>> speedBindings;
        
        speedBindings['q'] = {1.1,1.1};
        speedBindings['z'] = {.9,.9};
        speedBindings['w'] = {1.1,1};
        speedBindings['x'] = {.9,1};
        speedBindings['e'] = {1,1.1};
        speedBindings['c'] = {1,.9};

	ros::init(argc, argv, "keyTalker");
	
	ros::NodeHandle nh;
	
	ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("set_model_state", 100);

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
				for(auto elem : speedBindings) 
					{
					speed = speed * elem.second [0];
					turn = turn * elem.second [1];
					}
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

