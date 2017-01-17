#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <iostream>
#include <ncurses.h>
#include <math.h>

#include "std_msgs/String.h"
#include <sstream>



class Move
{
private:
  ros::NodeHandle m_nh;
  ros::Publisher cmd_pub;
  const int freq = 25; // getch blocking time, in ms (below 20 isn't working very well - no idea why)
  const float max_lin_vel = 0.5;
  const float max_ang_vel = 1.5;
  const float lin_accel = 0.1;
  const float ang_accel = 0.1;
  int input; // for reading input from terminal
  geometry_msgs::Twist cmd; // output Twist command

public:
  Move(ros::NodeHandle &nh)
  {
    m_nh = nh;
    cmd_pub = m_nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
  }

  int driveKeyboard()
  {
    std::cout << "Type a command and then press enter.  "
      "Use 'w','a','s', and 'd' to navigate and 'q' to exit.\n";

    initscr(); // get terminal environment variables
    cbreak(); // line buffering disabled; pass on everything
    keypad(stdscr, TRUE);
    timeout(freq);
    noecho(); // don't echo getch() inputs to screen

    printw("Type a command and then press enter. Use 'w','a','s', and 'd' to navigate and 'q' to exit.\n");
    refresh(); // print to screen, may or may not be necessary

    while(m_nh.ok())
    {
      // printw("input is: \r\n");
      input = getch();
      switch (input)
      {
        case KEY_UP:
        case 'w': cmd.linear.x += lin_accel; break;
        case KEY_DOWN:
        case 's': cmd.linear.x -= lin_accel; break;
        case KEY_LEFT:
        case 'a': cmd.angular.z += ang_accel; break;
        case KEY_RIGHT:
        case 'd': cmd.angular.z -= ang_accel; break;
        case 27: // escape key
        case 'q': // quit
          endwin();
          return 0;

        default:
          // linearly decline speed if no input is given:
          if(cmd.linear.x > 0)
          { cmd.linear.x -= (cmd.linear.x - lin_accel < 0) ? cmd.linear.x : lin_accel; }
          else if(cmd.linear.x < 0)
          { cmd.linear.x += (cmd.linear.x + lin_accel > 0) ? -cmd.linear.x : lin_accel; }

          if(cmd.angular.z > 0)
          { cmd.angular.z -= (cmd.angular.z - ang_accel < 0) ? cmd.angular.z : ang_accel; }
          else if(cmd.angular.z < 0)
          { cmd.angular.z += (cmd.angular.z + ang_accel > 0) ? -cmd.angular.z : ang_accel; }
      }

      if(cmd.linear.x > max_lin_vel) { cmd.linear.x = max_lin_vel; }
      if(cmd.linear.x < -max_lin_vel) { cmd.linear.x = -max_lin_vel; }
      if(cmd.angular.z > max_ang_vel) { cmd.angular.z = max_ang_vel; }
      if(cmd.angular.z < -max_ang_vel) { cmd.angular.z = -max_ang_vel; }

      std::cout << "linear: " << cmd.linear.x << "\tangular: " << cmd.angular.z << '\r' << "     " << std::flush; //write current speed command
      cmd_pub.publish(cmd);
    }
    // nocbreak(); //return terminal to "cooked" mode
    endwin();
    return 0;
  }

};



int main(int argc, char** argv)
{
  ros::init(argc, argv, "keyboard_control"); // initialize ROS node
  ros::NodeHandle nh; // create node handle
  // ros::Rate rate(5);

  Move move(nh);
  move.driveKeyboard();

  // nocbreak(); //return terminal to "cooked" mode
  // endwin();
  return 0;
}
