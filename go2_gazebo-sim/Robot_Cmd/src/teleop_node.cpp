#include "ros/ros.h"
#include "unitree_legged_msgs/keyboardCmd.h"
#include <termios.h>
#include <iostream>
#include <fcntl.h>
#include <signal.h>

int kfd = 0;
struct termios cooked, raw;
double linear_speed = 0.0;
double angular_speed = 0.0;
double lateral_speed = 0.0;
double roll = 0.0;
double pitch = 0.0;
double step_frequency = 0.64;
double step_Height = 0.06;
double body_height = 0.35;
int gait = 0, startFlag = 0, cppAD = 0;

ros::Publisher pub;
unitree_legged_msgs::keyboardCmd rlCmd;

void restoreTerminal(int)
{
  tcsetattr(kfd, TCSANOW, &cooked);
  ros::shutdown();
}

int setNonBlocking(int fd)
{
  int flags = fcntl(fd, F_GETFL, 0);
  if (flags == -1)
    return -1;

  flags |= O_NONBLOCK;
  if (fcntl(fd, F_SETFL, flags) == -1)
    return -1;

  return 0;
}

void printCmdINFO(){
  std::cout << "\n" << std::endl;
  std::cout << "----------- Command Change -----------"<< std::endl;
  if(startFlag)
    std::cout << "Algorithm startup : " << "true" << std::endl;
  else
    std::cout << "Algorithm startup : " << "false" << std::endl;
  if(cppAD)
    std::cout << "CPP AD            : " << "true " << std::endl;
  else
    std::cout << "CPP AD            : " << "false " << std::endl;
  std::cout << "Step Height       : " << step_Height << " m "<< std::endl;
  std::cout << "Body Height       : " << body_height << " m "<< std::endl;
  std::cout << "Forward speed     : " << linear_speed << " m/s "<< std::endl;
  std::cout << "Lateral speed     : " << lateral_speed << " m/s "<< std::endl;
  std::cout << "Angular speed     : " << angular_speed << " rad/s "<< std::endl;
  std::cout << "Roll angle        : " << roll << " rad "<< std::endl;
  std::cout << "Pitch angle       : " << pitch << " rad "<< std::endl;
  std::cout << "Gait type         : " << gait << std::endl;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "robot_cmd");
  std::cout << "----------- Robot Mode Control Starting ... -----------"<< std::endl;
  ros::NodeHandle nh;

  // Initialize the keyboard input
  kfd = fileno(stdin);
  tcgetattr(kfd, &cooked);
  memcpy(&raw, &cooked, sizeof(struct termios));
  raw.c_lflag &= ~(ICANON | ECHO);
  raw.c_cc[VEOL] = 1;
  raw.c_cc[VEOF] = 2;
  tcsetattr(kfd, TCSANOW, &raw);
  setNonBlocking(kfd);  // Set non-blocking mode for keyboard input
  pub = nh.advertise<unitree_legged_msgs::keyboardCmd>("/quad_cmd_vel", 10);

  signal(SIGINT, restoreTerminal);

  ros::Rate loop_rate(100);  // Adjust the loop rate as needed

  printCmdINFO();

  while (ros::ok())
  {
    char c;
    if (read(kfd, &c, 1) == 1)
    {
      switch (c)
      {
        case 'w':
          linear_speed += 0.1;
          printCmdINFO();
          break;
        case 's':
          linear_speed -= 0.1;
          printCmdINFO();
          break;
        case 'a':
          angular_speed += 0.1;
          printCmdINFO();
          break;
        case 'd':
          angular_speed -= 0.1;
          printCmdINFO();
          break;
        case 'j':
          lateral_speed += 0.1;
          printCmdINFO();
          break;
        case 'l':
          lateral_speed -= 0.1;
          printCmdINFO();
          break;
        case 'i':
          pitch += 0.1;
          if(pitch > 0.5) pitch = 0.5;
          printCmdINFO();
          break;
        case 'k':
          pitch -= 0.1;
          if(pitch < -0.5) pitch = -0.5;
          printCmdINFO();
          break;
        case 'u':
          roll -= 0.1;
          if(roll < -0.5) roll = -0.5;
          printCmdINFO();
          break;
        case 'o':
          roll += 0.1;
          if(roll > 0.5) roll = 0.5;
          printCmdINFO();
          break;
        case 'v':
          step_Height -= 0.01;
          if(step_Height < 0.01) step_Height = 0.01;
          printCmdINFO();
          break;
        case 'b':
          step_Height += 0.01;
          if(step_Height > 0.2) step_Height = 0.2;
          printCmdINFO();
          break;
        case 'g':
          body_height -= 0.01;
          if(body_height < 0.2) body_height = 0.2;
          printCmdINFO();
          break;
        case 'h':
          body_height += 0.01;
          if(body_height > 0.4) body_height = 0.4;
          printCmdINFO();
          break;
        case 'p':
          cppAD = !cppAD;
          printCmdINFO();
          break;
        case 'q':
          startFlag = !startFlag;
          printCmdINFO();
          break;
        case '0':
          gait = 3;
          printCmdINFO();
          break;
        case '1':
          gait = 1;
          printCmdINFO();
          break;
        case '2':
          gait = 2;
          printCmdINFO();
          break;
        case '3':
          gait = 4;
          printCmdINFO();
          break;
        case '4':
          gait = 5;
          printCmdINFO();
          break;
        case '5':
          gait = 7;
          printCmdINFO();
          break;
        case '6':
          gait =8;
          printCmdINFO();
          break;
        case '7':
          gait = 10;
          printCmdINFO();
          break;
        case '8':
          gait = 11;
          printCmdINFO();
          break;
        case '9':
          gait = 12;
          printCmdINFO();
          break;
        case ' ':
          linear_speed = 0.0;
          angular_speed = 0.0;
          lateral_speed = 0.0;
          body_height = 0.35;
          roll = 0.0;
          pitch = 0.0;
          printCmdINFO();
          // std::cout << "\n------- All speeds cleared.------- " << std::endl;
          break;
        default:
          break;
      }

    }

    rlCmd.linearVel_x = linear_speed;
    rlCmd.linearVel_y = lateral_speed;
    rlCmd.anguler_z = angular_speed;
    rlCmd.step_Frequency = step_frequency;
    rlCmd.step_Height = step_Height;
    rlCmd.cppAD = cppAD;
    rlCmd.gait = gait;
    rlCmd.startFlag = startFlag;
    rlCmd.roll = roll;
    rlCmd.pitch = pitch;
    rlCmd.body_Height = body_height;

    pub.publish(rlCmd);

    ros::spinOnce();  // Handle ROS events
    loop_rate.sleep();
  }

  return 0;
}
