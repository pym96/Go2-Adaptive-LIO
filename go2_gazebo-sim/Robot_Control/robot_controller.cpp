#include "robot_controller.hpp"


bool running = true;
// over watch the ctrl+c command
void ShutDown(int sig){
  (void) sig;
  std::cout << "stop the controller" << std::endl;
  running = false;
}
void setProcessScheduler(){
  pid_t pid = getpid();
  sched_param param;
  param.sched_priority = sched_get_priority_max(SCHED_FIFO);
  if (sched_setscheduler(pid, SCHED_FIFO, &param) == -1) {
    std::cout << "[ERROR] Function setProcessScheduler failed." << std::endl;
    return;
  }
}

int main(int argc, char **argv) {

  long long _startRosTime;
  double controller_dt = 0.002;
  printf("Robot_Controller main() begin ... \n");

  setProcessScheduler();
  ros::init(argc, argv, "Quad_gazebo_node");
  signal(SIGINT, ShutDown);
  IOInterface *ioInter;
  ioInter = new IOROS();
  // create my constructed controller
  QuadRobotRunner* robotRunner = new QuadRobotRunner(ioInter);
  // run the controller
  while (running) {
    _startRosTime = getRosTimeUsecond();
    robotRunner->run();
    absoluteRosWait(_startRosTime, (long long)(controller_dt * 1e9));
  };
  delete robotRunner;
  return 0;
}
