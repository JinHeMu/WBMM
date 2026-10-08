#include "JAKAZuRobot.h"
#include <chrono>
#include <thread>
#include <cstdio>
int main() {
  JAKAZuRobot robot;
  const int login = robot.login_in("10.5.5.100");
  if (login != 0) { std::fprintf(stderr, "login_error=%d\n", login); return 1; }
  int mode=-1;
  const int mr=robot.get_torque_sensor_mode(&mode);
  std::fprintf(stderr, "sensor_mode_return=%d sensor_mode=%d\n",mr,mode);
  std::puts("t,q1,q2,q3,q4,q5,q6,raw_fx,raw_fy,raw_fz,raw_tx,raw_ty,raw_tz,act_fx,act_fy,act_fz,act_tx,act_ty,act_tz,err,enabled,estop");
  const auto start=std::chrono::steady_clock::now();
  for(int i=0; i<60; ++i) {
    RobotStatus s{};
    const int ret=robot.get_robot_status(&s);
    if(ret!=0) { std::fprintf(stderr,"get_status_error=%d\n",ret); break; }
    std::printf("%.9f",std::chrono::duration<double>(std::chrono::steady_clock::now()-start).count());
    for(double q:s.joint_position) std::printf(",%.12g",q);
    for(double f:s.torq_sensor_monitor_data.torque) std::printf(",%.12g",f);
    for(double f:s.torq_sensor_monitor_data.actTorque) std::printf(",%.12g",f);
    std::printf(",%d,%d,%d\n",s.errcode,s.enabled,s.emergency_stop);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
  return robot.login_out()==0 ? 0 : 2;
}
