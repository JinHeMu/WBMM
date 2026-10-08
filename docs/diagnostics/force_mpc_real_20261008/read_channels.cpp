#include "JAKAZuRobot.h"
#include <cstdio>
int main(){
 JAKAZuRobot r;int ret=r.login_in("10.5.5.100");if(ret){printf("login_error=%d\n",ret);return 1;}
 for(int type=1;type<=3;++type){TorqSensorData d{};ret=r.get_torque_sensor_data(type,&d);printf("type=%d return=%d status=%d sensor_error=%d data=[%.9g,%.9g,%.9g,%.9g,%.9g,%.9g]\n",type,ret,d.status,d.errorCode,d.data.fx,d.data.fy,d.data.fz,d.data.tx,d.data.ty,d.data.tz);}
 float f=0;ret=r.get_torque_sensor_filter(&f);printf("force_filter_return=%d force_filter_hz=%.9g\n",ret,f);
 RobotStatus s{};ret=r.get_robot_status(&s);printf("status_return=%d sensor_status=%d sensor_error=%d\n",ret,s.torq_sensor_monitor_data.status,s.torq_sensor_monitor_data.errcode);
 return r.login_out();
}
