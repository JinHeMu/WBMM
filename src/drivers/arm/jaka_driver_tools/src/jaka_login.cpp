#include "JAKAZuRobot.h"
#include <cstdlib>
#include <string>
#include <thread>
#include <chrono>
#include <iostream>

using namespace std;

int main(int argc, char *argv[])
{
    setlocale(LC_ALL, "");
    const string robot_ip = argc > 1 ? argv[1] : "10.5.5.100";
    JAKAZuRobot robot;

    cout << "Connecting to JAKA at " << robot_ip << "..." << endl;
    const auto login_ret = robot.login_in(robot_ip.c_str());
    if (login_ret != 0) {
        cerr << "JAKA login failed, SDK error code: " << login_ret << endl;
        return EXIT_FAILURE;
    }
    cout << "JAKA login succeeded" << endl;

    const auto power_ret = robot.power_on();
    if (power_ret != 0) {
        cerr << "JAKA power-on failed, SDK error code: " << power_ret << endl;
        robot.login_out();
        return EXIT_FAILURE;
    }
    cout << "JAKA power-on command succeeded; waiting 8 seconds" << endl;
    std::this_thread::sleep_for(std::chrono::seconds(8));

    const auto enable_ret = robot.enable_robot();
    if (enable_ret != 0) {
        cerr << "JAKA enable failed, SDK error code: " << enable_ret << endl;
        robot.login_out();
        return EXIT_FAILURE;
    }
    cout << "JAKA enable command succeeded; waiting 4 seconds" << endl;
    std::this_thread::sleep_for(std::chrono::seconds(4));

    //Joint-space first-order low-pass filtering in robot servo mode
    const auto lpf_ret = robot.servo_move_use_joint_LPF(2);
    if (lpf_ret != 0) {
        cerr << "Failed to set joint LPF, SDK error code: " << lpf_ret << endl;
        robot.login_out();
        return EXIT_FAILURE;
    }

    const auto torque_ret = robot.set_torque_sensor_mode(1);
    if (torque_ret != 0) {
        cerr << "Failed to enable torque sensor mode, SDK error code: "
             << torque_ret << endl;
        robot.login_out();
        return EXIT_FAILURE;
    }

    // robot.servo_move_use_joint_NLF(45,30,30);
    //robot.servo_move_use_joint_NLF(45,45,45);
    cout << "JAKA communication initialization completed" << endl;
    return EXIT_SUCCESS;
}
