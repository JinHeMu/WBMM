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

    cout << "Connecting to JAKA at " << robot_ip << " for logout..." << endl;
    const auto login_ret = robot.login_in(robot_ip.c_str());
    if (login_ret != 0) {
        cerr << "JAKA logout login failed, SDK error code: " << login_ret << endl;
        return EXIT_FAILURE;
    }

    const auto disable_ret = robot.disable_robot();
    if (disable_ret != 0) {
        cerr << "JAKA disable failed, SDK error code: " << disable_ret << endl;
    }

    const auto power_ret = robot.power_off();
    if (power_ret != 0) {
        cerr << "JAKA power-off failed, SDK error code: " << power_ret << endl;
    }

    const auto shutdown_ret = robot.shut_down();
    if (shutdown_ret != 0) {
        cerr << "JAKA shutdown failed, SDK error code: " << shutdown_ret << endl;
    }

    const auto logout_ret = robot.login_out();
    if (logout_ret != 0) {
        cerr << "JAKA login-out failed, SDK error code: " << logout_ret << endl;
    }

    const bool ok = disable_ret == 0 && power_ret == 0 &&
                    shutdown_ret == 0 && logout_ret == 0;
    if (ok) {
        cout << "JAKA logout and power-off completed" << endl;
        return EXIT_SUCCESS;
    }
    return EXIT_FAILURE;
}
