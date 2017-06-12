#include "Board/boardGalgo.h"
#include <iostream>
#include <chrono>
#include <thread>
#include <string>

void RiseOfRobot(controller::BoardGalgo& bg){
    bg.setOperatingMode(std::vector<uint8_t>(12, 3));
    std::this_thread::sleep_for(std::chrono::milliseconds(50));

    bg.setSpeed(std::vector<double >(12, 0.01));
    bg.setPosition(std::vector<double>(12, 0));
    std::this_thread::sleep_for(std::chrono::seconds(10));

    bg.setSpeed(std::vector<double>(12, 0.005));
    bg.setPosition(0,0,1.57);
    bg.setPosition(1,0,1.57);
    bg.setPosition(2,0,-1.57);
    bg.setPosition(3,0,-1.57);
}

void Walk(controller::BoardGalgo& bg){
    bg.setOperatingMode(std::vector<uint8_t>(12, 3));
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    bg.setSpeed(std::vector<double>{
                    0.02, 0.02, 0.04,
                    0.02, 0.02, 0.04,
                    0.02, 0.02, 0.04,
                    0.02, 0.02, 0.04
                });

    bg.setPosition(std::vector<double>{
                       1.57, 0.0, 0.0,
                       1.57, 0.0, 0.0,
                       -1.57, 0.0, 0.0,
                       -1.57, 0.0, 0.0
                   });

    for(int t = 0; t < 4; t++){
        std::this_thread::sleep_for(std::chrono::milliseconds(2000));
        bg.setPosition(std::vector<double>{
                           1.57, 0.0, 0.0,
                           1.57, -0.8, 0.8,
                           -1.57, 0.0, 0.0,
                           -1.57, 0.8, -0.8
                       });

        std::this_thread::sleep_for(std::chrono::milliseconds(2000));
        bg.setPosition(std::vector<double>{
                           1.57, -0.8, 0.8,
                           1.57, 0.0, 0.0,
                           -1.57, 0.8, -0.8,
                           -1.57, 0.0, 0.0
                       });
    }
}

void Greeting(controller::BoardGalgo& bg){
    bg.setOperatingMode(std::vector<uint8_t>(12, 3));
    std::this_thread::sleep_for(std::chrono::milliseconds(50));

    bg.setSpeed(std::vector<double>(12, 0.05));
    bg.setPosition(std::vector<double>{
                       1.57, 0.0, 0.0,
                       1.57, 0.0, 0.0,
                       -1.57, 0.0, 0.0,
                       -1.57, 0.0, 0.0
                   });
    std::this_thread::sleep_for(std::chrono::milliseconds(2000));

    bg.setSpeed(std::vector<double>(12, 0.025));
    bg.setPosition(std::vector<double>{
                       0.0, -1.57, 0.0,
                       1.57, 0.0, 0.0,
                       -1.57, 0.0, 0.0,
                       -1.57, 0.0, 0.0
                   });

    std::this_thread::sleep_for(std::chrono::milliseconds(3000));
    bg.setOperatingMode(0, 0, 5);
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    bg.setTorqueLimit(0,0,0.04);
    std::this_thread::sleep_for(std::chrono::milliseconds(1500));
    bg.setOperatingMode(0, 0, 3);
    std::this_thread::sleep_for(std::chrono::milliseconds(50));

    bg.setSpeed(0, 0, 0.025);
    bg.setPosition(std::vector<double>{
                       1.57, 0.0, 0.0,
                       1.57, 0.0, 0.0,
                       -1.57, 0.0, 0.0,
                       -1.57, 0.0, 0.0
                   });
}

void SleepAndWakeUp(controller::BoardGalgo& bg){
    bg.setOperatingMode(std::vector<uint8_t>(12, 3));
    std::this_thread::sleep_for(std::chrono::milliseconds(50));

    bg.setSpeed(std::vector<double>(12, 0.07));
    bg.setPosition(std::vector<double>(12, 0.0));
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));

    bg.setOperatingMode(std::vector<uint8_t>(12, 5));
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    bg.setTorqueLimit(std::vector<double>(12, 0.02));

    std::this_thread::sleep_for(std::chrono::milliseconds(1500));
    bg.setTorqueLimit(std::vector<double>(12, 0.09));
}


void SetOffsets(controller::BoardGalgo& bg){
    bg.setOffset(std::vector<double>{
        0.0222482, 0.0176451, -0.313776,
        -0.0421948, 0.266211, -0.0145764,
        -0.307638, 0.29076, -0.0636757,
        -0.0069046, 0.301501, 0.450333});

    std::this_thread::sleep_for(std::chrono::milliseconds(50));
}

int main(){
    try {
        controller::BoardGalgo bg("/dev/ttyUSB0", "/dev/ttyUSB1", 3000000, 1);

        //SetOffsets(bg);

        //RiseOfRobot(bg);
        //Walk(bg);
        //Greeting(bg);
        SleepAndWakeUp(bg);

    } catch (const std::exception& ex) {
        std::cerr << ex.what() << std::endl;
        return 1;
    }

    return 0;
}
