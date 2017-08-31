///
/// \file
/// \brief Demos of BoardGalgo functionality.
/// \author Daniel Staśczak
/// \author Marcin Orczyk
///

#include <chrono>
#include <iostream>
#include <limits>
#include <string>
#include <thread>
#include "Board/boardGalgo.h"

void RiseOfRobot(controller::BoardGalgo& bg){
    bg.setOperatingMode(std::vector<uint8_t>(12, controller::BoardGalgo::OPERATINGMODE_POSITION));
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
    bg.setOperatingMode(std::vector<uint8_t>(12, controller::BoardGalgo::OPERATINGMODE_POSITION));
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
    bg.setOperatingMode(std::vector<uint8_t>(12, controller::BoardGalgo::OPERATINGMODE_POSITION));
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
    bg.setOperatingMode(0, 0, controller::BoardGalgo::OPERATINGMODE_CURRENT_BASED_POSITION);
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    bg.setTorqueLimit(0,0,0.04);
    std::this_thread::sleep_for(std::chrono::milliseconds(5000));
    bg.setOperatingMode(0, 0, controller::BoardGalgo::OPERATINGMODE_POSITION);
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
    bg.setOperatingMode(std::vector<uint8_t>(12, controller::BoardGalgo::OPERATINGMODE_POSITION));
    std::this_thread::sleep_for(std::chrono::milliseconds(50));

    bg.setSpeed(std::vector<double>(12, 0.07));
    bg.setPosition(std::vector<double>(12, 0.0));
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));

    bg.setOperatingMode(std::vector<uint8_t>(12, controller::BoardGalgo::OPERATINGMODE_CURRENT_BASED_POSITION));
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
        controller::BoardGalgo bg("/dev/ttyUSB0", "/dev/ttyUSB1", 3000000,
                {{0, {0, 15'000, 0, 0}}, {1, {1, 15'000, 0, 0}},
                 {2, {2, 15'000, 0, 0}}, {3, {3, 15'000, 0, 0}}}, 1);
        SetOffsets(bg);

        int selectedOption;
        do {
            std::cout << "\n"
                    << "[1] Rise of the robot\n"
                    << "[2] Walking\n"
                    << "[3] Greetings\n"
                    << "[4] Sleep and wake up\n\n"
                    << "Select the demo or [5] to quit:\n"
                    << "> ";
            std::cin >> selectedOption;
            std::cin.ignore( std::numeric_limits< std::streamsize >::max(), '\n' );
            switch( selectedOption ) {
                case 1:
                    RiseOfRobot(bg);
                    break;
                case 2:
                    Walk(bg);
                    break;
                case 3:
                    Greeting(bg);
                    break;
                case 4:
                    SleepAndWakeUp(bg);
                    break;
                case 5:
                    break;
                default:
                    std::cout << "Wrong option selected!\n";
                    break;
            }
        } while( selectedOption != 5 );

    } catch (const std::exception& ex) {
        std::cerr << ex.what() << std::endl;
        return 1;
    }

    return 0;
}
