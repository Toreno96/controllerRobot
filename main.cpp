#include "Board/boardGalgo.h"
#include <iostream>
#include <chrono>
#include <thread>
#include <string>

void PowstanieRobota(controller::BoardGalgo& bg){
    bg.setSpeed(std::vector< double >(12, 0.01));
    bg.setPosition(std::vector<double>(12, 0));
    std::this_thread::sleep_for(std::chrono::seconds(10));

    bg.setSpeed(std::vector< double >(12, 0.005));
    bg.setPosition(0,0,1.57);
    bg.setPosition(1,0,1.57);
    bg.setPosition(2,0,-1.57);
    bg.setPosition(3,0,-1.57);
}

int main(){
    try {
        controller::BoardGalgo bg( "/dev/ttyUSB0", "/dev/ttyUSB1", 3000000, 1);

        /*
        //Zapisane w EEPROM, nie trzeba wywolywac przy kazdym uruchomieniu
        bg.setOffset(std::vector<double>{
            0.0222482, 0.0176451, -0.313776,
            -0.0421948, 0.266211, -0.0145764,
            -0.307638, 0.29076, -0.0636757,
            -0.0069046, 0.301501, 0.450333});
        */

        PowstanieRobota(bg);

    } catch (const std::exception& ex) {
        std::cerr << ex.what() << std::endl;
        return 1;
    }

    return 0;
}
