#include "3rdParty/tinyXML/tinyxml2.h"
#include "Board/boardDynamixel.h"
#include "Board/boardGalgo.h"
#include <iostream>
#include <chrono>
#include <thread>

int main()
{
    try {
        using namespace std;

        /*
        tinyxml2::XMLDocument config;
        config.LoadFile("../../resources/configGlobal.xml");
        if (config.ErrorID())
            std::cout << "unable to load config file1.\n";
        std::string simConfig = config.FirstChildElement( "Environment" )->Attribute("config");
*/
        //Board* board = createBoardDynamixel();
        BoardGalgo bg = BoardGalgo("/dev/ttyUSB4", 3000000);
        //bg.setLED(1,1,1);
        //bg.setPosition( 1, 1, 90 );

        bg.setSpeed(1,1,300);
        std::this_thread::sleep_for(std::chrono::seconds(1));
        bg.setSpeed(1,1,0);

        //board->setPosition(0,0,1.2);
    }
    catch (const std::exception& ex) {
        std::cerr << ex.what() << std::endl;
        return 1;
    }

    return 0;
}
