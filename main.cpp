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
        BoardGalgo bg = BoardGalgo("/dev/ttyUSB0", 3000000);
        bg.setLED(1,1,1);
        double goalPositionAngle;
        std::cout << "Enter position angle:\n> ";
        std::cin >> goalPositionAngle;
        bg.setPosition( 1, 1, goalPositionAngle );
        std::this_thread::sleep_for( std::chrono::seconds( 1 ) );
        double presentPositionAngle;
        bg.readPosition( 1, 1, presentPositionAngle );
        std::cout << "Current position angle: " << presentPositionAngle << '\n';
    }
    catch (const std::exception& ex) {
        std::cerr << ex.what() << std::endl;
        return 1;
    }

    return 0;
}
