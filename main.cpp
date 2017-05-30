//#include "3rdParty/tinyXML/tinyxml2.h"
//#include "Board/boardDynamixel.h"
#include "Board/boardGalgo.h"
#include <iostream>
#include <chrono>
#include <thread>
#include <string>

int main()
{
    try {
        /*
        tinyxml2::XMLDocument config;
        config.LoadFile("../../resources/configGlobal.xml");
        if (config.ErrorID())
            std::cout << "unable to load config file1.\n";
        std::string simConfig = config.FirstChildElement( "Environment" )->Attribute("config");
*/
        //Board* board = createBoardDynamixel();

        controller::BoardGalgo bg( "/dev/ttyUSB0", "/dev/ttyUSB1", 3000000 );

        // Odkomentować wybrane funkcje w celu przeprowadzenia testów

    for( int i = 0; i < 5; ++i ) {
        bg.setLED(1, std::vector<bool>(2, 1));
        // bg.setLED(1, 1, 1);
        // bg.setLED(2, 1, 1);
        // bg.setLED(3, 1, 1);
        // bg.setLED(4, 1, 1);
        std::this_thread::sleep_for( std::chrono::milliseconds( 500 ) );
        bg.setLED(1, std::vector<bool>(2, 0));
        // bg.setLED(1, 1, 0);
        // bg.setLED(2, 1, 0);
        // bg.setLED(3, 1, 0);
        // bg.setLED(4, 1, 0);
        std::this_thread::sleep_for( std::chrono::milliseconds( 500 ) );
    }

        //double goalPositionAngle;
        //std::cout << "Enter position angle:\n> ";
        //std::cin >> goalPositionAngle;
        //bg.setPosition( 1, 1, goalPositionAngle );
        //std::this_thread::sleep_for( std::chrono::seconds( 1 ) );

        //double presentPositionAngle;
        //bg.readPosition( 1, 1, presentPositionAngle );
        //std::cout << "Current position angle: " << presentPositionAngle << '\n';

        //bg.setSpeed(1,1,300);
        //std::this_thread::sleep_for(std::chrono::seconds(1));
        //bg.setSpeed(1,1,0);

        //bg.setSpeed( 1, std::vector< double >{ 20.0, 100.0, 0.0 } );
        //std::this_thread::sleep_for( std::chrono::seconds( 1 ) );
        //bg.setSpeed( 1, std::vector< double >{ 0.0, 0.0, 0.0 } );

        // bg.toggleTorque( 1, 1, false );
        // std::cout << "11 joint torque disabled\n";
        // std::this_thread::sleep_for( std::chrono::seconds( 10 ) );
        // bg.toggleTorque( 1, 1, true );
        // std::cout << "11 joint torque enabled\n";
        // std::this_thread::sleep_for( std::chrono::seconds( 10 ) );
        // bg.toggleTorque( 1, 1, false );
        // std::cout << "11 joint torque disabled\n";
        // std::this_thread::sleep_for( std::chrono::seconds( 10 ) );
        // bg.toggleTorque( 1, std::vector< bool >( 3, true ) );
        // std::cout << "1 leg torque enabled\n";
        // std::this_thread::sleep_for( std::chrono::seconds( 10 ) );
        // bg.toggleTorque( std::vector< bool >( 4 * 3, false ) );
        // std::cout << "all legs torque disabled\n";
    }
    catch (const std::exception& ex) {
        std::cerr << ex.what() << std::endl;
        return 1;
    }

    return 0;
}
