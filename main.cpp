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

        controller::BoardGalgo bg( "/dev/ttyUSB0", "/dev/ttyUSB1", 3000000, 1);
        //bg.setOffset(std::vector<double>(12, 0));

    bg.setOffset( std::vector< double >{
            0.0222482, 0.0176451, -0.313776,
            -0.0421948, 0.266211, -0.0145764,
            -0.307638, 0.29076, -0.0636757,
            -0.0069046, 0.301501, 0.450333 } );
    //std::this_thread::sleep_for( std::chrono::seconds( 1 ) );

    // Odkomentować wybrane funkcje w celu przeprowadzenia testów

    // for( int i = 0; i < 5; ++i ) {
    //     // bg.setLED(1, std::vector<bool>(3, 1));
    //     bg.setLED( std::vector< uint8_t >( 12, 1 ) );
    //     // bg.setLED(1, 1, 1);
    //     // bg.setLED(2, 1, 1);
    //     // bg.setLED(3, 1, 1);
    //     // bg.setLED(4, 1, 1);
    //     std::this_thread::sleep_for( std::chrono::milliseconds( 500 ) );
    //     // bg.setLED(1, std::vector<bool>(3, 0));
    //     bg.setLED( std::vector< uint8_t >( 12, 0 ) );
    //     // bg.setLED(1, 1, 0);
    //     // bg.setLED(2, 1, 0);
    //     // bg.setLED(3, 1, 0);
    //     // bg.setLED(4, 1, 0);
    //     std::this_thread::sleep_for( std::chrono::milliseconds( 500 ) );
    // }

	// bg.setOperatingMode( std::vector<uint8_t>( 12, 3 ) );
	// bg.setTorqueLimit( std::vector< double >( 12, 0.05 ) );


	bg.setSpeed( std::vector< double >( 12, 0.05 ) );
	 
    double goalPositionAngle;
    std::cout << "Enter position angle:\n> ";
    std::cin >> goalPositionAngle;
    // bg.setPosition( std::vector< double >( 12, goalPositionAngle ) );
    bg.setPosition(std::vector<double>(12, goalPositionAngle));
    std::this_thread::sleep_for( std::chrono::seconds( 2 ) );


	while( true ) {
		std::vector< double > presentPosition( 3 );
		bg.readPosition( 2, presentPosition );
		for( auto position : presentPosition )
			std::cout << position << ' ';
		std::cout << '\n';
	}

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
