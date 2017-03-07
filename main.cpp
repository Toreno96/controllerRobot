#include "3rdParty/tinyXML/tinyxml2.h"
#include "Board/boardDynamixel.h"
#include <iostream>

int main()
{
    try {
        using namespace std;

        tinyxml2::XMLDocument config;
        config.LoadFile("../../resources/configGlobal.xml");
        if (config.ErrorID())
            std::cout << "unable to load config file1.\n";
        std::string simConfig = config.FirstChildElement( "Environment" )->Attribute("config");

        Board* board = createBoardDynamixel();

        board->setPosition(0,0,1.2);
    }
    catch (const std::exception& ex) {
        std::cerr << ex.what() << std::endl;
        return 1;
    }

    return 0;
}