#include <memory>
#include "dummies.h"
#include "syncWriter.h"

int main() {
  std::shared_ptr< dummy::PortHandler > portHandler;
  std::shared_ptr< dummy::PacketHandler > packetHandler;

  SyncWriter< uint8_t > rightLegsWriter( portHandler.get(),
      packetHandler.get(), 65 );
  SyncWriter< uint8_t > leftLegsWriter( portHandler.get(),
      packetHandler.get(), 65 );
  std::vector< tId > rightIds{ 11, 12, 21, 22 };
  std::vector< tId > leftIds{ 31, 32, 41, 42 };
  std::vector< uint8_t > powered{ 1, 2, 3, 4 };
  rightLegsWriter.send( rightIds, powered );
  leftLegsWriter.send( leftIds, powered );
}