#include "common/param_server.hpp"

int main( int argc, char** argv )
{
  lk_vio::common::ParamServer& param_server = lk_vio::common::ParamServer::getInstance();

  if ( argc < 2 )
  {
    std::cout << "Usage: " << argv[ 0 ] << " <param_name>" << std::endl;
    return 1;
  }
  else
  {
    param_server.readConfigFile( argv[ 1 ] );
  }
}