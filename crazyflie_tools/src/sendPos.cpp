#include <iostream>
#include <chrono>
#include <thread>

#include <boost/program_options.hpp>
#include <crazyflie_cpp/Crazyflie.h>

int main(int argc, char **argv)
{

  try
  {
    Crazyflie cf("radio://0/100/2M/FFE7E7E7E7");
    
    for(int i=0; i<1000; i++)
    {
        cf.sendBPositionUpdate(1.0, 2.0, 3.0);
        
        std::this_thread::sleep_for(std::chrono::microseconds(500));
    }
    std::cout << "Done" << std::endl;
    
    return 0;
    
  }
  catch(std::exception& e)
  {
    std::cerr << e.what() << std::endl;
    return 1;
  }
}
