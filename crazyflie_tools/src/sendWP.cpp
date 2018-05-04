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
    uint8_t seq = 0;
    for(int i=0; i<2000; i++)
    {
        cf.sendBroadcastSetpoint(seq, 1, i*0.01*1000, i*0.01*1000, 0, 0,0,0, 0,0,0, 0,0,0);
        seq++;
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
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
