#include <lcm/lcm-cpp.hpp>
#include "exlcm/example_t.hpp"

int main(int argc, char ** argv)
{
   lcm::LCM lcm;
   if(!lcm.good())
       return 1;

   return 0;
}
