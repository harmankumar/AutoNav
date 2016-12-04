#include <bits/stdc++.h>
#include <lcm/lcm-cpp.hpp>
#include "exlcm/example_t.hpp"

int main(int argc, char ** argv)
{
	lcm::LCM lcm;
    if(!lcm.good())
    	return 1;

    exlcm::example_t my_data;

    my_data.timestamp = 0;
    my_data.currImage = 1;
    my_data.distance = 26;
    my_data.angle = 0.76;

    lcm.publish("EXAMPLE", &my_data);

    return 0;
}
