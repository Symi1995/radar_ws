#ifndef RADAR_CONTI_DATA_H_
#define RADAR_CONTI_DATA_H_

#include <iostream>


union Byte
{
    unsigned char byte;

    struct
    {
        bool bit1 : 1;
        bool bit2 : 1;
        bool bit3 : 1;
        bool bit4 : 1;
        bool bit5 : 1;
        bool bit6 : 1;
        bool bit7 : 1;
        bool bit8 : 1;
    };
};

union Objects
{
    unsigned char objects;

    struct
    {
        uint obstacle_id;
        float longitude_dist;
        float lateral_dist;
        float longitude_vel;
        float lateral_vel;
        float rcs;
        int meas_state;
    };
};

#endif  
