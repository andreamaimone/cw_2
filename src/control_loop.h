#include <iostream>
#include <fstream>
#include <boost/thread.hpp>

using namespace std;

class CONTROLLER {
    public:
        CONTROLLER(float, float, float);
        
        void loop();                //Main loop function
        
        void system_start();       //start the system
        void set_xdes(double x);   //member to set the desired value

    private:
        double _xdes; //desired value to reach
        double _xmes; //current value of my system

        float _eps; //error threshold
        
        float _kp; // P gain
        float _kd; // D gain
        float _ki; // I gain

        bool _init_des_value;
};