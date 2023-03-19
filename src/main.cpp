#include "control_loop.h"

using namespace std;

// Sense: read a value from keyboard
// Plan:  generate the correct input
// Act:   set the input

int main(int argc, char** argv) {

    //New controller with the given gains for the PID
    CONTROLLER c(0.1,0.03,0.02);

    double x;
    while( true ) {
        cout << "Insert desired value for the controller: ";
        cin >> x;
        c.set_xdes(x);
    }

    return 0;
}
