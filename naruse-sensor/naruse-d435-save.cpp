/*
    naruse-d435-on.cpp
        Author: Keitaro Naruse
        Date:   2021-06-10
*/
#include <iostream>
#include <fstream>

#include <cnoid/SimpleController>
#include <cnoid/SpotLight>
#include <cnoid/Camera>
#include <cnoid/RangeCamera>
#include <cnoid/Joystick>


using namespace std;
using namespace cnoid;

class NaruseD435SaveController : public SimpleController
{
    SimpleControllerIO* io;
    Joystick joystick;
    RangeCamera* d435Device;
    bool d435PrevButtonState;
public:
    
    virtual bool initialize(SimpleControllerIO* io) override
    {
        this->io = io;
        Body* body = io->body();

        // Turn on D435
        d435Device = body->findDevice<RangeCamera>("D435");
        d435Device->on(true);
        d435Device->notifyStateChange();
        d435PrevButtonState = false;
        // Enable IO
        io->enableInput(d435Device);

        return true;
    }

    virtual bool control() override
    {
        joystick.readCurrentState();
        bool stateChanged = false;
        bool buttonState = joystick.getButtonState(Joystick::A_BUTTON);
        if(buttonState && !d435PrevButtonState){
            ostream& os = io->os();
            d435Device->constImage().save("test-image.png");
            os << "Saved an image file" << std::endl;
            ofstream ofs("test-points.pcd");
            ofs << "# .PCD v.7 - Point Cloud Data file format" << std::endl;
            ofs << "VERSION .7" << std::endl;
            ofs << "FIELDS x y z" << std::endl;
            ofs << "SIZE 4 4 4" << std::endl;
            ofs << "TYPE F F F" << std::endl;
            ofs << "COUNT 1 1 1" << std::endl;
            ofs << "WIDTH 428" << std::endl;
            ofs << "HEIGHT 240" << std::endl;
            ofs << "VIEWPOINT 0 0 0 1 0 0 0" << std::endl;
            ofs << "POINTS " << d435Device->constPoints().size() << std::endl;
            ofs << "DATA ascii" << std::endl; 
            for (const auto& e : d435Device->constPoints()) {
                ofs << e(0) << " " << e(1) << " " << e(2) << std::endl;
            }
            os << "Saved a pcd file" << std::endl;
        }
        d435PrevButtonState = buttonState;

        return true;
    }
};

CNOID_IMPLEMENT_SIMPLE_CONTROLLER_FACTORY(NaruseD435SaveController)
