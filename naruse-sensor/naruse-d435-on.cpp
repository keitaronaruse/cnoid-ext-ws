/*
    naruse-d435-on.cpp
        Author: Keitaro Naruse
        Date:   2021-06-10
*/

#include <cnoid/SimpleController>
#include <cnoid/SpotLight>
#include <cnoid/RangeCamera>
#include <cnoid/RangeSensor>
#include <cnoid/Joystick>

using namespace std;
using namespace cnoid;

class NaruseD435OnController : public SimpleController
{
    SimpleControllerIO* io;
    
public:
    
    virtual bool initialize(SimpleControllerIO* io) override
    {
        this->io = io;
        Body* body = io->body();

        // Turn on all the devices
        body->findDevice<RangeCamera>("D435")->on(true);
        body->findDevice<RangeCamera>("D435")->notifyStateChange();

        return true;
    }

    virtual bool control() override
    {
        return true;
    }
};

CNOID_IMPLEMENT_SIMPLE_CONTROLLER_FACTORY(NaruseD435OnController)
