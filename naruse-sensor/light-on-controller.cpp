/*
    light-on-controller.cpp
        Simple controller for d435 range camera
            - Save a point clound file as pcd when A button is pushed down
            - Point cloud is given by D435 sensor frame
        Author: Keitaro Naruse
        Date:   2021-08-16
*/
//  For file save in c++ language
#include <iostream>
#include <fstream>

//  For choreonoid
#include <cnoid/SimpleController>
#include <cnoid/SpotLight>

/*
    class LightOnController {}
        Simple controller for spot light, just turn it on
*/
class LightOnController : public cnoid::SimpleController
{
private:
    //  Class instance 
    // cnoid::SimpleControllerIO* io;
    cnoid::SpotLight* light;

public:
    virtual bool initialize(cnoid::SimpleControllerIO* io) override
    {
        // Find light and turn it on
        light = io->body()->findDevice<cnoid::SpotLight>("Light");

        return(true);
    }

    virtual bool control() override
    {
        if( !light->on() ) {
            light->on(true);
            light->notifyStateChange();
        }
        return(true);
    }
};

CNOID_IMPLEMENT_SIMPLE_CONTROLLER_FACTORY(LightOnController)
