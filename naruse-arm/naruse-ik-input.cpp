/*
    naruse-ik-input.cpp
        Simple controller of joystick for inputs of the inverse kinematics 
        Author: Keitaro Naruse
        Date:   2021-06-30
*/
//  For file save in c++ language
#include <iostream>
#include <fstream>

//  For choreonoid
#include <cnoid/SimpleController>
#include <cnoid/RangeCamera>
#include <cnoid/Joystick>
#include <cnoid/JointPath>

/*
    class NaruseIKInputController {}
        Simple controller for joystich for inputs of the inverse kinematics
*/
class NaruseIKInputController : public cnoid::SimpleController
{
private:
    //  Class instance 
    //  SimpleController
    cnoid::SimpleControllerIO* io;
    //  RangeCamera
    cnoid::RangeCamera* d435RangeCamera;
    //  Joystick
    cnoid::Joystick joystick;
    //  Button flag
    //  Joint
    cnoid::Link* joint[6];
    //  Operation mode of Joystick 0 == Position, 1 == Rotation
    int operation_mode;
    //  For smooth key input
    bool prevAbuttonState;

    //  Kinematics
    std::shared_ptr< cnoid::JointPath > jp;
    cnoid::Vector3d tip_ref_pos;
    cnoid::Matrix3d tip_ref_rot;
    
public:
    virtual bool initialize(cnoid::SimpleControllerIO* io) override
    {
        //  Set body model
        this->io = io;
        cnoid::Body* body = io->body();
        
        //  Set output stream
        std::ostream& os = io->os();

        //  Turn on and Enable IO of D435 for both Camera and RangeCamera
        d435RangeCamera = body-> findDevice<cnoid::RangeCamera>("D435");
        io->enableInput(d435RangeCamera);

        //  Enable joint output
        joint[0] = io->body()->link("Joint1");
        joint[1] = io->body()->link("Joint2");
        joint[2] = io->body()->link("Joint3");
        joint[3] = io->body()->link("Joint4");
        joint[4] = io->body()->link("Joint5");
        joint[5] = io->body()->link("Joint6");
        for(int i = 0; i < 6; i++)  {
            joint[i] -> setActuationMode(cnoid::Link::JointDisplacement);
        }

        //  Initial operation mode 0 == position
        operation_mode = 0;

        //  Initial state of prev button
        prevAbuttonState = false;

        //  Set Joint Path
        jp = cnoid::JointPath::getCustomPath(body, body->rootLink(), body->link("Tip"));

        return( true );
    }

    virtual bool control() override
    {
        //  Set body model
        this->io = io;
        cnoid::Body* body = io->body();

        //  Read joystick status 
        joystick.readCurrentState();
        
        //  Set output stream
        std::ostream& os = io->os();

        //  Check A button for save files
        bool currAbuttonState = joystick.getButtonState(cnoid::Joystick::A_BUTTON);
        if( !prevAbuttonState && currAbuttonState ) {
            operation_mode = ( 0 == operation_mode )? 1 : 0; 
            os << "Operation mode is " << operation_mode << std::endl;
            cnoid::Vector3 tip_ref_pos = body->link("Tip")->p();
            cnoid::Matrix3 tip_ref_rot = body->link("Tip")->R();
            os << tip_ref_pos << std::endl;
            os << tip_ref_rot << std::endl;
        }
        prevAbuttonState = currAbuttonState;

        return( true );
    }
};

CNOID_IMPLEMENT_SIMPLE_CONTROLLER_FACTORY(NaruseIKInputController)
