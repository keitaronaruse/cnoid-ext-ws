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
    //  Operation mode of Joystick 0 == Position, 1 == Rotation
    int operation_mode;
    //  Button flag
    bool prevAbuttonState;
    //  Reference angle
    double q_ref[6];
    //  Kinematics
    std::shared_ptr< cnoid::JointPath > jp;
    
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
        for(int i = 0; i < 6; i++)  {
            cnoid::Link* joint = body->joint(i);
            joint -> setActuationMode(cnoid::Link::JointAngle);
            io -> enableIO(joint);
            q_ref[i] = 0.7354;
        }
        io->enableInput(body->link("Tip"), cnoid::Link::LinkPosition);

        //  Initial operation mode 0 == position
        operation_mode = 0;

        //  Initial state of prev button
        prevAbuttonState = false;

        //  Set Joint Path
        // jp = cnoid::JointPath::getCustomPath(body, body->rootLink(), body->link("Tip"));

        return( true );
    }

    virtual bool start() override
    {
        return(true);
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

        for(int i = 0; i < 6; i++)  {
            cnoid::Link* joint = body->joint(i);
            joint->q_target() = q_ref[i];
        }
        
        //  Check A button for save files
        bool currAbuttonState = joystick.getButtonState(cnoid::Joystick::A_BUTTON);
        if( !prevAbuttonState && currAbuttonState ) {
            // jp -> calcForwardKinematics();
            operation_mode = ( 0 == operation_mode )? 1 : 0; 
            os << "Operation mode is " << operation_mode << std::endl;
            // cnoid::Vector3 tip_ref_pos = body->link("Tip")->p();
            // cnoid::Matrix3 tip_ref_rot = body->link("Tip")->R();
            cnoid::Position tip_T = body->link("Tip")->position();
            cnoid::Vector3 tip_p = tip_T.translation();
            cnoid::Matrix3 tip_R = tip_T.rotation();
            os << tip_p << std::endl;
            os << tip_R << std::endl;
        }
        prevAbuttonState = currAbuttonState;

        return( true );
    }
};

CNOID_IMPLEMENT_SIMPLE_CONTROLLER_FACTORY(NaruseIKInputController)
