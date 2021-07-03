/*
    naruse-arm-angle-control.cpp
        Simple controller of naruse-arm by joint angle
        Author: Keitaro Naruse
        Date:   2021-07-03
*/
//  For c++
#include <iostream>
#include <fstream>
#include <vector>

//  For choreonoid
#include <cnoid/SimpleController>
#include <cnoid/RangeCamera>
#include <cnoid/Joystick>
#include <cnoid/JointPath>

/*
    class NaruseArmAngleController {}
        Simple controller for joystich for inputs of the inverse kinematics
*/
class NaruseArmAngleController : public cnoid::SimpleController
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
    bool prevAButtonState;
    bool prevBButtonState;
    bool prevXButtonState;
    bool prevYButtonState;
    //  Reference angle
    std::vector<double> q_ref;
    //  Target joint id of angle control
    int target_joint_id;
    
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
        for(int i = 0; i < body -> numJoints(); i++)  {
            //  For each of the joints
            cnoid::Link* joint = body->joint(i);
            //  Control joints by angle
            joint -> setActuationMode(cnoid::Link::JointAngle);
            //  Enable input and output for the joint
            io -> enableIO(joint);
            //  Initial referece angle
            q_ref.push_back(0.0);
        }
        io->enableInput(body->link("Tip"), cnoid::Link::LinkPosition);

        //  Initial joint id of angle control is 0
        target_joint_id = 0;

        //  Initial state of prev button
        prevAButtonState = false;
        prevBButtonState = false;
        prevXButtonState = false;
        prevYButtonState = false;

        return( true );
    }

    virtual bool start() override
    {
        return( true );
    }
    
    virtual bool control() override
    {
        //  Set output stream
        std::ostream& os = io->os();

        //  Set body model
        this->io = io;
        cnoid::Body* body = io->body();

        //  Read joystick status 
        joystick.readCurrentState();
        
        //  Flag of showing a message at os
        bool isMessageOut = false;

        //  Push A button for decrease joint id 
        bool currAButtonState = joystick.getButtonState(cnoid::Joystick::A_BUTTON);
        if( !prevAButtonState && currAButtonState ) {
            if( -1 == --target_joint_id)    {
                target_joint_id = body->numJoints()-1;
            }
            isMessageOut = true;
        }
        prevAButtonState = currAButtonState;

        //  Push Y button for increase joint id
        bool currYButtonState = joystick.getButtonState(cnoid::Joystick::Y_BUTTON);
        if( !prevYButtonState && currYButtonState ) {
            if( body->numJoints() == ++target_joint_id)    {
                target_joint_id = 0;
            }
            isMessageOut = true;
        }
        prevYButtonState = currYButtonState;
        
        //  Push B button for increase referecen angle 
        bool currBButtonState = joystick.getButtonState(cnoid::Joystick::B_BUTTON);
        if( !prevBButtonState && currBButtonState ) {
            q_ref[target_joint_id] += 0.1;
            isMessageOut = true;
        }
        prevBButtonState = currBButtonState;

        //  Push X button for decrease referecen angle 
        bool currXButtonState = joystick.getButtonState(cnoid::Joystick::X_BUTTON);
        if( !prevXButtonState && currXButtonState ) {
            q_ref[target_joint_id] -= 0.1;
            isMessageOut = true;
        }
        prevXButtonState = currXButtonState;

        //  Set target angles as reference ones
        for(int i = 0; i < body->numJoints(); i++)  {
            cnoid::Link* joint = body->joint(i);
            joint->q_target() = q_ref[i];
        }
        
        //  Message out
        if(isMessageOut)    {
            cnoid::Isometry3 tip_T = body->link("Tip")->position();
            cnoid::Vector3 tip_p = tip_T.translation();
            cnoid::Matrix3 tip_R = tip_T.rotation();
            os << target_joint_id << ":" << q_ref[target_joint_id] << std::endl;
            os << tip_p << std::endl;
            os << tip_R << std::endl;
        }

        return( true );
    }
};

CNOID_IMPLEMENT_SIMPLE_CONTROLLER_FACTORY(NaruseArmAngleController)
