/*
    naruse-arm-angle-controller.cpp
        Simple controller of naruse-arm by joint angle
        Author: Keitaro Naruse
        Date:   2021-07-03
*/
//  C++ include files
//  For stream
#include <iostream>
//  For vector
#include <vector>

//  Choreonoid include files
//  For simple controller
#include <cnoid/SimpleController>
//  For range camera (D435)
#include <cnoid/RangeCamera>
//  For joystick
#include <cnoid/Joystick>
//  For kinematics (JointPath)
#include <cnoid/JointPath>

/*
    class NaruseArmAngleController
        Simple controller for joystick input of the inverse kinematics
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
    //  Reference joint angle
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
            //  For each of the joint links
            cnoid::Link* joint = body->joint(i);
            //  Set a control mode as a joint angle
            joint -> setActuationMode(cnoid::Link::JointAngle);
            //  Enable input and output
            io -> enableIO(joint);
            //  Set an initial referece angle
            q_ref.push_back(0.0);
        }
        //  Enable input of the link of Tip for reading a joint frame of T
        io->enableInput(body->link("Tip"), cnoid::Link::LinkPosition);

        //  Initial joint id of angle control is 0
        target_joint_id = 0;

        //  Initial value of a previous state of joystick buttons
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

        //  Read joystick state
        joystick.readCurrentState();
        
        //  Flag of messaging to os; Show a message if it is true
        bool isMessageOut = false;

        //  Decrease a joint id if A button is pushed down at this moment
        bool currAButtonState = joystick.getButtonState(cnoid::Joystick::A_BUTTON);
        if( !prevAButtonState && currAButtonState ) {
            //  target id changes as a ring: 0 -> numJoints()-1
            if( -1 == --target_joint_id)    {
                target_joint_id = body->numJoints()-1;
            }
            isMessageOut = true;
        }
        prevAButtonState = currAButtonState;

        //  Increase a joint id if Y button is pushed down at this moment
        bool currYButtonState = joystick.getButtonState(cnoid::Joystick::Y_BUTTON);
        if( !prevYButtonState && currYButtonState ) {
            //  target id changes as a ring: numJoints()-1 -> 0
            if( body->numJoints() == ++target_joint_id)    {
                target_joint_id = 0;
            }
            isMessageOut = true;
        }
        prevYButtonState = currYButtonState;
        
        //  Increase a reference angle of joint id if B button is pushed down at this moment
        bool currBButtonState = joystick.getButtonState(cnoid::Joystick::B_BUTTON);
        if( !prevBButtonState && currBButtonState ) {
            q_ref[target_joint_id] += 0.1;
            isMessageOut = true;
        }
        prevBButtonState = currBButtonState;

        //  Decrease a reference angle of joint id if X button is pushed down at this moment
        bool currXButtonState = joystick.getButtonState(cnoid::Joystick::X_BUTTON);
        if( !prevXButtonState && currXButtonState ) {
            q_ref[target_joint_id] -= 0.1;
            isMessageOut = true;
        }
        prevXButtonState = currXButtonState;

        //  For each of the joints, set a target angle as a reference one
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
