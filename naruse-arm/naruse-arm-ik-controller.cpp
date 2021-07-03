/*
    naruse-arm-ik-controller.cpp
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
class NaruseArmIKController : public cnoid::SimpleController
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
    //  For translation in x-axis
    double prevRightStickHorizontalPos;
    //  For translation in y-axis
    double prevRightStickVerticalPos;
    //  For translation in z-axis
    double prevLeftStickVerticalPos;
    //  For rotation around x-axis
    double prevLeftStickHorizontalPos;
    //  For rotaion around y-axis
    double prevPadVerticalPos;
    //  For rotaion around z-axis
    double prevPadHorizaontalPos;
    

    //  Reference joint angle
    std::vector<double> q_ref;
    //  Transform (fram) instance of reference (target) of Tip
    cnoid::Isometry3 tip_T_ref;
    
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

        //  Initial value of a previous state of joystick buttons
        prevAButtonState = false;
        prevBButtonState = false;
        prevXButtonState = false;
        prevYButtonState = false;
        prevRightStickHorizontalPos = 0.0;
        prevRightStickVerticalPos = 0.0;
        prevLeftStickVerticalPos = 0.0;
        prevLeftStickHorizontalPos = 0.0;
        prevPadHorizaontalPos = 0.0;
        prevPadVerticalPos = 0.0;

        //  Set initial value of tip_T_ref as Identity (No change transfor)
        tip_T_ref = cnoid::Isometry3::Identity();

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

        //  if A button is pushed down at this moment
        bool currAButtonState = joystick.getButtonState(cnoid::Joystick::A_BUTTON);
        if( !prevAButtonState && currAButtonState ) {
            isMessageOut = true;
        }
        prevAButtonState = currAButtonState;

        //  if Y button is pushed down at this moment
        bool currYButtonState = joystick.getButtonState(cnoid::Joystick::Y_BUTTON);
        if( !prevYButtonState && currYButtonState ) {
            ;
        }
        prevYButtonState = currYButtonState;
        
        //  If B button is pushed down at this moment
        bool currBButtonState = joystick.getButtonState(cnoid::Joystick::B_BUTTON);
        if( !prevBButtonState && currBButtonState ) {
            ;
        }
        prevBButtonState = currBButtonState;

        //  If X button is pushed down at this moment
        bool currXButtonState = joystick.getButtonState(cnoid::Joystick::X_BUTTON);
        if( !prevXButtonState && currXButtonState ) {
            ;
        }
        prevXButtonState = currXButtonState;

        //  right stick horizontal axis (j, l)
        double currRightStickHorizontalPos = joystick.getPosition(2);
        if( ( 0.0 == prevRightStickHorizontalPos ) && ( 0.0 != currRightStickHorizontalPos ) ) {
            //  Position mode
            if(0.0 < currRightStickHorizontalPos )    {
                tip_T_ref.translate( cnoid::Vector3( 0.1, 0.0, 0.0 ) );
            }
            else    {
                tip_T_ref.translate( cnoid::Vector3( -0.1, 0.0, 0.0 ) );
            }
            isMessageOut = true;
        }
        prevRightStickHorizontalPos = currRightStickHorizontalPos;

        //  right stick vertical axis (k, i)
        double currRightStickVerticalPos = joystick.getPosition(3);
        if( ( 0.0 == prevRightStickVerticalPos ) && ( 0.0 != currRightStickVerticalPos ) ) {
            if(0.0 < currRightStickVerticalPos )    {
                tip_T_ref.translate( cnoid::Vector3( 0.0, -0.1, 0.0 ) );
            }
            else    {
                tip_T_ref.translate( cnoid::Vector3( 0.0, 0.1, 0.0 ) );
            }
            isMessageOut = true;
        }
        prevRightStickVerticalPos = currRightStickVerticalPos;

        //  left stick vertical axis (e, d)
        double currLeftStickVerticalPos = joystick.getPosition(1);
        if( ( 0.0 == prevLeftStickVerticalPos ) && ( 0.0 != currLeftStickVerticalPos ) ) {
            if(0.0 < currLeftStickVerticalPos )    {
                tip_T_ref.translate( cnoid::Vector3( 0.0, 0.0, -0.1 ) );
            }
            else    {
                tip_T_ref.translate( cnoid::Vector3( 0.0, 0.0, 0.1 ) );
            }
            isMessageOut = true;
        }
        prevLeftStickVerticalPos = currLeftStickVerticalPos;

        //  left stick horizontal axis (s, f)
        double currLeftStickHorizontalPos = joystick.getPosition(0);
        if( ( 0.0 == prevLeftStickHorizontalPos ) && ( 0.0 != currLeftStickHorizontalPos ) ) {
            if(0.0 < currLeftStickHorizontalPos )    {
                tip_T_ref.rotate( cnoid::AngleAxis( 0.1, cnoid::Vector3::UnitX() ) );
            }
            else    {
                tip_T_ref.rotate( cnoid::AngleAxis( -0.1, cnoid::Vector3::UnitX() ) );
            }
            isMessageOut = true;
        }
        prevLeftStickHorizontalPos = currLeftStickHorizontalPos;
        
        //  Pad horizontal axis (Cursor: Left, Right)
        double currPadHorizontalPos = joystick.getPosition(4);
        if( ( 0.0 == prevPadHorizaontalPos ) && ( 0.0 != currPadHorizontalPos ) ) {
            //  Rotation around z axis
            if(0.0 < currPadHorizontalPos )    {
                tip_T_ref.rotate( cnoid::AngleAxis( 0.1, cnoid::Vector3::UnitZ() ) );
            }                
            else    {
                tip_T_ref.rotate( cnoid::AngleAxis( -0.1, cnoid::Vector3::UnitZ() ) );
            }
            isMessageOut = true;
        }
        prevPadHorizaontalPos = currPadHorizontalPos;

        //  Pad vertical axis (Cursor: Up, Down)
        double currPadVerticalPos = joystick.getPosition(5);
        if( ( 0.0 == prevPadVerticalPos ) && ( 0.0 != currPadVerticalPos ) ) {
            //  Rotation around y-axis
            if(0.0 < currPadVerticalPos )    {
                tip_T_ref.rotate( cnoid::AngleAxis( 0.1, cnoid::Vector3::UnitY() ) );
            }                
            else    {
                tip_T_ref.rotate( cnoid::AngleAxis( -0.1, cnoid::Vector3::UnitY() ) );
            }
            isMessageOut = true;
        }
        prevPadVerticalPos = currPadVerticalPos;

        //  Message out
        if(isMessageOut)    {
            cnoid::Vector3 tip_p_ref = tip_T_ref.translation();
            cnoid::Matrix3 tip_R_ref = tip_T_ref.rotation();
            os << tip_p_ref << std::endl;
            os << tip_R_ref << std::endl;
        }

        //  For each of the joints, set a target angle as a reference one
        for(int i = 0; i < body->numJoints(); i++)  {
            cnoid::Link* joint = body->joint(i);
            joint->q_target() = q_ref[i];
        }

        return( true );
    }
};

CNOID_IMPLEMENT_SIMPLE_CONTROLLER_FACTORY(NaruseArmIKController)
