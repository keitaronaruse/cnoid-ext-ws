/*
    naruse-arm-ik-controller.cpp
        Simple controller of naruse-arm by joint angle
        Author: Keitaro Naruse
        Date:   2021-07-06
        MIT-License
*/
//  C++ include files
//  For stream
#include <iostream>
#include <iomanip>
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
    //  Transform (frame) instance of reference (target) of Tip
    cnoid::Isometry3 tip_T_ref;

    //  Kinematics
    //  ikBody should be declared here because it is a smart pointer
    cnoid::BodyPtr ikBody;
    std::shared_ptr<cnoid::JointPath> baseToTip;

public:
    virtual bool initialize(cnoid::SimpleControllerIO* io) override
    {
        //  Set body model
        this->io = io;
        cnoid::Body* ioBody = io->body();
        
        //  Set output stream
        std::ostream& os = io->os();

        //  Turn on and Enable IO of D435 for both Camera and RangeCamera
        d435RangeCamera = ioBody-> findDevice<cnoid::RangeCamera>("D435");
        io->enableInput(d435RangeCamera);

        //  Enable joint output
        for(int i = 0; i < ioBody -> numJoints(); i++)  {
            //  For each of the joint links
            cnoid::Link* joint = ioBody->joint(i);
            //  Set a control mode as a joint angle
            joint->setActuationMode(cnoid::Link::JointAngle);
            //  Enable input and output
            io->enableIO(joint);
            //  Set an initial referece angle
            q_ref.push_back(0.0);
        }
        //  Set initial angles
        ioBody->joint(0)->q() = q_ref[0] =  0.2618;
        ioBody->joint(1)->q() = q_ref[1] = -1.0472;
        ioBody->joint(2)->q() = q_ref[2] =  2.0944;
        ioBody->joint(3)->q() = q_ref[3] =  0.2618;
        ioBody->joint(4)->q() = q_ref[4] =  0.5236;
        ioBody->joint(5)->q() = q_ref[5] = -1.5708;

        //  For Kinematics setting
        ikBody = ioBody->clone();
        //  Enable input of the link of Tip for reading a joint frame of T
        io->enableInput(ikBody->link("Tip"), cnoid::Link::LinkPosition);
        //  Copy joint angles of a robot model into the ones of an ik model 
        for(int i = 0; i < ioBody->numJoints(); ++i){
            ikBody->joint(i)->q() = ioBody->joint(i)->q();
        }
        baseToTip = cnoid::JointPath::getCustomPath(ikBody, ikBody->rootLink(), ikBody->link("Tip"));
        baseToTip->calcForwardKinematics();
        //  Copy T by forward kinematics of ikBody to tip_T_ref
        tip_T_ref = ikBody->link("Tip")->position();
        
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

        return( true );
    }

    virtual bool control() override
    {
        //  Set output stream
        std::ostream& os = io->os();

        //  Set body model
        this->io = io;
        cnoid::Body* ioBody = io->body();

        //  Read joystick state
        joystick.readCurrentState();
        
        //  Flag of messaging to os; Show a message if it is true
        bool isMessageOut = false;

        //  When A button is pushed down 
        //  Solve inverse kinematics
        bool currAButtonState = joystick.getButtonState(cnoid::Joystick::A_BUTTON);
        if( !prevAButtonState && currAButtonState ) {
            //  Solve inverse kinematics
            bool isIKSolved = baseToTip->calcInverseKinematics(tip_T_ref);
            os << "IK solve status: " << isIKSolved << std::endl;
            if(isIKSolved){
                //  Set joint angles solved by inverse kinematics to q_ref 
                for(int i = 0; i < baseToTip->numJoints(); ++i){
                    q_ref[i] = baseToTip->joint(i)->q();
                }
            }
            else{
                q_ref[0] =  0.2618;
                q_ref[1] = -1.0472;
                q_ref[2] =  2.0944;
                q_ref[3] =  0.2618;
                q_ref[4] =  0.5236;
                q_ref[5] = -1.5708;
            }
        }
        prevAButtonState = currAButtonState;

        //  if Y button is pushed down at this moment
        bool currYButtonState = joystick.getButtonState(cnoid::Joystick::Y_BUTTON);
        if( !prevYButtonState && currYButtonState ) {
            isMessageOut = true;
        }
        prevYButtonState = currYButtonState;
        
        //  When B button is pushed down 
        //  Copy actual T of tip to its reference(target) tip_T_ref 
        bool currBButtonState = joystick.getButtonState(cnoid::Joystick::B_BUTTON);
        if( !prevBButtonState && currBButtonState ) {
            //  Copy joint angles of a robot model into the ones of an ik model 
            for(int i = 0; i < ioBody->numJoints(); ++i){
                ikBody->joint(i)->q() = ioBody->joint(i)->q();
            }
            baseToTip->calcForwardKinematics();
            //  Copy T by forward kinematics of ikBody to tip_T_ref
            tip_T_ref = ikBody->link("Tip")->position();
            os << "Copy an ikBody T of tip to refrece of it." << std::endl;
            isMessageOut = true;
        }
        prevBButtonState = currBButtonState;

        //  If X button is pushed down at this moment
        bool currXButtonState = joystick.getButtonState(cnoid::Joystick::X_BUTTON);
        if( !prevXButtonState && currXButtonState ) {
            isMessageOut = true;
        }
        prevXButtonState = currXButtonState;

        //  right stick horizontal axis (j, l)
        double currRightStickHorizontalPos = joystick.getPosition(2);
        if( ( 0.0 == prevRightStickHorizontalPos ) && ( 0.0 != currRightStickHorizontalPos ) ) {
            //  Position mode
            if(0.0 < currRightStickHorizontalPos )    {
                // tip_T_ref.translate( cnoid::Vector3( 0.1, 0.0, 0.0 ) );
                tip_T_ref.translation().x() += 0.1;
            }
            else    {
                // tip_T_ref.translate( cnoid::Vector3( -0.1, 0.0, 0.0 ) );
                tip_T_ref.translation().x() -= 0.1;
            }
            isMessageOut = true;
        }
        prevRightStickHorizontalPos = currRightStickHorizontalPos;

        //  right stick vertical axis (k, i)
        double currRightStickVerticalPos = joystick.getPosition(3);
        if( ( 0.0 == prevRightStickVerticalPos ) && ( 0.0 != currRightStickVerticalPos ) ) {
            if(0.0 < currRightStickVerticalPos )    {
                // tip_T_ref.translate( cnoid::Vector3( 0.0, -0.1, 0.0 ) );
                tip_T_ref.translation().y() -= 0.1;
            }
            else    {
                // tip_T_ref.translate( cnoid::Vector3( 0.0, 0.1, 0.0 ) );
                tip_T_ref.translation().y() += 0.1;
            }
            isMessageOut = true;
        }
        prevRightStickVerticalPos = currRightStickVerticalPos;

        //  left stick vertical axis (e, d)
        //  Camera roll
        double currLeftStickVerticalPos = joystick.getPosition(1);
        if( ( 0.0 == prevLeftStickVerticalPos ) && ( 0.0 != currLeftStickVerticalPos ) ) {
            if(0.0 < currLeftStickVerticalPos )    {
                // tip_T_ref.translate( cnoid::Vector3( 0.0, 0.0, -0.1 ) );
                tip_T_ref.translation().z() -= 0.1;
            }
            else    {
                // tip_T_ref.translate( cnoid::Vector3( 0.0, 0.0, 0.1 ) );
                tip_T_ref.translation().z() += 0.1;
            }
            isMessageOut = true;
        }
        prevLeftStickVerticalPos = currLeftStickVerticalPos;

        //  left stick horizontal axis (s, f)
        double currLeftStickHorizontalPos = joystick.getPosition(0);
        if( ( 0.0 == prevLeftStickHorizontalPos ) && ( 0.0 != currLeftStickHorizontalPos ) ) {
            //  Rotation around z axis
            if(0.0 < currLeftStickHorizontalPos )    {
                tip_T_ref.rotate( cnoid::AngleAxis( 0.1, cnoid::Vector3::UnitZ() ) );
            }
            else    {
                tip_T_ref.rotate( cnoid::AngleAxis( -0.1, cnoid::Vector3::UnitZ() ) );
            }
            isMessageOut = true;
        }
        prevLeftStickHorizontalPos = currLeftStickHorizontalPos;
        
        //  Pad horizontal axis (Cursor: Left, Right)
        //  Camera pan
        double currPadHorizontalPos = joystick.getPosition(4);
        if( ( 0.0 == prevPadHorizaontalPos ) && ( 0.0 != currPadHorizontalPos ) ) {
            //  Rotation around x axis
            if(0.0 < currPadHorizontalPos )    {
                tip_T_ref.rotate( cnoid::AngleAxis( 0.1, cnoid::Vector3::UnitY() ) );
            }                
            else    {
                tip_T_ref.rotate( cnoid::AngleAxis( -0.1, cnoid::Vector3::UnitY() ) );
            }
            isMessageOut = true;
        }
        prevPadHorizaontalPos = currPadHorizontalPos;

        //  Pad vertical axis (Cursor: Up, Down)
        //  Camera tilt
        double currPadVerticalPos = joystick.getPosition(5);
        if( ( 0.0 == prevPadVerticalPos ) && ( 0.0 != currPadVerticalPos ) ) {
            //  Rotation around y-axis
            if(0.0 < currPadVerticalPos )    {
                tip_T_ref.rotate( cnoid::AngleAxis( 0.1, cnoid::Vector3::UnitX() ) );
            }                
            else    {
                tip_T_ref.rotate( cnoid::AngleAxis( -0.1, cnoid::Vector3::UnitX() ) );
            }
            isMessageOut = true;
        }
        prevPadVerticalPos = currPadVerticalPos;

        //  For each of the joints, set a target angle as a reference one
        for(int i = 0; i < ioBody->numJoints(); i++)  {
            ioBody->joint(i)->q_target() = q_ref[i];
        }

        //  Message out
        if(isMessageOut)    {
            os << "T_tip_ref" << std::endl;
            os << std::setprecision(3) << tip_T_ref.translation() << std::endl;
            os << std::setprecision(3) << tip_T_ref.linear() << std::endl;
        }

        return( true );
    }
};

CNOID_IMPLEMENT_SIMPLE_CONTROLLER_FACTORY(NaruseArmIKController)
