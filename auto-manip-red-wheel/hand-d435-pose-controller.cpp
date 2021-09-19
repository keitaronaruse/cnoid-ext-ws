/*
    hand-d435-pose-controller.cpp
        Choreonoid simple controller for pose control of hand-d435.body
        Author: Keitaro Naruse
        Date:   2021-09-19
*/
//  For file save in c++ language
#include <iostream>
#include <fstream>
#include <vector>
#include <string>
//  Eigen
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>
//  For choreonoid
#include <cnoid/SimpleController>
#include <cnoid/RangeCamera>
#include <cnoid/Joystick>
/*
    class HandD435PoseController
        Simple controller for pose control of hand-d435.body
*/
class HandD435PoseController : public cnoid::SimpleController
{
private:
    //  Class instance 
    cnoid::SimpleControllerIO* io;
    cnoid::Joystick joystick;
    bool PrevAButtonState;
    bool PrevBButtonState;
    bool PrevXButtonState;
    bool PrevYButtonState;
    //  Link object
    cnoid::Link* graspPointLink;
    //  Reference of pose
    Eigen::Vector3f t_ref;
    Eigen::Matrix3f r_ref;
    Eigen::Vector3f euler_angles_ref;

public:
    virtual bool initialize(cnoid::SimpleControllerIO* io) override
    {
        this->io = io;
        cnoid::Body* body = io->body();

        PrevAButtonState = false;
        PrevBButtonState = false;
        
        // Initial reference
        t_ref << 0.0f, 0.0f, 0.0f;
        r_ref << 
            1.0f, 0.0f, 0.0f, 
            0.0f, 1.0f, 0.0f, 
            0.0f, 0.0f, 1.0f;
        euler_angles_ref << 0.0f, 0.0f, 0.0f;

        //  Joint initialize
        for(int i = 0; i < io -> body() -> numJoints(); ++i){
            cnoid::Link* joint = io -> body() -> joint(i);
            joint -> setActuationMode(cnoid::Link::JointDisplacement);
            io -> enableIO(joint);
        }
        graspPointLink = io -> body( )-> link("GraspingPoint");
        graspPointLink -> setActuationMode(cnoid::Link::LinkPosition);
        io -> enableIO(graspPointLink);

        return true;
    }

    virtual bool control() override
    {
        joystick.readCurrentState();
        bool stateChanged = false;
        bool AbuttonState = joystick.getButtonState(cnoid::Joystick::A_BUTTON);
        bool BbuttonState = joystick.getButtonState(cnoid::Joystick::B_BUTTON);
        bool XbuttonState = joystick.getButtonState(cnoid::Joystick::X_BUTTON);
        bool YbuttonState = joystick.getButtonState(cnoid::Joystick::Y_BUTTON);

        //  When A button is pushed down at this moment
        //  Console out current reference values
        if(AbuttonState && !PrevAButtonState){
            //  Set os to choreonoid message window
            std::ostream& os = io -> os();
            os << "A button has pushed." << std::endl;
            os << "Ref translation:" << std::endl << t_ref << std::endl;
            os << "Ref rotation:" << std::endl << r_ref << std::endl;
        }
        PrevAButtonState = AbuttonState;

        //  When B button is pushed down at this moment
        //  Console out options strings
        if(BbuttonState && !PrevBButtonState){
            //  Set os to choreonoid message window
            std::ostream& os = io -> os();
            os << "B button has pushed." << std::endl;

            //  Get option strings given in project > simple controller > options
            std::vector<std::string> options = io -> options();
            if(12 == options.size())    {
                os << "Options are assigned to reference." << std::endl;
                t_ref << std::stof(options[0]), std::stof(options[1]), std::stof(options[2]);
                r_ref <<
                    std::stof(options[ 3]), std::stof(options[ 4]), std::stof(options[ 5]), 
                    std::stof(options[ 6]), std::stof(options[ 7]), std::stof(options[ 8]), 
                    std::stof(options[ 9]), std::stof(options[10]), std::stof(options[11]);
                //  Findl Euler angles (roll, pitch, yaw) from the rotation matrix r_ref
                Eigen::Vector3f euler_angles = r_ref.eulerAngles(0, 1, 2);
                euler_angles_ref(0) = euler_angles(0);
                euler_angles_ref(1) = euler_angles(1);
                euler_angles_ref(2) = euler_angles(2);
            }
            else    {
                os << "Wrong options string format." << std::endl;
            }
        }
        PrevBButtonState = BbuttonState;

        //  When X button is pushed down at this moment
        //  Calculate Euler angle from r_ref
        if(XbuttonState && !PrevXButtonState){
            //  Set os to choreonoid message window
            std::ostream& os = io -> os();
            os << "X button has pushed." << std::endl;
            Eigen::Vector3f euler_angles = r_ref.eulerAngles(0, 1, 2);
            os << "X: roll:  " << euler_angles(0) / 3.14159 * 180.0f << std::endl;
            os << "Y: pitch: " << euler_angles(1) / 3.14159 * 180.0f << std::endl;
            os << "Z: yaw:   " << euler_angles(2) / 3.14159 * 180.0f << std::endl;
        }
        PrevXButtonState = XbuttonState;

        //  When Y button is pushed down at this moment
        if(YbuttonState && !PrevYButtonState){
            ;
        }
        PrevYButtonState = YbuttonState;
        
        //  Joint Control
        //  x, y, z positions
        io -> body() -> joint(0) -> q_target() = t_ref(0);
        io -> body() -> joint(1) -> q_target() = t_ref(1);
        io -> body() -> joint(2) -> q_target() = t_ref(2);
        //  roll, pitch, yaw angles
        io -> body() -> joint(3) -> q_target() = euler_angles_ref(0);
        io -> body() -> joint(4) -> q_target() = euler_angles_ref(1);
        io -> body() -> joint(5) -> q_target() = euler_angles_ref(2);       
        
        return true;
    }
};

CNOID_IMPLEMENT_SIMPLE_CONTROLLER_FACTORY(HandD435PoseController)
