/*
    human-hold.cpp
        Simple controller for human to hold a give pose
        Author: Keitaro Naruse
        Date:   2021-08-01
*/
//  C++ include files
//  For stream
#include <iostream>
//  For vector
#include <vector>

//  Choreonoid include files
//  For simple controller
#include <cnoid/SimpleController>

/*
    class HumanHoldController
        Simple controller for human to hold a give pose
*/
class HumanHoldController : public cnoid::SimpleController
{
private:
    //  Class instance 
    //  SimpleController
    cnoid::SimpleControllerIO* io;
    //  Reference joint angle
    std::vector<double> q_ref;
    
public:
    virtual bool initialize(cnoid::SimpleControllerIO* io) override
    {
        //  Set body model
        this->io = io;
        cnoid::Body* body = io->body();
        
        //  Set output stream
        std::ostream& os = io->os();

        //  Enable joint output
        for(int i = 0; i < body -> numJoints(); i++)  {
            //  For each of the joint links
            cnoid::Link* joint = body->joint(i);
            //  Set a control mode as a joint angle
            joint -> setActuationMode(cnoid::Link::JointAngle);
            //  Enable input and output
            io -> enableIO(joint);
            //  Set an initial referece angle
            q_ref.push_back(joint->q());
        }
        q_ref[0] =  0.0;
        q_ref[1] = -2.0944;
        q_ref[2] =  0.5236;
        q_ref[3] =  0.0;

        return( true );
    }

    virtual bool control() override
    {
        //  Set output stream
        std::ostream& os = io->os();

        //  Set body model
        this->io = io;
        cnoid::Body* body = io->body();

        //  For each of the joints, set a target angle as a reference one
        for(int i = 0; i < body->numJoints(); i++)  {
            cnoid::Link* joint = body->joint(i);
            joint->q_target() = q_ref[i];
        }
        
        return( true );
    }
};

CNOID_IMPLEMENT_SIMPLE_CONTROLLER_FACTORY(HumanHoldController)
