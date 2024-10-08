/*
    naruse-pt-d435-save.cpp
        Simple controller for d435 range camera with pan and tilt control
            - Pan angle: +Delta by L button, -Delta by   
            - Save an image file and pcd one when A button is pushed down
        Author: Keitaro Naruse
        Date:   2021-06-14
*/
//  For file save in c++ language
#include <iostream>
#include <fstream>

//  For choreonoid
#include <cnoid/SimpleController>
#include <cnoid/Camera>
#include <cnoid/RangeCamera>
#include <cnoid/Joystick>

//  For pcl
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

/*
    class NarusePT-D435SaveController {}
        Simple controller for Naruse D435 Range Camera
            Save an image file and a PCD file 
            when A-button of joystick is pressed down
*/
class NarusePTD435SaveController : public cnoid::SimpleController
{
private:
    //  Class instance 
    //  SimpleController
    cnoid::SimpleControllerIO* io;
    //  Joystick
    cnoid::Joystick joystick;
    //  RangeCamera
    cnoid::RangeCamera* d435RangeCamera;
    //  Button flag
    bool d435PrevButtonState;
    //  Joint
    cnoid::Link* panPlate;
    cnoid::Link* tiltPlate;

    //  Reference
    double qref_pan;
    double qref_tilt;

    //  Make a colored point cloud of a current scene
    pcl::PointCloud<pcl::PointXYZRGB> makeColoredPointCloudOfCurrentScene() {
        //  Get a cuurent scene as image 
        const cnoid::Image& d435RGBImage = d435RangeCamera->constImage();

        //  Image width and height
        const int width  = d435RGBImage.width();
        const int height = d435RGBImage.height();
        //  Pixel pointer 
        //  Each pixel i has unsgined r[i], g[i], b[i];
        const unsigned char* pixels = d435RGBImage.pixels();
        
        // Make a colored point cloud from a scene
        // Make an instance of a point cloud
        pcl::PointCloud<pcl::PointXYZRGB> cloud;
        // Initialize the instance
        cloud.width    = width;
        cloud.height   = height;
        cloud.is_dense = false;
        cloud.points.resize(cloud.width * cloud.height);
        
        //  Conver a 3D point in RangeCamera and color in Camera to a colored point cloud
        //  Each point i in cloud is represented as
        //      cloud[i].x, cloud[i].y, cloud[i].z: (x,y,z) position 
        //      cloud[i].r, cloud[i].g, cloud[i].b: color data
        //  points are alligned in 1D as Vector3f of x, y, and z
        //  pixels are alligned in 1D with 3 components of r,g,b
        std::size_t i = 0;
        //  for each point e from points in RangeCamera
        for(const auto& e: d435RangeCamera->constPoints()) {
            //  Set x, y, z from e
            cloud[i].x = e(0);
            cloud[i].y = e(1);
            cloud[i].z = e(2);
            //  Find a corresponding pixel and set color 
            cloud[i].r = pixels[3*i + 0];
            cloud[i].g = pixels[3*i + 1];
            cloud[i].b = pixels[3*i + 2];
            ++i;
        }
        return(cloud);
    };
public:
    virtual bool initialize(cnoid::SimpleControllerIO* io) override
    {
        this->io = io;
        cnoid::Body* body = io->body();

        // Turn on and Enable IO of D435 for both Camera and RangeCamera
        d435RangeCamera = body-> findDevice<cnoid::RangeCamera>("D435");
        io->enableInput(d435RangeCamera);
        d435PrevButtonState = false;

        //  Set pan and tilt joints
        panPlate = io->body()->link("PanPlate");
        tiltPlate = io->body()->link("TiltPlate");
        panPlate -> setActuationMode(cnoid::Link::JointDisplacement);
        tiltPlate -> setActuationMode(cnoid::Link::JointDisplacement);
        io->enableOutput(panPlate);
        io->enableOutput(tiltPlate);
        //  Set initial reference angles of pan and tilt
        qref_pan = 0.0;
        qref_tilt = 0.0;

        return true;
    }

    virtual bool control() override
    {
        //  Read joystick status 
        joystick.readCurrentState();
        
        //  Set output stream
        std::ostream& os = io->os();

        //  Check A button for save files
        bool stateChanged = false;
        bool buttonState = joystick.getButtonState(cnoid::Joystick::A_BUTTON);
        if(buttonState && !d435PrevButtonState){
            //  Get a cuurent scene as image 
            const cnoid::Image& d435RGBImage = d435RangeCamera->constImage();
            //  Save it as an image file
            d435RGBImage.save("test-image.png");
            os << "Saved an image file" << std::endl;

            //  Make a cuurent scene as colored point cloud
            pcl::PointCloud<pcl::PointXYZRGB> cloud = makeColoredPointCloudOfCurrentScene();
            //  Save it as PCD file in binary and compressed format
            pcl::io::savePCDFileBinaryCompressed ("test-image.pcd", cloud);
            //  Save it as PCD file in the ASCII format
            // pcl::io::savePCDFileASCII("test-image.pcd", cloud);
            os << "Saved a pcd file" << std::endl;
        }
        d435PrevButtonState = buttonState;
        
        //  L key, Rotate pan negative
        if(joystick.getPosition(2) > 0.5)  {
            qref_pan -= 0.001;
            os << "qref_pan[deg]:" << qref_pan/3.14*180.0 << std::endl;
        }
        //  J key, Rotate pan positive
        else if(joystick.getPosition(2) < -0.5) {
            qref_pan += 0.001;
            os << "qref_pan[deg]:" << qref_pan/3.14*180.0 << std::endl;
        } 
        //  K key, Rotate tilt positive
        if(joystick.getPosition(3) > 0.5)  {
            qref_tilt += 0.001;
            os << "qref_tilt[deg]:" << qref_tilt/3.14*180.0 << std::endl;
        }
        //  I key, Rotatetilt negative
        else if(joystick.getPosition(3) < -0.5) {
            qref_tilt -= 0.001;
            os << "qref_tilt[deg]:" << qref_tilt/3.14*180.0 << std::endl;
        } 

        //  Acutation by angle
        panPlate -> q_target() = qref_pan;
        tiltPlate -> q_target() = qref_tilt;

        return true;
    }
};

CNOID_IMPLEMENT_SIMPLE_CONTROLLER_FACTORY(NarusePTD435SaveController)
