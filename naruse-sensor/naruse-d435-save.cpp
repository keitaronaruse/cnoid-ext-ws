/*
    naruse-d435-save.cpp
        Simple controller for d435 range camera
            - Sensor on
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
    class NaruseD435SaveController {}
        Simple controller for Naruse D435 Range Camera
            Save an image file and a PCD file 
            when A-button of joystick is pressed down
*/
class NaruseD435SaveController : public cnoid::SimpleController
{
private:
    //  Class instance 
    cnoid::SimpleControllerIO* io;
    cnoid::Joystick joystick;
    cnoid::RangeCamera* d435RangeCamera;
    bool d435PrevButtonState;

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

        return true;
    }

    virtual bool control() override
    {
        joystick.readCurrentState();
        bool stateChanged = false;
        bool buttonState = joystick.getButtonState(cnoid::Joystick::A_BUTTON);
        if(buttonState && !d435PrevButtonState){
            std::ostream& os = io->os();
            //  Get a cuurent scene as image 
            const cnoid::Image& d435RGBImage = d435RangeCamera->constImage();
            //  Save it as an image file
            d435RGBImage.save("test-image.png");
            os << "Saved an image file" << std::endl;

            //  Make a cuurent scene as colored point cloud
            pcl::PointCloud<pcl::PointXYZRGB> cloud = makeColoredPointCloudOfCurrentScene();
            //  Save it as PCD file in binary and compressed format
            pcl::io::savePCDFileBinaryCompressed ("test-image.pcd", cloud);
            // pcl::io::savePCDFileASCII ("test-image.pcd", cloud);
            os << "Saved a pcd file" << std::endl;
        }
        d435PrevButtonState = buttonState;

        return true;
    }
};

CNOID_IMPLEMENT_SIMPLE_CONTROLLER_FACTORY(NaruseD435SaveController)
