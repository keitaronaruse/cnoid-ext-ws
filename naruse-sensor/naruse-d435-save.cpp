/*
    naruse-d435-on.cpp
        Author: Keitaro Naruse
        Date:   2021-06-10
*/
#include <iostream>
#include <fstream>

#include <cnoid/SimpleController>
#include <cnoid/SpotLight>
#include <cnoid/Camera>
#include <cnoid/RangeCamera>
#include <cnoid/Joystick>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

using namespace std;
using namespace cnoid;

class NaruseD435SaveController : public SimpleController
{
    SimpleControllerIO* io;
    Joystick joystick;
    RangeCamera* d435Device;
    bool d435PrevButtonState;
public:
    
    virtual bool initialize(SimpleControllerIO* io) override
    {
        this->io = io;
        Body* body = io->body();

        // Turn on D435
        d435Device = body->findDevice<RangeCamera>("D435");
        d435Device->on(true);
        d435Device->notifyStateChange();
        d435PrevButtonState = false;
        // Enable IO
        io->enableInput(d435Device);

        return true;
    }

    virtual bool control() override
    {
        joystick.readCurrentState();
        bool stateChanged = false;
        bool buttonState = joystick.getButtonState(Joystick::A_BUTTON);
        if(buttonState && !d435PrevButtonState){
            ostream& os = io->os();
            const Image& d435RGBImage = d435Device->constImage();

            d435RGBImage.save("test-image.png");
            os << "Saved an image file" << std::endl;

            const int width = d435RGBImage.width(), 
                height = d435RGBImage.height(); 
            //  pixel array stored in an image
            const unsigned char* pixels = d435RGBImage.pixels();
            
            //  PCD file save
            //  Initialize point cloud
            pcl::PointCloud<pcl::PointXYZRGB> cloud;
            // Fill in the cloud data
            cloud.width    = width;
            cloud.height   = height;
            cloud.is_dense = false;
            cloud.points.resize(cloud.width * cloud.height);
            std::size_t i = 0;
            for(const auto& e: d435Device->constPoints()) {
                cloud[i].x = e(0);
                cloud[i].y = e(1);
                cloud[i].z = e(2);
                cloud[i].r = pixels[3*i + 0];
                cloud[i].g = pixels[3*i + 1];
                cloud[i].b = pixels[3*i + 2];
                ++i;
            }
            os << "cloud.size(): " << cloud.points.size() << std::endl;
            pcl::io::savePCDFileBinaryCompressed ("test-pcd.pcd", cloud);
            os << "Saved a pcd file" << std::endl;
        }
        d435PrevButtonState = buttonState;

        return true;
    }
};

CNOID_IMPLEMENT_SIMPLE_CONTROLLER_FACTORY(NaruseD435SaveController)
