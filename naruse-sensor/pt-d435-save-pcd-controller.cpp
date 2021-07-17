/*
    pt-d435-save.cpp
        Simple controller for d435 range camera with pan and tilt control
            - Pan angle: +Delta by L button, -Delta by   
            - Save an image file and pcd one when A button is pushed down
        Author: Keitaro Naruse
        Date:   2021-07-17
*/
//  For file save in c++ language
#include <iostream>
#include <fstream>

//  For choreonoid
#include <cnoid/SimpleController>
#include <cnoid/RangeCamera>
#include <cnoid/Joystick>

//  For pcl
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
//  For frame transformation
#include <pcl/common/transforms.h>
//  For filter
#include <pcl/filters/passthrough.h>

/*
    class PTD435SavePCDController {}
        Simple controller for Naruse D435 Range Camera
            Save an image file and a PCD file 
            when A-button of joystick is pressed down
*/
class PTD435SavePCDController : public cnoid::SimpleController
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

    //  Reference angle
    double qref_pan;
    double qref_tilt;

    //  Make a colored point cloud of a current scene
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr makeColoredPointCloudOfCurrentSceneInCnoidFrame() {
         // Make an instance of a point cloud
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr pc_ptr(new pcl::PointCloud<pcl::PointXYZRGB>);

        //  Image width and height
        const int width  = d435RangeCamera->constImage().width();
        const int height =  d435RangeCamera->constImage().height();
        //  Pixel pointer 
        //  Each pixel i has unsgined r[i], g[i], b[i];
        const unsigned char* pixels =  d435RangeCamera->constImage().pixels();
        
        // Make a colored point cloud from a scene
        // Initialize the instance
        pc_ptr -> width    = width * height;
        pc_ptr -> height   = 1;
        pc_ptr -> is_dense = false;
        pc_ptr -> points.resize(pc_ptr -> width * pc_ptr -> height);

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
            pc_ptr -> points[i].x = e(0);
            pc_ptr -> points[i].y = e(1);
            pc_ptr -> points[i].z = e(2);
            //  Find a corresponding pixel and set color 
            pc_ptr -> points[i].r = pixels[3*i + 0];
            pc_ptr -> points[i].g = pixels[3*i + 1];
            pc_ptr -> points[i].b = pixels[3*i + 2];
            ++i;
        }

        return(pc_ptr);
    };

    //  Convert a colored point cloud from choreonoid frame to D435 frame
    //  Reference: https://pointclouds.org/documentation/tutorials/matrix_transform.html 
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr convertFromCnoidFrameToD435Frame(pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr pc_ptr_from) {
        //  A frame transdered point clound from a given one 
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr pc_ptr_to(new pcl::PointCloud<pcl::PointXYZRGB>);
        //  Matrix 4f for affine transformation
        Eigen::Matrix4f transform;
        transform << 
             1.0,  0.0,  0.0,  0.0, 
             0.0, -1.0,  0.0,  0.0, 
             0.0,  0.0, -1.0,  0.0, 
             0.0,  0.0,  0.0,  1.0;

        //  Make frame transformation
        pcl::transformPointCloud(*pc_ptr_from, *pc_ptr_to, transform);
        
        //  Return a frame transferred point cloud
        return(pc_ptr_to);
    }
    
    //  Remove outliers in a given point cloud
    //  Reference: https://pcl.readthedocs.io/projects/tutorials/en/latest/passthrough.html#passthrough
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr applyPassThroughFilter(pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr pc_ptr_from) {
        //  Make a pass-throughed point cloud from a given one 
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr pc_ptr_to(new pcl::PointCloud<pcl::PointXYZRGB>);
        // Create a filtering object
        pcl::PassThrough<pcl::PointXYZRGB> pass;
        pass.setInputCloud(pc_ptr_from);
        pass.setFilterFieldName("z");
        pass.setFilterLimits(0, 3.0);
        pass.setFilterLimitsNegative (true);
        pass.filter (*pc_ptr_to);

        //  Return an outlier removed point cloud
        return(pc_ptr_to);
    }

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
            // const cnoid::Image& d435RGBImage = d435RangeCamera->constImage();
            //  Save it as an image file
            // d435RGBImage.save("test-image.png");
            // os << "Saved an image file" << std::endl;

            //  Make a cuurent scene as colored point cloud
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcPtrCnoidFrame = makeColoredPointCloudOfCurrentSceneInCnoidFrame();
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcPtrPassThroughCnoidFrame = applyPassThroughFilter(pcPtrCnoidFrame);
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcPtrPassThroughD435Frame = convertFromCnoidFrameToD435Frame(pcPtrPassThroughCnoidFrame);
            //  Save it as PCD file in binary and compressed format
            pcl::io::savePCDFileBinaryCompressed("test-cnoid-frame.pcd", *pcPtrCnoidFrame);
            pcl::io::savePCDFileBinaryCompressed("test-pass-through-cnoid-frame.pcd", *pcPtrPassThroughCnoidFrame);
            pcl::io::savePCDFileBinaryCompressed("test-pass-through-d435-frame.pcd", *pcPtrPassThroughD435Frame);
            pcl::io::savePCDFileASCII ("test-pass-through-d435-frame.ascii.pcd", *pcPtrPassThroughD435Frame);
            os << "Saved pcd files" << std::endl;
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

CNOID_IMPLEMENT_SIMPLE_CONTROLLER_FACTORY(PTD435SavePCDController)
