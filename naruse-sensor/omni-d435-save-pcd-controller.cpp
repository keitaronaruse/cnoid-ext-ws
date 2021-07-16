/*
    omni-d435-save-pcd-controller.cpp
        Simple controller for omini-d435 range camera
            - omni-d435 includes four d435 range cameras
            - Generate a registered point cloud from the four cameras 
            - Save the registered point clound in a pcd file when A button is pushed down
        Author: Keitaro Naruse
        Date:   2021-07-15
*/
//  For file save in c++ language
#include <iostream>
#include <fstream>

//  For choreonoid
#include <cnoid/SimpleController>
#include <cnoid/RangeCamera>
#include <cnoid/Joystick>

//  pcl related header files
//  For basic IO
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
//  For frame transformation
#include <pcl/common/transforms.h>
//  For filter
#include <pcl/filters/passthrough.h>

/*
    class OmniD435SavePCDController {}
        Simple controller for d435.yaml
            Save an image file and a PCD file 
            when A-button of joystick is pressed down
*/
class OmniD435SavePCDController : public cnoid::SimpleController
{
private:
    //  Class instance 
    cnoid::SimpleControllerIO* io;
    cnoid::Joystick joystick;
    //  Range camera instances for the four D435 cameras
    cnoid::RangeCamera* d435RangeCamera[4];
    //  Transformation matrix for the four D435 cameras
    Eigen::Matrix4f d435TransformationMatrix[4];
    bool d435PrevButtonState;

    //  Make a colored point cloud of a current scene in the choreonoid frame
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr makeColoredPointCloudInCnoidFrame( cnoid::RangeCamera* range_camera ) {
        // Make an instance of a point cloud
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr pc_ptr( new pcl::PointCloud<pcl::PointXYZRGB> );

        //  Image width and height
        const int width  = range_camera->constImage().width();
        const int height =  range_camera->constImage().height();
        //  Pixel pointer 
        //  Each pixel i has unsgined r[i], g[i], b[i];
        const unsigned char* pixels =  range_camera->constImage().pixels();
        
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
        for(const auto& e: range_camera->constPoints()) {
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
        return( pc_ptr );
    }

    //  Convert a colored point cloud from choreonoid frame to D435 frame
    //  Reference: https://pointclouds.org/documentation/tutorials/matrix_transform.html 
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr convertFromCnoidFrameToD435Frame( pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr pc_ptr_from ) {
        //  A frame transdered point clound from a given one 
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr pc_ptr_to( new pcl::PointCloud<pcl::PointXYZRGB> );
        //  Matrix 4f for affine transformation
        Eigen::Matrix4f transform;
        transform << 
             1.0,  0.0,  0.0,  0.0, 
             0.0, -1.0,  0.0,  0.0, 
             0.0,  0.0, -1.0,  0.0, 
             0.0,  0.0,  0.0,  1.0;

        //  Make frame transformation
        pcl::transformPointCloud (*pc_ptr_from, *pc_ptr_to, transform );
        
        //  Return a frame transferred point cloud
        return( pc_ptr_to );
    }
    
    //  Remove outliers in a given point cloud
    //  Reference: https://pcl.readthedocs.io/projects/tutorials/en/latest/passthrough.html#passthrough
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr applyPassThroughFilter(pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr pc_ptr_from) {
        //  Make a pass-throughed point cloud from a given one 
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr pc_ptr_to(new pcl::PointCloud<pcl::PointXYZRGB>);
        // Create a filtering object
        pcl::PassThrough<pcl::PointXYZRGB> pass;
        pass.setInputCloud( pc_ptr_from );
        pass.setFilterFieldName( "z" );
        pass.setFilterLimits( 0, 3.0 );
        pass.setFilterLimitsNegative( true );
        pass.filter( *pc_ptr_to );

        //  Return an outlier removed point cloud
        return( pc_ptr_to );
    }

public:
    virtual bool initialize(cnoid::SimpleControllerIO* io) override
    {
        this->io = io;
        cnoid::Body* body = io->body();
        //  Front D435
        //  Range camera instance
        d435RangeCamera[0] = body-> findDevice<cnoid::RangeCamera>("FrontD435");
        //  Transformation matrix
        d435TransformationMatrix[0] << 
             0.0,  0.0,  1.0,  0.058, 
            -1.0,  0.0,  0.0,  0.0, 
             0.0, -1.0,  0.0,  0.013, 
             0.0,  0.0,  0.0,  1.0;

        //  Left D435
        //  Range camera instance
        d435RangeCamera[1] = body-> findDevice<cnoid::RangeCamera>("LeftD435");
        //  Transformation matrix
        d435TransformationMatrix[1] << 
             1.0,  0.0,  0.0,  0.0, 
             0.0,  0.0,  1.0,  0.058, 
             0.0, -1.0,  0.0,  0.013, 
             0.0,  0.0,  0.0,  1.0;
        //  Rear D435
        //  Range camera instance
        d435RangeCamera[2] = body-> findDevice<cnoid::RangeCamera>("RearD435");
        //  Transformation matrix
        d435TransformationMatrix[2] << 
             0.0,  0.0, -1.0, -0.058, 
             1.0,  0.0,  0.0,  0.0, 
             0.0, -1.0,  0.0,  0.013, 
             0.0,  0.0,  0.0,  1.0;
        //  Right D435
        //  Range camera instance
        d435RangeCamera[3] = body-> findDevice<cnoid::RangeCamera>("RightD435");
        //  Transformation matrix
        d435TransformationMatrix[3] << 
            -1.0,  0.0, 0.0,   0.0, 
             0.0,  0.0, -1.0, -0.058, 
             0.0, -1.0,  0.0,  0.013, 
             0.0,  0.0,  0.0,  1.0;

        //  Enable sensor
        for(auto e:d435RangeCamera){
            io->enableInput(e);
        }

        //  Set initial state of button
        d435PrevButtonState = false;

        return( true );
    }

    virtual bool control() override
    {
        joystick.readCurrentState();
        bool stateChanged = false;
        bool buttonState = joystick.getButtonState(cnoid::Joystick::A_BUTTON);
        if(buttonState && !d435PrevButtonState){
            //  Set os to choreonoid message window
            std::ostream& os = io->os();
            
            //  Point cloud data of each of the four D435 range camers in choreonoid frame 
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcPtrCnoidFrame[4];
            //  Filtered (far point removal) point cloud data of the above 
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcPtrPassThroughCnoidFrame[4];
            //  Filtred point cloud data in a D435 local frame 
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcPtrPassThroughD435Frame[4];
            //  Transformed point cloud in the sensor base frame 
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcPtrPassThroughSensorFrame[4];
            //  Registered point cloud in the sensor base frame;
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcRegiseredinSensorFrame(new pcl::PointCloud<pcl::PointXYZRGB>); 
            
            //  Merge of point clounds
            for(int i = 0; 4 != i; ++i) {
                //  Get a point cloud from each of the range cameras 
                pcPtrCnoidFrame[i] = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>);;
                pcPtrCnoidFrame[i] = makeColoredPointCloudInCnoidFrame(d435RangeCamera[i]);
                //  Apply pass filter for removing far points
                pcPtrPassThroughCnoidFrame[i] = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>);;
                pcPtrPassThroughCnoidFrame[i] = applyPassThroughFilter(pcPtrCnoidFrame[i]);
                //  Convert it into the D435 local frame
                pcPtrPassThroughD435Frame[i] = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>);;
                pcPtrPassThroughD435Frame[i] = convertFromCnoidFrameToD435Frame(pcPtrPassThroughCnoidFrame[i]);
                //  Transform it to the sensor local frame
                pcPtrPassThroughSensorFrame[i] = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
                pcl::transformPointCloud(*pcPtrPassThroughD435Frame[i], *pcPtrPassThroughSensorFrame[i], d435TransformationMatrix[i]);
                //  Merge
                *pcRegiseredinSensorFrame += *pcPtrPassThroughSensorFrame[i];
            }
            //  Make a cuurent scene as colored point cloud
            //  Save it as PCD file in binary and compressed format
            // pcl::io::savePCDFileBinaryCompressed("test-cnoid-frame.pcd", *pcPtrCnoidFrame);
            // pcl::io::savePCDFileBinaryCompressed("test-pass-through-cnoid-frame.pcd", *pcPtrPassThroughCnoidFrame);
            // pcl::io::savePCDFileBinaryCompressed("test-pass-through-d435-frame.pcd", *pcPtrPassThroughD435Frame);
            pcl::io::savePCDFileASCII ("test-omni-d435-frame.ascii.pcd", *pcRegiseredinSensorFrame);
            os << "Saved pcd files" << std::endl;
        }
        d435PrevButtonState = buttonState;

        return true;
    }
};
 
CNOID_IMPLEMENT_SIMPLE_CONTROLLER_FACTORY(OmniD435SavePCDController)
