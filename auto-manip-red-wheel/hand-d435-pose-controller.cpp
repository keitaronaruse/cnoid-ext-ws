/*
    hand-d435-pose-controller.cpp
        Choreonoid simple controller for pose control of hand-d435.body
        Author: Keitaro Naruse
        Date:   2021-09-19 > 09-20
*/
//  For file save in c++ language
#include <iostream>
#include <fstream>
#include <vector>
#include <string>
//  Eigen
#include <Eigen/Core>
#include <Eigen/LU>
#include <Eigen/Dense>
#include <Eigen/Geometry>
//  For choreonoid
#include <cnoid/SimpleController>
#include <cnoid/RangeCamera>
#include <cnoid/Joystick>
//  pcl related header files
//  PCL Fundamentals
#include <pcl/pcl_base.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
//  For frame transformation
#include <pcl/common/transforms.h>
//  PCL Segmentation
#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/region_growing_rgb.h>
//  PCL RANSAC
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_circle3d.h>

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
    cnoid::RangeCamera* d435RangeCamera;
    bool PrevAButtonState;
    bool PrevBButtonState;
    bool PrevXButtonState;
    bool PrevYButtonState;
    cnoid::Link *Base;

    //  Reference of pose
    Eigen::Vector3f t_ref;
    Eigen::Vector3f euler_angles_ref;
    
    //  c_7 is the center of the valve (red wheel handle) in the camera frame
    Eigen::Vector3f c_7;
    //  o_7 is the offset of the ceter position in the camera frame
    Eigen::Vector3f o_7;
    //  p_7 is the stadby position of the hand in the camera frame
    //  0.2[m] away from the center point in the direction of the normal vector
    Eigen::Vector3f p_7;
    //  n_7 is the normal vector of the valve in the camera frame
    Eigen::Vector3f n_7;
    //  r_7 is the radius of the red wheel in the camera frame
    float r_7;

    //  Frame transformation
    //  from the hand frame(6) to the base frame(0) 
    Eigen::Isometry3f T06;
    //  from the camera frame(7) to the hand frame(6)
    Eigen::Isometry3f T67;
    //  Inverse matrix of T67
    Eigen::Isometry3f T67inv;
    //  from the new camera frame(8) to the current frame(7)
    Eigen::Isometry3f T78;
    //  from the new hand frame(6) to the base frame(0) 
    Eigen::Isometry3f S06;

    //  Class methods
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr makeColoredPointCloudOfCurrentSceneInCnoidFrame(void);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr convertFromCnoidFrameToD435Frame(pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr pc_ptr_from); 
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr applyPassThroughFilter(pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr pc_ptr_from); 
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr segment_and_find_target(pcl::PointCloud<pcl::PointXYZRGB>::Ptr scene_pc_ptr);
    static void rgb2hsv(float r, float g, float b, float &h, float &s, float &v);
    void ransac_circle(pcl::PointCloud<pcl::PointXYZRGB>::Ptr target_pc_ptr, Eigen::Vector3f& circle_center_vector, float& circle_radius, Eigen::Vector3f& circle_norm_vector);
    Eigen::Isometry3f get_T06(void);
    Eigen::Isometry3f get_T67(void);
    Eigen::Isometry3f get_T78(void);

public:
    virtual bool initialize(cnoid::SimpleControllerIO* io) override
    {
        this->io = io;
        cnoid::Body* body = io->body();
        
        // Turn on and Enable IO of D435 for both Camera and RangeCamera
        d435RangeCamera = body -> findDevice<cnoid::RangeCamera>("D435");
        d435RangeCamera -> on(true);
        d435RangeCamera -> notifyStateChange();
        io->enableInput(d435RangeCamera);

        //  Initilize button states
        PrevAButtonState = false;
        PrevBButtonState = false;
        PrevXButtonState = false;
        PrevYButtonState = false;
        
        // Initialize reference of translation, rotation, and Euler angles
        t_ref << 0.0f, 0.0f, 0.0f;
        euler_angles_ref << 0.0f, 0.0f, 0.0f;


        //  c_7 is the center of the valve (red wheel handle) in the camera frame
        c_7 = Eigen::Vector3f(0.0f, 0.0f, 0.3f);
        //  o_7 is the offeset position of the centerin the camera frame
        o_7 = Eigen::Vector3f(0.010f, -0.073f, -0.078f);
        //  p_7 is the stadby position of the hand in the camera frame
        p_7 = Eigen::Vector3f(0.0f, 0.0f, 0.2f);
        //  n_7 is the normal vector of the valve in the camera frame
        n_7 = Eigen::Vector3f(0.0f, 0.0f, 1.0f);
        //  r_7 is the radius of the red wheel in the camera frame
        r_7 = 0.03f;

        //  Set transformation matrix
        //  From the camera frame(7) to the hand frame(6)
        T06 = get_T06();
        //  From the camera frame(7) to the hand frame(6)
        T67 = get_T67();
        T67inv = T67.inverse();
        //  From the new camera frame(8) to the camera frame(7)
        T78 = get_T78();

        //  Initialize joint actuation mode and io
        for(int i = 0; i < io -> body() -> numJoints(); ++i){
            cnoid::Link* joint = io -> body() -> joint(i);
            joint -> setActuationMode(cnoid::Link::JointDisplacement);
            io -> enableIO(joint);
        }
        Base = io -> body() -> link("Base");
        Base -> setActuationMode(cnoid::Link::LinkPosition);
        io->enableIO(Base);

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
        //  Get and save point cloud
        if(AbuttonState && !PrevAButtonState){
            //  Set os to choreonoid message window
            std::ostream& os = io -> os();
            os << "A button has pushed." << std::endl;

            //  Get a scene point cloud in choreonoid frame
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr 
                pcPtrOriginalCnoidFrame = makeColoredPointCloudOfCurrentSceneInCnoidFrame();
            //  Apply pass through filter
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr 
                pcPtrCnoidFrame = applyPassThroughFilter(pcPtrOriginalCnoidFrame);
            //  If point cloud is empty
            if(pcPtrCnoidFrame -> empty())   {
                os << "pcPtrCnoidFrame is empty!" << std::endl;
            }
            else    {
                pcl::io::savePCDFileBinaryCompressed(std::string("debug.cnoid.pcd"), *pcPtrCnoidFrame);
                os << "debug.cnoid.pcd has been saved." << std::endl;
                //  Convert the scene point cloud to camera frame
                pcl::PointCloud<pcl::PointXYZRGB>::Ptr 
                    pcPtrD435Frame = convertFromCnoidFrameToD435Frame(pcPtrCnoidFrame);
                //  Empty point cloud never happnes
                pcl::io::savePCDFileBinaryCompressed(std::string("debug.d435.pcd"), *pcPtrD435Frame);
                os << "debug.d435.pcd has been saved." << std::endl;

                //  Segmentation and find a target cluster of the largest r-ratio
                pcl::PointCloud<pcl::PointXYZRGB>::Ptr 
                    pcPtrTargetD435Frame = segment_and_find_target(pcPtrD435Frame);
                if(pcPtrTargetD435Frame->empty())  {
                    os << "pcPtrTargetD435Frame is empty!" << std::endl;
                }
                else    {
                    //  Save it as a PCD file
                    pcl::io::savePCDFileBinaryCompressed(std::string("debug.target.pcd"), *pcPtrTargetD435Frame);
                    os << "Saved: debug.target.pcd" << std::endl;
                    //  Pose estimation of circle
                    float wheel_radius;

                    ransac_circle(pcPtrTargetD435Frame, c_7, r_7, n_7);
                    //  Calculate standby position
                    const float standby_distance = 0.1;
                    p_7 = c_7 - standby_distance * n_7.normalized();
                    //  p_7 is the stadby position of the hand in the camera frame
                    //  0.1[m] away from the center point in the direction of the normal vector

                    //  Disply the results of the pose estimation
                    os << "Red wheel position vector in Camera Frame: " << std::endl << c_7 << std::endl;
                    os << "Red wheel radius in Camera Frame: " << std::endl << r_7 << std::endl;
                    os << "Red wheel normal vector in Camera Frame: " << std::endl << n_7 << std::endl;
                    os << "Standby position in Camera Frame: " << std::endl << p_7 << std::endl;
                }
            }
        }
        PrevAButtonState = AbuttonState;

        //  When B button is pushed down at this moment
        //  Console out options strings
        if(BbuttonState && !PrevBButtonState){
            //  Set os to choreonoid message window
            std::ostream& os = io -> os();
            //  Frame transformation
            //  Display T06: From the hand frame (6) to the base frame (0)
            T06 = get_T06();
            //  From the camera frame(7) to the hand frame(6)
            //  T67 is fixed and already calculated
            // T67 = get_T67();
            T78 = get_T78();
            //  New hand frame
            S06 = (T06 * T67 * T78) * T67inv;
            os << "S06.translation()" << std::endl << S06.translation() << std::endl;
            os << "S06.linear()" << std::endl << S06.linear() << std::endl;
            //  Set to the reference
            t_ref = S06.translation();
            euler_angles_ref = S06.linear().eulerAngles(0, 1, 2);
        }
        PrevBButtonState = BbuttonState;

        //  When X button is pushed down at this moment
        if(XbuttonState && !PrevXButtonState){
            //  Set os to choreonoid message window
            std::ostream& os = io -> os();
            os << "X button has pushed." << std::endl;
            if(0 != io->options().size()) {
                pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcPtrD435Frame(new pcl::PointCloud<pcl::PointXYZRGB>);
                os << io->options()[0] << std::endl;
                pcl::io::loadPCDFile(io->options()[0], *pcPtrD435Frame);
                //  Segmentation and find a target cluster of the largest r-ratio
                pcl::PointCloud<pcl::PointXYZRGB>::Ptr 
                    pcPtrTargetD435Frame = segment_and_find_target(pcPtrD435Frame);
                if(pcPtrTargetD435Frame->empty())  {
                    os << "pcPtrTargetD435Frame is empty!" << std::endl;
                }
                else {
                    //  Pose estimation of circle
                    float wheel_radius;

                    ransac_circle(pcPtrTargetD435Frame, c_7, r_7, n_7);
                    //  Calculate standby position
                    const float standby_distance = 0.1;
                    p_7 = c_7 - standby_distance * n_7.normalized();
                    
                    //  Disply the results of the pose estimation
                    os << "Red wheel position vector in Camera Frame: " << std::endl << c_7 << std::endl;
                    os << "Red wheel radius in Camera Frame: " << std::endl << r_7 << std::endl;
                    os << "Red wheel normal vector in Camera Frame: " << std::endl << n_7 << std::endl;
                    os << "Standby position in Camera Frame: " << std::endl << p_7 << std::endl;
                }
            }
        }
        PrevXButtonState = XbuttonState;

        //  When Y button is pushed down at this moment
        if(YbuttonState && !PrevYButtonState){
            std::ostream& os = io -> os();
            os << "Y button has pushed." << std::endl;
        }
        PrevYButtonState = YbuttonState;

        double pos = 0.0;
        //  Angle control
        //  Pan = Yaw angle
        pos = joystick.getPosition(cnoid::Joystick::R_STICK_H_AXIS);
        euler_angles_ref(2) += 0.001 * pos;
        //  Tilt = pitch angle
        pos = joystick.getPosition(cnoid::Joystick::R_STICK_V_AXIS);
        euler_angles_ref(1) += 0.001 * pos;
        
        //  Position control
        //  X direction
        pos = joystick.getPosition(cnoid::Joystick::L_STICK_H_AXIS);
        t_ref(0) -= 0.0001 * pos;
        //  Z direction
        pos = joystick.getPosition(cnoid::Joystick::L_STICK_V_AXIS);
        t_ref(2) -= 0.0001 * pos;
        
        //  Control joints
        //  x, y, z positions by prismatic joint
        io -> body() -> joint(0) -> q_target() = t_ref(0);
        io -> body() -> joint(1) -> q_target() = t_ref(1);
        io -> body() -> joint(2) -> q_target() = t_ref(2);
        //  roll, pitch, yaw angles by revolutional joints
        io -> body() -> joint(3) -> q_target() = euler_angles_ref(0);
        io -> body() -> joint(4) -> q_target() = euler_angles_ref(1);
        io -> body() -> joint(5) -> q_target() = euler_angles_ref(2);       
        
        return true;
    }
};

//  Make a colored point cloud of a current scene in the choreonoid frame
pcl::PointCloud<pcl::PointXYZRGB>::Ptr 
    HandD435PoseController::makeColoredPointCloudOfCurrentSceneInCnoidFrame(void) 
{
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
}

//  Convert a colored point cloud from choreonoid frame to D435 frame
//  Reference: https://pointclouds.org/documentation/tutorials/matrix_transform.html 
pcl::PointCloud<pcl::PointXYZRGB>::Ptr 
    HandD435PoseController::convertFromCnoidFrameToD435Frame(pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr pc_ptr_from) 
{
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
pcl::PointCloud<pcl::PointXYZRGB>::Ptr 
    HandD435PoseController::applyPassThroughFilter(pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr pc_ptr_from) 
{
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

/*
    segment_and_find_target()
        Segment a given color point cloud by colored-based region growing segmantation and
        Find a target point clound
        Input: 
            Argument 1: a point cloud of a scene
        Output:   
            Return: a target point cloud
        Reference: 
            https://pcl.readthedocs.io/projects/tutorials/en/latest/region_growing_rgb_segmentation.html
            almost same but parameters
            https://pcl.readthedocs.io/projects/tutorials/en/latest/extract_indices.html
*/
pcl::PointCloud<pcl::PointXYZRGB>::Ptr 
    HandD435PoseController::segment_and_find_target(pcl::PointCloud<pcl::PointXYZRGB>::Ptr scene_pc_ptr)
{
    //  Segmentation by RegionGrowingRGB method
    //  Segmentation object of RegtionGrowingRGB
    pcl::RegionGrowingRGB<pcl::PointXYZRGB> reg;
    //  Search object for RegtionGrowingRGB
    pcl::search::Search<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>);
    reg.setInputCloud(scene_pc_ptr);
    reg.setSearchMethod(tree);

    //  Shishiki parameters
    reg.setDistanceThreshold(100);
    reg.setPointColorThreshold(20); // 10//25
    reg.setRegionColorThreshold(19); // 9//24
    // reg.setMinClusterSize(1500); // 1000
    // reg.setMaxClusterSize(5000); // 5000
    //  Naruse parameters
    reg.setMinClusterSize(300);

    //  Extract clusters in indices and point clouds
    //  Object of vector of PointClound::Ptr
    std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> pc_ptr_clusters;    
    //  Object of vector of indices
    std::vector<pcl::PointIndices> clusters;
    //  Extract segments and store them in indices
    reg.extract(clusters);
    
    //  For finding a target cluseter of the largest r ratio = r/(r+g+b)
    std::vector<float> h_list, s_list, v_list;
    //  For each of the clusters
    for(int i = 0; i != clusters.size(); ++i)   {
        //  New point cloud for each of the clsuters
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr pc_ptr_cluster(new pcl::PointCloud<pcl::PointXYZRGB>);
        //  Convert the indices of the cluster to the designated format
        pcl::PointIndices::Ptr cluster_indices(new pcl::PointIndices(clusters[i]));
        //  Object of extracting point clouds from the indices
        pcl::ExtractIndices<pcl::PointXYZRGB> extract;
        // Extract a point cloud from the indices
        extract.setInputCloud(scene_pc_ptr);
        extract.setIndices(cluster_indices);
        extract.setNegative(false);
        //  Extract a point cloud of this cluster in pc_ptr_cluster
        extract.filter(*pc_ptr_cluster);
        //  Add it to vector of point clouds
        pc_ptr_clusters.push_back(pc_ptr_cluster);

        //  Find the most red componet by the HSV color model
        float h = 0.0f, s = 0.0f, v = 0.0f, ave_h = 0.0f, ave_s = 0.0f, ave_v = 0.0f;
        //  Sum up the componets of h, s, v
        for(auto& point: *pc_ptr_cluster)  {
            HandD435PoseController::rgb2hsv((float)point.r/255.0f, (float)point.g/255.0f, (float)point.b/255.0f, h, s, v);
            ave_h += h;
            ave_s += s;
            ave_v += v;
        }
        //  Calculate average h, s, v
        if(0 != pc_ptr_cluster -> size())   {
            ave_h /= (float)pc_ptr_cluster -> size();
            ave_s /= (float)pc_ptr_cluster -> size();
            ave_v /= (float)pc_ptr_cluster -> size();
        }
        else {
            ave_h = 0.0f;
            ave_s = 0.0f;
            ave_v = 0.0f;
        }

        //  Put the average h, s, v to list for compare
        h_list.push_back(ave_h);
        s_list.push_back(ave_s);
        v_list.push_back(ave_v);
    }

    //  Return an index of the most red cluster
    std::size_t target_index = 0;
    for(std::size_t i = 0; i != h_list.size(); ++i)   {
        //  Find a cluster close to the red
        if( ( (0.0f <= h_list[i]) && (h_list[i] < 0.08f) && (0.2f < s_list[i]) && (0.1f < v_list[i]) ) 
            || ( (0.92f <= h_list[i]) && (h_list[i] <= 1.0f)) && (0.2f < s_list[i]) && (0.1f < v_list[i]) )  {
            target_index = i;
        }

    }

    return(pc_ptr_clusters[target_index]);
}

/*
    rgb2hsv()
        Convert a color model from rgb to hsv
*/
void HandD435PoseController::rgb2hsv(float r, float g, float b, float &h, float &s, float &v)
{
    // (float r, float g, float b)
    float max = r > g ? r : g;
    max = max > b ? max : b;

    float min = r < g ? r : g;
    min = min < b ? min : b;

    //  H
    h = max - min;
    if (h > 0.0f) {
        if (max == r) {
            h = (g - b) / h;
            if (h < 0.0f) {
                h += 6.0f;
            }
        } 
        else if (max == g) {
            h = 2.0f + (b - r) / h;
        } else {
            h = 4.0f + (r - g) / h;
        }
    }
    h /= 6.0f;

    //  S
    s = (max - min);
    if (max != 0.0f)
        s /= max;
    
    //  V
    v = max;
}

/*
    ransac_circle()
        https://pcl.readthedocs.io/projects/tutorials/en/latest/random_sample_consensus.html
*/
void HandD435PoseController::ransac_circle(pcl::PointCloud<pcl::PointXYZRGB>::Ptr target_pc_ptr, 
    Eigen::Vector3f& circle_center_vector, float& circle_radius, Eigen::Vector3f& circle_norm_vector)
{
    //  Point cloud for inliers
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr inliers_pc_ptr(new pcl::PointCloud<pcl::PointXYZRGB>);

    // Created RandomSampleConsensus object and compute the appropriated model
    pcl::SampleConsensusModelCircle3D<pcl::PointXYZRGB>::Ptr
        model_circle3d(new pcl::SampleConsensusModelCircle3D<pcl::PointXYZRGB> (target_pc_ptr));   

    //  Apply RANSAC and result is stored in inliers
    pcl::RandomSampleConsensus<pcl::PointXYZRGB> ransac(model_circle3d);
    //  Threshold 0.01[m] = 1[cm], setting in the tutorial
    // ransac.setDistanceThreshold(.01);
    //  Threshold 0.001[m] = 1[mm]
    ransac.setDistanceThreshold(0.001);
    ransac.computeModel();
    //  Extract inliers as indices
    std::vector<int> inliers;
    ransac.getInliers(inliers);
    //  Extract points for making the best model
    std::vector<int> model_points;
    ransac.getModel(model_points);
    //  Extract a model
    Eigen::VectorXf circle_model(7);
    model_circle3d->computeModelCoefficients(model_points, circle_model);
    //  Circle center position vector
    circle_center_vector = Eigen::Vector3f( circle_model(0), circle_model(1), circle_model(2) );
    //  Circle normal vector
    circle_norm_vector = Eigen::Vector3f( circle_model(4), circle_model(5), circle_model(6) );
    //  We suppose the normal vector is face to the positive z-direction
    //  If it is the negative one, we negate it 
    if(0.0 > circle_model(6))   {
        circle_norm_vector *= -1.0f;
    }
    //  Circle radius
    circle_radius = circle_model(3);
}


/*
    get_T06()
        Return a frame transformation from the hand frame to the base frame
*/
Eigen::Isometry3f HandD435PoseController::get_T06(void)
{
    //  Hand position in the base frame
    Eigen::Translation<float, 3> trans06(t_ref);
    //  Hand orientation in the base frame
    Eigen::AngleAxisf a06; 
    a06 = Eigen::AngleAxisf(euler_angles_ref(0), Eigen::Vector3f::UnitX())
        * Eigen::AngleAxisf(euler_angles_ref(1), Eigen::Vector3f::UnitY())
        * Eigen::AngleAxisf(euler_angles_ref(2), Eigen::Vector3f::UnitZ());
    //  Make a transformation matrix 
    Eigen::Isometry3f t06 = trans06 * a06;

    return(t06);
}

/*
    get_T67()
        Return a frame transformation from the camera frame to the hand frame
*/
Eigen::Isometry3f HandD435PoseController::get_T67(void)
{
    //  Camera positon in the hand frame
    Eigen::Translation<float, 3> trans67(0.078f, 0.010f, 0.073f);
    //  Camera orientation in the hand frame
    Eigen::Matrix3f m67;
    m67 <<  0.0f,  0.0f, -1.0f, 
            1.0f,  0.0f,  0.0f,
            0.0f, -1.0f,  0.0f;
    //  Convert rotation matrix to angle rotation
    Eigen::AngleAxisf a67(m67);
    Eigen::Isometry3f t67 = trans67 * a67;
    
    return(t67);
}

/*
    get_T78()
        Return a frame transformation from the new camera frame (8) to the camera frame (7)
*/
Eigen::Isometry3f HandD435PoseController::get_T78(void)
{
    //  Translation of the new camera frame in the camera frame
    //  Standby point
    Eigen::Translation<float, 3> trans78(p_7 + o_7);
    // Eigen::Translation<float, 3> trans78(p_7);

    //  Orientation of the new camer frame in the camera frame
    //  It is calculated as follows: 
    //  z8 is given by the normal vector of the red wheel 
    Eigen::Vector3f z8 = n_7.normalized();
    //  x8 is set to (1, 0, 0) always
    Eigen::Vector3f x8 = Eigen::Vector3f::UnitX();
    //  x8 is set to be normal to both of UnitZ and n_7
    // Eigen::Vector3f x8 = Eigen::Vector3f::UnitZ().cross(n_7);
    //  y8 is found by corss of z8 and x8
    Eigen::Vector3f y8 = z8.cross(x8).normalized(); 
    //  Rotation matrix
    Eigen::Matrix3f m78;
    m78 << x8(0), y8(0), z8(0), 
            x8(1), y8(1), z8(1),
            x8(2), y8(2), z8(2);
    //  Convert rotation matrix to angle rotation
    Eigen::AngleAxisf a78(m78);
    
    //  Tranformation matrix
    Eigen::Isometry3f t78 = trans78 * a78;
    return(t78);
}

CNOID_IMPLEMENT_SIMPLE_CONTROLLER_FACTORY(HandD435PoseController)
