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
    cnoid::Link *CameraBody;
    cnoid::Link *Base;

    //  Reference of pose
    Eigen::Vector3f t_ref;
    Eigen::Matrix3f r_ref;
    Eigen::Vector3f euler_angles_ref;
    
    //  Frame transformation
    //  from the hand frame to the base frame 
    Eigen::Isometry3f T06;
    //  from the camera frame to the hand frame
    Eigen::Isometry3f T67;
    //  from the camera frame to the base frame
    Eigen::Isometry3f T07;
    //  from the base frame to the camera frame
    Eigen::Isometry3f T70;

    //  Class methods
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr makeColoredPointCloudOfCurrentSceneInCnoidFrame(void);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr convertFromCnoidFrameToD435Frame(pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr pc_ptr_from); 
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr applyPassThroughFilter(pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr pc_ptr_from); 
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr segment_and_find_target(pcl::PointCloud<pcl::PointXYZRGB>::Ptr scene_pc_ptr);
    static void rgb2hsv(float r, float g, float b, float &h, float &s, float &v);
    void ransac_circle(pcl::PointCloud<pcl::PointXYZRGB>::Ptr target_pc_ptr, Eigen::Vector3f& circle_center_vector, float& circle_radius, Eigen::Vector3f& circle_norm_vector);
    Eigen::Isometry3f find_trans_mat_from_new_cam_frame_to_hand_frame(const Eigen::Vector3f& pos_vect_cam_frame, const Eigen::Vector3f& norm_vect_cam_frame);
    Eigen::Isometry3f get_T06(void);
    Eigen::Isometry3f get_T67(void);

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
        r_ref << 
            1.0f, 0.0f, 0.0f, 
            0.0f, 1.0f, 0.0f, 
            0.0f, 0.0f, 1.0f;
        euler_angles_ref << 0.0f, 0.0f, 0.0f;

        //  Frame transformation
        T06 = get_T06();
        T67 = get_T67();
        T07 = T06 * T67;
        T70 = T07.inverse();

        //  Initialize joint actuation mode and io
        for(int i = 0; i < io -> body() -> numJoints(); ++i){
            cnoid::Link* joint = io -> body() -> joint(i);
            joint -> setActuationMode(cnoid::Link::JointDisplacement);
            io -> enableIO(joint);
        }
        CameraBody = io -> body() -> link("CameraBody");
        CameraBody -> setActuationMode(cnoid::Link::LinkPosition);
        io->enableIO(CameraBody);

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
                    Eigen::Vector3f wheel_pos_vect_cam_frame;
                    float wheel_radius;
                    Eigen::Vector3f wheel_norm_vect_cam_frame;

                    ransac_circle(pcPtrTargetD435Frame, wheel_pos_vect_cam_frame, wheel_radius, wheel_norm_vect_cam_frame);
                    //  Disply the results of the pose estimation
                    os << "Red wheel position vector in Camera Frame: " << std::endl << wheel_pos_vect_cam_frame << std::endl;
                    os << "Red wheel normal vector in Camera Frame: " << std::endl << wheel_norm_vect_cam_frame << std::endl;
                    os << "Red wheel radius in Camera Frame: " << std::endl << wheel_radius << std::endl;

                    //  Find a transformation from a camera frame to the base
                    Eigen::Translation<float, 3> trans06(t_ref);
                    Eigen::AngleAxisf a06;
                    a06.fromRotationMatrix(r_ref);
                    Eigen::Isometry3f t06 = trans06 * a06;

                    os << "t_ref" << std::endl << t_ref << std::endl;
                    os << "t06.translation()" << std::endl << t06.translation() << std::endl;
                    os << "t06.linear()" << std::endl << t06.linear() << std::endl;

                    //  Calculate standby position
                    const float standby_distance = 0.1;
                    Eigen::Vector3f standby_pos_vect_cam_frame 
                        = wheel_pos_vect_cam_frame + standby_distance * wheel_norm_vect_cam_frame;
                    Eigen::Vector3f standby_pos_vect_base_frame;     
                    os << "Standby position in Camera Frame: " << std::endl << standby_pos_vect_cam_frame << std::endl;
                    Eigen::Isometry3f t68 = 
                        find_trans_mat_from_new_cam_frame_to_hand_frame(standby_pos_vect_cam_frame, wheel_norm_vect_cam_frame);
                    // os << "t68.translation()" << std::endl << t68.translation() << std::endl;
                    // os << "t68.linear()" << std::endl << t68.linear() << std::endl;
                }
            }
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
            d435RangeCamera->notifyStateChange();

        }
        PrevBButtonState = BbuttonState;

        //  When X button is pushed down at this moment
        //  Console out Euler angles claculated from r_ref
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
            std::ostream& os = io -> os();
            os << "Y button has pushed." << std::endl;
            cnoid::Isometry3 t_base = Base -> position();
            os << "t_base.translation()" << std::endl << t_base.translation() << std::endl;
            os << "t_base.linear()" << std::endl << t_base.linear() << std::endl;
            cnoid::Isometry3 t_camera = CameraBody -> position();
            os << "t_camera.translation()" << std::endl << t_camera.translation() << std::endl;
            os << "t_camera.linear()" << std::endl << t_camera.linear() << std::endl;
        }
        PrevYButtonState = YbuttonState;
        
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

    //  Frame transformation
    // T06 = get_T06();
    // T67 = get_T67();
    //  From camera base frame to the base frame
    // T07 = T06 * T67;
    //  From the base frame to the camera frame
    // T70 = T07.inverse();

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
    a06.fromRotationMatrix(r_ref);
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
    Eigen::Translation<float, 3> trans67(0.01, 0.060, -0.065);
    //  Camera orientation in the hand frame
    Eigen::Matrix3f m67;
    m67 <<  1.0f,  0.0f,  0.0f, 
            0.0f, -1.0f,  0.0f,
            0.0f,  0.0f, -1.0f;
    Eigen::AngleAxisf a67(m67); 
    //  Make a transformation matrix
    Eigen::Isometry3f t67 = trans67 * a67;
    
    return(t67);
}

Eigen::Isometry3f HandD435PoseController::find_trans_mat_from_new_cam_frame_to_hand_frame(
    const Eigen::Vector3f& pos_vect_cam_frame, const Eigen::Vector3f& norm_vect_cam_frame)
{
    //  Find a transformation matrix from the camera frame to the hand
    //  Camera position in the hand frame
    Eigen::Translation<float, 3> trans67(0.01, -0.073, -0.078);
    //  Camera orientation in the hand frame
    Eigen::AngleAxisf a67 = Eigen::AngleAxisf(M_PI, Eigen::Vector3f::UnitZ());
    Eigen::Isometry3f t67 = trans67 * a67;

    //  Find a transformation matrix from the new camera to the current camera
    //  Translation
    Eigen::Translation<float, 3> trans78(pos_vect_cam_frame);
    //  Orientation = x, y, z axis
    //  z8 is given by the normal vector of the red wheel 
    Eigen::Vector3f z8 = norm_vect_cam_frame;
    //  x8 is set to (1, 0, 0) always
    Eigen::Vector3f x8(1.0f, 0.0f, 0.0f);
    //  y8 is found by corss of z8 and x8
    Eigen::Vector3f y8 = z8.cross(x8); 
    //  Rotation
    Eigen::Matrix3f rot78;
    rot78 << x8(0), y8(0), z8(0), 
            x8(1), y8(1), z8(1),
            x8(2), y8(2), z8(2);
    Eigen::AngleAxisf a78;
    a78.fromRotationMatrix(rot78);
    Eigen::Isometry3f t78 = trans78 * a78;

    //  Find a transformation matrix from the new camera to the hand
    Eigen::Isometry3f t68 = t67 * t78;

    return(t68);
}

CNOID_IMPLEMENT_SIMPLE_CONTROLLER_FACTORY(HandD435PoseController)
