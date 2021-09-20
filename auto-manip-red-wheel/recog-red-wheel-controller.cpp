/*
    recog-red-wheel-controller.cpp
        Simple controller for recognition of red wheel handle from d435 range camera
            - When A button is pushed,
                - Save a current scene as a PCD file 
                - Console out 
                    - A position vector of the center of the red wheel handle in the camera frame
                    - A normal vector of the red wheel handle circle in the camera frame
        Author: Keitaro Naruse
        Date:   2021-09-18
*/
//  For file save in c++ language
#include <iostream>
#include <fstream>
#include <vector>
#include <string>
//  Eigen
#include <Eigen/Core>
#include <Eigen/Dense>
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
    class RecogRedWheelController {}
        Simple controller for d435.body
            Save an image file and a PCD file 
            when A-button of joystick is pressed down
*/
class RecogRedWheelController : public cnoid::SimpleController
{
private:
    //  Class instance 
    cnoid::SimpleControllerIO* io;
    cnoid::Joystick joystick;
    cnoid::RangeCamera* d435RangeCamera;
    cnoid::Link* rootLink;
    bool PrevAButtonState;
    bool PrevBButtonState;

    //  Point cloud instances
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcPtrCnoidFrame;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcPtrD435Frame;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcPtrTargetD435Frame;
    
    //  Estimated circle parameters
    Eigen::Vector3f circle_center_vector;
    Eigen::Vector3f circle_norm_vector;
    float circle_radius;

    //  Make a colored point cloud of a current scene in the choreonoid frame
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
    }

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

    /*
        rgb2hsv()
            Convert a color model from rgb to hsv
    */
    void rgb2hsv(float r, float g, float b, float &h, float &s, float &v)
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
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr segment_and_find_target(pcl::PointCloud<pcl::PointXYZRGB>::Ptr scene_pc_ptr)
    {
        //  Apply pass through (range) filter for retrieving ROI(region of interest)
        //  We assume that the depth is in the z-direction and set a range from 0.1 to 0.5
        pcl::IndicesPtr pass_indices_ptr(new std::vector<int>);
        pcl::PassThrough<pcl::PointXYZRGB> pass;
        pass.setInputCloud(scene_pc_ptr);
        pass.setFilterFieldName("z");
        pass.setFilterLimits (0.1, 0.5);
        pass.filter(*pass_indices_ptr);

        //  Segmentation by RegionGrowingRGB method
        //  Segmentation object of RegtionGrowingRGB
        pcl::RegionGrowingRGB<pcl::PointXYZRGB> reg;
        //  Search object for RegtionGrowingRGB
        pcl::search::Search<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>);
        reg.setInputCloud(scene_pc_ptr);
        reg.setIndices(pass_indices_ptr);
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
            //  Sum up the componets of (r, g, b)
            for(auto& point: *pc_ptr_cluster)  {
                rgb2hsv((float)point.r/255.0f, (float)point.g/255.0f, (float)point.b/255.0f, h, s, v);
                ave_h += h;
                ave_s += s;
                ave_v += v;
            }
            // std::cout << ave_h << std::endl;
            ave_h /= (float)pc_ptr_cluster -> size();
            ave_s /= (float)pc_ptr_cluster -> size();
            ave_v /= (float)pc_ptr_cluster -> size();

            //  Calcute the r-ratio and put it to r-ratio vector
            h_list.push_back(ave_h);
            s_list.push_back(ave_s);
            v_list.push_back(ave_v);
        }

        //  Return an index of the most red cluster
        std::size_t target_index = 0;
        for(std::size_t i = 0; i != h_list.size(); ++i)   {
            if( ( (0.0f <= h_list[i]) && (h_list[i] < 0.08f) && (0.2f < s_list[i]) && (0.1f < v_list[i]) ) 
                || ( (0.92f <= h_list[i]) && (h_list[i] <= 1.0f)) && (0.2f < s_list[i]) && (0.1f < v_list[i]) )  {
                target_index = i;
            }
        }

        return(pc_ptr_clusters[target_index]);
    }

    /*
        ransac_circle()
            https://pcl.readthedocs.io/projects/tutorials/en/latest/random_sample_consensus.html
    */
    void ransac_circle(pcl::PointCloud<pcl::PointXYZRGB>::Ptr target_pc_ptr, 
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
    
public:
    virtual bool initialize(cnoid::SimpleControllerIO* io) override
    {
        this->io = io;
        cnoid::Body* body = io->body();

        PrevAButtonState = false;
        PrevBButtonState = false;
        
        // Turn on and Enable IO of D435 for both Camera and RangeCamera
        d435RangeCamera = body -> findDevice<cnoid::RangeCamera>("D435");
        io->enableInput(d435RangeCamera);

        //  Link
        rootLink = io -> body() -> rootLink();
        rootLink -> setActuationMode(cnoid::Link::LinkPosition);
        io -> enableIO(rootLink);

        return true;
    }

    virtual bool control() override
    {
        joystick.readCurrentState();
        bool stateChanged = false;
        bool AbuttonState = joystick.getButtonState(cnoid::Joystick::A_BUTTON);
        bool BbuttonState = joystick.getButtonState(cnoid::Joystick::B_BUTTON);
        if(AbuttonState && !PrevAButtonState){
            //  Set os to choreonoid message window
            std::ostream& os = io->os();

            //  Make a cuurent scene as colored point cloud
            pcPtrCnoidFrame = makeColoredPointCloudOfCurrentSceneInCnoidFrame();
            pcPtrD435Frame = convertFromCnoidFrameToD435Frame(pcPtrCnoidFrame);
            //  Save it as PCD file in binary and compressed format
            // pcl::PointCloud<pcl::PointXYZRGB>::Ptr scene_pc_ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
            // pcl::io::loadPCDFile("test-d435-frame.pcd", *scene_pc_ptr);
            // os << "Loaded: test-d435-frame.pcd" << std::endl;

            //  Segmentation and find a target cluster of the largest r-ratio
            pcPtrTargetD435Frame = segment_and_find_target(pcPtrD435Frame);
            //  Save it as a PCD file
            pcl::io::savePCDFileBinaryCompressed(std::string("debug.target.pcd"), *pcPtrTargetD435Frame);
            os << "Saved: debug.target.pcd" << std::endl;

            //  Pose estimation of circle
            ransac_circle(pcPtrTargetD435Frame, circle_center_vector, circle_radius, circle_norm_vector);

            //  Disply the results of the pose estimation
            os << "Red handle center position vector: " << std::endl << circle_center_vector << std::endl;
            os << "Red handle normal vector: " << std::endl << circle_norm_vector << std::endl;
            os << "Red handle radius: " << std::endl << circle_radius << std::endl;

            //  Find a new camera position
            //  Position p_graspint_point
            const float distance_standby = 0.1f;
            Eigen::Vector3f p_standby = circle_center_vector - distance_standby * circle_norm_vector;
            
            //  Orientation = x, y, z axis
            //  z8 is given by the normal vector of the red wheel 
            Eigen::Vector3f z8 = circle_norm_vector;
            //  x8 is set to (1, 0, 0) always
            Eigen::Vector3f x8(1.0f, 0.0f, 0.0f);
            //  y8 is found by corss of z8 and x8
            Eigen::Vector3f y8 = z8.cross(x8); 
            
            Eigen::Translation<float, 3> trans78(p_standby);
            Eigen::Matrix3f rot78;
            rot78 << x8(0), y8(0), z8(0), 
                x8(1), y8(1), z8(1),
                x8(2), y8(2), z8(2);
            Eigen::AngleAxisf a78;
            a78.fromRotationMatrix(rot78);
            Eigen::Isometry3f t78 = trans78 * a78;

            os << "t78.translation(): " << std::endl << t78.translation() << std::endl;
            os << "t78.linear(): " << std::endl << t78.linear() << std::endl;

            rootLink -> setTranslation(p_standby);
            rootLink -> setRotation(a78);
        }
        PrevAButtonState = AbuttonState;

        if(BbuttonState && !PrevBButtonState){
            //  Set os to choreonoid message window
            std::ostream& os = io->os();
            os << "T.translation(): " << std::endl << rootLink -> translation() << std::endl;
            os << "T.rotation(): " << std::endl << rootLink -> rotation() << std::endl;
        }
        PrevBButtonState = BbuttonState;

        return true;
    }
};

CNOID_IMPLEMENT_SIMPLE_CONTROLLER_FACTORY(RecogRedWheelController)
