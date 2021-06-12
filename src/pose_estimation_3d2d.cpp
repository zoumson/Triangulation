#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <g2o/core/base_vertex.h>
#include <g2o/core/base_unary_edge.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/solvers/csparse/linear_solver_csparse.h>
#include <g2o/types/sba/types_six_dof_expmap.h>
#include <chrono>

using namespace std;
using namespace cv;

void find_feature_matches (
    const Mat& img_1, const Mat& img_2,
    std::vector<KeyPoint>& keypoints_1,
    std::vector<KeyPoint>& keypoints_2,
    std::vector< DMatch >& matches );

// Pixel coordinates to camera normalized coordinates
Point2d pixel2cam ( const Point2d& p, const Mat& K );

void bundleAdjustment (
    const vector<Point3f> points_3d,
    const vector<Point2f> points_2d,
    const Mat& K,
    Mat& R, Mat& t
);

int main ( int argc, char** argv )
{
         String keys =
        "{i image1 |<none>           | input image1 path}"                 
        "{j image2 |<none>           | input image2 path}"                
        "{d depth |<none>           | input depth path}"                 
        //"{e depth2 |<none>           | input depth2 path}"                 
        "{help h usage ?    |      | show help message}";      
  
    CommandLineParser parser(argc, argv, keys);
    parser.about("Pose estimation 2D-2D");
    if (parser.has("help")) 
    {
        parser.printMessage();
        return 0;
    }
 
    String imagePath1 = parser.get<String>("image1");
    String imagePath2 = parser.get<String>("image2");
    String imageDepth = parser.get<String>("depth");
    // always after variable, required variable are checked here
    if (!parser.check()) 
    {
        parser.printErrors();
        return -1;
    }
  
    //-- Read image
    Mat img_1 = imread ( imagePath1, IMREAD_COLOR );
    Mat img_2 = imread ( imagePath2, IMREAD_COLOR );
   
    vector<KeyPoint> keypoints_1, keypoints_2;
    vector<DMatch> matches;
    find_feature_matches ( img_1, img_2, keypoints_1, keypoints_2, matches );
    cout<<"Total of good matches: "<<matches.size() <<"\n";

    // Create 3D points
    // The depth image is a 16-bit unsigned number, single-channel image
    Mat d1 = imread ( imageDepth, IMREAD_UNCHANGED );       
 
    Mat K = ( Mat_<double> ( 3,3 ) << 520.9, 0, 325.1, 0, 521.0, 249.7, 0, 0, 1 );
    vector<Point3f> pts_3d;
    vector<Point2f> pts_2d;
    for ( DMatch m:matches )
    {
        ushort d = d1.ptr<unsigned short> (int ( keypoints_1[m.queryIdx].pt.y )) [ int ( keypoints_1[m.queryIdx].pt.x ) ];
        if ( d == 0 )   // bad depth
            continue;
        float dd = d/5000.0;
        Point2d p1 = pixel2cam ( keypoints_1[m.queryIdx].pt, K );
        pts_3d.push_back ( Point3f ( p1.x*dd, p1.y*dd, dd ) );
        pts_2d.push_back ( keypoints_2[m.trainIdx].pt );
    }

    cout<<"3d-2d pairs: "<<pts_3d.size() <<endl;

    Mat r, t;
    // Call OpenCV's PnP solution, you can choose EPNP, DLS and other methods
    solvePnP ( pts_3d, pts_2d, K, Mat(), r, t, false ); 
    Mat R;
    // r is in the form of a rotation vector, converted to a matrix using Rodrigues formula
    Rodrigues ( r, R ); 

    cout<<"R="<<endl<<R<<endl;
    cout<<"t="<<endl<<t<<endl;

    cout<<"calling bundle adjustment"<<endl;

    bundleAdjustment ( pts_3d, pts_2d, K, R, t );
    return 0; 
}

void find_feature_matches ( const Mat& img_1, const Mat& img_2,
                            std::vector<KeyPoint>& keypoints_1,
                            std::vector<KeyPoint>& keypoints_2,
                            std::vector< DMatch >& matches )
{
    //-- initialization
    Mat descriptors_1, descriptors_2;
    // used in OpenCV4
    Ptr<FeatureDetector> detector = ORB::create();
    Ptr<DescriptorExtractor> descriptor = ORB::create();
    // use this if you are in OpenCV2
    // Ptr<FeatureDetector> detector = FeatureDetector::create ( "ORB" );
    // Ptr<DescriptorExtractor> descriptor = DescriptorExtractor::create ( "ORB" );
    Ptr<DescriptorMatcher> matcher  = DescriptorMatcher::create ( "BruteForce-Hamming" );
    //-- Step 1: Detect the position of the Oriented FAST corner point
    detector->detect ( img_1,keypoints_1 );
    detector->detect ( img_2,keypoints_2 );

    //-- Step 2: Calculate the BRIEF descriptor based on the position of the corner point
    descriptor->compute ( img_1, keypoints_1, descriptors_1 );
    descriptor->compute ( img_2, keypoints_2, descriptors_2 );

    //-- Step 3: Match the BRIEF descriptors in the two images, using Hamming distance
    vector<DMatch> match;
    // BFMatcher matcher ( NORM_HAMMING );
    matcher->match ( descriptors_1, descriptors_2, match );

    //-- The fourth step: matching point pair screening
    double min_dist=10000, max_dist=0;

    //Find the minimum and maximum distances between all matches, 
    //that is, the distance between the most similar and least similar two sets of points
    for ( int i = 0; i < descriptors_1.rows; i++ )
    {
        double dist = match[i].distance;
        if ( dist < min_dist ) min_dist = dist;
        if ( dist > max_dist ) max_dist = dist;
    }

    printf ( "-- Max dist : %f \n", max_dist );
    printf ( "-- Min dist : %f \n", min_dist );

    //When the distance between the descriptors is greater than twice the minimum distance, 
    //it is considered that the matching is wrong. 
    //But sometimes the minimum distance will be very small, 
    //and an empirical value of 30 is set as the lower limit.
    for ( int i = 0; i < descriptors_1.rows; i++ )
    {
        if ( match[i].distance <= max ( 2*min_dist, 30.0 ) )
        {
            matches.push_back ( match[i] );
        }
    }
}

Point2d pixel2cam ( const Point2d& p, const Mat& K )
{
    return Point2d
           (
               ( p.x - K.at<double> ( 0,2 ) ) / K.at<double> ( 0,0 ),
               ( p.y - K.at<double> ( 1,2 ) ) / K.at<double> ( 1,1 )
           );
}

void bundleAdjustment (
    const vector< Point3f > points_3d,
    const vector< Point2f > points_2d,
    const Mat& K,
    Mat& R, Mat& t )
{
    // Initialize g2o
    // Build graph optimization, first set g2o
    // The dimension of the optimized variable for each error term is 3, and the dimension of the error value is 1
    // pose dimension is 6, landmark dimension is 3
    typedef g2o::BlockSolver< g2o::BlockSolverTraits<6,3> > Block;  

    // Linear equation solver
    std::unique_ptr<Block::LinearSolverType> linearSolver 
				(new g2o::LinearSolverCSparse<Block::PoseMatrixType>());

    // Matrix block solver
    std::unique_ptr<Block> solver_ptr (new Block(std::move(linearSolver)));
	
    // Gradient descent method, choose from GN, LM, DogLeg
    g2o::OptimizationAlgorithmLevenberg * solver = new g2o::OptimizationAlgorithmLevenberg(std::move(solver_ptr));
    
    
    g2o::SparseOptimizer optimizer;
    optimizer.setAlgorithm ( solver );

    // vertex
    g2o::VertexSE3Expmap* pose = new g2o::VertexSE3Expmap(); // camera pose
    Eigen::Matrix3d R_mat;
    R_mat <<
          R.at<double> ( 0,0 ), R.at<double> ( 0,1 ), R.at<double> ( 0,2 ),
               R.at<double> ( 1,0 ), R.at<double> ( 1,1 ), R.at<double> ( 1,2 ),
               R.at<double> ( 2,0 ), R.at<double> ( 2,1 ), R.at<double> ( 2,2 );
    pose->setId ( 0 );
    pose->setEstimate ( g2o::SE3Quat (
                            R_mat,
                            Eigen::Vector3d ( t.at<double> ( 0,0 ), t.at<double> ( 1,0 ), t.at<double> ( 2,0 ) )
                        ) );
    optimizer.addVertex ( pose );

    int index = 1;
    for ( const Point3f p:points_3d )   // landmarks
    {
        g2o::VertexPointXYZ* point = new g2o::VertexPointXYZ();
        point->setId ( index++ );
        point->setEstimate ( Eigen::Vector3d ( p.x, p.y, p.z ) );
        point->setMarginalized ( true ); // g2o Must be set in marg, see the tenth lecture content
        optimizer.addVertex ( point );
    }

    // parameter: camera intrinsics
    g2o::CameraParameters* camera = new g2o::CameraParameters (
        K.at<double> ( 0,0 ), Eigen::Vector2d ( K.at<double> ( 0,2 ), K.at<double> ( 1,2 ) ), 0
    );
    camera->setId ( 0 );
    optimizer.addParameter ( camera );

    // edges
    index = 1;
    for ( const Point2f p:points_2d )
    {
        g2o::EdgeProjectXYZ2UV* edge = new g2o::EdgeProjectXYZ2UV();
        edge->setId ( index );
        edge->setVertex ( 0, dynamic_cast<g2o::VertexPointXYZ*> ( optimizer.vertex ( index ) ) );
        edge->setVertex ( 1, pose );
        edge->setMeasurement ( Eigen::Vector2d ( p.x, p.y ) );
        edge->setParameterId ( 0,0 );
        edge->setInformation ( Eigen::Matrix2d::Identity() );
        optimizer.addEdge ( edge );
        index++;
    }

    chrono::steady_clock::time_point t1 = chrono::steady_clock::now();
    optimizer.setVerbose ( true );
    optimizer.initializeOptimization();
    optimizer.optimize ( 100 );
    chrono::steady_clock::time_point t2 = chrono::steady_clock::now();
    chrono::duration<double> time_used = chrono::duration_cast<chrono::duration<double>> ( t2-t1 );
    cout<<"optimization costs time: "<<time_used.count() <<" seconds."<<endl;

    cout<<endl<<"after optimization:"<<endl;
    cout<<"T="<<endl<<Eigen::Isometry3d ( pose->estimate() ).matrix() <<endl;
}
