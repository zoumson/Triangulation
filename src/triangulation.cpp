#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/core/utility.hpp>
// #include "extra.h" // used in opencv2 
using namespace std;
//using namespace cv;

void find_feature_matches (
    const cv::Mat& img_1, const cv::Mat& img_2,
    std::vector<cv::KeyPoint>& keypoints_1,
    std::vector<cv::KeyPoint>& keypoints_2,
    std::vector< cv::DMatch >& matches );

void pose_estimation_2d2d (
    const std::vector<cv::KeyPoint>& keypoints_1,
    const std::vector<cv::KeyPoint>& keypoints_2,
    const std::vector< cv::DMatch >& matches,
    cv::Mat& R, cv::Mat& t );

void triangulation (
    const vector<cv::KeyPoint>& keypoint_1,
    const vector<cv::KeyPoint>& keypoint_2,
    const std::vector< cv::DMatch >& matches,
    const cv::Mat& R, const cv::Mat& t,
    vector<cv::Point3d>& points
);

// Pixel coordinates to camera normalized coordinates
cv::Point2f pixel2cam( const cv::Point2d& p, const cv::Mat& K );

int main ( int argc, char** argv )
{
    cv::String keys =
        "{i image1 |<none>           | input image1 path}"                 
        "{j image2 |<none>          | input image2 path}"                 
        "{help h usage ?    |      | show help message}";      
  
    cv::CommandLineParser parser(argc, argv, keys);
    parser.about("Pose estimation 2D-2D");
    if (parser.has("help")) 
    {
        parser.printMessage();
        return 0;
    }
 
    cv::String imagePath1 = parser.get<cv::String>("image1");
    cv::String imagePath2 = parser.get<cv::String>("image2");
    // always after variable, required variable are checked here
    if (!parser.check()) 
    {
        parser.printErrors();
        return -1;
    }
   
    //-- read image 
    cv::Mat img_1 = cv::imread ( imagePath1, cv::IMREAD_COLOR );
    cv::Mat img_2 = cv::imread ( imagePath2, cv::IMREAD_COLOR );


    vector<cv::KeyPoint> keypoints_1, keypoints_2;
    vector<cv::DMatch> matches;
    find_feature_matches ( img_1, img_2, keypoints_1, keypoints_2, matches );
    cout<<"Total of good matches: "<<matches.size() <<"\n";

    //-- Estimate the motion between two images
    cv::Mat R,t;
 
    pose_estimation_2d2d ( keypoints_1, keypoints_2, matches, R, t );
   

    //-- Triangulate
 
    vector<cv::Point3d> points;
    triangulation( keypoints_1, keypoints_2, matches, R, t, points );
 
    //-- Verify the reprojection relationship between triangulated points and feature points
    cv::Mat K = ( cv::Mat_<double> ( 3,3 ) << 520.9, 0, 325.1, 0, 521.0, 249.7, 0, 0, 1 );
    for ( int i=0; i<matches.size(); i++ )
    {
        cv::Point2d pt1_cam = pixel2cam( keypoints_1[ matches[i].queryIdx ].pt, K );
        cv::Point2d pt1_cam_3d(
            points[i].x/points[i].z, 
            points[i].y/points[i].z 
        );
        
        cout<<"point in the first camera frame: "<<pt1_cam<<endl;
        cout<<"point projected from 3D "<<pt1_cam_3d<<", d="<<points[i].z<<endl;
        
        // Second picture
        cv::Point2f pt2_cam = pixel2cam( keypoints_2[ matches[i].trainIdx ].pt, K );
        cv::Mat pt2_trans = R*( cv::Mat_<double>(3,1) << points[i].x, points[i].y, points[i].z ) + t;
        pt2_trans /= pt2_trans.at<double>(2,0);
        cout<<"point in the second camera frame: "<<pt2_cam<<endl;
        cout<<"point reprojected from second frame: "<<pt2_trans.t()<<endl;
        cout<<endl;
    }
   
   //zouma
    return 0;
}

void find_feature_matches ( const cv::Mat& img_1, const cv::Mat& img_2,
                            std::vector<cv::KeyPoint>& keypoints_1,
                            std::vector<cv::KeyPoint>& keypoints_2,
                            std::vector< cv::DMatch >& matches )
{
    //-- initialization
    cv::Mat descriptors_1, descriptors_2;
    // used in OpenCV 
    cv::Ptr<cv::FeatureDetector> detector = cv::ORB::create();
    cv::Ptr<cv::DescriptorExtractor> descriptor = cv::ORB::create();
    // use this if you are in OpenCV2 
    // cv::Ptr<cv::FeatureDetector> detector = cv::FeatureDetector::create ( "cv::ORB" );
    // cv::Ptr<cv::DescriptorExtractor> descriptor = cv::DescriptorExtractor::create ( "cv::ORB" );
    cv::Ptr<cv::DescriptorMatcher> matcher  = cv::DescriptorMatcher::create("BruteForce-Hamming");
    //-- Step 1: Detect the position of the Oriented FAST corner point
    detector->detect ( img_1,keypoints_1 );
    detector->detect ( img_2,keypoints_2 );

    //-- Step 2: Calculate the BRIEF descriptor based on the position of the corner point
    descriptor->compute ( img_1, keypoints_1, descriptors_1 );
    descriptor->compute ( img_2, keypoints_2, descriptors_2 );

    //-- Step 3: Match the BRIEF descriptors in the two images, using Hamming distance
    vector<cv::DMatch> match;
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

void pose_estimation_2d2d (
    const std::vector<cv::KeyPoint>& keypoints_1,
    const std::vector<cv::KeyPoint>& keypoints_2,
    const std::vector< cv::DMatch >& matches,
    cv::Mat& R, cv::Mat& t )
{
    // Camera internal reference, TUM Freiburg2
    cv::Mat K = ( cv::Mat_<double> ( 3,3 ) << 520.9, 0, 325.1, 0, 521.0, 249.7, 0, 0, 1 );
    //-- Convert the matching point to the form of vector<cv::Point2f>
    vector<cv::Point2f> points1;
    vector<cv::Point2f> points2;

    for ( int i = 0; i < ( int ) matches.size(); i++ )
    {
        points1.push_back ( keypoints_1[matches[i].queryIdx].pt );
        points2.push_back ( keypoints_2[matches[i].trainIdx].pt );
    }

    //-- Calculate the fundamental matrix
    cv::Mat fundamental_matrix;Calculate the fundamental matrix
    fundamental_matrix = cv::findFundamentalMat ( points1, points2, cv::FM_8POINT );
    cout<<"fundamental_matrix is "<<endl<< fundamental_matrix<<endl;
    //-- Calculate the essential matrix
    cv::Point2d principal_point ( 325.1, 249.7 );				//Camera principal point, TUM dataset calibration value
    int focal_length = 521;						//Camera focal length, TUM dataset calibration value
    cv::Mat essential_matrix;
    essential_matrix = cv::findEssentialMat ( points1, points2, focal_length, principal_point );

    cout<<"essential_matrix is "<<endl<< essential_matrix<<endl;

    //-- Calculate the homography matrix
    cv::Mat homography_matrix;
    homography_matrix = cv::findHomography ( points1, points2, cv::RANSAC, 3 );
    cout <<"3\n";
    cout<<"homography_matrix is "<<endl<<homography_matrix<<endl;

    //-- Recover rotation and translation information from the essential matrix.
    recoverPose ( essential_matrix, points1, points2, R, t, focal_length, principal_point );
    cout <<"4\n";
    cout<<"R is "<<endl<<R<<endl;
    cout<<"t is "<<endl<<t<<endl;
}

void triangulation ( 
    const vector< cv::KeyPoint >& keypoint_1, 
    const vector< cv::KeyPoint >& keypoint_2, 
    const std::vector< cv::DMatch >& matches,
    const cv::Mat& R, const cv::Mat& t, 
    vector< cv::Point3d >& points )
{
    cv::Mat T1 = (cv::Mat_<float> (3,4) <<
        1,0,0,0,
        0,1,0,0,
        0,0,1,0);
    cv::Mat T2 = (cv::Mat_<float> (3,4) <<
        R.at<double>(0,0), R.at<double>(0,1), R.at<double>(0,2), t.at<double>(0,0),
        R.at<double>(1,0), R.at<double>(1,1), R.at<double>(1,2), t.at<double>(1,0),
        R.at<double>(2,0), R.at<double>(2,1), R.at<double>(2,2), t.at<double>(2,0)
    );
    
    cv::Mat K = ( cv::Mat_<double> ( 3,3 ) << 520.9, 0, 325.1, 0, 521.0, 249.7, 0, 0, 1 );
    vector<cv::Point2f> pts_1, pts_2;
    for ( cv::DMatch m:matches )
    {
        // Convert pixel coordinates to camera coordinates
        pts_1.push_back ( pixel2cam( keypoint_1[m.queryIdx].pt, K) );
        pts_2.push_back ( pixel2cam( keypoint_2[m.trainIdx].pt, K) );
    }
    
    cv::Mat pts_4d;
    triangulatePoints( T1, T2, pts_1, pts_2, pts_4d );
    
    // Convert to non-homogeneous coordinates
    for ( int i=0; i<pts_4d.cols; i++ )
    {
        cv::Mat x = pts_4d.col(i);
        x /= x.at<float>(3,0); // 归一化
        cv::Point3d p (
            x.at<float>(0,0), 
            x.at<float>(1,0), 
            x.at<float>(2,0) 
        );
        points.push_back( p );
    }
}

cv::Point2f pixel2cam ( const cv::Point2d& p, const cv::Mat& K )
{
    return cv::Point2f
    (
        ( p.x - K.at<double>(0,2) ) / K.at<double>(0,0), 
        ( p.y - K.at<double>(1,2) ) / K.at<double>(1,1) 
    );
}

