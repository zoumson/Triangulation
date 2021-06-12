#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace std;
using namespace cv;

int main ( int argc, char** argv )
{
        String keys =
        "{i image1 |<none>           | input image path}"                 
        "{j image2 |<none>           | input image path}"                 
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
    // always after variable, required variable are checked here
    if (!parser.check()) 
    {
        parser.printErrors();
        return -1;
    }

    //-- Read image
    Mat img_1 = imread ( imagePath1, IMREAD_COLOR );
    Mat img_2 = imread ( imagePath2, IMREAD_COLOR );
        
    //-- Initialization
    std::vector<KeyPoint> keypoints_1, keypoints_2;
    Mat descriptors_1, descriptors_2;
    Ptr<FeatureDetector> detector = ORB::create();
    Ptr<DescriptorExtractor> descriptor = ORB::create();
    // Ptr<FeatureDetector> detector = FeatureDetector::create(detector_name);
    // Ptr<DescriptorExtractor> descriptor = DescriptorExtractor::create(descriptor_name);
    Ptr<DescriptorMatcher> matcher  = DescriptorMatcher::create ( "BruteForce-Hamming" );

    //-- Step 1: Detect the position of the Oriented FAST corner point
    detector->detect ( img_1,keypoints_1 );
    detector->detect ( img_2,keypoints_2 );

    //-- Step 2: Calculate the BRIEF descriptor based on the position of the corner point
    descriptor->compute ( img_1, keypoints_1, descriptors_1 );
    descriptor->compute ( img_2, keypoints_2, descriptors_2 );

    Mat outimg1;
    drawKeypoints( img_1, keypoints_1, outimg1, Scalar::all(-1), DrawMatchesFlags::DEFAULT );
    imshow("Image 1: ORB feature points",outimg1);

    //-- Step 3: Match the BRIEF descriptors in the two images, using Hamming distance
    vector<DMatch> matches;
    //BFMatcher matcher ( NORM_HAMMING );
    matcher->match ( descriptors_1, descriptors_2, matches );

    //-- The fourth step: matching point pair screening
    double min_dist=10000, max_dist=0;

    //Find the minimum and maximum distances between all matches, that is, 
    //the distance between the most similar and least similar two sets of points
    for ( int i = 0; i < descriptors_1.rows; i++ )
    {
        double dist = matches[i].distance;
        if ( dist < min_dist ) min_dist = dist;
        if ( dist > max_dist ) max_dist = dist;
    }
    
    // Another way to find max_dist and min_dist
    min_dist = min_element( matches.begin(), matches.end(), [](const DMatch& m1, const DMatch& m2) {return m1.distance<m2.distance;} )->distance;
    max_dist = max_element( matches.begin(), matches.end(), [](const DMatch& m1, const DMatch& m2) {return m1.distance<m2.distance;} )->distance;

    printf ( "-- Max dist : %f \n", max_dist );
    printf ( "-- Min dist : %f \n", min_dist );

    //When the distance between the descriptors is greater than twice the minimum distance, 
    //it is considered that the matching is wrong. But sometimes the minimum distance will be very small, 
    //and an empirical value of 30 is set as the lower limit.
    std::vector< DMatch > good_matches;
    for ( int i = 0; i < descriptors_1.rows; i++ )
    {
        if ( matches[i].distance <= max ( 2*min_dist, 30.0 ) )
        {
            good_matches.push_back ( matches[i] );
        }
    }

    //-- Step 5: draw matching results
    Mat img_match;
    Mat img_goodmatch;
    drawMatches ( img_1, keypoints_1, img_2, keypoints_2, matches, img_match );
    drawMatches ( img_1, keypoints_1, img_2, keypoints_2, good_matches, img_goodmatch );
    imshow ( "All matching point pairs", img_match );
    imshow ( "Match point pairs after optimization", img_goodmatch );
    waitKey(0);

    return 0;
}
