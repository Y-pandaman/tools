#include <iostream>
#include <stdio.h>
#include <opencv2/opencv.hpp>
#include <vector>
#include "opencv2/core.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/features2d.hpp"
#include "opencv2/xfeatures2d.hpp"

using namespace std;
using namespace cv;
using namespace cv::xfeatures2d;
using std::cout;
using std::endl;

#define SQRT2 1.41

struct NormalizedData {
	Mat T1;
    Mat T2;
	vector<pair<Point2f, Point2f>> newPtPairs;
};

struct RANSACDiffs
{
    int inliersNum;  // number of inliers
    vector<float> distances; // euclidean distances
    vector<bool> isInliers; // vector of inlier/outlier segmentation results; true: inlier false: outlier
};

struct NormalizedData NormalizeData(vector<pair<Point2f, Point2f>> pts2D)
{
	int ptsNum = pts2D.size();

	// calculate means (they will be the center of coordinate systems)
	float mean1x = 0.0, mean1y = 0.0, meanx_tick = 0.0, meany_tick = 0.0;
	for (int i = 0; i < ptsNum; i++)
    {
		mean1x += pts2D[i].first.x;
        mean1y += pts2D[i].first.y;

		meanx_tick += pts2D[i].second.x;
		meany_tick += pts2D[i].second.y;
	}

	mean1x /= ptsNum;
	mean1y /= ptsNum;

	meanx_tick /= ptsNum;
	meany_tick /= ptsNum;

    // determine the spread for x and y

	float spread1x = 0.0, spread1y = 0.0, spreadx_tick = 0.0, spready_tick = 0.0;

	for (int i = 0; i < ptsNum; i++)
    {
		spread1x += (pts2D[i].first.x - mean1x) * (pts2D[i].first.x - mean1x);
		spread1y += (pts2D[i].first.y - mean1y) * (pts2D[i].first.y - mean1y);

		spreadx_tick += (pts2D[i].second.x - meanx_tick) * (pts2D[i].second.x - meanx_tick);
		spready_tick += (pts2D[i].second.y - meany_tick) * (pts2D[i].second.y - meany_tick);
	}

	spread1x /= ptsNum;
	spread1y /= ptsNum;

	spreadx_tick /= ptsNum;
	spready_tick /= ptsNum;

    // offset (translation) and scale matrices (3x3)

	Mat offs1 = Mat::eye(3, 3, CV_32F);
    Mat offs_tick = Mat::eye(3, 3, CV_32F);
    Mat scale1 = Mat::eye(3, 3, CV_32F);
    Mat scale_tick = Mat::eye(3, 3, CV_32F);

	offs1.at<float>(0, 2) = -mean1x;
	offs1.at<float>(1, 2) = -mean1y;

	offs_tick.at<float>(0, 2) = -meanx_tick;
	offs_tick.at<float>(1, 2) = -meany_tick;

	scale1.at<float>(0, 0) = SQRT2 / sqrt(spread1x);
	scale1.at<float>(1, 1) = SQRT2 / sqrt(spread1y);

	scale_tick.at<float>(0, 0) = SQRT2 / sqrt(spreadx_tick);
	scale_tick.at<float>(1, 1) = SQRT2 / sqrt(spready_tick);

    struct NormalizedData ret;

    // T matrices are scale*translation

	ret.T1 = scale1 * offs1;
	ret.T2 = scale_tick * offs_tick;

    // apply the normalization to each point (point pair)

	for (int i = 0; i < ptsNum; i++)
    {
        pair<Point2f, Point2f> newPair;

		newPair.first.x = SQRT2 * (pts2D[i].first.x - mean1x) / sqrt(spread1x);
		newPair.first.y = SQRT2 * (pts2D[i].first.y - mean1y) / sqrt(spread1y);

		newPair.second.x = SQRT2 * (pts2D[i].second.x - meanx_tick) / sqrt(spreadx_tick);
		newPair.second.y = SQRT2 * (pts2D[i].second.y - meany_tick) / sqrt(spready_tick);

		ret.newPtPairs.push_back(newPair);
	}

	return ret;
}

Mat calcHomography(vector<pair<Point2f, Point2f>> pointPairs)
{
    const int ptsNum = pointPairs.size();
    Mat A(2 * ptsNum, 9, CV_32F);

    for (int i = 0; i < ptsNum; i++)
    {
        float u1 = pointPairs[i].first.x;
        float v1 = pointPairs[i].first.y;

        float u2 = pointPairs[i].second.x;
        float v2 = pointPairs[i].second.y;

        A.at<float>(2 * i, 0) = u1;
        A.at<float>(2 * i, 1) = v1;
        A.at<float>(2 * i, 2) = 1.0f;
        A.at<float>(2 * i, 3) = 0.0f;
        A.at<float>(2 * i, 4) = 0.0f;
        A.at<float>(2 * i, 5) = 0.0f;
        A.at<float>(2 * i, 6) = -u2 * u1;
        A.at<float>(2 * i, 7) = -u2 * v1;
        A.at<float>(2 * i, 8) = -u2;

        A.at<float>(2 * i + 1, 0) = 0.0f;
        A.at<float>(2 * i + 1, 1) = 0.0f;
        A.at<float>(2 * i + 1, 2) = 0.0f;
        A.at<float>(2 * i + 1, 3) = u1;
        A.at<float>(2 * i + 1, 4) = v1;
        A.at<float>(2 * i + 1, 5) = 1.0f;
        A.at<float>(2 * i + 1, 6) = -v2 * u1;
        A.at<float>(2 * i + 1, 7) = -v2 * v1;
        A.at<float>(2 * i + 1, 8) = -v2;
    }

    // linear estimation of planar homography (homogoeneous linear system of equations)
    // optimal solution in the least squares sense is the eigenvalue of A^T A corresponding to the smallest eigenvalue

    Mat eVecs(9, 9, CV_32F), eVals(9, 9, CV_32F);
    // cout << "A\n" << A << endl;
    eigen(A.t() * A, eVals, eVecs);

    Mat H(3, 3, CV_32F);
    for (int i = 0; i < 9; i++)
    {
        H.at<float>(i / 3, i % 3) = eVecs.at<float>(8, i);
    }

    // Normalize:
    H = H * (1.0 / H.at<float>(2, 2));

    return H;
}

// Tranformation of images

void transformImage(Mat origImg, Mat& newImage, Mat tr, bool isPerspective)
{
    Mat invTr = tr.inv();
    const int WIDTH = origImg.cols;
    const int HEIGHT = origImg.rows;

    const int newWIDTH = newImage.cols;
    const int newHEIGHT = newImage.rows;

    for (int x = 0; x < newWIDTH; x++)
    {
        for (int y = 0; y < newHEIGHT; y++)
        {
            Mat pt(3, 1, CV_32F);
            pt.at<float>(0, 0) = x;
            pt.at<float>(1, 0) = y;
            pt.at<float>(2, 0) = 1.0;

            Mat ptTransformed = invTr * pt;
            if (isPerspective)
            {
                ptTransformed = (1.0 / ptTransformed.at<float>(2, 0)) * ptTransformed;
            }

            int newX = round(ptTransformed.at<float>(0, 0));
            int newY = round(ptTransformed.at<float>(1, 0));

            if ((newX >= 0) && (newX < WIDTH) && (newY >= 0) && (newY < HEIGHT))
            {
                newImage.at<Vec3b>(y, x) = origImg.at<Vec3b>(newY, newX);
            }
        }
    }
}

RANSACDiffs Distance(vector<pair<Point2f, Point2f>> goodMatches, Mat H, float threshold)
{
    int num = goodMatches.size();

    RANSACDiffs ret;

    vector<bool> isInliers;
    vector<float> distances;

    int inlierCounter = 0;
    for (int idx = 0; idx < num; idx++)
    {
        Point2f u = goodMatches[idx].first;
        // cout << "u\n" << u << endl;

        Mat u_tick(3, 1, CV_32F);
        u_tick.at<float>(0, 0) = goodMatches[idx].second.x;
        u_tick.at<float>(1, 0) = goodMatches[idx].second.y;
        u_tick.at<float>(2, 0) = 1.0;

        Mat ptTransformed = H.inv() * u_tick; // apply the homography
        ptTransformed = (1.0 / ptTransformed.at<float>(2, 0)) * ptTransformed;

        float u_tick_x = ptTransformed.at<float>(0, 0);
        float u_tick_y = ptTransformed.at<float>(1, 0);

        Point2f u_tick_p(u_tick_x, u_tick_y); // convert from Mat to Point2f
        // cout << "u_tick\n" << u_tick_p << endl;

        // find the Eucliedean distance between the original point u and the point determined using homography and u_tick
        double diff = cv::norm(u - u_tick_p);
        // cout << "diff\n" << diff << endl;

        // determine if point is inlier or outlier given a threshold value
        distances.push_back(diff);
        if (diff < threshold)
        {
            isInliers.push_back(true);
            inlierCounter++;
        }
        else
        {
            isInliers.push_back(false);
        }

    }

    ret.distances = distances;
    ret.isInliers = isInliers;
    ret.inliersNum = inlierCounter;

    return ret;
}

// robust method for estimating homography matrix H
Mat EstimateHRANSAC(vector<pair<Point2f, Point2f>> goodMatches, float threshold, int iterateNum)
{
    int num = goodMatches.size();

    int bestSampleInlierNum = 0;
    Mat bestH;

    for (int iter = 0; iter < iterateNum; iter++)
    {
        // four points needed for planar homography
        float rand1 = (float)(rand()) / RAND_MAX;
        float rand2 = (float)(rand()) / RAND_MAX;
        float rand3 = (float)(rand()) / RAND_MAX;
        float rand4 = (float)(rand()) / RAND_MAX;

        int index1 = (int)(rand1 * num);
        int index2 = (int)(rand2 * num);

        while (index2 == index1)
        {
            rand2 = (float)(rand()) / RAND_MAX; index2 = (int)(rand2 * num);
        }

        int index3 = (int)(rand3 * num);
        while ((index3 == index1) || (index3 == index2))
        {
            rand3 = (float)(rand()) / RAND_MAX; index3 = (int)(rand3 * num);
        }
        int index4 = (int)(rand4 * num);
        while ((index4 == index1) || (index4 == index2) || (index4 == index3))
        {
            rand4 = (float)(rand()) / RAND_MAX; index4 = (int)(rand4 * num);
        }

        pair<Point2f, Point2f> pt1 = goodMatches.at(index1);
        pair<Point2f, Point2f> pt2 = goodMatches.at(index2);
        pair<Point2f, Point2f> pt3 = goodMatches.at(index3);
        pair<Point2f, Point2f> pt4 = goodMatches.at(index4);

        // In each RANSAC cycle, a minimal sample with 4 point pairs are formed
        vector<pair<Point2f, Point2f>> minimalSample;

        minimalSample.push_back(pt1);
        minimalSample.push_back(pt2);
        minimalSample.push_back(pt3);
        minimalSample.push_back(pt4);

        // Data normalization
        NormalizedData norm = NormalizeData(minimalSample);
        vector<pair<Point2f, Point2f> > normPointPairs = norm.newPtPairs;
        Mat T1 = norm.T1;
        Mat T2 = norm.T2;

        cout << "T1\n" << T1 << endl;
        cout << "T2\n" << T2 << endl;

        Mat H_tick = calcHomography(normPointPairs);

        Mat HT1 = H_tick * T1;
        Mat H = T2.inv() * HT1;

        // Compute consensus set
        RANSACDiffs sampleResult = Distance(goodMatches, H, threshold);

        // Check if the new test is larger than the best one

        if (sampleResult.inliersNum > bestSampleInlierNum)
        {
            bestSampleInlierNum = sampleResult.inliersNum;
            bestH = H;
            // cout << "best H:\n" << H << endl;
        }
    }

    // Finally, inliers are found with best H
    RANSACDiffs bestResult = Distance(goodMatches, bestH, threshold);

    vector<pair<Point2f, Point2f>> inlierPts;

    for (int idx = 0; idx < num; idx++)
    {
        if (bestResult.isInliers.at(idx))
        {
            inlierPts.push_back(goodMatches.at(idx));
        }
    }

    // Final H is found with inlier point pairs
    NormalizedData norm_final = NormalizeData(inlierPts);
    vector<pair<Point2f, Point2f> > normPointPairs_final = norm_final.newPtPairs;
    Mat T1_final = norm_final.T1;
    Mat T2_final = norm_final.T2;

    Mat H_tick_final = calcHomography(normPointPairs_final);

    Mat HT1_final = H_tick_final * T1_final;
    Mat H_final = T2_final.inv() * HT1_final;

    return H_final;
}

int main(int argc, char* argv[])
{
    for (int i = 1; i < 4; i++)
    {
        Mat img1 = imread("/home/bdca/workspace/stitch/0000" + std::to_string(i) + ".jpg", IMREAD_COLOR);
        Mat img2 = imread("/home/bdca/workspace/stitch/1000" + std::to_string(i+1) + ".jpg", IMREAD_COLOR);

        if (img1.empty() || img2.empty())
        {
            cout << "Could not open or find the image!\n" << endl;
            return -1;
        }

        Mat img1_gray, img2_gray;
        cvtColor(img1, img1_gray, COLOR_BGR2GRAY);
        cvtColor(img2, img2_gray, COLOR_BGR2GRAY);

        // Detect the keypoints using SURF Detector, compute the descriptors
        int minHessian = 400;
        Ptr<SURF> detector = SURF::create(minHessian);
        std::vector<KeyPoint> keypoints1, keypoints2;
        Mat descriptors1, descriptors2;
        detector->detectAndCompute(img1_gray, noArray(), keypoints1, descriptors1);
        detector->detectAndCompute(img2_gray, noArray(), keypoints2, descriptors2);

        // Matching descriptor vectors with a FLANN based matcher
        // FLANN stands for Fast Library for Approximate Nearest Neighbors
        // Since SURF is a floating-point descriptor NORM_L2 is used
        Ptr<DescriptorMatcher> matcher = DescriptorMatcher::create(DescriptorMatcher::FLANNBASED);
        std::vector< std::vector<DMatch> > knn_matches;
        matcher->knnMatch(descriptors1, descriptors2, knn_matches, 2);

        // Filter matches using the Lowe's ratio test
        // rejects poor matches by computing the ratio between the best and second-best match
        // If the ratio is below some threshold, the match is discarded as being low-quality
        const float ratio_thresh = 0.5f;
        std::vector<DMatch> good_matches;
        for (size_t i = 0; i < knn_matches.size(); i++)
        {
            if (knn_matches[i][0].distance < ratio_thresh * knn_matches[i][1].distance)
            {
                good_matches.push_back(knn_matches[i][0]);
            }
        }
        vector<pair<Point2f, Point2f> > pointPairs;

        for (size_t i = 0; i < good_matches.size(); i++)
        {
            pair<Point2f, Point2f> currPts;
            currPts.first = keypoints2[good_matches[i].trainIdx].pt;
            currPts.second = keypoints1[good_matches[i].queryIdx].pt;
            pointPairs.push_back(currPts);
        }

        Mat H = EstimateHRANSAC(pointPairs, 0.2, 500);

        Mat transformedImage = Mat::zeros(1.5 * img1.size().height, 2.0 * img1.size().width, img1.type());
        // First, add img1 to transformedImage without changing it
        transformImage(img1, transformedImage, Mat::eye(3, 3, CV_32F), true);
        // Then, add img2 and apply homography
        transformImage(img2, transformedImage, H, true);

        // imwrite("stitchedImage_"+std::to_string(i)+".png", transformedImage);
        imwrite("demo_stitchedImage_" + std::to_string(i) + ".png", transformedImage);
    }

    // namedWindow("Display window", WINDOW_AUTOSIZE); // Create a window for display.
    // imshow("Display window", transformedImage); // Show our image inside it.
    // waitKey(0); // Wait for a keystroke in the window

    // return 0;
}

