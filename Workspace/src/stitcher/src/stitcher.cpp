#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/stitching.hpp"

#include <iostream>

using namespace std;
using namespace cv;

Stitcher::Mode mode = Stitcher::PANORAMA;

void stitch_images(vector<Mat> imgs, bool try_use_gpu, Mat &pano)
{
    // check for valid input

    // empty vector
    if (imgs.empty())
    {
        cout << "Empty vector";
        return;
    }

    else
    {
        // loop through each element check image validity
        for (int i = 0; i < imgs.size(); i++)
        {
            if (imgs[i].empty())
            {
                cout << "Can't read image: " << i << "'\n";
                return;
            }
        }
    }

    // stitching done by class
    Mat pano_temp;
    Ptr<Stitcher> stitcher = Stitcher::create(mode, try_use_gpu);
    Stitcher::Status status = stitcher->stitch(imgs, pano_temp);

    // investigate status

    // if there was a problem, exit
    if (status != Stitcher::OK)
    {
        switch(status)
        {
            case 1: cout << "Not enough images";
            case 2: cout << "Homography estimate fail";
            case 3: cout << "Camera param adjust fail";
        }
        
        return;
    }

    // if stitching was successful, assign to pano
    pano = pano_temp;
}