#include "stitcher_code.cpp"

int main()
{
	Mat img1 = imread("/home/tibi/MarsRover2019/Workspace/src/stitcher/testImages/img1.jpg", 1);
	Mat img2 = imread("/home/tibi/MarsRover2019/Workspace/src/stitcher/testImages/img2.jpg", 1);
	Mat img3 = imread("/home/tibi/MarsRover2019/Workspace/src/stitcher/testImages/img3.jpg", 1);
	Mat img4 = imread("/home/tibi/MarsRover2019/Workspace/src/stitcher/testImages/img4.jpg", 1);

	vector<Mat> imgs;
	imgs.push_back(img1);
	imgs.push_back(img2);
	imgs.push_back(img3);
	imgs.push_back(img4);
	Mat result;
	stitch_images(imgs, false, result);
	imwrite("Result.jpg", result);
	

	/*
	namedWindow( "Display window", CV_WINDOW_AUTOSIZE );// Create a window for display.
	imshow( "Display window", img1); 
	waitKey(0);
	*/
}

