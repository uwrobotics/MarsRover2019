#include "stitcher/src/stitcher_code.cpp"

// test function with local images
int main()
{
	Mat img1 = imread("/home/tibi/MarsRover2019/Workspace/src/stitcher/testImages/pic1.jpg", 1);
	Mat img2 = imread("/home/tibi/MarsRover2019/Workspace/src/stitcher/testImages/pic2.jpg", 1);

	
	vector<Mat> imgs = {img1, img2};
	Mat result;
	stitch_images(imgs, false, result);
	imwrite("Result.jpg", result);
	

	/*
	namedWindow( "Display window", CV_WINDOW_AUTOSIZE );// Create a window for display.
	imshow( "Display window", img1); 
	waitKey(0);
	*/
}
