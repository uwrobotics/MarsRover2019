#include "cameraviewwidget.hpp"
#include "sensor_msgs/Image.h"
#include "ui_cameraviewwidget.h"
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>

CameraViewWidget::CameraViewWidget(QWidget *parent)
    : QWidget(parent), ui(new Ui::CameraViewWidget) {
  ui->setupUi(this);
}

CameraViewWidget::~CameraViewWidget() { delete ui; }

void CameraViewWidget::subscribe(ros::NodeHandle &guiHandle,
                                 std::string cameraTopic, bool bDepthImg, bool bCostmap) {
  sub = guiHandle.subscribe(cameraTopic, 1, &CameraViewWidget::imageCallback,
                            this);
  mbDepthImg = bDepthImg;
  mbCostmap = bCostmap;
}

static void depthToCV8UC1(const cv::Mat &float_img, cv::Mat &mono8_img, double scale) {
  // Process images
  if (mono8_img.rows != float_img.rows || mono8_img.cols != float_img.cols) {
    mono8_img = cv::Mat(float_img.size(), CV_8UC1);
  }
  cv::convertScaleAbs(float_img, mono8_img, scale, 0.0);
  // The following doesn't work due to NaNs
  // double minVal, maxVal;
  // minMaxLoc(float_img, &minVal, &maxVal);
  // ROS_DEBUG("Minimum/Maximum Depth in current image: %f/%f", minVal, maxVal);
  // mono8_img = cv::Scalar(0);
  // cv::line( mono8_img, cv::Point2i(10,10),cv::Point2i(200,100),
  // cv::Scalar(255), 3, 8);
}

void CameraViewWidget::imageCallback(const sensor_msgs::Image::ConstPtr &msg) {
  QImage image;

  if (!mbDepthImg && !mbCostmap) {
    QImage::Format format = QImage::Format_RGB888;
    if (msg->encoding == sensor_msgs::image_encodings::BGRA8)
    {
      format = QImage::Format_ARGB32;
    }
    // ROS_INFO("%s", msg->encoding.c_str());
    QImage temp(&(msg->data[0]), msg->width, msg->height,
                format);
    image = temp; //.scaled(ui->label->width(), ui->label->height(),
                  //Qt::AspectRatioMode::IgnoreAspectRatio);
    ui->label->setScaledContents(true);
    ui->label->setPixmap(
        QPixmap::fromImage(image).scaledToHeight(ui->label->height() - 4));
  } else if (mbCostmap) {
    cv_bridge::CvImagePtr cv_ptr;
    ROS_INFO("received costmap");
    // Convert from the ROS image message to a CvImage suitable for working with
    // OpenCV for processing
    try {
      // Always copy, returning a mutable CvImage
      // OpenCV expects color images to use BGR channel order.
      cv_ptr = cv_bridge::toCvCopy(msg, msg->encoding);
    } catch (cv_bridge::Exception &e) {
      // if there is an error during conversion, display it
      ROS_ERROR("tutorialROSOpenCV::main.cpp::cv_bridge exception: %s",
                e.what());
      return;
    }
    // ROS_INFO("%f", cv_ptr->image.at<float>(0, 0));
    // Copy the image.data to imageBuf.
    cv::Mat float_img = cv_ptr->image;
    ROS_INFO("image: w %d, h %d, c %d", float_img.size().width, float_img.size().height, float_img.channels());
    cv::Mat split[3];
    cv::split(float_img, split);
    cv::Mat mono8_img;
    depthToCV8UC1(split[2], mono8_img, 255);
    cv::Mat rgb_img;
    cv::Mat in[] = {mono8_img, mono8_img, mono8_img};
    cv::merge(in, 3, rgb_img);
    cv::flip(rgb_img, rgb_img, 0);
    QImage temp = QImage((uchar *)rgb_img.data, rgb_img.cols, rgb_img.rows,
                         rgb_img.step, QImage::Format_RGB888);
    image = temp;
    ui->label->setScaledContents(true);
    ui->label->setPixmap(
        QPixmap::fromImage(image).scaledToHeight(ui->label->height() - 4));
  } else {
    cv_bridge::CvImagePtr cv_ptr;
    // Convert from the ROS image message to a CvImage suitable for working with
    // OpenCV for processing
    try {
      // Always copy, returning a mutable CvImage
      // OpenCV expects color images to use BGR channel order.
      cv_ptr = cv_bridge::toCvCopy(msg, msg->encoding);
    } catch (cv_bridge::Exception &e) {
      // if there is an error during conversion, display it
      ROS_ERROR("tutorialROSOpenCV::main.cpp::cv_bridge exception: %s",
                e.what());
      return;
    }
    // ROS_INFO("%f", cv_ptr->image.at<float>(0, 0));
    // Copy the image.data to imageBuf.
    cv::Mat depth_float_img = cv_ptr->image;
    cv::Mat depth_mono8_img;
    depthToCV8UC1(depth_float_img, depth_mono8_img, 10);
    cv::Mat rgb_img;
    cv::Mat in[] = {depth_mono8_img, depth_mono8_img, depth_mono8_img};
    cv::merge(in, 3, rgb_img);

    QImage temp = QImage((uchar *)rgb_img.data, rgb_img.cols, rgb_img.rows,
                         rgb_img.step, QImage::Format_RGB888);
    image = temp;
    ui->label->setScaledContents(true);
    ui->label->setPixmap(
        QPixmap::fromImage(image).scaledToHeight(ui->label->height() - 4));
    // ui->label->setPixmap(QPixmap::fromImage(image));
  }
}

// void CameraViewWidget::resizeEvent(QResizeEvent *event) {
//  ui->label->resize(event->size());
//  ui->label->setPixmap(ui->label->pixmap()->scaledToHeight(event->size().height())
//  - 4);
//}