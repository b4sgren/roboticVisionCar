#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <cmath>

int mouse_X, mouse_Y;

void on_mouse(int evt, int x, int y, int flags, void* param) {
   if(evt == cv::EVENT_LBUTTONDOWN) { //CV_EVENT_LBUTTONDOWN
       mouse_X = x;
       mouse_Y = y;
   }
}

cv::Point2f calcMoment(const cv::Mat& img)
{
  cv::Moments m{cv::moments(img, true)};
  cv::Point2f center;
  if (m.m00 != 0.0)
    center = cv::Point2f{float(m.m10/m.m00), float(m.m01/m.m00)};
  else
    center = cv::Point2f{0,0};

  return center;
}

int main()
{
  cv::VideoCapture cap("../final_project1.avi");
  cv::Mat img, hsv, bw_img, cropped_img;
  cap >> img;
  cv::Rect roi; // Determine where to initialize this
  roi.x = 0; //for images taken on my laptop
  roi.y = 366;
  roi.width = img.cols;
  roi.height = 4;

  double x0{img.cols/2.0}, y0{double(img.rows)};
  std::cout << "x0: " << x0 << "\ty0: " << y0 << std::endl;

  while(true)
  {
    cap >> img;
    if(img.empty())
      break;

    cv::cvtColor(img, hsv, cv::COLOR_BGR2HSV);
    cv::inRange(hsv, cv::Scalar(80, 0, 0), cv::Scalar(115, 255, 255), bw_img); //Captures the table legs

    cropped_img = bw_img(roi);
    cv::Point2f center = calcMoment(cropped_img);
    center.x += roi.x;
    center.y += roi.y;
    std::cout << "Center point: " << center << std::endl;
    double phi = atan2(center.x - x0, y0 - center.y);
    std::cout << "Angle: " << phi * 180/3.14159265 << std::endl;
    /*
    Note: This works. Sometimes the angle seems a little bit big. We also may
    want to low pass filter phi before sending it over to the controller.
    TODO Take care of the situation where only one lane is visible in the FOV
    of the camera.
     */

    cv::imshow("Window", bw_img);
    cv::imshow("Color", img);
    cv::imshow("Cropped", cropped_img);
    cv::setMouseCallback("Color", on_mouse);
    cv::waitKey(0);
    // std::cout << mouse_X << "\t" << mouse_Y << "\n";
    // std::cout << img.at<cv::Vec3b>(mouse_Y, mouse_X) << "\n";
  }

  return 0;
}
