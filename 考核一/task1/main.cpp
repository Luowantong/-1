#include<opencv2/opencv.hpp>
#include<iostream>
int main()
{
std::cout<<"OpenCV runs successfully"<<std::endl;
std::cout<<"OpenCV version:"<<CV_VERSION<<std::endl;
cv::Mat image=cv::Mat::zeros(300,400,CV_8UC3);
cv::putText(image,"Hello OpenCV!",cv::Point(50,150),cv::FONT_HERSHEY_SIMPLEX,1.0,cv::Scalar(0,255,0),2);
std::cout<<"images have been created successfully"<<std::endl;
return 0;
}
