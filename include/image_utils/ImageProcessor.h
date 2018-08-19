#ifndef IMAGE_PROCESSOR_H
#define IMAGE_PROCESSOR_H

#include <image_utils/Utilities.h>

#include <opencv2/cudaimgproc.hpp>
#include <opencv2/cudawarping.hpp>
#include <opencv2/cudaarithm.hpp>
#include <opencv2/cudafilters.hpp>
#include <opencv2/core/opengl.hpp>

#include <dlib/opencv/cv_image.h>
#include <dlib/image_processing.h>

class ImageProcessor{
  public:
    ImageProcessor();
    ~ImageProcessor();
    
    void updateFilters(cv::cuda::GpuMat& im);
    
    // GPU Functions
    void convertTo(cv::cuda::GpuMat& im, int method=CV_BGR2GRAY);
    void convertTo(cv::cuda::GpuMat& src, cv::cuda::GpuMat& dst, int method=CV_BGR2GRAY);
    void convertTo(cv::Mat& im, int method=CV_BGR2GRAY);
    void convertTo(cv::Mat& src, cv::Mat& dst, int method=CV_BGR2GRAY);
    void equalizeHist(cv::cuda::GpuMat& im);
    void equalizeHist(cv::cuda::GpuMat& src, cv::cuda::GpuMat& dst);    
    void flip(cv::cuda::GpuMat& im, int flip_code=1);
    void flip(cv::cuda::GpuMat& src, cv::cuda::GpuMat& dst, int flip_code=1);
    void resize(cv::cuda::GpuMat& im, cv::Size dsize);
    void resize(cv::cuda::GpuMat& src, cv::cuda::GpuMat& dst, cv::Size dsize);    
    void rescale(cv::cuda::GpuMat& im, double fx, double fy=0);
    void rescale(cv::cuda::GpuMat& src, cv::cuda::GpuMat& dst, double fx, double fy=0);
    void minMax(cv::cuda::GpuMat& im, double& minVal, double& maxVal);
    void multiply(cv::cuda::GpuMat& src, double val, cv::cuda::GpuMat& dst);
    
    void normalize(cv::cuda::GpuMat& src, cv::cuda::GpuMat& dst, double alpha=1, double beta=0, int norm_type=cv::NORM_L2, int dtype=-1);
    void threshold(cv::cuda::GpuMat& im, double thresh, double max=1, int type=CV_THRESH_BINARY_INV);
    
    void gaussianBlur(cv::cuda::GpuMat& im);
    void medianFilter(cv::cuda::GpuMat& im, int kernel);
    void dilate(cv::cuda::GpuMat& im, cv::cuda::GpuMat& element);
    void canny(cv::cuda::GpuMat& src, cv::cuda::GpuMat& dst);
    void matchTemplate(cv::cuda::GpuMat& im, cv::cuda::GpuMat& templ, cv::cuda::GpuMat &result, int method = 2);
    void match3DTemplate(cv::cuda::GpuMat& im, cv::cuda::GpuMat& templ, cv::cuda::GpuMat &result, int method = 2);

    // Conversion Functions
    cv::Mat toMat(cv::cuda::GpuMat& im);
    cv::cuda::GpuMat toGpuMat(cv::Mat& im);
    dlib::array2d<uchar> toDlib(cv::Mat& im);

    dlib::rectangle toDlibRect(cv::Rect& rect);
    cv::Rect toCvRect(dlib::rectangle& rect);
  
    // CPU Functions
    void rgb2bw (cv::Mat &im);
    cv::Mat overlayImages(cv::Mat& src1, cv::Mat& src2, double alpha=0.5);
    
    // Depth CPU Functions
    double getAvgDist(cv::Mat& im, cv::Rect& rect, double granularity=2);

    // OpenCV Drawing Functions
    void drawLegend(cv::Mat &im, std::vector<std::string> legend_text, std::vector<cv::Scalar> legend_color);
    void drawRects (cv::Mat &im, std::vector<cv::Rect> targets, cv::Scalar color);
    cv::Mat createBlankMat(cv::Mat& im, int type=CV_8UC3);
    cv::Mat createBlankMat(int rows=1080, int cols=1920, int type=CV_8UC1);
    
  public:
    // Canny Parameters
    bool canny_ready_ = false;
    double canny_low_ = 35;
    double canny_high_ = 200;
    int canny_aper_size_ = 3;
    bool canny_L2_grad_ = false;
    
    bool update_filters_ = false;
   
    // Median Filter Parameters
    int medianFilter_kernelsize_ = 5;

    // Gaussian Blur Parameters
    int gaussianBlur_border_ = cv::BorderTypes::BORDER_DEFAULT;
    cv::Size gaussianBlur_kernelsize_ = cv::Size(3,3);
    
    cv::Ptr<cv::cuda::CannyEdgeDetector> gpuCanny_;
    cv::Ptr<cv::cuda::Filter> gpuMedianFilter_;
    cv::Ptr<cv::cuda::Filter> gpuGaussianBlur_;
    cv::Ptr<cv::cuda::TemplateMatching> gpuTemplateMatching_;
    cv::Ptr<cv::cuda::TemplateMatching> gpu3DTemplateMatching_;
    cv::Ptr<cv::cuda::Filter> gpuDilate_;
  
  private:
    bool mf_ready_ = false;
    bool gaussian_ready_ = false;
    bool dilate_ready_ = false;
    bool templ_ready_ = false;
    bool templ3D_ready_ = false; 
};

#endif //IMAGE_PROCESSOR_H 
