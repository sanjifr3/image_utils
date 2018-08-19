#include <image_utils/ImageProcessor.h>

ImageProcessor::ImageProcessor(){
  ROS_INFO("[ImageProcessor] Initialized!");
}

ImageProcessor::~ImageProcessor(){
}

void ImageProcessor::updateFilters(cv::cuda::GpuMat& im){
 gpuMedianFilter_ = cv::cuda::createMedianFilter(im.depth(), medianFilter_kernelsize_);
 gpuGaussianBlur_ = cv::cuda::createGaussianFilter(im.depth(), im.depth(), gaussianBlur_kernelsize_, gaussianBlur_border_);
}

void ImageProcessor::convertTo(cv::cuda::GpuMat& im, int method){
  cv::cuda::cvtColor(im, im, method);
}

void ImageProcessor::convertTo(cv::cuda::GpuMat& src, cv::cuda::GpuMat& dst, int method){
  cv::cuda::cvtColor(src, dst, method);
}

void ImageProcessor::convertTo(cv::Mat& im, int method){
  cv::cvtColor(im, im, method);
}

void ImageProcessor::convertTo(cv::Mat& src, cv::Mat& dst, int method){
  cv::cvtColor(src, dst, method);
}

void ImageProcessor::equalizeHist(cv::cuda::GpuMat& im){
  cv::cuda::equalizeHist(im, im);
}

void ImageProcessor::equalizeHist(cv::cuda::GpuMat& src, cv::cuda::GpuMat& dst){
  cv::cuda::equalizeHist(src, dst);
}

void ImageProcessor::flip(cv::cuda::GpuMat& im, int flip_code){
  cv::cuda::flip(im, im, flip_code);
}

void ImageProcessor::flip(cv::cuda::GpuMat& src, cv::cuda::GpuMat& dst, int flip_code){
  cv::cuda::flip(src, dst, flip_code);
}

void ImageProcessor::resize(cv::cuda::GpuMat& im, cv::Size dsize){
  cv::cuda::resize(im, im, dsize);
}

void ImageProcessor::resize(cv::cuda::GpuMat& src, cv::cuda::GpuMat& dst, cv::Size dsize){
  cv::cuda::resize(src, dst, dsize);
}

void ImageProcessor::rescale(cv::cuda::GpuMat& im, double fx, double fy){
  if (fy == 0) fy = fx;
  cv::cuda::resize(im, im, cv::Size(0,0), fx, fy);
}

void ImageProcessor::rescale(cv::cuda::GpuMat& src, cv::cuda::GpuMat& dst, double fx, double fy){
  if (fy == 0) fy = fx;
  cv::cuda::resize(src, dst, cv::Size(0,0), fx, fy);
}

void ImageProcessor::minMax(cv::cuda::GpuMat& im, double& minVal, double& maxVal){
  cv::cuda::minMax(im, &minVal, &maxVal);
}

void ImageProcessor::multiply(cv::cuda::GpuMat& src, double val, cv::cuda::GpuMat& dst){
  cv::cuda::multiply(src, val, dst);
}

void ImageProcessor::normalize(cv::cuda::GpuMat& src, cv::cuda::GpuMat& dst, double alpha, double beta, int norm_type, int dtype){
  cv::cuda::normalize(src, dst, alpha, beta, norm_type, dtype);
}

void ImageProcessor::threshold(cv::cuda::GpuMat& im, double thresh, double max, int type){
  cv::cuda::threshold(im, im, thresh, max, type);
}

void ImageProcessor::gaussianBlur(cv::cuda::GpuMat& im){
  if (!gaussian_ready_){
    gpuGaussianBlur_ = cv::cuda::createGaussianFilter(im.depth(), im.depth(), gaussianBlur_kernelsize_, gaussianBlur_border_);
    gaussian_ready_ = true;
  }
  gpuGaussianBlur_->apply(im, im);
}

void ImageProcessor::medianFilter(cv::cuda::GpuMat& im, int kernel){
  if (!mf_ready_ || kernel != medianFilter_kernelsize_){
    medianFilter_kernelsize_ = kernel;
    gpuMedianFilter_ = cv::cuda::createMedianFilter(im.depth(), medianFilter_kernelsize_);
    mf_ready_ = true;
  }
  gpuMedianFilter_->apply(im, im);
}

void ImageProcessor::dilate(cv::cuda::GpuMat& im, cv::cuda::GpuMat& element){
  if(!dilate_ready_){
    gpuDilate_ = cv::cuda::createMorphologyFilter(1, im.type(), element);
    dilate_ready_ = true;
  }
  gpuDilate_->apply(im, im);
}

void ImageProcessor::canny(cv::cuda::GpuMat& src, cv::cuda::GpuMat& dst){
  if (!canny_ready_){
    gpuCanny_ = cv::cuda::createCannyEdgeDetector(canny_low_, canny_high_, canny_aper_size_, canny_L2_grad_);
    canny_ready_ = true;
  }
  gpuCanny_->detect(src, dst);
}

void ImageProcessor::matchTemplate(cv::cuda::GpuMat& im, cv::cuda::GpuMat& templ, cv::cuda::GpuMat &result, int method){
  if (!templ_ready_){
    gpuTemplateMatching_ = cv::cuda::createTemplateMatching(im.type(), method);
    templ_ready_ = true;
  }
  gpuTemplateMatching_ -> match(im, templ, result);
  return;
}

void ImageProcessor::match3DTemplate(cv::cuda::GpuMat& im, cv::cuda::GpuMat& templ, cv::cuda::GpuMat &result, int method){
  if (!templ3D_ready_){
    gpu3DTemplateMatching_ = cv::cuda::createTemplateMatching(im.type(), method);
    templ3D_ready_ = true;
  }
  gpu3DTemplateMatching_ -> match(im, templ, result);
  return;
}

cv::Mat ImageProcessor::toMat(cv::cuda::GpuMat& im){
  cv::Mat cpu_im;
  im.download(cpu_im);
  return cpu_im;
}

cv::cuda::GpuMat ImageProcessor::toGpuMat(cv::Mat& im){
  cv::cuda::GpuMat gpu_im(im);
  return gpu_im;
}

dlib::array2d<uchar> ImageProcessor::toDlib(cv::Mat& im){
  dlib::cv_image<unsigned char> temp(im);
  dlib::array2d<unsigned char> dlib_im;
  dlib::assign_image(dlib_im, temp);
  return dlib_im;
}

dlib::rectangle ImageProcessor::toDlibRect(cv::Rect& rect){
  int l = rect.x;
  int t = rect.y;
  int r = rect.x + rect.width;
  int b = rect.y + rect.height;

  return dlib::rectangle(l, t, r, b);
}

cv::Rect ImageProcessor::toCvRect(dlib::rectangle& rect){
  int x = rect.left();
  int y = rect.top();
  int width = rect.width();
  int height = rect.height();

  return cv::Rect(x,y,width,height);
}

void ImageProcessor::rgb2bw(cv::Mat& im){
  if ( im.channels() == 3 )
    cvtColor ( im, im, CV_RGB2GRAY );

  cv::threshold ( im, im, 128, 255, CV_THRESH_BINARY );
  return;
}

cv::Mat ImageProcessor::overlayImages(cv::Mat& src1, cv::Mat& src2, double alpha){
  cv::Mat dst;
  cv::addWeighted(src1, alpha, src2, 1.0 - alpha, 0.0, dst);
  return dst;
}

double ImageProcessor::getAvgDist(cv::Mat& im, cv::Rect& rect, double granularity){ 
  int min_x = rect.x + rect.width/3;
  int min_y = rect.y + rect.height/3;
  int max_x = rect.x + rect.width - rect.width/3;
  int max_y = rect.y + rect.height - rect.height/3;

  float x_spacing = float(max_x - min_x)/float(granularity+1);
  float y_spacing = float(max_y - min_y)/float(granularity+1);  

  double validPts = 0;
  double avgDist = 0;

  for(int i = 1; i < granularity +1; i++){
    for(int j = 1; j < granularity +1; j++){
      float dist = im.at<uint16_t>(
        int(min_y + y_spacing*j),
        int(min_x + x_spacing*i) 
      );
      if(dist!=0){
        avgDist += dist;
        validPts++;
      }
    } 
  }

  if(validPts != 0)
    avgDist/=validPts;

  return avgDist/1000.0;
}

void ImageProcessor::drawLegend(cv::Mat& im, std::vector<std::string> legend_text, std::vector<cv::Scalar> legend_color){
  int L_i = 20; // x starting point //480
  int L_j = 20; // y starting point
  int L_len = 45; // length of line
  int L_vspace = 20; // spacing between lines
  int L_tj = 25; // text starting height
  double L_thick = 2; // thickness of line
  int L_hspace = 10; // spacing between line and text
  double L_theight = 0.5; // text height

  for (unsigned int i = 1; i < legend_text.size() + 1; i++){
    cv::line(im, cv::Point(L_i, L_j + i * L_vspace), cv::Point(L_i + L_len, L_j + i * L_vspace), legend_color[i-1], L_thick);
    cv::putText(im, cv::format("%s", legend_text[i-1].c_str()), cv::Point(L_i + L_len + L_hspace, L_tj + i * L_vspace), 2.0, L_theight, legend_color[i-1], 1.0);
  }
  return;
}

void ImageProcessor::drawRects(cv::Mat &im, std::vector<cv::Rect> targets, cv::Scalar color){
  for(unsigned int i = 0; i < targets.size(); i++)
    cv::rectangle(im,targets[i],color,2);
  return;
}

cv::Mat ImageProcessor::createBlankMat(cv::Mat& im, int type){
  return createBlankMat(im.rows, im.cols, type);
}

cv::Mat ImageProcessor::createBlankMat(int rows, int cols, int type){
  return cv::Mat(rows, cols, type, cv::Scalar::all(255));
}