// Small helper for cv::omnidir::calibrate (OpenCV 4.2 Python binding is broken).
// Input text format:
//   nviews
//   npoints
//   ox oy oz ix iy   (repeated npoints times)
//   ... repeated nviews times
// Output: one-line JSON.
#include <opencv2/opencv.hpp>
#include <opencv2/ccalib/omnidir.hpp>
#include <fstream>
#include <iostream>
#include <sstream>
#include <vector>

int main(int argc, char** argv) {
  if (argc < 6) {
    std::cerr << "usage: omni_calib points.txt width height fx fy [cx cy]\n";
    return 2;
  }
  std::ifstream in(argv[1]);
  if (!in) { std::cerr << "cannot open " << argv[1] << "\n"; return 2; }
  int width = std::atoi(argv[2]);
  int height = std::atoi(argv[3]);
  double fx = std::atof(argv[4]);
  double fy = std::atof(argv[5]);
  double cx = (argc > 7) ? std::atof(argv[6]) : (width - 1) / 2.0;
  double cy = (argc > 7) ? std::atof(argv[7]) : (height - 1) / 2.0;

  int nviews = 0;
  in >> nviews;
  std::vector<std::vector<cv::Point3d>> objectPoints;
  std::vector<std::vector<cv::Point2d>> imagePoints;
  for (int v = 0; v < nviews; ++v) {
    int n = 0;
    in >> n;
    std::vector<cv::Point3d> obj;
    std::vector<cv::Point2d> img;
    obj.reserve(n); img.reserve(n);
    for (int i = 0; i < n; ++i) {
      double ox, oy, oz, ix, iy;
      in >> ox >> oy >> oz >> ix >> iy;
      obj.emplace_back(ox, oy, oz);
      img.emplace_back(ix, iy);
    }
    objectPoints.push_back(obj);
    imagePoints.push_back(img);
  }
  if (!in) { std::cerr << "bad points file\n"; return 2; }

  cv::Mat K = (cv::Mat_<double>(3, 3) << fx, 0, cx, 0, fy, cy, 0, 0, 1);
  cv::Mat xi = (cv::Mat_<double>(1, 1) << 1.0);
  cv::Mat D = cv::Mat::zeros(1, 4, CV_64F);
  std::vector<cv::Mat> rvecs, tvecs;
  cv::Mat idx;
  int flags = cv::omnidir::CALIB_USE_GUESS | cv::omnidir::CALIB_FIX_SKEW;
  cv::TermCriteria criteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS,
                            200, 1e-9);
  double rms = 0.0;
  try {
    rms = cv::omnidir::calibrate(objectPoints, imagePoints,
                                 cv::Size(width, height), K, xi, D,
                                 rvecs, tvecs, flags, criteria, idx);
  } catch (const cv::Exception& e) {
    std::cerr << "calibrate failed: " << e.what() << "\n";
    return 3;
  }

  std::ostringstream os;
  os.precision(17);
  os << "{\"rms\":" << rms << ",\"K\":[";
  for (int r = 0; r < 3; ++r)
    for (int c = 0; c < 3; ++c)
      os << (r || c ? "," : "") << K.at<double>(r, c);
  os << "],\"xi\":" << xi.at<double>(0, 0) << ",\"D\":[";
  for (int c = 0; c < 4; ++c) os << (c ? "," : "") << D.at<double>(0, c);
  os << "],\"rvecs\":[";
  for (size_t i = 0; i < rvecs.size(); ++i) {
    os << (i ? ",[" : "[");
    for (int r = 0; r < 3; ++r) os << (r ? "," : "") << rvecs[i].at<double>(r, 0);
    os << "]";
  }
  os << "],\"tvecs\":[";
  for (size_t i = 0; i < tvecs.size(); ++i) {
    os << (i ? ",[" : "[");
    for (int r = 0; r < 3; ++r) os << (r ? "," : "") << tvecs[i].at<double>(r, 0);
    os << "]";
  }
  os << "],\"idx\":[";
  for (int i = 0; i < (idx.empty() ? 0 : (int)idx.total()); ++i)
    os << (i ? "," : "") << idx.at<int>(i);
  os << "]}";
  std::cout << os.str() << std::endl;
  return 0;
}
