// Fisheye stereo calibration helper (avoids OpenCV 4.2 Python binding bugs).
// Input text:
//   nviews
//   npoints
//   ox oy oz ix1 iy1 ix2 iy2
// Output JSON: {"rms":...,"R":[9],"T":[3]}
#include <opencv2/opencv.hpp>
#include <fstream>
#include <iostream>
#include <sstream>
#include <vector>

int main(int argc, char** argv) {
  if (argc < 20) {
    std::cerr << "usage: fisheye_stereo points.txt width height "
                 "fx1 fy1 cx1 cy1 k11 k12 k13 k14 fx2 fy2 cx2 cy2 k21 k22 k23 k24\n";
    return 2;
  }
  std::ifstream in(argv[1]);
  if (!in) { std::cerr << "cannot open " << argv[1] << "\n"; return 2; }
  int width = std::atoi(argv[2]);
  int height = std::atoi(argv[3]);
  cv::Mat K1 = (cv::Mat_<double>(3, 3) << std::atof(argv[4]), 0, std::atof(argv[6]),
                0, std::atof(argv[5]), std::atof(argv[7]), 0, 0, 1);
  cv::Mat D1 = (cv::Mat_<double>(4, 1) << std::atof(argv[8]), std::atof(argv[9]),
                std::atof(argv[10]), std::atof(argv[11]));
  cv::Mat K2 = (cv::Mat_<double>(3, 3) << std::atof(argv[12]), 0, std::atof(argv[14]),
                0, std::atof(argv[13]), std::atof(argv[15]), 0, 0, 1);
  cv::Mat D2 = (cv::Mat_<double>(4, 1) << std::atof(argv[16]), std::atof(argv[17]),
                std::atof(argv[18]), std::atof(argv[19]));

  int nviews = 0;
  in >> nviews;
  std::vector<std::vector<cv::Point3f>> objectPoints;
  std::vector<std::vector<cv::Point2f>> imagePoints1, imagePoints2;
  for (int v = 0; v < nviews; ++v) {
    int n = 0;
    in >> n;
    std::vector<cv::Point3f> obj;
    std::vector<cv::Point2f> img1, img2;
    obj.reserve(n); img1.reserve(n); img2.reserve(n);
    for (int i = 0; i < n; ++i) {
      double ox, oy, oz, ix1, iy1, ix2, iy2;
      in >> ox >> oy >> oz >> ix1 >> iy1 >> ix2 >> iy2;
      obj.emplace_back((float)ox, (float)oy, (float)oz);
      img1.emplace_back((float)ix1, (float)iy1);
      img2.emplace_back((float)ix2, (float)iy2);
    }
    objectPoints.push_back(obj);
    imagePoints1.push_back(img1);
    imagePoints2.push_back(img2);
  }
  if (!in) { std::cerr << "bad points file\n"; return 2; }

  cv::Mat R, T;
  cv::TermCriteria criteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 200, 1e-9);
  double rms = 0.0;
  try {
    rms = cv::fisheye::stereoCalibrate(objectPoints, imagePoints1, imagePoints2,
                                       K1, D1, K2, D2, cv::Size(width, height),
                                       R, T, cv::fisheye::CALIB_FIX_INTRINSIC, criteria);
  } catch (const cv::Exception& e) {
    std::cerr << "stereoCalibrate failed: " << e.what() << "\n";
    return 3;
  }
  std::ostringstream os;
  os.precision(17);
  os << "{\"rms\":" << rms << ",\"R\":[";
  for (int r = 0; r < 3; ++r)
    for (int c = 0; c < 3; ++c)
      os << (r || c ? "," : "") << R.at<double>(r, c);
  os << "],\"T\":[" << T.at<double>(0) << "," << T.at<double>(1) << ","
     << T.at<double>(2) << "]}";
  std::cout << os.str() << std::endl;
  return 0;
}
