#include <opencv2/opencv.hpp>
#include <cstdio>
#include <unistd.h>

static long rss_kb() {
  FILE* f = fopen("/proc/self/status", "r");
  if (!f) return -1;
  char line[256];
  long v = -1;
  while (fgets(line, sizeof(line), f)) {
    if (sscanf(line, "VmRSS: %ld kB", &v) == 1) break;
  }
  fclose(f);
  return v;
}

int main() {
  cv::Mat src(1200, 1920, CV_8UC3);
  cv::randu(src, 0, 255);
  cv::Mat mx(480, 960, CV_32FC1), my(480, 960, CV_32FC1);
  cv::randu(mx, -1, 1921); cv::randu(my, -1, 1201);
  cv::Mat mx16, my16;
  cv::convertMaps(mx, my, mx16, my16, CV_16SC2);
  cv::Mat dst(480, 960, CV_8UC3);
  cv::Mat ext(1200, 1920, CV_8UC3, src.data);
  printf("start rss=%ld src.u=%p ext.u=%p\n", rss_kb(), (void*)src.u, (void*)ext.u);

  printf("A: Mat alloc/free\n");
  for (int i = 0; i < 200; ++i) {
    cv::Mat m(1200, 1920, CV_8UC3);
    m.setTo(cv::Scalar(i & 255));
    if (i % 40 == 0) printf("  %d rss=%ld\n", i, rss_kb());
  }

  printf("B: remap float full\n");
  for (int i = 0; i < 120; ++i) {
    cv::remap(src, dst, mx, my, cv::INTER_LINEAR, cv::BORDER_CONSTANT);
    if (i % 20 == 0) printf("  %d rss=%ld\n", i, rss_kb());
  }

  printf("C: remap i16 full\n");
  for (int i = 0; i < 120; ++i) {
    cv::remap(src, dst, mx16, my16, cv::INTER_LINEAR, cv::BORDER_CONSTANT);
    if (i % 20 == 0) printf("  %d rss=%ld\n", i, rss_kb());
  }

  printf("D: remap i16 external src full\n");
  for (int i = 0; i < 120; ++i) {
    cv::remap(ext, dst, mx16, my16, cv::INTER_LINEAR, cv::BORDER_CONSTANT);
    if (i % 20 == 0) printf("  %d rss=%ld\n", i, rss_kb());
  }

  cv::Mat big(480, 960, CV_8UC3);
  cv::Mat mxr(480, 240, CV_32FC1), myr(480, 240, CV_32FC1);
  cv::randu(mxr, -1, 1921); cv::randu(myr, -1, 1201);
  cv::Mat mxr16, myr16;
  cv::convertMaps(mxr, myr, mxr16, myr16, CV_16SC2);
  printf("E: remap i16 into ROI\n");
  for (int i = 0; i < 120; ++i) {
    cv::Mat roi = big(cv::Rect((i % 4) * 240, 0, 240, 480));
    cv::remap(src, roi, mxr16, myr16, cv::INTER_LINEAR, cv::BORDER_CONSTANT);
    if (i % 20 == 0) printf("  %d rss=%ld\n", i, rss_kb());
  }
  printf("end rss=%ld\n", rss_kb());
  return 0;
}