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
  printf("start rss=%ld threads=%d\n", rss_kb(), cv::getNumThreads());

  for (int nt : {8, 4, 1}) {
    cv::setNumThreads(nt);
    printf("threads=%d remap i16 full\n", nt);
    for (int i = 0; i < 120; ++i) {
      cv::remap(src, dst, mx16, my16, cv::INTER_LINEAR, cv::BORDER_CONSTANT);
      if (i % 20 == 0) printf("  %d rss=%ld\n", i, rss_kb());
    }
  }
  printf("end rss=%ld\n", rss_kb());
  return 0;
}