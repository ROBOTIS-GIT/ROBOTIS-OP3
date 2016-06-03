#ifndef _DETECTOR_CONFIG_H_
#define _DETECTOR_CONFIG_H_

namespace robotis_op {

//constants
const int GAUSSIAN_BLUR_SIZE_DEFAULT = 7;
const double GAUSSIAN_BLUR_SIGMA_DEFAULT = 2;
const double CANNY_EDGE_TH_DEFAULT = 130;
const double HOUGH_ACCUM_RESOLUTION_DEFAULT = 2;
const double MIN_CIRCLE_DIST_DEFAULT = 30;
const double HOUGH_ACCUM_TH_DEFAULT = 120;
const int MIN_RADIUS_DEFAULT = 30;
const int MAX_RADIUS_DEFAULT = 400;
const unsigned int IMG_MONO = 0;
const unsigned int IMG_RGB8 = 1;
const int FILTER_THRESHOLD_DEFAULT = 160;

class DetectorConfig
{
public:
    int gaussian_blur_size;         // size of gaussian blur kernel mask [pixel]
    double gaussian_blur_sigma;     // sigma of gaussian blur kernel mask [pixel]
    double canny_edge_th;           // threshold of the edge detector.
    double hough_accum_resolution;  // resolution of the Hough accumulator, in terms of inverse ratio of image resolution
    double min_circle_dist;         // Minimum distance between circles
    double hough_accum_th;          // accumulator threshold to decide circle detection
    int min_radius;                 // minimum circle radius allowed
    int max_radius;                 // maximum circle radius allowed
    int filter_threshold;           // red filter threshold 0~255
    bool debug;                     // to debug log

    DetectorConfig()
        : canny_edge_th(CANNY_EDGE_TH_DEFAULT)
          , hough_accum_resolution(HOUGH_ACCUM_RESOLUTION_DEFAULT)
          , min_circle_dist(MIN_CIRCLE_DIST_DEFAULT)
          , hough_accum_th(HOUGH_ACCUM_TH_DEFAULT)
          , min_radius(MIN_RADIUS_DEFAULT)
          , max_radius(MAX_RADIUS_DEFAULT)
          , filter_threshold(FILTER_THRESHOLD_DEFAULT)
          , debug(false)
    { }

    ~DetectorConfig() {}
};

}
#endif  // _DETECTOR_CONFIG_H_
