#ifndef DELAUNY_H
#define DELAUNY_H

#include <map>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <chrono>
#include "Edge.h"
#include "Frame.h"
namespace ORB_SLAM2
{
    class Frame;
    class Delauny{
    public:
        Delauny();
        static void UpdatePointRelation(cv::Rect rect,Frame &currentFrame);
        static void DrawLines(cv::Mat &img,std::vector<Edge> &edgelist);
        static bool ContainPoint(cv::Rect rect,cv::Point2f p);
    };
}

#endif