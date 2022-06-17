#ifndef EDGE_H
#define EDGE_H

#include<map>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <chrono>
namespace ORB_SLAM2{
class Edge{
    public:
        static int pre_id;
        //keypoint1_2
        cv::KeyPoint keyPoint_1;
        cv::KeyPoint keyPoint_2;
        Edge(cv::KeyPoint &keyPoint_1,cv::KeyPoint &keyPoint_2);
        Edge(cv::KeyPoint &keyPoint_1,int key1_id,cv::KeyPoint &keyPoint_2,int key2_id);
        Edge();
        int get_edge_ID();
        int set_edge_ID();
        void get_keyPoint(cv::KeyPoint &keyPoint_1,cv::KeyPoint &keyPoint_2);
        std::vector<int,int> get_edge_vector();
        float get_edge_val();
        //计算边长度
        float cal_edge_val();
        //两点在图上的index
        int key1_id,key2_id;
        
        bool isOutlier;
        
        void SetOutlier();
    private:
        //边的id
        int edge_id;
        
        
        //边向量
        //std::vector<int,int> edge_vector;
        //边的值
        float edge_val;
};
}
#endif