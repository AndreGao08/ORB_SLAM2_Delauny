#include"Edge.h"

namespace ORB_SLAM2{
int Edge::pre_id=-1;
Edge::Edge(cv::KeyPoint &keyPoint_1,int key1_id,cv::KeyPoint &keyPoint_2,int key2_id){
    pre_id++;
    this->edge_id = pre_id;
    this->keyPoint_1 = keyPoint_1;
    this->key1_id = key1_id;
    this->keyPoint_2 = keyPoint_2;
    this->key2_id = key2_id;
    this->edge_val = cal_edge_val();
    this->isOutlier=false;
}

//初始化函数
Edge::Edge(cv::KeyPoint &keyPoint_1,cv::KeyPoint &keyPoint_2){
    pre_id++;
    this->edge_id = pre_id;
    this->keyPoint_1 = keyPoint_1;
    this->keyPoint_2 = keyPoint_2;
    this->edge_val = cal_edge_val();
    this->isOutlier=false;
}
//空初始化函数
Edge::Edge()
{

}
//获取边id
int Edge::get_edge_ID(){
    return this->edge_id;
}
//得到边的两个关键点
void Edge::get_keyPoint(cv::KeyPoint &keyPoint_1,cv::KeyPoint &keyPoint_2){
    keyPoint_1=this->keyPoint_1;
    keyPoint_2=this->keyPoint_2;
    return ;
}
//计算边长度
float Edge::cal_edge_val(){
    cv::Point2f p1 = this->keyPoint_1.pt;
    cv::Point2f p2 = this->keyPoint_2.pt;
    return pow(p1.x-p2.x,2)+pow(p1.y-p2.y,2);
}
//得到edge长度 马是距离
float Edge::get_edge_val(){
    if(this->edge_val!=0){
        return this->edge_val;
    }
    else{
        return this->cal_edge_val();
    }
}

void Edge::SetOutlier(){
    this->isOutlier=true;
}

}