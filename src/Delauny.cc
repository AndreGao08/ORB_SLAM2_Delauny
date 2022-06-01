#include "Delauny.h"
#include <map>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <chrono>
#include <iostream>
namespace ORB_SLAM2
{
    void Delauny::UpdatePointRelation(cv::Rect rect,Frame &currentFrame){
        //所有特征点
        //std::vector<cv::KeyPoint> keyPoints = currentFrame.mvKeys;
        //当前帧的地图点
        //std::vector<MapPoint*> mapPoints = currentFrame.mvpMapPoints;
        //存储特征点和vertexID的对应关系
        //map的结构{key:关键点在帧中的index , value:关键点在三角剖分中的vertexIndex}
        map<int,int> pointIndex;
        for(int i=0;i<currentFrame.N;i++){
            MapPoint *mp = currentFrame.mvpMapPoints[i];
            //必须要是存在地图点的才进行三角剖分
            if(mp){
                pointIndex[i]=currentFrame.subdiv.insert(currentFrame.mvKeys[i].pt);
            }
        }
       
        //获取TriangleList
        std::vector<cv::Vec6f> triangleList;
        currentFrame.subdiv.getTriangleList(triangleList);
        //std::cout<<triangleList.size()<<std::endl;
        //存储一个三角形中的三个点
        std::vector<cv::Point2f> pt(3);
        int vertex[3];
        int edge;
        //遍历triangleList
        for(int i=0;i<triangleList.size();i++){
            cv::Vec6f t = triangleList[i];
            //pt[0],pt[1],pt[2]
            pt[0] = cv::Point2f(t[0],t[1]);
            pt[1] = cv::Point2f(t[2],t[3]);
            pt[2] = cv::Point2f(t[4],t[5]);
            //std::cout<<"group"<<i<<std::endl;
            //在图内的点才能进行三角剖分
            if(ContainPoint(rect,pt[0]) && ContainPoint(rect,pt[1]) && ContainPoint(rect,pt[2]))
            {
                //得到各个点的序号
                for(int index=0;index<3;index++){
                    currentFrame.subdiv.locate(pt[index],edge,vertex[index]);
                    //std::cout<<pt[index]<<":vertex="<<vertex[index]<<std::endl;
                }
                //通过每个vertex来建立Edge
                for(int index=0;index<3;index++){
                    for(map<int,int>::iterator it=pointIndex.begin();it!=pointIndex.end();it++){
                         if(it->second==vertex[index]){
                             vertex[index]=it->first;
                             break;
                         }
                    }
                }
                //创建Edge对象
                Edge edge1(currentFrame.mvKeys[vertex[0]],vertex[0],currentFrame.mvKeys[vertex[1]],vertex[1]);
                Edge edge2(currentFrame.mvKeys[vertex[2]],vertex[2],currentFrame.mvKeys[vertex[0]],vertex[0]);
                Edge edge3(currentFrame.mvKeys[vertex[1]],vertex[1],currentFrame.mvKeys[vertex[2]],vertex[2]);
                currentFrame.mvEdgeList.push_back(edge1);
                currentFrame.mvEdgeList.push_back(edge2);
                currentFrame.mvEdgeList.push_back(edge3);
                
                //点关系表存储
                currentFrame.mvPointRelations[vertex[0]][vertex[1]]=edge1.get_edge_ID();
                currentFrame.mvPointRelations[vertex[1]][vertex[0]]=edge1.get_edge_ID();
                currentFrame.mvPointRelations[vertex[1]][vertex[2]]=edge3.get_edge_ID();
                currentFrame.mvPointRelations[vertex[2]][vertex[1]]=edge3.get_edge_ID();
                currentFrame.mvPointRelations[vertex[0]][vertex[2]]=edge2.get_edge_ID();
                currentFrame.mvPointRelations[vertex[2]][vertex[0]]=edge2.get_edge_ID();

            }
        
            
        }

    }

    void Delauny::DrawLines(cv::Mat &img, std::vector<Edge> &edgelist){
        cv::Scalar delaunay_color(255,0,0);
        std::cout<<edgelist.size()<<std::endl;
        if(!edgelist.empty()){
            for(std::vector<Edge>::iterator edge=edgelist.begin();edge!=edgelist.end();edge++){
                cv::line(img,edge->keyPoint_1.pt,edge->keyPoint_2.pt,delaunay_color,1,cv::LINE_AA,0);
            }
        }
        else{
            return;
        }
    }

    bool Delauny::ContainPoint(cv::Rect rect,cv::Point2f p){
        if(p.x<0){
            return false;
        }else if(p.x>rect.width){
            return false;
        }else if(p.y<0){
            return false;
        }else if(p.y>rect.height){
            return false;
        }
        return true;
    }

    Delauny::Delauny(){}
}
