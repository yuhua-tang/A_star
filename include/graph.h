#ifndef GRAPH_H
#define GRAPH_H

#include <vector>
#include <cassert>
#include <iostream>
#include <Eigen/Core>
using namespace std;

namespace global_planner{

    template <typename _LM_Index, typename _Point>
    struct LocalMarkNode{
        _LM_Index id;
        _Point local_point;
        LocalMarkNode(const _LM_Index& _id, const _Point& _local_point)
            : id(_id), local_point(_local_point)
        {}
        LocalMarkNode(){}
        LocalMarkNode& operator=(const LocalMarkNode& l){
            this->id = l.id;
            this->local_point = l.local_point;
            return *this;
        }
    };

    struct OneTrajectory{
        Eigen::Vector3f start_point;
        Eigen::Vector3f end_point;
        Eigen::Vector3f control_point1;
        Eigen::Vector3f control_point2;
        OneTrajectory(const Eigen::Vector3f& _start_point,const Eigen::Vector3f& _end_point,
                    const Eigen::Vector3f& _control_point1, const Eigen::Vector3f& _control_point2)
            : start_point(_start_point),end_point(_end_point),
            control_point1(_control_point1),control_point2(_control_point2)
        {}
        OneTrajectory(const Eigen::Vector3f& _start_point,const Eigen::Vector3f& _end_point,
                    const Eigen::Vector3f& _control_point1)
            : start_point(_start_point),end_point(_end_point),
            control_point1(_control_point1)
        {}
        OneTrajectory(const Eigen::Vector3f& _start_point,const Eigen::Vector3f& _end_point)
            : start_point(_start_point),end_point(_end_point)
        {}
    };


    // W -- 边对应权值的类型
    template <typename W>
    struct EdgeNode
    {
        W                weight;  // 边所对应权值
        size_t startIndex;  // 边起点的索引
        size_t   endIndex;  // 边终点的索引
        EdgeNode<W>*   nextNode;  // 指向下个结点
        EdgeNode(size_t start, size_t end, const W& _weight)
            : startIndex(start)
            , endIndex(end)
            , weight(_weight)
        {}
    };

    // V -- 图顶点的数据类型
    // W -- 图边权值的类型
    template <typename V, typename W, typename Z>
    class GraphLink
    {
    public:
        typedef EdgeNode<W> node;

        GraphLink(const V* _vertex, size_t _size, bool _isDirected = false);
        // 打印图中的边
        void printEdge();
        // 向图中添加一条边
        void addEdge(const Z& z1, const Z& z2, const W& weight);
        // 获取无向图
        vector<LocalMarkNode<string, Eigen::Vector3f>> GetAllNode();
        vector<node*> GetEggeLinkTable();

    private:
        // 获取顶点所在索引
        size_t getIndexOfVertex(const Z& z);
        // 添加一条边
        void __addEdge(int startIndex, int endIndex, const W& weight);

    private:
        bool         isDirected; 
        vector<LocalMarkNode<string, Eigen::Vector3f>>        vertex; // 存储所有顶点
        vector<node*> linkTable; // 存储顶点的边
    };

    void TestGraphLink();
    double GetDistanceWeight(const Eigen::Vector3f& start,const Eigen::Vector3f&end); 
    void CreateLocalMarkNodeListAndTrajectoryList(vector<LocalMarkNode<string, Eigen::Vector3f>>& node_list,
                                                vector<OneTrajectory>& trajectory_list);

}
#endif //GRAPH_H