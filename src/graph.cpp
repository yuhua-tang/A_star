#include "graph.h"
#include <string>
#include <vector>
#include <iostream>
#include <cstring>


namespace global_planner{
/*
 *   public 函数
 */

    template <typename V, typename W, typename Z>
    GraphLink<V, W, Z>::GraphLink(const V *_vertex, size_t _size, bool _isDirected)
    {
        // 开辟空间并初始化数据
        this->vertex.resize(_size);
        this->linkTable.resize(_size);
        this->isDirected = _isDirected;

        for (size_t i = 0; i < _size; i++)
        {
            this->vertex[i] = (*_vertex)[i];
            // this->vertex[i].id = *_vertex->[i].id;
            // this->vertex[i].local_point = *_vertex->[i].local_point;
        }
    }

    template <typename V, typename W, typename Z>
    void GraphLink<V, W, Z>::printEdge()
    {
        for (size_t idx = 0; idx < vertex.size(); ++idx)
        {
            cout << "[" << vertex[idx].id << "]";

            node *pEdge = linkTable[idx];
            while (pEdge)
            {
                cout << pEdge->weight << "-->[" << vertex[pEdge->endIndex].id << "]";
                pEdge = pEdge->nextNode;
            }
            cout << "-->NULL" << endl;
        }
        cout << endl;
    }

    template <typename V, typename W, typename Z>
    vector<LocalMarkNode<string, Eigen::Vector3f>> GraphLink<V, W, Z>::GetAllNode()
    {
        vector<LocalMarkNode<string, Eigen::Vector3f>> ans = vertex;
        return ans;
    }

    template <typename V, typename W, typename Z>
    vector<typename GraphLink<V, W, Z>::node *> GraphLink<V, W, Z>::GetEggeLinkTable()
    {
        return linkTable;
    }

    template <typename V, typename W, typename Z>
    void GraphLink<V, W, Z>::addEdge(const Z &z1, const Z &z2, const W &weight)
    {
        size_t startIndex = getIndexOfVertex(z1);
        size_t endIndex = getIndexOfVertex(z2);

        // 防止填加自己指向自己的边
        assert(startIndex != endIndex);

        __addEdge(startIndex, endIndex, weight);

        // 无向图需要添加对称的一条边
        if (!isDirected)
            __addEdge(endIndex, startIndex, weight);
    }

    /*
    *   private 函数
    */

    template <typename V, typename W, typename Z>
    void GraphLink<V, W, Z>::__addEdge(int startIndex, int endIndex, const W &weight)
    {
        // 头插的方式添加边到链表中
        node *pNewEdge = new node(startIndex, endIndex, weight);
        pNewEdge->nextNode = linkTable[startIndex];
        linkTable[startIndex] = pNewEdge;
    }

    template <typename V, typename W, typename Z>
    size_t GraphLink<V, W, Z>::getIndexOfVertex(const Z &z)
    {
        for (int idx = 0; idx < vertex.size(); idx++)
        {
            if (vertex[idx].local_point == z)
                return idx;
        }

        // 如果没有找到就说明发生了错误
        assert(false);
        return -1;
    }

    double GetDistanceWeight(const Eigen::Vector3f &start, const Eigen::Vector3f &end)
    {
        return sqrt(pow((end.x() - start.x()), 2) + pow((end.y() - start.y()), 2));
    }

    void CreateLocalMarkNodeListAndTrajectoryList(vector<LocalMarkNode<string, Eigen::Vector3f>> &node_list,
                                                vector<OneTrajectory> &trajectory_list)
    {
        // Node
        // string LM2 = "LM2";
        // Eigen::Vector3f LM2_Point(90.0, 0.0, 0.0);
        // LocalMarkNode<string, Eigen::Vector3f> LMNode2(LM2, LM2_Point);
        // node_list.emplace_back(LMNode2);
        // string LM3 = "LM3";
        // Eigen::Vector3f LM3_Point(50.0, 20.0, 0.0);
        // LocalMarkNode<string, Eigen::Vector3f> LMNode3(LM3, LM3_Point);
        // node_list.emplace_back(LMNode3);
        // string LM4 = "LM4";
        // Eigen::Vector3f LM4_Point(70.0, 35.0, 0.0);
        // LocalMarkNode<string, Eigen::Vector3f> LMNode4(LM4, LM4_Point);
        // node_list.emplace_back(LMNode4);
        // string LM5 = "LM5";
        // Eigen::Vector3f LM5_Point(-10, -35.0, 0.0);
        // LocalMarkNode<string, Eigen::Vector3f> LMNode5(LM5, LM5_Point);
        // node_list.emplace_back(LMNode5);
        // string LM6 = "LM6";
        // Eigen::Vector3f LM6_Point(-75, 20.0, 0.0);
        // LocalMarkNode<string, Eigen::Vector3f> LMNode6(LM6, LM6_Point);
        // node_list.emplace_back(LMNode6);
        // string LM1 = "LM1";
        // Eigen::Vector3f LM1_Point(0.0, 0.0, 0.0);
        // LocalMarkNode<string, Eigen::Vector3f> LMNode1(LM1, LM1_Point);
        // node_list.emplace_back(LMNode1);
        string LM1 = "LM1";
        Eigen::Vector3f LM1_Point(2.97232, 1.34877, 0.0);
        LocalMarkNode<string, Eigen::Vector3f> LMNode1(LM1, LM1_Point);
        node_list.emplace_back(LMNode1);
        string LM2 = "LM2";
        Eigen::Vector3f LM2_Point(0.408456, 1.13672, 0.0);
        LocalMarkNode<string, Eigen::Vector3f> LMNode2(LM2, LM2_Point);
        node_list.emplace_back(LMNode2);
        string LM3 = "LM3";
        Eigen::Vector3f LM3_Point(0.466288, -1.15726, 0.0);
        LocalMarkNode<string, Eigen::Vector3f> LMNode3(LM3, LM3_Point);
        node_list.emplace_back(LMNode3);
        string LM0 = "LM0";
        Eigen::Vector3f LM0_Point(0.0, 0.0, 0.0);
        LocalMarkNode<string, Eigen::Vector3f> LMNode0(LM0, LM0_Point);
        node_list.emplace_back(LMNode0);
        // Path list
        // OneTrajectory ot1(LM1_Point, LM2_Point);
        // trajectory_list.emplace_back(ot1);
        // OneTrajectory ot1_1(LM2_Point, LM1_Point);
        // trajectory_list.emplace_back(ot1_1);
        // OneTrajectory ot2(LM6_Point, LM4_Point);
        // trajectory_list.emplace_back(ot2);
        // OneTrajectory ot2_2(LM4_Point, LM6_Point);
        // trajectory_list.emplace_back(ot2_2);
        // OneTrajectory ot3(LM5_Point, LM3_Point);
        // trajectory_list.emplace_back(ot3);
        // OneTrajectory ot3_3(LM3_Point, LM5_Point);
        // trajectory_list.emplace_back(ot3_3);
        // OneTrajectory ot4(LM4_Point, LM2_Point);
        // trajectory_list.emplace_back(ot4);
        // OneTrajectory ot4_4(LM2_Point, LM4_Point);
        // trajectory_list.emplace_back(ot4_4);
        // OneTrajectory ot5(LM1_Point, LM3_Point);
        // trajectory_list.emplace_back(ot5);
        // OneTrajectory ot5_5(LM3_Point, LM1_Point);
        // trajectory_list.emplace_back(ot5_5);
        // OneTrajectory ot6(LM5_Point, LM6_Point);
        // trajectory_list.emplace_back(ot6);
        // OneTrajectory ot7(LM6_Point, LM5_Point);
        // trajectory_list.emplace_back(ot7);

        OneTrajectory ot1(LM1_Point, LM2_Point);
        trajectory_list.emplace_back(ot1);
        OneTrajectory ot2(LM2_Point, LM1_Point);
        trajectory_list.emplace_back(ot2);
        OneTrajectory ot3(LM2_Point, LM3_Point);
        trajectory_list.emplace_back(ot3);
        OneTrajectory ot4(LM3_Point, LM2_Point);
        trajectory_list.emplace_back(ot4);

    }

    void TestGraphLink()
    {
        // // 无向图
        // char* str = "ABCDE";
        // GraphLink<char, int> graph1(str, std::strlen(str));
        // graph1.addEdge('A', 'C', 2);
        // graph1.addEdge('D', 'B', 6);
        // graph1.addEdge('A', 'B', 4);
        // graph1.addEdge('E', 'D', 9);
        // graph1.printEdge();

        // cout << "------------" << endl;
        // // 有向图
        // GraphLink<char, int> graph2(str, std::strlen(str), true);
        // graph2.addEdge('D', 'C', 2);
        // graph2.addEdge('B', 'E', 6);
        // graph2.addEdge('A', 'D', 4);
        // graph2.addEdge('A', 'C', 4);
        // graph2.addEdge('E', 'D', 9);
        // graph2.addEdge('E', 'A', 5);
        // graph2.printEdge();

        // test data struct
        // zhandian ["LM1" start_point end_point] ["LM2" start_point end_point]
        // quxian [start_point end_point control_1 control_2] [start_point end_point control_1 control_2] [start_point end_point control_1 control_2] ...
        // find adj tab to use A*

        // create Node
        vector<LocalMarkNode<string, Eigen::Vector3f>> node_list;
        // create path list
        vector<OneTrajectory> trajectory_list;

        CreateLocalMarkNodeListAndTrajectoryList(node_list, trajectory_list);

        // 无向图
        GraphLink<vector<LocalMarkNode<string, Eigen::Vector3f>>, double, Eigen::Vector3f> graph1(&node_list, node_list.size());
        for (int i = 0; i < trajectory_list.size(); i++)
        {
            OneTrajectory one_traj = trajectory_list[i];

            double dist_weigth = GetDistanceWeight(one_traj.start_point, one_traj.end_point);

            graph1.addEdge(one_traj.start_point, one_traj.end_point, dist_weigth);
        }
        graph1.printEdge();

        // no direction
    }

    template class GraphLink<vector<LocalMarkNode<string, Eigen::Vector3f>>, double, Eigen::Vector3f>;

}