#ifndef A_STAR_H
#define A_STAR_H
#include <iostream>
#include <Eigen/Core>
#include "matplotlibcpp.h"

#include <fstream>
#include <vector>
#include <string>
#include <cmath>
#include <algorithm>
#include <stack>
#include <time.h>

#include "graph.h"

using namespace std;
namespace plt = matplotlibcpp;

namespace global_planner{
    class AStar{
    public:
        struct Node{
            double f;//path all length f = g + h
            double g;//cost
            double h;//euler distance
            string id;//the site id
            string father_id;//the father site id
            Node(){
                f = 0;
                g = 0;
                h = 0;
                id = -1;
                father_id = -1;
            }
            Node(double g, double h, string id, string father_id){
                this->h = h;
                this->g = g;
                this->id = id;
                this->f = g + h;
                this->father_id = father_id;
            }
        };

        //compare fucntion
        static bool cmp(const Node& node1, const Node& node2);

        bool node_in_list(string id, vector<Node> list);
        Node find_node_by_id(vector<Node> list, string id);

        bool a_star(string start, string end,const vector<LocalMarkNode<string, Eigen::Vector3f>>& vertex, vector<EdgeNode<double>*> link_table);//start_lm end_lm
        bool CheckLM(const string& lm, const vector<LocalMarkNode<string, Eigen::Vector3f>>& vertex);
        Eigen::Vector2f GetPositionFromVertexs(const string& lm, const vector<LocalMarkNode<string, Eigen::Vector3f>>& vertex);
        
        double get_h(string start, string end,const vector<LocalMarkNode<string, Eigen::Vector3f>>& vertex);
        int GetIndexFromVertexs(const string& lm, const vector<LocalMarkNode<string, Eigen::Vector3f>>& vertexs);
        string GetNameFromIndex(const int& index, const vector<LocalMarkNode<string, Eigen::Vector3f>>& vertexs);

    public:
        vector<Node> openLists;
        vector<Node> closeLists;
    };

    void GetAStarPath();
    void plotVertexs(const vector<LocalMarkNode<string, Eigen::Vector3f>>& vertexs);
    void plotEdges(const vector<OneTrajectory>& trajectorys);

}




#endif