#include "A_star.h"


namespace global_planner{
    bool AStar::cmp(const Node& node1, const Node& node2){
        if(node1.f < node2.f) return true;
        else if(node1.f > node2.f) return false;
        else{
            if(node1.h < node2.h) return true;
        }
        return false;
    }


    double AStar::get_h(string start, string end, const vector<LocalMarkNode<string, Eigen::Vector3f>>& vertex){
        Eigen::Vector2f start_point = GetPositionFromVertexs(start,vertex);
        Eigen::Vector2f end_point = GetPositionFromVertexs(end,vertex);
        return abs((end_point - start_point).norm());
    }

    Eigen::Vector2f AStar::GetPositionFromVertexs(const string& lm, const vector<LocalMarkNode<string, Eigen::Vector3f>>& vertex){
        Eigen::Vector2f point(0.0,0.0);
        for(auto node : vertex){
            if(node.id == lm){
                point.x() = node.local_point.x();
                point.y() = node.local_point.y();
                return point;
            }
        }
    }

    int AStar::GetIndexFromVertexs(const string& lm, const vector<LocalMarkNode<string, Eigen::Vector3f>>& vertexs){
        int index(0);
        for(int i = 0; i < vertexs.size(); i++){
            if(vertexs[i].id == lm){
                return i;
            }
        }
    }

    string AStar::GetNameFromIndex(const int& index, const vector<LocalMarkNode<string, Eigen::Vector3f>>& vertexs){
        string name = vertexs[index].id;
        return name;
    }

    bool AStar::node_in_list(string id, vector<Node> list){
        for(auto it = list.begin(); it != list.end(); ++it){
            if(it->id == id){
                return true;
            }
        }
        return false;
    }

    AStar::Node AStar::find_node_by_id(vector<Node> list, string id){
        for(int i = 0; i < list.size(); ++i){
            if(id == list[i].id) return list[i];
        }
        return Node(-1,-1,"-1","-1");
    }


    bool AStar::a_star(string start, string end,const vector<LocalMarkNode<string, Eigen::Vector3f>>& vertex, vector<EdgeNode<double>*> link_table){
        //check LM
        if(!CheckLM(start,vertex) || !CheckLM(end,vertex)){
            return false;
        }
        //get start to end dist
        Node start_node(0, get_h(start,end,vertex), start, start);
        openLists.push_back(start_node);
        Node center_node;
        
        while(1){
            sort(openLists.begin(),openLists.end(),AStar::cmp);
            center_node = *openLists.begin();
            openLists.erase(openLists.begin());
            closeLists.push_back(center_node);
            string temp_start = center_node.id;
            //A* core
            //遍历邻接表
            //先找到对应顶点所在的索引，从索引的数值去遍历邻接表
            int temp_start_index = GetIndexFromVertexs(temp_start,vertex);
            //vertex node name
            vertex[temp_start_index].id;
            EdgeNode<double> *pEdge = link_table[temp_start_index];

            while (pEdge)
            {
                cout << pEdge->weight << "-->[" << vertex[pEdge->endIndex].id << "]";
                int next_index = pEdge->endIndex;
                string next_name = vertex[pEdge->endIndex].id;
                if(node_in_list(next_name, closeLists)){
                    //upadte 
                    pEdge = pEdge->nextNode;
                    continue;
                } 
                if(!node_in_list(next_name,openLists)){
                    string next_node_name = GetNameFromIndex(next_index,vertex);
                    Node new_node(pEdge->weight + center_node.g, get_h(next_node_name,end,vertex),next_name ,center_node.id);
                    openLists.push_back(new_node);
                }else{
                    Node node = find_node_by_id(openLists,next_name);
                    if(center_node.g + pEdge->weight < node.g){
                        node.g = center_node.g + pEdge->weight;
                        node.father_id = center_node.id;
                        node.f = node.g + node.h;
                        sort(openLists.begin(),openLists.end(), AStar::cmp);
                    }
                }
                
                //upadte 
                pEdge = pEdge->nextNode;
            }
            if(node_in_list(end,openLists)) return true;
            if(openLists.empty()) return false; 
        }
    }

    bool AStar::CheckLM(const string& lm, const vector<LocalMarkNode<string, Eigen::Vector3f>>& vertex){
        for(auto node : vertex){
            if(node.id == lm){
                return true;
            }
        }
        return false;
    }


    void GetAStarPath(){
        //prepared data;
        // create Node
        vector<LocalMarkNode<string, Eigen::Vector3f>> node_list;
        // create path list
        vector<OneTrajectory> trajectory_list;
        CreateLocalMarkNodeListAndTrajectoryList(node_list,trajectory_list);
        // 无向图
        GraphLink<vector<LocalMarkNode<string, Eigen::Vector3f>>, double, Eigen::Vector3f> graph1(&node_list, node_list.size());
        for (int i = 0; i < trajectory_list.size(); i++)
        {
            OneTrajectory one_traj = trajectory_list[i];

            double dist_weigth = GetDistanceWeight(one_traj.start_point, one_traj.end_point);

            graph1.addEdge(one_traj.start_point, one_traj.end_point, dist_weigth);
        }
        graph1.printEdge();
        vector<LocalMarkNode<string, Eigen::Vector3f>> vertex = graph1.GetAllNode();
        cout<<"All vertex"<<endl;
        for(int i = 0; i < vertex.size(); i++){
            cout<<"Name: "<<vertex[i].id<<" Position x: "<<vertex[i].local_point.x()<<" y: "<<vertex[i].local_point.y()<<endl;
        }
        vector<GraphLink<vector<LocalMarkNode<string, Eigen::Vector3f>>, double, Eigen::Vector3f>::node*> linkTable = graph1.GetEggeLinkTable();
        vector<EdgeNode<double>*> link_table2 = linkTable;
        cout<<"All edge"<<endl;
        for (size_t idx = 0; idx < vertex.size(); ++idx)
        {
            cout << "[" << vertex[idx].id << "]";

            EdgeNode<double> *pEdge = linkTable[idx];
            while (pEdge)
            {
                cout << pEdge->weight << "-->[" << vertex[pEdge->endIndex].id << "]";
                pEdge = pEdge->nextNode;
            }
            cout << "-->NULL" << endl;
        }
        cout << endl;

        global_planner::AStar a_start;

        string start = "LM1";
        string end = "LM6";

        std::vector<double> x_path,y_path;

        if(a_start.a_star(start,end,vertex,link_table2)){
            global_planner::AStar::Node node = a_start.find_node_by_id(a_start.openLists,end);
            cout<<"最短路线长度为: "<<node.f<<endl;
            stack<string> track;
            stack<string> path_id;
            stack<double> path_cost;

            track.push(node.id);
            path_id.push(node.id);
            path_cost.push(node.g);
            
            while(node.id != start){
                node = a_start.find_node_by_id(a_start.closeLists,node.father_id);
                cout<<"node id: "<<node.id<<" ,node_father id: "<<node.father_id<<endl;
                track.push(node.id);
                path_id.push(node.id);
                path_cost.push(node.g);
            }
            cout<<"行进路线应为: ";
            cout<<track.top()<<" id: "<<path_id.top()<<" cost: "<<path_cost.top();
            Eigen::Vector2f tmp_point = a_start.GetPositionFromVertexs(path_id.top(),vertex);
            x_path.emplace_back(tmp_point.x());
            y_path.emplace_back(tmp_point.y());
            track.pop();
            path_id.pop();
            path_cost.pop();
            while(!track.empty()) {
                cout<<" -> "<<track.top()<<" id: "<<path_id.top()<<" cost: "<<path_cost.top();
                tmp_point = a_start.GetPositionFromVertexs(path_id.top(),vertex);
                x_path.emplace_back(tmp_point.x());
                y_path.emplace_back(tmp_point.y());
                track.pop();
                path_id.pop();
                path_cost.pop();
            }
            cout<<endl;
        }else{
            cout<<"search failed!"<<endl;
        }
        plotVertexs(node_list);
        plotEdges(trajectory_list);
        //plot path
        plt::figure(1);
        plt::plot(x_path,y_path,"-b");
        plt::show();

    }


    void plotVertexs(const vector<LocalMarkNode<string, Eigen::Vector3f>>& vertexs){
        std::vector<double> x,y;
        std::vector<string> mark_names;
        for(auto vertex : vertexs){
            x.emplace_back(vertex.local_point.x());
            y.emplace_back(vertex.local_point.y());
            mark_names.emplace_back(vertex.id);
        }
        plt::figure(1);
        std::map<std::string, std::string> mp;
        plt::scatter(x,y,50);
        for(int i = 0; i < mark_names.size(); i++){
            plt::annotate(mark_names[i],x[i],y[i]);
        }
        plt::grid(1);
    }

    void plotEdges(const vector<OneTrajectory>& trajectorys){
        std::vector<double> x,y;
        plt::figure(1);
        for(int i = 0; i < trajectorys.size(); i++){
            x.clear();
            y.clear();
            OneTrajectory one_traj = trajectorys[i];
            x.emplace_back(one_traj.start_point.x());
            y.emplace_back(one_traj.start_point.y());
            x.emplace_back(one_traj.end_point.x());
            y.emplace_back(one_traj.end_point.y());
            plt::plot(x,y,"--r");
        }
    }

}


