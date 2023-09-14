#pragma once
#include "global.h"
#include "obj/Design.h"
#include "GridGraph.h"
#include "GRNet.h"
#include "PatternRoute.h"

class GlobalRouter {
public:
    GlobalRouter(const Design& design, const Parameters& params);
    void route();
    void write(std::string guide_file = "");
    
private:
    const Parameters& parameters;
    GridGraph gridGraph;
    vector<GRNet> nets;
    
    int areaOfPinPatches;
    int areaOfWirePatches;
    std::map<int,std::map<int,std::vector<std::vector<std::vector<int>>>>> Empty_DGR_result;
    std::map<int,std::map<int,std::vector<std::vector<std::vector<int>>>>>& DGR_result = Empty_DGR_result;
    std::map<int,std::vector<std::shared_ptr<SteinerTreeNode>>> DGR_tree; // steiner tree selected by DGR, NetID -> PinID -> SteinerNode
    void sortNetIndices(vector<int>& netIndices) const;
    void getGuides(const GRNet& net, vector<std::pair<int, utils::BoxT<int>>>& guides);
    void printStatistics() const;
    void readDGRResult();
    void readDGRTree();
};