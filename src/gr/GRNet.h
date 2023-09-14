#pragma once
#include "global.h"
#include "obj/Design.h"
#include "GridGraph.h"

class GRNet {
public:
    GRNet(const Net& baseNet, const Design& design, const GridGraph& gridGraph);
    int getIndex() const { return index; }
    std::string getName() const { return name; }
    int getNumPins() const { return pinAccessPoints.size(); }
    const vector<vector<GRPoint>>& getPinAccessPoints() const { return pinAccessPoints; }
    const utils::BoxT<int>& getBoundingBox() const { return boundingBox; }
    const std::shared_ptr<GRTreeNode>& getRoutingTree() const { return routingTree; }
    // void getGuides(vector<std::pair<int, utils::BoxT<int>>>& guides) const;
    
    void setRoutingTree(std::shared_ptr<GRTreeNode> tree) { routingTree = tree; }
    void clearRoutingTree() { routingTree = nullptr; }
    int num_paths; // number of available paths in the graph, used for sorting nets. if DGR is not enabled, this is the same as numPins; otherwise, will be set when reading.
    int num_gcell_pins; // number of pins in the gcell tree. getNumPins() returns the physical pins. they are different. Now it is only used for sorting nets. order = num_paths - num_gcell_pins.
private:
    int index;
    std::string name;
    vector<vector<GRPoint>> pinAccessPoints;
    utils::BoxT<int> boundingBox;
    std::shared_ptr<GRTreeNode> routingTree;
};