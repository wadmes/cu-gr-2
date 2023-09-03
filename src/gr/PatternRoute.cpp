#include "PatternRoute.h"

void SteinerTreeNode::preorder(
    std::shared_ptr<SteinerTreeNode> node, 
    std::function<void(std::shared_ptr<SteinerTreeNode>)> visit
) {
    visit(node);
    for (auto& child : node->children) preorder(child, visit);
}

std::string SteinerTreeNode::getPythonString(std::shared_ptr<SteinerTreeNode> node) {
    vector<std::pair<utils::PointT<int>, utils::PointT<int>>> edges;
    preorder(node, [&] (std::shared_ptr<SteinerTreeNode> node) {
        for (auto child : node->children) {
            edges.emplace_back(*node, *child);
        }
    });
    std::stringstream ss;
    ss << "[";
    for (int i = 0; i < edges.size(); i++) {
        auto& edge = edges[i];
        ss << "[" << edge.first << ", " << edge.second << "]";
        ss << (i < edges.size() - 1 ? ", " : "]");
    }
    return ss.str();
}

std::string PatternRoutingNode::getPythonString(std::shared_ptr<PatternRoutingNode> routingDag) {
    vector<std::pair<utils::PointT<int>, utils::PointT<int>>> edges;
    std::function<void(std::shared_ptr<PatternRoutingNode>)> getEdges = 
        [&] (std::shared_ptr<PatternRoutingNode> node) {
            for (auto& childPaths : node->paths) {
                for (auto& path : childPaths) {
                    edges.emplace_back(*node, *path);
                    getEdges(path);
                }
            }
        };
    getEdges(routingDag);
    std::stringstream ss;
    ss << "[";
    for (int i = 0; i < edges.size(); i++) {
        auto& edge = edges[i];
        ss << "[" << edge.first << ", " << edge.second << "]";
        ss << (i < edges.size() - 1 ? ", " : "]");
    }
    return ss.str();
}

void PatternRoute::constructSteinerTree() {
    // 1. Select access points
    robin_hood::unordered_map<uint64_t, std::pair<utils::PointT<int>, utils::IntervalT<int>>> selectedAccessPoints;
    gridGraph.selectAccessPoints(net, selectedAccessPoints);
    
    // 2. Construct Steiner tree
    const int degree = selectedAccessPoints.size();
    if (degree == 1) {
        for (auto& accessPoint : selectedAccessPoints) {
            steinerTree = std::make_shared<SteinerTreeNode>(accessPoint.second.first, accessPoint.second.second);
        }
    } else {
        int xs[degree * 4];
        int ys[degree * 4];
        int i = 0;
        for (auto& accessPoint : selectedAccessPoints) {
            xs[i] = accessPoint.second.first.x;
            ys[i] = accessPoint.second.first.y;
            i++;
        }
        Tree flutetree = flute(degree, xs, ys, ACCURACY);
        const int numBranches = degree + degree - 2;
        vector<utils::PointT<int>> steinerPoints;
        steinerPoints.reserve(numBranches);
        vector<vector<int>> adjacentList(numBranches);
        for (int branchIndex = 0; branchIndex < numBranches; branchIndex++) {
            const Branch& branch = flutetree.branch[branchIndex];
            steinerPoints.emplace_back(branch.x, branch.y);
            if (branchIndex == branch.n) continue;
            adjacentList[branchIndex].push_back(branch.n);
            adjacentList[branch.n].push_back(branchIndex);
        }
        std::function<void(std::shared_ptr<SteinerTreeNode>&, int, int)> constructTree = [&] (
            std::shared_ptr<SteinerTreeNode>& parent, int prevIndex, int curIndex
        ) {
            std::shared_ptr<SteinerTreeNode> current = std::make_shared<SteinerTreeNode>(steinerPoints[curIndex]);
            if (parent != nullptr && parent->x == current->x && parent->y == current->y) {
                for (int nextIndex : adjacentList[curIndex]) {
                    if (nextIndex == prevIndex) continue;
                    constructTree(parent, curIndex, nextIndex);
                }
                return;
            }
            // Build subtree
            for (int nextIndex : adjacentList[curIndex]) {
                if (nextIndex == prevIndex) continue;
                constructTree(current, curIndex, nextIndex);
            }
            // Set fixed layer interval
            uint64_t hash = gridGraph.hashCell(current->x, current->y);
            if (selectedAccessPoints.find(hash) != selectedAccessPoints.end()) {
                current->fixedLayers = selectedAccessPoints[hash].second;
            }
            // Connect current to parent
            if (parent == nullptr) {
                parent = current;
            } else {
                parent->children.emplace_back(current);
            }
        };
        // Pick a root having degree 1
        int root = 0;
        std::function<bool(int)> hasDegree1 = [&] (int index) {
            if (adjacentList[index].size() == 1) {
                int nextIndex = adjacentList[index][0];
                if (steinerPoints[index] == steinerPoints[nextIndex]) {
                    return hasDegree1(nextIndex);
                } else {
                    return true;
                }
            } else {
                return false;
            }
        };
        for (int i = 0; i < steinerPoints.size(); i++) {
            if (hasDegree1(i)) {
                root = i;
                break;
            }
        }
        constructTree(steinerTree, -1, root);
    }
}

void PatternRoute::constructRoutingDAG() {
    std::function<void(std::shared_ptr<PatternRoutingNode>&, std::shared_ptr<SteinerTreeNode>&)> constructDag = [&] (
        std::shared_ptr<PatternRoutingNode>& dstNode, std::shared_ptr<SteinerTreeNode>& steiner
    ) {
        std::shared_ptr<PatternRoutingNode> current = std::make_shared<PatternRoutingNode>(
            *steiner, steiner->fixedLayers, numDagNodes++
        );
        for (auto steinerChild : steiner->children) {
            constructDag(current, steinerChild);
        }
        if (dstNode == nullptr) {
            dstNode = current;
        } else {
            dstNode->children.emplace_back(current);
            constructPaths(dstNode, current);
        }
    };
    constructDag(routingDag, steinerTree);
}

void PatternRoute::constructDAGFromDGR(){
    numSteinerNodes = 0;
    std::function<void(std::shared_ptr<PatternRoutingNode>&, std::shared_ptr<SteinerTreeNode>&, int)> constructDagFromDGR = [&] (
            std::shared_ptr<PatternRoutingNode>& dstNode, std::shared_ptr<SteinerTreeNode>& steiner,int parent
        ) {
            std::shared_ptr<PatternRoutingNode> current = std::make_shared<PatternRoutingNode>(
                *steiner, steiner->fixedLayers, numDagNodes++
            );
            int current_idx = numSteinerNodes;
            numSteinerNodes += 1;
            for (auto steinerChild : steiner->children) {
                constructDagFromDGR(current, steinerChild,current_idx);
            }
            if (dstNode == nullptr) {
                dstNode = current;
            } else {
                dstNode->children.emplace_back(current);
                constructPathsFromDGR(dstNode, current,current_idx);
            }
        };
    constructDagFromDGR(routingDag, steinerTree,-1);
    // std::cout << net.getName()<< " " << net.getIndex()<< " numSteinerNodes: " << numSteinerNodes << std::endl;
}


// current_idx here is the child pin index
void PatternRoute::constructPathsFromDGR(std::shared_ptr<PatternRoutingNode>& start, std::shared_ptr<PatternRoutingNode>& end,int current_idx) {
    std::vector<std::vector<std::vector<int>>>& DGRPath = DGR_net_result[current_idx]; // DGR_net_result: pin_idx -> path_idx -> node_idx -> node coordinates
    
    int childIndex = start->paths.size();
    // std::cout << net.getIndex() << " " << current_idx << " " << DGRPath.size() <<" " << start->x << " " << start->y << " " << end->x << " " << end->y << " " << childIndex<<std::endl;
    start->paths.emplace_back();
    vector<std::shared_ptr<PatternRoutingNode>>& childPaths = start->paths[childIndex]; // childPaths: path_idx -> first routing node of this routing path
    // DGRPath is empty: not exists in DGR file; DGRPath[0] is empty: No additional node; assert start->x == end->x || start->y == end->y
    if (DGRPath.empty() || DGRPath[0].empty()) {
        assert(start->x == end->x || start->y == end->y);
        childPaths.push_back(end);
        } 
    else {
        assert(start->x != end->x && start->y != end->y);
        assert(DGRPath.size() == 2);
        for (int pathIndex = 0; pathIndex < DGRPath.size(); pathIndex++) {
            vector<std::shared_ptr<PatternRoutingNode>>* last_path = &childPaths;
            for (int nodeIdx = 0; nodeIdx < DGRPath[pathIndex].size(); nodeIdx++) {
                utils::PointT<int> midPoint = utils::PointT<int>(DGRPath[pathIndex][nodeIdx][0], DGRPath[pathIndex][nodeIdx][1]);
                // std::cout<< "midPoint: " << midPoint << std::endl;
                assert(DGRPath[pathIndex][nodeIdx][0] == start -> x || DGRPath[pathIndex][nodeIdx][1] == start -> y);
                assert(DGRPath[pathIndex][nodeIdx][0] == end -> x || DGRPath[pathIndex][nodeIdx][1] == end -> y);
                std::shared_ptr<PatternRoutingNode> mid = std::make_shared<PatternRoutingNode>(midPoint, numDagNodes++, true);
                (*last_path).push_back(mid);
                mid->paths.emplace_back();
                last_path = &(mid->paths[0]);
                // std::cout << "Path pushed" << std::endl;
            }
            (*last_path).push_back(end);
            // std::cout << *((*last_path)[0]) << std::endl;
        }
        // print path info from start to end
        // std::cout<< "start: " << start->x << " " << start->y << std::endl;
        // for (int pathIdx = 0; pathIdx < start->paths[childIndex].size(); pathIdx++) {
        //     std::cout << "pathIdx: " << pathIdx << std::endl;
        //     std::cout <<"this path "<< pathIdx<<" "<< start->paths[childIndex][pathIdx]->x << " " << start->paths[childIndex][pathIdx]->y << std::endl;
        //     std::cout << (start->paths[childIndex][pathIdx])->paths.size() << std::endl;
        //     std::cout << (start->paths[childIndex][pathIdx])->paths[0].size()<< std::endl;
        //     std::cout <<" end node "<<  *((start->paths[childIndex][pathIdx])->paths[0][0]) << std::endl;
        // }
    }
}

/*
default childIndex is -1, which means no path is constructed at this moment
*/
void PatternRoute::constructPaths(std::shared_ptr<PatternRoutingNode>& start, std::shared_ptr<PatternRoutingNode>& end, int childIndex) {
    if (childIndex == -1) {
        childIndex = start->paths.size();
        start->paths.emplace_back(); // initialize an empty path object
    }
    vector<std::shared_ptr<PatternRoutingNode>>& childPaths = start->paths[childIndex]; // paths is encoded by childIndex -> pathIndex -> path, so here, childPaths means all paths from start to this end child node, note that "path" is encoded by the next node, (not necessarily to encode all nodes)
    if (start->x == end->x || start->y == end->y) {
        childPaths.push_back(end); // if the two nodes are on the same 2D gcell node, then there is only one path
    } else {
        for (int pathIndex = 0; pathIndex <= 1; pathIndex++) { // two paths of different L-shape
            utils::PointT<int> midPoint = pathIndex ? utils::PointT<int>(start->x, end->y) : utils::PointT<int>(end->x, start->y);
            std::shared_ptr<PatternRoutingNode> mid = std::make_shared<PatternRoutingNode>(midPoint, numDagNodes++, true);
            mid->paths = {{end}};
            childPaths.push_back(mid);
        }
    }
}

void PatternRoute::constructDetours(GridGraphView<bool>& congestionView) {
    struct ScaffoldNode {
        std::shared_ptr<PatternRoutingNode> node;
        vector<std::shared_ptr<ScaffoldNode>> children;
        ScaffoldNode(std::shared_ptr<PatternRoutingNode> n): node(n) {}
    };
    
    vector<vector<std::shared_ptr<ScaffoldNode>>> scaffolds(2);
    vector<vector<std::shared_ptr<ScaffoldNode>>> scaffoldNodes(
        2, vector<std::shared_ptr<ScaffoldNode>>(numDagNodes, nullptr)
    ); // direction -> numDagNodes -> scaffold node
    vector<bool> visited(numDagNodes, false);
    
    std::function<void(std::shared_ptr<PatternRoutingNode>)> buildScaffolds = 
        [&] (std::shared_ptr<PatternRoutingNode> node) {
            if (visited[node->index]) return;
            visited[node->index] = true;
            
            if (node->optional) {
                assert(node->paths.size() == 1 && node->paths[0].size() == 1 && !node->paths[0][0]->optional);
                auto& path = node->paths[0][0];
                buildScaffolds(path);
                unsigned direction = (node->y == path->y ? MetalLayer::H : MetalLayer::V);
                if (!scaffoldNodes[direction][path->index] && congestionView.check(*node, *path)) {
                    scaffoldNodes[direction][path->index] = std::make_shared<ScaffoldNode>(path);
                }
            } else {
                for (auto& childPaths : node->paths) {
                    for (auto& path : childPaths) {
                        buildScaffolds(path);
                        unsigned direction = (node->y == path->y ? MetalLayer::H : MetalLayer::V);
                        if (path->optional) {
                            if (!scaffoldNodes[direction][node->index] && congestionView.check(*node, *path)) {
                                scaffoldNodes[direction][node->index] = std::make_shared<ScaffoldNode>(node);
                            }
                        } else {
                            if (congestionView.check(*node, *path)) {
                                if (!scaffoldNodes[direction][node->index]) {
                                    scaffoldNodes[direction][node->index] = std::make_shared<ScaffoldNode>(node);
                                }
                                if (!scaffoldNodes[direction][path->index]) {
                                    scaffoldNodes[direction][node->index]->children.emplace_back(std::make_shared<ScaffoldNode>(path));
                                } else {
                                    scaffoldNodes[direction][node->index]->children.emplace_back(scaffoldNodes[direction][path->index]);
                                    scaffoldNodes[direction][path->index] = nullptr;
                                }
                            }
                        }
                    }
                    for (auto& child : node->children) {
                        for (unsigned direction = 0; direction < 2; direction++) {
                            if (scaffoldNodes[direction][child->index]) {
                                scaffolds[direction].emplace_back(std::make_shared<ScaffoldNode>(node));
                                scaffolds[direction].back()->children.emplace_back(scaffoldNodes[direction][child->index]);
                                scaffoldNodes[direction][child->index] = nullptr;
                            }
                        }
                    }
                }
            }
        };
        
    buildScaffolds(routingDag);
    for (unsigned direction = 0; direction < 2; direction++) {
        if (scaffoldNodes[direction][routingDag->index]) {
            scaffolds[direction].emplace_back(std::make_shared<ScaffoldNode>(nullptr));
            scaffolds[direction].back()->children.emplace_back(scaffoldNodes[direction][routingDag->index]);
        }
    }
    
    
    
    std::function<void(std::shared_ptr<ScaffoldNode>, utils::IntervalT<int>&, vector<int>&, unsigned, bool)> getTrunkAndStems = 
        [&] (std::shared_ptr<ScaffoldNode> scaffoldNode, utils::IntervalT<int>& trunk, vector<int>& stems, unsigned direction, bool starting) {
            if (starting) {
                if (scaffoldNode->node) {
                    stems.emplace_back((*scaffoldNode->node)[1 - direction]);
                    trunk.Update((*scaffoldNode->node)[direction]);
                }
                for (auto& scaffoldChild : scaffoldNode->children) getTrunkAndStems(scaffoldChild, trunk, stems, direction, false);
            } else {
                trunk.Update((*scaffoldNode->node)[direction]);
                if (scaffoldNode->node->fixedLayers.IsValid()) {
                    stems.emplace_back((*scaffoldNode->node)[1 - direction]);
                }
                for (auto& treeChild : scaffoldNode->node->children) {
                    bool scaffolded = false;
                    for (auto& scaffoldChild : scaffoldNode->children) {
                        if (treeChild == scaffoldChild->node) {
                            getTrunkAndStems(scaffoldChild, trunk, stems, direction, false);
                            scaffolded = true;
                            break;
                        }
                    }
                    if (!scaffolded) {
                        stems.emplace_back((*treeChild)[1 - direction]);
                        trunk.Update((*treeChild)[direction]);
                    }
                }
            }
        };
    
    auto getTotalStemLength = [&] (const vector<int>& stems, const int pos) {
        int length = 0;
        for (int stem : stems) length += abs(stem - pos);
        return length;
    };
    
    std::function<std::shared_ptr<PatternRoutingNode>(std::shared_ptr<ScaffoldNode>, unsigned, int)> buildDetour = 
        [&] (std::shared_ptr<ScaffoldNode> scaffoldNode, unsigned direction, int shiftAmount) {
            std::shared_ptr<PatternRoutingNode> treeNode = scaffoldNode->node;
            if (treeNode->fixedLayers.IsValid()) {
                std::shared_ptr<PatternRoutingNode> dupTreeNode = 
                    std::make_shared<PatternRoutingNode>((utils::PointT<int>)*treeNode, treeNode->fixedLayers, numDagNodes++);
                std::shared_ptr<PatternRoutingNode> shiftedTreeNode = 
                    std::make_shared<PatternRoutingNode>((utils::PointT<int>)*treeNode, numDagNodes++);
                (*shiftedTreeNode)[1 - direction] += shiftAmount;
                constructPaths(shiftedTreeNode, dupTreeNode);
                for (auto& treeChild : treeNode->children) {
                    bool built = false;
                    for (auto& scaffoldChild : scaffoldNode->children) {
                        if (treeChild == scaffoldChild->node) {
                            auto shiftedChildTreeNode = buildDetour(scaffoldChild, direction, shiftAmount);
                            constructPaths(shiftedTreeNode, shiftedChildTreeNode);
                            built = true;
                            break;
                        }
                    }
                    if (!built) {
                        constructPaths(shiftedTreeNode, treeChild);
                    }
                }
                return shiftedTreeNode;
            } else {
                std::shared_ptr<PatternRoutingNode> shiftedTreeNode = 
                    std::make_shared<PatternRoutingNode>((utils::PointT<int>)*treeNode, numDagNodes++);
                (*shiftedTreeNode)[1 - direction] += shiftAmount;
                for (auto& treeChild : treeNode->children) {
                    bool built = false;
                    for (auto& scaffoldChild : scaffoldNode->children) {
                        if (treeChild == scaffoldChild->node) {
                            auto shiftedChildTreeNode = buildDetour(scaffoldChild, direction, shiftAmount);
                            constructPaths(shiftedTreeNode, shiftedChildTreeNode);
                            built = true;
                            break;
                        }
                    }
                    if (!built) {
                        constructPaths(shiftedTreeNode, treeChild);
                    }
                }
                return shiftedTreeNode;
            }
        };
        
    for (unsigned direction = 0; direction < 2; direction++) {
        for (std::shared_ptr<ScaffoldNode> scaffold : scaffolds[direction]) {
            assert (scaffold->children.size() == 1);
            
            utils::IntervalT<int> trunk;
            vector<int> stems;
            getTrunkAndStems(scaffold, trunk, stems, direction, true);
            std::sort(stems.begin(), stems.end());
            int trunkPos = (*scaffold->children[0]->node)[1 - direction];
            int originalLength = getTotalStemLength(stems, trunkPos);
            utils::IntervalT<int> shiftInterval(trunkPos);
            int maxLengthIncrease = trunk.range() * parameters.max_detour_ratio;
            while (shiftInterval.low - 1 >= 0 && getTotalStemLength(stems, shiftInterval.low - 1) - originalLength <= maxLengthIncrease) shiftInterval.low--;
            while (shiftInterval.high + 1 < gridGraph.getSize(1 - direction) && getTotalStemLength(stems, shiftInterval.high - 1) - originalLength <= maxLengthIncrease) shiftInterval.high++;
            int step = 1;
            while ((trunkPos - shiftInterval.low) / (step + 1) + (shiftInterval.high - trunkPos) / (step + 1) >= parameters.target_detour_count) step++;
            utils::IntervalT<int> dupShiftInterval = shiftInterval;
            shiftInterval.low = trunkPos - (trunkPos - shiftInterval.low) / step * step;
            shiftInterval.high = trunkPos + (shiftInterval.high - trunkPos) / step * step;
            for (double pos = shiftInterval.low; pos <= shiftInterval.high; pos += step) {
                int shiftAmount = (pos - trunkPos); 
                if (shiftAmount == 0) continue;
                if (scaffold->node) {
                    auto& scaffoldChild = scaffold->children[0];
                    if ((*scaffoldChild->node)[1 - direction] + shiftAmount < 0 || 
                        (*scaffoldChild->node)[1 - direction] + shiftAmount >= gridGraph.getSize(1 - direction)) {
                        continue;
                    }
                    for (int childIndex = 0; childIndex < scaffold->node->children.size(); childIndex++) {
                        auto& treeChild = scaffold->node->children[childIndex];
                        if (treeChild == scaffoldChild->node) {
                            std::shared_ptr<PatternRoutingNode> shiftedChild = buildDetour(scaffoldChild, direction, shiftAmount);
                            constructPaths(scaffold->node, shiftedChild, childIndex);
                        }
                    }
                } else {
                    std::shared_ptr<ScaffoldNode> scaffoldNode = scaffold->children[0];
                    auto treeNode = scaffoldNode->node;
                    if (treeNode->children.size() == 1) {
                        if ((*treeNode)[1 - direction] + shiftAmount < 0 || 
                            (*treeNode)[1 - direction] + shiftAmount >= gridGraph.getSize(1 - direction)) {
                            continue;
                        }
                        std::shared_ptr<PatternRoutingNode> shiftedTreeNode = 
                            std::make_shared<PatternRoutingNode>((utils::PointT<int>)*treeNode, numDagNodes++);
                        (*shiftedTreeNode)[1 - direction] += shiftAmount;
                        constructPaths(treeNode, shiftedTreeNode, 0);
                        for (auto& treeChild : treeNode->children) {
                            bool built = false;
                            for (auto& scaffoldChild : scaffoldNode->children) {
                                if (treeChild == scaffoldChild->node) {
                                    auto shiftedChildTreeNode = buildDetour(scaffoldChild, direction, shiftAmount);
                                    constructPaths(shiftedTreeNode, shiftedChildTreeNode);
                                    built = true;
                                    break;
                                }
                            }
                            if (!built) {
                                constructPaths(shiftedTreeNode, treeChild);
                            }
                        }
                    
                    } else {
                        log() << "Warning: the root has not exactly one child." << std::endl;
                    }
                }
            }
        }
    }
}

void PatternRoute::run() {
    calculateRoutingCosts(routingDag);
    net.setRoutingTree(getRoutingTree(routingDag));
}

void PatternRoute::calculateRoutingCosts(std::shared_ptr<PatternRoutingNode>& node) {
    if (node->costs.size() != 0) return;
    vector<vector<std::pair<CostT, int>>> childCosts; // childIndex -> layerIndex -> (cost, pathIndex)
    // Calculate child costs
    if (node->paths.size() > 0) childCosts.resize(node->paths.size());
    for (int childIndex = 0; childIndex < node->paths.size(); childIndex++) {
        auto& childPaths = node->paths[childIndex];
        auto& costs = childCosts[childIndex];
        costs.assign(gridGraph.getNumLayers(), {std::numeric_limits<CostT>::max(), -1});
        for (int pathIndex = 0; pathIndex < childPaths.size(); pathIndex++) {
            std::shared_ptr<PatternRoutingNode>& path = childPaths[pathIndex];
            // std::cout << path->x << " " << path->y << " " << node->x << " " << node->y << std::endl;
            calculateRoutingCosts(path);
            unsigned direction = node->x == path->x ? MetalLayer::V : MetalLayer::H;
            assert((*node)[1 - direction] == (*path)[1 - direction]);
            for (int layerIndex = parameters.min_routing_layer; layerIndex < gridGraph.getNumLayers(); layerIndex++) {
                if (gridGraph.getLayerDirection(layerIndex) != direction) continue;
                CostT cost = path->costs[layerIndex] + gridGraph.getWireCost(layerIndex, *node, *path);
                if (cost < costs[layerIndex].first) costs[layerIndex] = std::make_pair(cost, pathIndex);
            }
        }
    }
    
    node->costs.assign(gridGraph.getNumLayers(), std::numeric_limits<CostT>::max());
    node->bestPaths.resize(gridGraph.getNumLayers());
    if (node->paths.size() > 0) {
        for (int layerIndex = 1; layerIndex < gridGraph.getNumLayers(); layerIndex++) {
            node->bestPaths[layerIndex].assign(node->paths.size(), {-1, -1});
        }
    }
    // Calculate the partial sum of the via costs
    vector<CostT> viaCosts(gridGraph.getNumLayers());
    viaCosts[0] = 0;
    for (int layerIndex = 1; layerIndex < gridGraph.getNumLayers(); layerIndex++) {
        viaCosts[layerIndex] = viaCosts[layerIndex - 1] + gridGraph.getViaCost(layerIndex - 1, *node);
    }
    utils::IntervalT<int> fixedLayers = node->fixedLayers;
    fixedLayers.low = min(fixedLayers.low, static_cast<int>(gridGraph.getNumLayers()) - 1);
    fixedLayers.high = max(fixedLayers.high, parameters.min_routing_layer);
    
    for (int lowLayerIndex = 0; lowLayerIndex <= fixedLayers.low; lowLayerIndex++) {
        vector<CostT> minChildCosts; 
        vector<std::pair<int, int>> bestPaths; 
        if (node->paths.size() > 0) {
            minChildCosts.assign(node->paths.size(), std::numeric_limits<CostT>::max());
            bestPaths.assign(node->paths.size(), {-1, -1});
        }
        for (int layerIndex = lowLayerIndex; layerIndex < gridGraph.getNumLayers(); layerIndex++) {
            for (int childIndex = 0; childIndex < node->paths.size(); childIndex++) {
                if (childCosts[childIndex][layerIndex].first < minChildCosts[childIndex]) {
                    minChildCosts[childIndex] = childCosts[childIndex][layerIndex].first;
                    bestPaths[childIndex] = std::make_pair(childCosts[childIndex][layerIndex].second, layerIndex);
                }
            }
            if (layerIndex >= fixedLayers.high) {
                CostT cost = viaCosts[layerIndex] - viaCosts[lowLayerIndex];
                for (CostT childCost : minChildCosts) cost += childCost;
                if (cost < node->costs[layerIndex]) {
                    node->costs[layerIndex] = cost;
                    node->bestPaths[layerIndex] = bestPaths;
                }
            }
        }
        for (int layerIndex = gridGraph.getNumLayers() - 2; layerIndex >= lowLayerIndex; layerIndex--) {
            if (node->costs[layerIndex + 1] < node->costs[layerIndex]) {
                node->costs[layerIndex] = node->costs[layerIndex + 1];
                node->bestPaths[layerIndex] = node->bestPaths[layerIndex + 1];
            }
        }
    }
}

/*
Write the tree information into a file for differentiable GR
*/
void PatternRoute::writeTree(std::ofstream& os){
    os <<"tree "<< net.getName() << " " << net.getNumPins() << " " << net.getIndex() << std::endl;
    numSteinerNodes = 0;
    std::function<void(std::ofstream&, std::shared_ptr<SteinerTreeNode>&, int)> writeTreeNode = [&] (
            std::ofstream& os, std::shared_ptr<SteinerTreeNode>& steiner, int parent
        ) {
            int current_idx = numSteinerNodes;
            os << steiner->x <<" " << steiner->y << " " << steiner->fixedLayers.low << " " << steiner->fixedLayers.high << " " << current_idx << " " << parent << std::endl;
            numSteinerNodes += 1;
            for (auto steinerChild : steiner->children) {
                writeTreeNode(os,steinerChild,current_idx);
            }
        };
    writeTreeNode(os,steinerTree,-1);
    os << std::endl;
}


/*
Write the path information into a file for differentiable GR (we need translate 3D path to 2D)
*/
void PatternRoute::writePath(std::ofstream& os){
    os <<"tree "<< net.getName() << " " << net.getNumPins() << " " << net.getIndex() << std::endl;
    numSteinerNodes = 0;
    std::shared_ptr<GRTreeNode> root = net.getRoutingTree();
    std::map<int, std::map<int, std::vector<int>>> layerMap; // x -> y -> (layermin, layermax). encoding the pin info
    std::function<void(std::ofstream&, std::shared_ptr<SteinerTreeNode>&, int)> readNode = [&] (
            std::ofstream& os, std::shared_ptr<SteinerTreeNode>& steiner, int parent
        ) {
            int current_idx = numSteinerNodes;
            layerMap[steiner->x][steiner->y].push_back(steiner->fixedLayers.low);
            layerMap[steiner->x][steiner->y].push_back(steiner->fixedLayers.high);
            numSteinerNodes += 1;
            for (auto steinerChild : steiner->children) {
                readNode(os,steinerChild,current_idx);
            }
        };
    readNode(os,steinerTree,-1);

    numSteinerNodes = 0;
    std::function<void(std::ofstream&, std::shared_ptr<GRTreeNode>&, int,int, int)> writePathNode = [&] (
            std::ofstream& os, std::shared_ptr<GRTreeNode>& steiner, int parent, int parent_x, int parent_y
        ) {
            int current_idx = parent; // the index of the current node, if in the same via, then, the idx should be the same
            // only consider nodes whose children size is larger than 1 or it is not a steiner node (layerMap[steiner->x][steiner->y].size() == 0)
            if ( layerMap[steiner->x][steiner->y].size() > 0 || steiner->children.size() > 1){
                // skip nodes with the same x and y, which mean they are connected by a via
                if (parent_x != steiner->x || parent_y != steiner->y || parent == -1){
                    // if layerMap is empty, it means this node is a steiner point, then low = std::numeric_limits<T>::max(); high = std::numeric_limits<T>::min();
                    int low;
                    int high;
                    current_idx = numSteinerNodes;
                    if (layerMap[steiner->x][steiner->y].size() == 0){
                        low = std::numeric_limits<int>::max();
                        high = std::numeric_limits<int>::min();
                    }
                    else{
                        low =layerMap[steiner->x][steiner->y][0];
                        high = layerMap[steiner->x][steiner->y][1];
                    }
                    
                    os << steiner->x <<" " << steiner->y << " " << low << " " << high << " " << current_idx << " " << parent << std::endl;
                    numSteinerNodes += 1;
                }
            }
            

            for (auto steinerChild : steiner->children) {
                writePathNode(os,steinerChild,current_idx, steiner->x, steiner->y);
            }
        };
    writePathNode(os,root,-1,root->x,root->y);
    os << std::endl;
}
std::shared_ptr<GRTreeNode> PatternRoute::getRoutingTree(std::shared_ptr<PatternRoutingNode>& node, int parentLayerIndex) {
    if (parentLayerIndex == -1) {
        CostT minCost = std::numeric_limits<CostT>::max();
        for (int layerIndex = 0; layerIndex < gridGraph.getNumLayers(); layerIndex++) {
            if (routingDag->costs[layerIndex] < minCost) {
                minCost = routingDag->costs[layerIndex];
                parentLayerIndex = layerIndex;
            }
        }
    }
    std::shared_ptr<GRTreeNode> routingNode = std::make_shared<GRTreeNode>(parentLayerIndex, node->x, node->y);
    std::shared_ptr<GRTreeNode> lowestRoutingNode = routingNode;
    std::shared_ptr<GRTreeNode> highestRoutingNode = routingNode;
    if (node->paths.size() > 0) {
        int pathIndex, layerIndex;
        vector<vector<std::shared_ptr<PatternRoutingNode>>> pathsOnLayer(gridGraph.getNumLayers());
        for (int childIndex = 0; childIndex < node->paths.size(); childIndex++) {
            std::tie(pathIndex, layerIndex) = node->bestPaths[parentLayerIndex][childIndex];
            pathsOnLayer[layerIndex].push_back(node->paths[childIndex][pathIndex]);
        }
        if (pathsOnLayer[parentLayerIndex].size() > 0) {
            for (auto& path : pathsOnLayer[parentLayerIndex]) {
                routingNode->children.push_back(getRoutingTree(path, parentLayerIndex));
            }
        }
        for (int layerIndex = parentLayerIndex - 1; layerIndex >= 0; layerIndex--) {
            if (pathsOnLayer[layerIndex].size() > 0) {
                lowestRoutingNode->children.push_back(std::make_shared<GRTreeNode>(layerIndex, node->x, node->y));
                lowestRoutingNode = lowestRoutingNode->children.back();
                for (auto& path : pathsOnLayer[layerIndex]) {
                    lowestRoutingNode->children.push_back(getRoutingTree(path, layerIndex));
                }
            }
        }
        for (int layerIndex = parentLayerIndex + 1; layerIndex < gridGraph.getNumLayers(); layerIndex++) {
            if (pathsOnLayer[layerIndex].size() > 0) {
                highestRoutingNode->children.push_back(std::make_shared<GRTreeNode>(layerIndex, node->x, node->y));
                highestRoutingNode = highestRoutingNode->children.back();
                for (auto& path : pathsOnLayer[layerIndex]) {
                    highestRoutingNode->children.push_back(getRoutingTree(path, layerIndex));
                }
            }
        }
    }
    if (lowestRoutingNode->layerIdx > node->fixedLayers.low) {
        lowestRoutingNode->children.push_back(std::make_shared<GRTreeNode>(node->fixedLayers.low, node->x, node->y));
    }
    if (highestRoutingNode->layerIdx < node->fixedLayers.high) {
        highestRoutingNode->children.push_back(std::make_shared<GRTreeNode>(node->fixedLayers.high, node->x, node->y));
    }
    return routingNode;
}
