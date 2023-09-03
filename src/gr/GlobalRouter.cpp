#include "GlobalRouter.h"
#include "PatternRoute.h"
#include "MazeRoute.h"
#include <chrono>

GlobalRouter::GlobalRouter(const Design& design, const Parameters& params): 
    gridGraph(design, params), parameters(params) {
    // Instantiate the global routing netlist
    const vector<Net>& baseNets = design.getAllNets();
    nets.reserve(baseNets.size());
    for (const Net& baseNet : baseNets) {
        nets.emplace_back(baseNet, design, gridGraph);
    }
}


void GlobalRouter::route() {
    int n1 = 0, n2 = 0, n3 = 0;
    double t1 = 0, t2 = 0, t3 = 0;
    
    auto t = std::chrono::high_resolution_clock::now();
    
    vector<int> netIndices;
    netIndices.reserve(nets.size());
    for (const auto& net : nets) netIndices.push_back(net.getIndex());
    // Stage 1: Pattern routing
    n1 = netIndices.size();
    PatternRoute::readFluteLUT();
    log() << "stage 1: pattern routing" << std::endl;
    if (parameters.dgr_file != ""){readDGRResult();}
    if (parameters.dgr_tree_file != ""){readDGRTree();}
    sortNetIndices(netIndices);
    std::ofstream* os;
    os = new std::ofstream("tree.txt");
    int num_DGR_tree = 0;
    for (const int netIndex : netIndices) {
        PatternRoute patternRoute(nets[netIndex], gridGraph, parameters, DGR_result[nets[netIndex].getIndex()]);
        // if DGR_tree[nets[netIndex].getIndex()] is not empty, set patternRoute.steinerTree = DGR_tree[nets[netIndex].getIndex()]
        if (DGR_tree[nets[netIndex].getIndex()].size() != 0){
            patternRoute.setSteinerTree(DGR_tree[nets[netIndex].getIndex()][0]); // the first pin is the root pin
            num_DGR_tree += 1;
        }
        else {patternRoute.constructSteinerTree();}
        
        // patternRoute.writeTree(*os);
        if (parameters.dgr_file != "") {
            // log() << "construct DAG from DGR file: " << parameters.dgr_file << std::endl;
            patternRoute.constructDAGFromDGR();
        }
        else {
            // log() << "construct DAG from Steiner tree" << std::endl;
            patternRoute.constructRoutingDAG();
        }
        patternRoute.run();
        gridGraph.commitTree(nets[netIndex].getRoutingTree());
    }
    delete os;
    log() << num_DGR_tree << " / " << netIndices.size() << " nets are DGR trees." << std::endl;
    netIndices.clear();
    for (const auto& net : nets) {
        if (gridGraph.checkOverflow(net.getRoutingTree()) > 0) {
            netIndices.push_back(net.getIndex());
        }
    }
    log() << netIndices.size() << " / " << nets.size() << " nets have overflows." << std::endl;
    logeol();
    
    t1 = std::chrono::duration<double>(std::chrono::high_resolution_clock::now() - t).count();
    t = std::chrono::high_resolution_clock::now();
    
    // Stage 2: Pattern routing with possible detours
    if (parameters.phase2 != 0){
        n2 = netIndices.size();
        std::string filename;
        // filename == dgr_tree.txt if dgr_file is specified, else filename == CUGR2_tree.txt
        if (parameters.dgr_file != ""){filename = "dgr_tree.txt";}
        else {filename = "CUGR2_tree.txt";}
        os = new std::ofstream(filename);
        if (netIndices.size() > 0) {
            log() << "stage 2: pattern routing with possible detours" << std::endl;
            GridGraphView<bool> congestionView; // (2d) direction -> x -> y -> has overflow?
            gridGraph.extractCongestionView(congestionView);
            sortNetIndices(netIndices);
            for (const int netIndex : netIndices) {
                GRNet& net = nets[netIndex];
                gridGraph.commitTree(net.getRoutingTree(), true);
                PatternRoute patternRoute(net, gridGraph, parameters,DGR_result[nets[netIndex].getIndex()]);
                patternRoute.constructSteinerTree();
                patternRoute.constructRoutingDAG();
                patternRoute.constructDetours(congestionView); // KEY DIFFERENCE compared to stage 1
                patternRoute.run();
                patternRoute.writePath(*os);
                gridGraph.commitTree(net.getRoutingTree());
            }
            
            netIndices.clear();
            for (const auto& net : nets) {
                if (gridGraph.checkOverflow(net.getRoutingTree()) > 0) {
                    netIndices.push_back(net.getIndex());
                }
            }
            log() << netIndices.size() << " / " << nets.size() << " nets have overflows." << std::endl;
            logeol();
        }
        
        t2 = std::chrono::duration<double>(std::chrono::high_resolution_clock::now() - t).count();
        t = std::chrono::high_resolution_clock::now();
    }

    
    // Stage 3: maze routing on sparsified routing graph
    n3 = netIndices.size();
    if (netIndices.size() > 0) {
        log() << "stage 3: maze routing on sparsified routing graph" << std::endl;
        for (const int netIndex : netIndices) {
            GRNet& net = nets[netIndex];
            gridGraph.commitTree(net.getRoutingTree(), true);
        }
        GridGraphView<CostT> wireCostView;
        gridGraph.extractWireCostView(wireCostView);
        sortNetIndices(netIndices);
        SparseGrid grid(10, 10, 0, 0);
        std::string maze_file;
        if (parameters.dgr_file != ""){maze_file = "dgr_maze.txt";}
        else {maze_file = "CUGR2_maze.txt";}
        os = new std::ofstream(maze_file);
        for (const int netIndex : netIndices) {
            GRNet& net = nets[netIndex];
            // gridGraph.commitTree(net.getRoutingTree(), true);
            // gridGraph.updateWireCostView(wireCostView, net.getRoutingTree());
            MazeRoute mazeRoute(net, gridGraph, parameters);
            mazeRoute.constructSparsifiedGraph(wireCostView, grid);
            mazeRoute.run();
            std::shared_ptr<SteinerTreeNode> tree = mazeRoute.getSteinerTree();
            assert(tree != nullptr);
            
            PatternRoute patternRoute(net, gridGraph, parameters,DGR_result[nets[netIndex].getIndex()]);
            patternRoute.setSteinerTree(tree);
            patternRoute.writeTree(*os);
            patternRoute.constructRoutingDAG();
            patternRoute.run();
            
            gridGraph.commitTree(net.getRoutingTree());
            gridGraph.updateWireCostView(wireCostView, net.getRoutingTree());
            grid.step();
        }
        netIndices.clear();
        for (const auto& net : nets) {
            if (gridGraph.checkOverflow(net.getRoutingTree()) > 0) {
                netIndices.push_back(net.getIndex());
            }
        }
        log() << netIndices.size() << " / " << nets.size() << " nets have overflows." << std::endl;
        logeol();
    }
    
    t3 = std::chrono::duration<double>(std::chrono::high_resolution_clock::now() - t).count();
    t = std::chrono::high_resolution_clock::now();
    
    // std::cout << "iteration statistics " 
    //     << n1 << " " << std::setprecision(3) << std::fixed << t1 << " " 
    //     << n2 << " " << std::setprecision(3) << std::fixed << t2 << " " 
    //     << n3 << " " << std::setprecision(3) << std::fixed << t3 << std::endl;
    
    printStatistics();
    if (parameters.write_heatmap) gridGraph.write();
}

void GlobalRouter::sortNetIndices(vector<int>& netIndices) const {
    vector<int> halfParameters(nets.size());
    vector<int> numPaths(nets.size());
    for (int netIndex : netIndices) {
        auto& net = nets[netIndex];
        halfParameters[netIndex] = net.getBoundingBox().hp();
        numPaths[netIndex] = net.num_paths;
    }
    sort(netIndices.begin(), netIndices.end(), [&](int lhs, int rhs) {
        if (parameters.new_sort == 1) {if (numPaths[lhs] != numPaths[rhs]) {return numPaths[lhs] < numPaths[rhs];}}
        return halfParameters[lhs] < halfParameters[rhs];
    });
}

void GlobalRouter::getGuides(const GRNet& net, vector<std::pair<int, utils::BoxT<int>>>& guides) {
    auto& routingTree = net.getRoutingTree();
    if (!routingTree) return;
    // 0. Basic guides
    GRTreeNode::preorder(routingTree, [&](std::shared_ptr<GRTreeNode> node) {
        for (const auto& child : node->children) {
            if (node->layerIdx == child->layerIdx) {
                guides.emplace_back(
                    node->layerIdx, utils::BoxT<int>(
                        min(node->x, child->x), min(node->y, child->y),
                        max(node->x, child->x), max(node->y, child->y)
                    )
                );
            } else {
                int maxLayerIndex = max(node->layerIdx, child->layerIdx);
                for (int layerIdx = min(node->layerIdx, child->layerIdx); layerIdx <= maxLayerIndex; layerIdx++) {
                    guides.emplace_back(layerIdx, utils::BoxT<int>(node->x, node->y));
                }
            }
        }
    });
    
    
    auto getSpareResource = [&] (const GRPoint& point) {
        double resource = std::numeric_limits<double>::max();
        unsigned direction = gridGraph.getLayerDirection(point.layerIdx);
        if (point[direction] + 1 < gridGraph.getSize(direction)) {
            resource = min(resource, gridGraph.getEdge(point.layerIdx, point.x, point.y).getResource());
        }
        if (point[direction] > 0) {
            GRPoint lower = point;
            lower[direction] -= 1;
            resource = min(resource, gridGraph.getEdge(lower.layerIdx, point.x, point.y).getResource());
        }
        return resource;
    };
    
    // 1. Pin access patches
    assert(parameters.min_routing_layer + 1 < gridGraph.getNumLayers());
    for (auto& gpts : net.getPinAccessPoints()) {
        for (auto& gpt : gpts) {
            if (gpt.layerIdx < parameters.min_routing_layer) {
                int padding = 0;
                if (getSpareResource({parameters.min_routing_layer, gpt.x, gpt.y}) < parameters.pin_patch_threshold) {
                    padding = parameters.pin_patch_padding;
                }
                for (int layerIdx = gpt.layerIdx; layerIdx <= parameters.min_routing_layer + 1; layerIdx++) {
                    guides.emplace_back(layerIdx, utils::BoxT<int>(
                        max(gpt.x - padding, 0),
                        max(gpt.y - padding, 0),
                        min(gpt.x + padding, (int)gridGraph.getSize(0) - 1),
                        min(gpt.y + padding, (int)gridGraph.getSize(1) - 1)
                    ));
                    areaOfPinPatches += (guides.back().second.x.range() + 1) * (guides.back().second.y.range() + 1);
                }
            }
        }
    }
    
    // 2. Wire segment patches
    GRTreeNode::preorder(routingTree, [&](std::shared_ptr<GRTreeNode> node) {
        for (const auto& child : node->children) {
            if (node->layerIdx == child->layerIdx) {
                double wire_patch_threshold = parameters.wire_patch_threshold;
                unsigned direction = gridGraph.getLayerDirection(node->layerIdx);
                int l = min((*node)[direction], (*child)[direction]);
                int h = max((*node)[direction], (*child)[direction]);
                int r = (*node)[1 - direction];
                for (int c = l; c <= h; c++) {
                    bool patched = false;
                    GRPoint point = (direction == MetalLayer::H ? GRPoint(node->layerIdx, c, r) : GRPoint(node->layerIdx, r, c));
                    if (getSpareResource(point) < wire_patch_threshold) {
                        for (int layerIndex = node->layerIdx - 1; layerIndex <= node->layerIdx + 1; layerIndex += 2) {
                            if (layerIndex < parameters.min_routing_layer || layerIndex >= gridGraph.getNumLayers()) continue;
                            if (getSpareResource({layerIndex, point.x, point.y}) >= 1.0) {
                                guides.emplace_back(layerIndex, utils::BoxT<int>(point.x, point.y));
                                areaOfWirePatches += 1;
                                patched = true;
                            }
                        }
                    } 
                    if (patched) {
                        wire_patch_threshold = parameters.wire_patch_threshold;
                    } else {
                        wire_patch_threshold *= parameters.wire_patch_inflation_rate;
                    }
                }
            }
        }
    });
}

void GlobalRouter::printStatistics() const {
    log() << "routing statistics" << std::endl;
    loghline();

    // wire length and via count
    uint64_t wireLength = 0;
    int viaCount = 0;
    vector<vector<vector<int>>> wireUsage;
    wireUsage.assign(
        gridGraph.getNumLayers(), vector<vector<int>>(gridGraph.getSize(0), vector<int>(gridGraph.getSize(1), 0))
    );
    for (const auto& net : nets) {
        GRTreeNode::preorder(net.getRoutingTree(), [&] (std::shared_ptr<GRTreeNode> node) {
            for (const auto& child : node->children) {
                if (node->layerIdx == child->layerIdx) {
                    unsigned direction = gridGraph.getLayerDirection(node->layerIdx);
                    int l = min((*node)[direction], (*child)[direction]);
                    int h = max((*node)[direction], (*child)[direction]);
                    int r = (*node)[1 - direction];
                    for (int c = l; c < h; c++) {
                        wireLength += gridGraph.getEdgeLength(direction, c);
                        int x = direction == MetalLayer::H ? c : r;
                        int y = direction == MetalLayer::H ? r : c;
                        wireUsage[node->layerIdx][x][y] += 1;
                    }
                } else {
                    viaCount += abs(node->layerIdx - child->layerIdx);
                }
            }
        });
    }
    
    // resource
    CapacityT overflow = 0;

    CapacityT minResource = std::numeric_limits<CapacityT>::max();
    GRPoint bottleneck(-1, -1, -1);
    for (int layerIndex = parameters.min_routing_layer; layerIndex < gridGraph.getNumLayers(); layerIndex++) {
        unsigned direction = gridGraph.getLayerDirection(layerIndex);
        for (int x = 0; x < gridGraph.getSize(0) - 1 + direction; x++) {
            for (int y = 0; y < gridGraph.getSize(1) - direction; y++) {
                CapacityT resource = gridGraph.getEdge(layerIndex, x, y).getResource();
                if (resource < minResource) {
                    minResource = resource;
                    bottleneck = {layerIndex, x, y};
                }
                CapacityT usage = wireUsage[layerIndex][x][y];
                CapacityT capacity = max(gridGraph.getEdge(layerIndex, x, y).capacity, 0.0);
                if (usage > 0.0 && usage > capacity) {
                    overflow += usage - capacity;
                }
            }
        }
    }
    
    log() << "wire length (metric):  " << wireLength / gridGraph.getM2Pitch() << std::endl;
    log() << "total via count:       " << viaCount << std::endl;
    log() << "total wire overflow:   " << (int)overflow << std::endl;
    logeol();

    log() << "min resource: " << minResource << std::endl;
    log() << "bottleneck:   " << bottleneck << std::endl;

    logeol();
}

void GlobalRouter::write(std::string guide_file) {
    log() << "generating route guides..." << std::endl;
    if (guide_file == "") guide_file = parameters.out_file;
    
    areaOfPinPatches = 0;
    areaOfWirePatches = 0;
    std::stringstream ss;
    for (const GRNet& net : nets) {
        vector<std::pair<int, utils::BoxT<int>>> guides;
        getGuides(net, guides);
        
        ss << net.getName() << std::endl;
        ss << "(" << std::endl;
        for (const auto& guide : guides) {
            ss << gridGraph.getGridline(0, guide.second.x.low) << " "
                 << gridGraph.getGridline(1, guide.second.y.low) << " "
                 << gridGraph.getGridline(0, guide.second.x.high + 1) << " "
                 << gridGraph.getGridline(1, guide.second.y.high + 1) << " "
                 << gridGraph.getLayerName(guide.first) << std::endl;
        }
        ss << ")" << std::endl;
    }
    log() << "total area of pin access patches: " << areaOfPinPatches << std::endl;
    log() << "total area of wire segment patches: " << areaOfWirePatches << std::endl;
    log() << std::endl;
    log() << "writing output..." << std::endl;
    std::ofstream fout(guide_file);
    fout << ss.str();
    fout.close();
    log() << "finished writing output..." << std::endl;
}

void GlobalRouter::readDGRResult() {
    // read parameters.dgr_file
    log() << "reading dgr result..." << std::endl;
    std::ifstream inputFile(parameters.dgr_file);
    if (!inputFile) {
        std::cerr << "Error opening the file " << parameters.dgr_file<< " " <<std::endl;
        return;
    }
    //reset num_paths of all net in nets as 0
    for (auto& net : nets) {
        net.num_paths = 0;
    }
    std::map<int, int> netID2num_paths;
    std::string line;
    int netID;
    int pinID;
    while (std::getline(inputFile, line)) {
        std::istringstream iss(line);
        std::string token;
        std::vector<std::string> tokens;
        // Tokenize the line based on spaces
        while (iss >> token) {
            tokens.push_back(token);
        }
        if (tokens.size() == 3) {
            // It's a netname netID pinID line
            netID = std::stoi(tokens[1]);
            pinID = std::stoi(tokens[2]);
            // std::cout << "netID: " << netID << " pinID: " << pinID << std::endl;
            // Create an empty vector for this netID and pinID if it doesn't exist
            // append an empty vector for DGR_result[netID][pinID]
            
            DGR_result[netID][pinID].emplace_back();
            netID2num_paths[netID] += 1;
        } else if (tokens.size() == 2) {
            // It's a via_x via_y line
            int via_x = std::stoi(tokens[0]);
            int via_y = std::stoi(tokens[1]);
            int path_num = DGR_result[netID][pinID].size() - 1;
            // Add the via_x and via_y to the vector for the current netID and pinID
            // std::cout << "netID: " << netID << " pinID: " << pinID << " path_num: "<< path_num<<" via_x: " << via_x << " via_y: " << via_y << std::endl;
            DGR_result[netID][pinID][path_num].push_back({via_x, via_y});
        }
    }
    //update num_paths of all net in nets
    for (auto& net : nets) {
        // if net.idx is not in netID2num_paths, then net.num_paths = 0
        if (netID2num_paths.find(net.getIndex()) == netID2num_paths.end()) {
            net.num_paths = 0;
            continue;
        }
        net.num_paths = netID2num_paths[net.getIndex()];
    }
    log() << "finish reading dgr result..." << std::endl;
    // std::cout << "DGR_result size: " << DGR_result.size() << std::endl;
}


// Read steiner tree selected by DGR and update DGR_tree
void GlobalRouter::readDGRTree(){
    log() << "reading dgr tree..." << std::endl;
    std::ifstream inputFile(parameters.dgr_tree_file);
    if (!inputFile) {
        std::cerr << "Error opening the file " << parameters.dgr_tree_file<< " " <<std::endl;
        return;
    }
    int netID;
    int pin_num; // num of pins in this net
    std::string line;
    vector<utils::PointT<int>> steinerPoints;
    while (std::getline(inputFile, line)) {
        std::istringstream iss(line);
        std::string token;
        std::vector<std::string> tokens;
        // Tokenize the line based on spaces
        while (iss >> token) {
            tokens.push_back(token);
        }
        if (tokens.size() == 3) {
            // netname, netId, pin_num
            netID = std::stoi(tokens[1]);
            pin_num = std::stoi(tokens[2]);
            //initialize 
            DGR_tree[netID].resize(pin_num);
            for (int i = 0; i < pin_num; i++) {
                    DGR_tree[netID][i] = std::make_shared<SteinerTreeNode>(-1,-1);
                }
            // std::cout << "netID: " << netID << " pin_num: " << pin_num << std::endl;
        }
        else{
            // otherwise, the line is pinIdx, x, y, layer_min, layer_max, child1, child2, childx ...
            int pinIdx = std::stoi(tokens[0]);
            int x = std::stoi(tokens[1]);
            int y = std::stoi(tokens[2]);
            int layer_min = std::stoi(tokens[3]);
            int layer_max = std::stoi(tokens[4]);
            // std::cout << "pinIdx: " << pinIdx << " x: " << x << " y: " << y << " layer_min: " << layer_min << " layer_max: " << layer_max << " " <<tokens.size()<<std::endl;
            for (int i = 5; i < tokens.size(); i++){
                int child = std::stoi(tokens[i]);
                DGR_tree[netID][pinIdx]->children.emplace_back(DGR_tree[netID][child]);
                // std::cout << "child: " << child << std::endl;
            }
            // std::cout << "start assign values" << std::endl;
            DGR_tree[netID][pinIdx]->x = x;
            DGR_tree[netID][pinIdx]->y = y;
            // std::cout << "DGR_tree[netID][pinIdx]->x: " << DGR_tree[netID][pinIdx]->x << " DGR_tree[netID][pinIdx]->y: " << DGR_tree[netID][pinIdx]->y << std::endl;
            DGR_tree[netID][pinIdx]->fixedLayers.low = layer_min;
            DGR_tree[netID][pinIdx]->fixedLayers.high = layer_max;
            // std::cout << "DGR_tree[netID][pinIdx]->fixedLayers.low: " << DGR_tree[netID][pinIdx]->fixedLayers.low << " DGR_tree[netID][pinIdx]->fixedLayers.high: " << DGR_tree[netID][pinIdx]->fixedLayers.high << std::endl;

        }
    }
    log() << "finish reading dgr tree..." << std::endl;
}
