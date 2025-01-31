#include "application.h"

#include <algorithm>
#include <cassert>
#include <cstdlib>
#include <cstring>
#include <iomanip> /*setprecision*/
#include <iostream>
#include <map>
#include <queue>
#include <set>
#include <stack>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "dist.h"
#include "graph.h"
#include "osm.h"
#include "tinyxml2.h"

using namespace std;
using namespace tinyxml2;

double INF = numeric_limits<double>::max();

class prioritize {
   public:
    bool operator()(const pair<long long, double>& p1,
                    const pair<long long, double>& p2) const {
        return p1.second > p2.second;
    }
};

graph<long long, double> buildGraph(
    const map<long long, Coordinates>& Nodes,
    const vector<FootwayInfo>& Footways,
    const vector<BuildingInfo>& Buildings) {
    graph<long long, double> G;
    double edgeWeight = 0.0;

    // insert all nodes as vertices in the graph
    for (const auto& node : Nodes) {
        G.addVertex(node.first);
    }

    // convert 'FootwayInfo' into edges between consecutive pairs of nodes
    for (const auto& footway : Footways) {
        for (long unsigned int i = 0; i + 1 < footway.Nodes.size(); i++) {
            edgeWeight = distBetween2Points(Nodes.at(footway.Nodes.at(i)).Lat, Nodes.at(footway.Nodes.at(i)).Lon, Nodes.at(footway.Nodes.at(i + 1)).Lat, Nodes.at(footway.Nodes.at(i + 1)).Lon);
            G.addEdge(footway.Nodes.at(i), footway.Nodes.at(i + 1), edgeWeight);
            G.addEdge(footway.Nodes.at(i + 1), footway.Nodes.at(i), edgeWeight); // third parameter incorrect. weight is something else
        }
    }


    // convert 'Buildings' into vertices with edges
    for (const auto& building : Buildings) {
        // add vertex for buildings
        G.addVertex(building.Coords.ID);
        // create an edge from the building’s node to any other node within 0.041 miles that is on a footway
        for (const auto& node : Nodes) {
            if (node.second.OnFootway) {
                // we are on a footway, check nodes' distance from building
                edgeWeight = distBetween2Points(building.Coords.Lat, building.Coords.Lon, node.second.Lat, node.second.Lon);
                if (distBetween2Points(building.Coords.Lat, building.Coords.Lon, node.second.Lat, node.second.Lon) < 0.041) {
                    G.addEdge(building.Coords.ID, node.first, edgeWeight);
                }
                if (distBetween2Points(node.second.Lat, node.second.Lon, building.Coords.Lat, building.Coords.Lon) < 0.041) {
                    G.addEdge(node.first, building.Coords.ID, edgeWeight);
                }
            }
        }
    }

    return G;
}

vector<long long> dijkstra(
    const graph<long long, double>& G,
    long long start,
    long long target,
    const set<long long>& ignoreNodes) {
    
    // edge case: if the start is the same as the target
    if (start == target) {
        return vector<long long> {start};
    }

    map<long long, double> dist; // distance map to store the shortest path distance to each node from the start.
    map<long long, long long> predV; // predecessor map to reconstruct the shortest path.
    priority_queue<pair<long long, double>, vector<pair<long long, double>>, prioritize> worklist; // priority queue to select the next node with the shortest distance efficiently.
    double edgeWeight = 0.0; // weight of the edge from currentV to currentNeighbor

    // initialize distances to all vertices as infinity except for the start node.
    vector<long long> vertices = G.getVertices();
    for (const auto& currentV : vertices) {
        dist[currentV] = INF;
        predV[currentV] = -1;
        worklist.push({currentV, dist[currentV]});
    }
    
    // startV has a distance of 0 from itself
    dist[start] = 0.0; // dist of start to itself is zero.
    worklist.push({start, 0.0}); // start queue with the start node.

    while (!worklist.empty()) {
        // visit vertex with minimum distance from startV
        pair<long long, double> currentNode = worklist.top(); // node in the queue with the smallest distance.
        long long currentV = currentNode.first;
        double currentDist = currentNode.second;
        worklist.pop();

        // if the popped vertex's distance is INF, it's not reachable, so break
        if (currentDist == INF) {
            break;
        }

        // skip currentV if the node is in ignoreNodes, unless it's the start
        if (ignoreNodes.find(currentV) != ignoreNodes.end() && currentV != start) {
            continue;
        }

        // iterate through all adjacent vertices (neighbors) of the current node
        set<long long> adjacentVertices = G.neighbors(currentV);
        for (const auto& adjV : adjacentVertices) {
            if (G.getWeight(currentV, adjV, edgeWeight)) { // get the weight of the edge, if possible
                // if the calculated distance is less than the known distance, update it.
                if (dist[currentV] + edgeWeight < dist[adjV]) {
                    dist[adjV] = dist[currentV] + edgeWeight; // update the distance
                    predV[adjV] = currentV; // update the predecessor
                    worklist.push({adjV, dist[adjV]}); // push the updated distance to the queue
                }
            }
        }
    }

    // edge case: target is unreachable
    if (dist[target] == INF) {
        return {};
    }

    // reconstruct the shortest path from the target back to the start using the predecessors.
    vector<long long> path;
    for (long long i = target; i != start; i = predV[i]) {
        path.push_back(i); //cout << "line 161 added: " << i << endl;
    }
    path.push_back(start); // include the start node in the path
    reverse(path.begin(), path.end());

    return path; // return the shortest path
}

double pathLength(const graph<long long, double>& G, const vector<long long>& path) {
    double length = 0.0;
    double weight;
    for (size_t i = 0; i + 1 < path.size(); i++) {
        bool res = G.getWeight(path.at(i), path.at(i + 1), weight);
        assert(res);
        length += weight;
    }
    return length;
}

void outputPath(const vector<long long>& path) {
    for (size_t i = 0; i < path.size(); i++) {
        cout << path.at(i);
        if (i != path.size() - 1) {
            cout << "->";
        }
    }
    cout << endl;
}

void application(
    const vector<BuildingInfo>& Buildings,
    const graph<long long, double>& G) {
    string person1Building, person2Building;

    set<long long> buildingNodes;
    for (const auto& building : Buildings) {
        buildingNodes.insert(building.Coords.ID);
    }

    cout << endl;
    cout << "Enter person 1's building (partial name or abbreviation), or #> ";
    getline(cin, person1Building);

    while (person1Building != "#") {
        cout << "Enter person 2's building (partial name or abbreviation)> ";
        getline(cin, person2Building);

        //
        // find the building coordinates
        //
        bool foundP1 = false;
        bool foundP2 = false;
        Coordinates P1Coords, P2Coords;
        string P1Name, P2Name;

        for (const BuildingInfo& building : Buildings) {
            if (building.Abbrev == person1Building) {
                foundP1 = true;
                P1Name = building.Fullname;
                P1Coords = building.Coords;
            }
            if (building.Abbrev == person2Building) {
                foundP2 = true;
                P2Name = building.Fullname;
                P2Coords = building.Coords;
            }
        }

        for (const BuildingInfo& building : Buildings) {
            if (!foundP1 &&
                building.Fullname.find(person1Building) != string::npos) {
                foundP1 = true;
                P1Name = building.Fullname;
                P1Coords = building.Coords;
            }
            if (!foundP2 && building.Fullname.find(person2Building) != string::npos) {
                foundP2 = true;
                P2Name = building.Fullname;
                P2Coords = building.Coords;
            }
        }

        if (!foundP1) {
            cout << "Person 1's building not found" << endl;
        } else if (!foundP2) {
            cout << "Person 2's building not found" << endl;
        } else {
            cout << endl;
            cout << "Person 1's point:" << endl;
            cout << " " << P1Name << endl;
            cout << " (" << P1Coords.Lat << ", " << P1Coords.Lon << ")" << endl;
            cout << "Person 2's point:" << endl;
            cout << " " << P2Name << endl;
            cout << " (" << P2Coords.Lat << ", " << P2Coords.Lon << ")" << endl;

            string destName;
            Coordinates destCoords;

            Coordinates centerCoords = centerBetween2Points(
                P1Coords.Lat, P1Coords.Lon, P2Coords.Lat, P2Coords.Lon);

            double minDestDist = numeric_limits<double>::max();

            for (const BuildingInfo& building : Buildings) {
                double dist = distBetween2Points(
                    centerCoords.Lat, centerCoords.Lon,
                    building.Coords.Lat, building.Coords.Lon);
                if (dist < minDestDist) {
                    minDestDist = dist;
                    destCoords = building.Coords;
                    destName = building.Fullname;
                }
            }

            cout << "Destination Building:" << endl;
            cout << " " << destName << endl;
            cout << " (" << destCoords.Lat << ", " << destCoords.Lon << ")" << endl;

            vector<long long> P1Path = dijkstra(G, P1Coords.ID, destCoords.ID, buildingNodes);
            vector<long long> P2Path = dijkstra(G, P2Coords.ID, destCoords.ID, buildingNodes);

            // This should NEVER happen with how the graph is built
            if (P1Path.empty() || P2Path.empty()) {
                cout << endl;
                cout << "At least one person was unable to reach the destination building. Is an edge missing?" << endl;
                cout << endl;
            } else {
                cout << endl;
                cout << "Person 1's distance to dest: " << pathLength(G, P1Path);
                cout << " miles" << endl;
                cout << "Path: ";
                outputPath(P1Path);
                cout << endl;
                cout << "Person 2's distance to dest: " << pathLength(G, P2Path);
                cout << " miles" << endl;
                cout << "Path: ";
                outputPath(P2Path);
            }
        }

        //
        // another navigation?
        //
        cout << endl;
        cout << "Enter person 1's building (partial name or abbreviation), or #> ";
        getline(cin, person1Building);
    }
}
