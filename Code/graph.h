#pragma once

#include <iostream>
#include <map>
#include <set>
#include <unordered_map>
#include <vector>

using namespace std;

/// @brief Simple directed graph using an adjacency list.
/// @tparam VertexT vertex type
/// @tparam WeightT edge weight type
template <typename VertexT, typename WeightT>
class graph {
   private:
    map<VertexT, unordered_map<VertexT, WeightT>> adjList; // map of vertices to an unordered_map of adjacent vertices and weights.

   public:
    /// Default constructor
    graph() = default;

    /// @brief Add the vertex `v` to the graph, must run in at most O(log |V|).
    /// @param v
    /// @return true if successfully added; false if it existed already
    bool addVertex(VertexT v) {
        // check if vertex already exists
        if (adjList.find(v) != adjList.end()) {
            return false;
        }
        // insert vertex with an empty map
        adjList[v];
        return true;
    }

    /// @brief Add or overwrite directed edge in the graph, must run in at most O(log |V|).
    /// @param from starting vertex
    /// @param to ending vertex
    /// @param weight edge weight / label
    /// @return true if successfully added or overwritten;
    ///         false if either vertices isn't in graph
    bool addEdge(VertexT from, VertexT to, WeightT weight) {
        // make sure starting and ending vertices exist
        if (adjList.find(from) == adjList.end() || adjList.find(to) == adjList.end()) {
            return false;
        }
        adjList[from][to] = weight;
        return true;
    }

    /// @brief Maybe get the weight associated with a given edge, must run in at most O(log |V|).
    /// @param from starting vertex
    /// @param to ending vertex
    /// @param weight output parameter
    /// @return true if the edge exists, and `weight` is set;
    ///         false if the edge does not exist
    bool getWeight(VertexT from, VertexT to, WeightT& weight) const {
        if (adjList.find(from) != adjList.end() && adjList.at(from).find(to) != adjList.at(from).end()) { // check if 'from' and 'to' exist
            weight = adjList.at(from).at(to); // set the weight
            return true;
        } else {
            return false;
        }
    }

    /// @brief Get the out-neighbors of `v`. Must run in at most O(|V|).
    /// @param v
    /// @return vertices that v has an edge to
    set<VertexT> neighbors(VertexT v) const {
        set<VertexT> S;
        auto it = adjList.find(v);
        if (it != adjList.end()) { // check if vertex exists
            // insert all neighbors into set
            for (const auto& pair : it->second) {
                S.insert(pair.first);
            }
        }
        return S;
    }

    /// @brief Return a vector containing all vertices in the graph
    vector<VertexT> getVertices() const {
        vector<VertexT> vertices;
        // push back all vertices in our adjacency list
        for (const auto& pair : adjList) {
            vertices.push_back(pair.first);
        }
        return vertices;
    }

    /// @brief Get the number of vertices in the graph. Runs in O(1).
    size_t NumVertices() const {
        return adjList.size();
    }

    /// @brief Get the number of directed edges in the graph. Runs in at most O(|V|).
    size_t NumEdges() const {
        size_t ct = 0;
        // iterate through all vertices in our adjacency list 
        for (const auto& pair : adjList) {
            // pair.second.size() gives us the number of adjacent vertices to the original vertex of pair.first
            ct += pair.second.size();
        }
        return ct;
    }
};
