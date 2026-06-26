#pragma once
#include <unordered_map>
#include <vector>
#include <queue>
#include <chrono>
#include <limits>
#include <algorithm>
#include <bits/stdc++.h>
#include "parser.hpp"
using namespace std;

struct Result{
    vector<long long> path;
    double distance;
    int nodesexp;
    double timems;
};

Result dijkstra(const Graph& g, long long src, long long tgt){
    auto start = chrono::high_resolution_clock::now();
    unordered_map<long long, double> dist;
    unordered_map<long long, long long>prev;
    dist[src] = 0.0;
    int explored = 0;

    using P = pair<double, long long>;
    priority_queue<P, vector<P>, greater<P>> pq;
    pq.push({0.0, src});
    while (!pq.empty()){
        auto [cost, u] = pq.top();
        pq.pop();
        if (cost>dist[u]) continue;
        explored++;
        if (u == tgt) break;
        auto it = g.adj.find(u);
        if (it == g.adj.end()) continue;
        for (auto& [v, w]: it->second){
            double nd = dist[u] + w;
            if (!dist.count(v) || nd<dist[v]){
                dist[v]=nd;
                prev[v] = u;
                pq.push({nd, v});
            }
        }
    }

    vector<long long> path;
    if (dist.count(tgt)){
        for (long long cur = tgt; cur!= src; cur = prev[cur]){
            path.push_back(cur);
        }
        path.push_back(src);
        reverse(path.begin(), path.end());
    }

    auto end = chrono::high_resolution_clock::now();
    double ms = chrono:: duration<double, milli>(end-start).count();
    return {path, dist.count(tgt) ? dist[tgt] : -1.0, explored, ms};

}

Result astar(const Graph& g, long long src, long long tgt){
    auto start = chrono::high_resolution_clock::now();
    
    if (!g.nodes.count(tgt)) return {{}, -1, 0, 0.0};
    unordered_map<long long, double> dist;
    unordered_map<long long, long long> prev;
    dist[src] = 0.0;
    int explored = 0;

    double tlat = g.nodes.at(tgt).lat;
    double tlon = g.nodes.at(tgt).lon;

    using P = pair<double, long long>;
    priority_queue<P, vector<P>, greater<P>> pq;
    pq.push({0.0, src});
    while (!pq.empty()){
        auto [f, u] = pq.top();
        pq.pop();

        double g_u = dist.count(u) ? dist[u] : numeric_limits<double>::infinity();
        if (f>g_u + haversine(g.nodes.at(u).lat, g.nodes.at(u).lon, tlat, tlon) + 1e-9) continue;
        explored++;

        if (u == tgt) break;

        auto it = g.adj.find(u);
        if (it == g.adj.end()) continue;
        for (auto& [v ,w]: it->second){
            double nd = dist[u] + w;
            if (!dist.count(v) || nd < dist[v]){
                dist[v] = nd;
                prev[v] = u;
                double h = haversine(g.nodes.at(v).lat, g.nodes.at(v).lon, tlat, tlon);
                pq.push({nd + h, v});
            }
        }
    }

    vector<long long> path;
    if (dist.count(tgt)){
        for (long long cur = tgt; cur!= src; cur = prev[cur]){
            path.push_back(cur);
        }
        path.push_back(src);
        reverse(path.begin(), path.end());
    }
    auto end = chrono::high_resolution_clock::now();
    double ms = chrono::duration<double, milli>(end - start).count();

    return {path, dist.count(tgt) ? dist[tgt] : -1.0, explored, ms};

}

Result bidijkstra(const Graph& g, long long src, long long tgt) {
    auto start = chrono::high_resolution_clock::now();

    unordered_map<long long, double> dist_f, dist_b;
    unordered_map<long long, long long> prev_f, prev_b;
    unordered_set<long long> settled_f, settled_b;

    dist_f[src] = 0.0;
    dist_b[tgt] = 0.0;

    using P = pair<double, long long>;
    priority_queue<P, vector<P>, greater<P>> pq_f, pq_b;
    pq_f.push({0.0, src});
    pq_b.push({0.0, tgt});

    double best = numeric_limits<double>::infinity();
    long long meetNode = -1;
    int explored = 0;

    auto relax = [&](
        long long u,
        const unordered_map<long long, vector<pair<long long, double>>>& adj,
        unordered_map<long long, double>& dist_mine,
        unordered_map<long long, double>& dist_other,
        unordered_map<long long, long long>& prev,
        unordered_set<long long>& settled_mine,
        priority_queue<P, vector<P>, greater<P>>& pq
    ) {
        auto it = adj.find(u);
        if (it == adj.end()) return;
        for (auto& [v, w] : it->second) {
            double nd = dist_mine[u] + w;
            if (!dist_mine.count(v) || nd < dist_mine[v]) {
                dist_mine[v] = nd;
                prev[v] = u;
                pq.push({nd, v});
            }
            if (dist_other.count(v)) {
                double total = dist_mine[u] + w + dist_other[v];
                if (total < best) { best = total; meetNode = v; }
            }
        }
    };

    while (!pq_f.empty() && !pq_b.empty()) {
        if (pq_f.top().first + pq_b.top().first >= best) break;

        {
            auto [c, u] = pq_f.top(); pq_f.pop();
            if (!settled_f.count(u)) {
                settled_f.insert(u); explored++;
                relax(u, g.adj, dist_f, dist_b, prev_f, settled_f, pq_f);
            }
        }

        {
            auto [c, u] = pq_b.top(); pq_b.pop();
            if (!settled_b.count(u)) {
                settled_b.insert(u); explored++;
                relax(u, g.radj, dist_b, dist_f, prev_b, settled_b, pq_b);
            }
        }
    }

    vector<long long> path;
    if (meetNode != -1) {
        vector<long long> fwd;
        for (long long cur = meetNode; cur != src; ) {
            fwd.push_back(cur);
            if (!prev_f.count(cur)) { fwd.clear(); break; }
            cur = prev_f[cur];
        }
        if (!fwd.empty() || meetNode == src) {
            fwd.push_back(src);
            reverse(fwd.begin(), fwd.end());

            vector<long long> bwd;
            for (long long cur = meetNode; cur != tgt; ) {
                if (!prev_b.count(cur)) { bwd.clear(); break; }
                cur = prev_b[cur];
                bwd.push_back(cur);
            }
            path = fwd;
            path.insert(path.end(), bwd.begin(), bwd.end());
        }
    }

    auto end = chrono::high_resolution_clock::now();
    double ms = chrono::duration<double, milli>(end - start).count();

    return {path, best == numeric_limits<double>::infinity() ? -1.0 : best, explored, ms};
}

long long nearestNode(const Graph& g, double lat, double lon) {
    long long best = -1;
    double bestDist = numeric_limits<double>::max();
    for (auto& [id, n] : g.nodes) {
        double d = haversine(n.lat, n.lon, lat, lon);
        if (d < bestDist) { bestDist = d; best = id; }
    }
    return best;
}