#pragma once
#include <unordered_map>
#include <vector>
#include <string>
#define _USE_MATH_DEFINES
#include <cmath>
#include "../pugixml.hpp"
using namespace std;

const double R = 6371000.0;

struct Node {
    long long id;
    double lat, lon;
};

double haversine(double lat1, double lon1, double lat2, double lon2) {
    double dlat = (lat2 - lat1) * M_PI / 180.0;
    double dlon = (lon2 - lon1) * M_PI / 180.0;
    lat1 = lat1 * M_PI / 180.0;
    lat2 = lat2 * M_PI / 180.0;
    double a = sin(dlat/2)*sin(dlat/2) +
               cos(lat1)*cos(lat2)*sin(dlon/2)*sin(dlon/2);
    return 2 * R * asin(sqrt(a));
}

struct Graph {
    unordered_map<long long, Node> nodes;
    unordered_map<long long, vector<pair<long long, double>>> adj;
    unordered_map<long long, vector<pair<long long, double>>> radj;
};

Graph parseOSM(const string& filepath) {
    Graph g;
    pugi::xml_document doc;

    if (!doc.load_file(filepath.c_str())) {
        throw runtime_error("Failed to load OSM file: " + filepath);
    }

    for (auto& xnode : doc.child("osm").children("node")) {
        Node n;
        n.id  = xnode.attribute("id").as_llong();
        n.lat = xnode.attribute("lat").as_double();
        n.lon = xnode.attribute("lon").as_double();
        g.nodes[n.id] = n;
    }

    for (auto& way : doc.child("osm").children("way")) {
        bool isRoad = false;

        for (auto& tag : way.children("tag")) {
            string k = tag.attribute("k").as_string();
            if (k == "highway") isRoad = true;
        }

        if (!isRoad) continue;

        vector<long long> refs;
        for (auto& nd : way.children("nd"))
            refs.push_back(nd.attribute("ref").as_llong());

        for (size_t i = 0; i + 1 < refs.size(); i++) {
            long long u = refs[i], v = refs[i+1];
            if (g.nodes.count(u) && g.nodes.count(v)) {
                double dist = haversine(
                    g.nodes[u].lat, g.nodes[u].lon,
                    g.nodes[v].lat, g.nodes[v].lon
                );
                g.adj[u].push_back({v, dist});
                g.adj[v].push_back({u, dist});
                g.radj[v].push_back({u, dist});
                g.radj[u].push_back({v, dist});
            }
        }
    }

    return g;
}