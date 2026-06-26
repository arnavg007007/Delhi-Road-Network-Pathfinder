#include <iostream>
#include <iomanip>
#include "parser.hpp"
#include "algorithms.hpp"
using namespace std;
//28.6139 77.2090 28.5355 77.3910 (reference input) -> output 18.676 km (cp -> noida sec 18)
int main(){
    Graph g = parseOSM("data/export.osm");
    auto it = g.nodes.begin();
    cout << "Enter 2 locations lat1  lon1, lat2  lon2 (space between every number):\n";
    double lat1, lat2, lon1, lon2;
    
    cin >> lat1;
    cin >> lon1;
    cin >> lat2;
    cin >> lon2;
    long long src = nearestNode(g, lat1, lon1);
    long long tgt = nearestNode(g, lat2, lon2);

    cout << "src: " << src << "\n";
    cout << "tgt: " << tgt << "\n";

    Result r1 = dijkstra(g, src, tgt);
    Result r2 = astar(g, src, tgt);
    Result r3 = bidijkstra(g, src, tgt);

    cout << fixed << setprecision(3);
    cout << "\n--- Dijkstra ---\n";
    cout << "Distance: " << r1.distance/1000.0 << " km\n";
    cout << "Nodes explored: " << r1.nodesexp << "\n";
    cout << "Time: " << r1.timems << " ms\n";

    cout << "\n--- A* ---\n";
    cout << "Distance: " << r2.distance/1000.0 << " km\n";
    cout << "Nodes explored: " << r2.nodesexp << "\n";
    cout << "Time: " << r2.timems << " ms\n";

    cout << "\n--- Bidirectional Dijkstra ---\n";
    cout << "Distance: " << r3.distance/1000.0 << " km\n";
    cout << "Nodes explored: " << r3.nodesexp << "\n";
    cout << "Time: " << r3.timems << " ms\n";
    cout << "\n";

    cout << "Reduction in time by A* over Dijkstra: " << 100*(r1.timems-r2.timems)/r1.timems << "%\n";
    cout << "Reduction in visited nodes by A* over Dijkstra: " << 100*(r1.nodesexp-r2.nodesexp)/(double)r1.nodesexp << "%\n";


}