# Delhi Road Network Pathfinder

A command-line C++ program that loads Delhi's real road network from OpenStreetMap and finds the shortest path between any two locations using three algorithms, benchmarked against each other.

## Algorithms

- **Dijkstra** — standard shortest path using a min-heap priority queue
- **Bidirectional Dijkstra** — simultaneous forward and backward search, meets in the middle
- **A\*** — Dijkstra with a haversine straight-line distance heuristic to guide the search

## Benchmark Results

Tested on 5 source-target pairs across Delhi (avg. distance ~18km):

| Algorithm | Nodes Explored | Time |
|---|---|---|
| Dijkstra | ~490,000 | ~500ms |
| Bidirectional Dijkstra | ~200,000 | ~280ms |
| A* | ~41,000 | ~40ms |

**A* explored 85% fewer nodes than Dijkstra** by using haversine distance as an admissible heuristic, never overestimating the true cost to the target.

Sample query — Connaught Place → Noida Sector 18 (18.7 km):
```
--- Dijkstra ---
Distance: 18.676 km
Nodes explored: 488986
Time: 460.207 ms

--- A* ---
Distance: 18.676 km
Nodes explored: 41419
Time: 39.927 ms

--- Bidirectional Dijkstra ---
Distance: 18.676 km
Nodes explored: 102064
Time: 81.753 ms
```

## Build

Requires g++ with C++17 support.

```bash
g++ -std=c++17 -O2 -o pathfinder src/main.cpp pugixml.cpp
```

## Getting OSM Data

1. Go to [overpass-turbo.eu](https://overpass-turbo.eu)
2. Run this query:
```
[out:xml][timeout:60];
area["name"="Delhi"]["admin_level"="4"]->.searchArea;
(
  way["highway"](area.searchArea);
  node(w);
);
out body;
```
3. Export → download as `export.osm`
4. Place the file in the `data/` directory

## Usage

```bash
./pathfinder
```

Enter two locations as `lat1 lon1 lat2 lon2`:
```
Enter 2 locations lat1 lon1, lat2 lon2 (space between every number):
28.6139 77.2090 28.5355 77.3910
```

## Project Structure

```
delhi-pathfinder/
├── src/
│   ├── main.cpp
│   ├── parser.hpp       # OSM XML parsing, graph construction
│   └── algorithms.hpp   # Dijkstra, A*, Bidirectional Dijkstra
├── data/
│   └── export.osm
├── pugixml.hpp
└── pugixml.cpp
```

## Dependencies

- [pugixml](https://pugixml.org) — XML parsing (included in repo)
- C++17 standard library only
