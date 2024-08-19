#include <algorithm>
#include <iostream>
#include <limits>
#include <map>
#include <numeric>
#include <queue>
#include <set>
#include <string>
#include <vector>

template <typename Vertex = int, typename Weight = double>
class Graph {
public:
    using vertex_type = Vertex;
    using weight_type = Weight;
    using neighbor_type = std::pair<Vertex, Weight>;
    using neighbor_list_type = std::vector<neighbor_type>;

public:
   void AddEdge(Vertex source, Vertex target, Weight weight, bool bidirectional = true) {
      adjacency_list_[source].push_back(std::make_pair(target, weight));
      adjacency_list_[target].push_back(std::make_pair(source, weight));
   }

   size_t VertexCount() const { return adjacency_list_.size(); }
   std::vector<Vertex> Verteces() const {
      std::vector<Vertex> keys;
      for (auto const & kvp : adjacency_list_) {
         keys.push_back(kvp.first);
      }
      return keys;
   }

    const neighbor_list_type& neighbors(Vertex& v) const {
      auto pos = adjacency_list_.find(v);
      if (pos == adjacency_list_.end()) {
         throw std::runtime_error("vertex not found");
      }

      return pos->second;
   }

   constexpr static Weight Infinity = std::numeric_limits<Weight>::infinity();

private:
   std::map<vertex_type, neighbor_list_type> adjacency_list_;
};

template <typename Vertex, typename Weight>
void ShortestPath(
   const Graph<Vertex, Weight>& g,
   Vertex source,
   std::map<Vertex, Weight>& min_distance,
   std::map<Vertex, Vertex>& previous) {
   const auto n = g.VertexCount();
   const auto verteces = g.Verteces();

   min_distance.clear();
   for (const auto& v : verteces) {
      min_distance[v] = Graph<Vertex, Weight>::Infinity;
   }
   min_distance[source] = 0;

   previous.clear();

   std::set<std::pair<Weight, Vertex> > vertex_queue;
   vertex_queue.insert(std::make_pair(min_distance[source], source));

   while (!vertex_queue.empty()) {
      auto dist = vertex_queue.begin()->first;
      auto u = vertex_queue.begin()->second;

      vertex_queue.erase(std::begin(vertex_queue));

      const auto& neighbors = g.neighbors(u);
      for (const auto& neighbor : neighbors) {
         auto v = neighbor.first;
         auto w = neighbor.second;
         auto dist_via_u = dist + w;
         if (dist_via_u < min_distance[v]) {
            vertex_queue.erase(std::make_pair(min_distance[v], v));

            min_distance[v] = dist_via_u;
            previous[v] = u;
            vertex_queue.insert(std::make_pair(min_distance[v], v));
         }
      }
   }
}

template <typename Vertex>
void BuildPath(
   const std::map<Vertex, Vertex>& prev, Vertex v,
   std::vector<Vertex>& result) {
   result.push_back(v);

   auto pos = prev.find(v);
   if (pos == std::end(prev)) {
      return;
   }

   BuildPath(prev, pos->second, result);
}

template <typename Vertex>
std::vector<Vertex> BuildPath(
   const std::map<Vertex, Vertex>& prev, Vertex v) {
   std::vector<Vertex> result;
   BuildPath(prev, v, result);
   std::reverse(std::begin(result), std::end(result));
   return result;
}

template <typename Vertex>
void PrintPath(const std::vector<Vertex>& path) {
   for (size_t i = 0; i < path.size(); ++i) {
      std::cout << path[i];
      if (i < path.size() - 1) {
         std::cout << " -> ";
      }
   }
}

Graph<char, double> MakeGraph() {
   Graph<char, double> g;
   g.AddEdge('A', 'B', 4);
   g.AddEdge('A', 'H', 8);
   g.AddEdge('B', 'C', 8);
   g.AddEdge('B', 'H', 11);
   g.AddEdge('C', 'D', 7);
   g.AddEdge('C', 'F', 4);
   g.AddEdge('C', 'J', 2);
   g.AddEdge('D', 'E', 9);
   g.AddEdge('D', 'F', 14);
   g.AddEdge('E', 'F', 10);
   g.AddEdge('F', 'G', 2);
   g.AddEdge('G', 'J', 6);
   g.AddEdge('G', 'H', 1);
   g.AddEdge('H', 'J', 7);

   return g;
}

Graph<char, double> MakeGraphWiki() {
   Graph<char, double> g;
   g.AddEdge('A', 'B', 7);
   g.AddEdge('A', 'C', 9);
   g.AddEdge('A', 'F', 14);
   g.AddEdge('B', 'C', 10);
   g.AddEdge('B', 'D', 15);
   g.AddEdge('C', 'D', 11);
   g.AddEdge('C', 'F', 2);
   g.AddEdge('D', 'E', 6);
   g.AddEdge('E', 'F', 9);

   return g;
}

Graph<std::string, double> MakeGraphMap() {
   Graph<std::string, double> g;

   g.AddEdge("London", "Reading", 41);
   g.AddEdge("London", "Oxford", 57);
   g.AddEdge("Reading", "Swindon", 40);
   g.AddEdge("Swindon", "Bristol", 40);
   g.AddEdge("Oxford", "Swindon", 30);
   g.AddEdge("London", "Southampton", 80);
   g.AddEdge("Southampton", "Bournemouth", 33);
   g.AddEdge("Bournemouth", "Exeter", 89);
   g.AddEdge("Bristol", "Exeter", 83);
   g.AddEdge("Bristol", "Bath", 12);
   g.AddEdge("Swindon", "Bath", 35);
   g.AddEdge("Reading", "Southampton", 50);

   return g;
}

int main() {
   {
      auto g = MakeGraph();

      char source = 'A';
      std::map<char, double>  min_distance;
      std::map<char, char> previous;
      ShortestPath(g, source, min_distance, previous);

      for (auto const & kvp : min_distance) {
         std::cout << source << " -> " << kvp.first << " : "
            << kvp.second << '\t';

         PrintPath(BuildPath(previous, kvp.first));

         std::cout << std::endl;
      }
   }
   std::cout << std::endl;
   {
      auto g = MakeGraphWiki();

      char source = 'A';
      std::map<char, double>  min_distance;
      std::map<char, char> previous;
      ShortestPath(g, source, min_distance, previous);

      for (auto const & kvp : min_distance) {
         std::cout << source << " -> " << kvp.first << " : "
            << kvp.second << '\t';

         PrintPath(BuildPath(previous, kvp.first));

         std::cout << std::endl;
      }
   }
   std::cout << std::endl;
   {
      auto g = MakeGraphMap();

      std::string source = "London";
      std::map<std::string, double>  min_distance;
      std::map<std::string, std::string> previous;
      ShortestPath(g, source, min_distance, previous);

      for (auto const & kvp : min_distance) {
         std::cout << source << " -> " << kvp.first << " : "
            << kvp.second << '\t';

         PrintPath(BuildPath(previous, kvp.first));

         std::cout << std::endl;
      }
   }
}
