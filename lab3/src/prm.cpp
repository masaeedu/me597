#include <Eigen/Dense>
#include <tuple>
#include <vector>
#include <nav_msgs/OccupancyGrid.h>

using namespace std;

typedef tuple<int, int> indices;
typedef tuple<double, double> coord;
typedef map<int, map<int, double>> edgeset;

short sgn(int x) { return x >= 0 ? 1 : -1; }

void bresenham(indices idx1, indices idx2, vector<int>& x, vector<int>& y) {
    // Unpack indices
    int x0, y0, x1, y1;
    tie(x0, y0) = idx1;
    tie(x1, y1) = idx2;
    
    int dx = abs(x1 - x0);
    int dy = abs(y1 - y0);
    int dx2 = x1 - x0;
    int dy2 = y1 - y0;
    
    const bool s = abs(dy) > abs(dx);
    
    if (s) {
        int dx2 = dx;
        dx = dy;
        dy = dx2;
    }
    
    int inc1 = 2 * dy;
    int d = inc1 - dx;
    int inc2 = d - dx;
    
    x.push_back(x0);
    y.push_back(y0);
    
    while (x0 != x1 || y0 != y1) {
        if (s) 
            y0+=sgn(dy2); 
        else 
            x0+=sgn(dx2);
            

        if (d < 0) 
            d += inc1;
        else {
            d += inc2;
            if (s) x0+=sgn(dx2); else y0+=sgn(dy2);
        }
        
        //Add point to vector
        x.push_back(x0);
        y.push_back(y0);
    }
}

double get_distance(coord a, coord b) {
    return sqrt(pow(get<0>(a) - get<0>(b), 2) + pow(get<1>(a) - get<1>(b), 2));
}

vector<int> find_nearest_neighbors(coord point, vector<coord> candidates, int k) {
    // Accommodate self as an edge
    k += 1;

    vector<int> idx(candidates.size()); 
    iota(idx.begin(), idx.end(), 0);

    partial_sort(idx.begin(), idx.begin() + k, idx.end(), [point, candidates](int a, int b) {
        return get_distance(point, candidates[a]) < get_distance(point, candidates[b]);
    });

    return vector<int>(idx.begin() + 1, idx.begin() + k);
}

coord sample_point(nav_msgs::OccupancyGrid grid) {
    thread_local static mt19937 rng(std::random_device{}());
    thread_local static uniform_real_distribution<double> dist(0.0, 1.0);
        
    double mapWidth = grid.info.resolution * grid.info.width;
    double mapHeight = grid.info.resolution * grid.info.height;
    
    return coord {
        dist(rng) * mapWidth + grid.info.origin.position.x, 
        dist(rng) * mapHeight + grid.info.origin.position.y
    };
}

indices get_indices(coord c, nav_msgs::OccupancyGrid grid) {
    return indices {
        (int)((get<0>(c) - grid.info.origin.position.x) / grid.info.resolution), 
        (int)((get<1>(c) - grid.info.origin.position.y) / grid.info.resolution)
    };
}

coord get_coordinates(indices i, nav_msgs::OccupancyGrid grid) {
    return coord {
        (0.5 + get<0>(i)) * grid.info.resolution + grid.info.origin.position.x, 
        (0.5 + get<1>(i)) * grid.info.resolution + grid.info.origin.position.y
    };
}

int get_og_index(indices idx, nav_msgs::OccupancyGrid grid) {
    auto x = get<0>(idx);
    auto y = get<1>(idx);
    return y * grid.info.width + x;
}

vector<indices> get_radius_neighbors(indices idx, int width, int height, int radius) {
    vector<indices> result;
    for (int i = max(get<0>(idx) - radius, 0); i < min(get<0>(idx) + radius, width); i++) {
        for (int j = max(get<1>(idx) - radius, 0); j < min(get<1>(idx) + radius, height); j++) {
            result.push_back(indices{i, j});
        }
    }
    
    return result;
}

void fuck_up_occupancygrid(nav_msgs::OccupancyGrid &grid) {
    for (int i = 0; i < grid.info.width; i++) {
        for (int j = 0; j < grid.info.height; j++) {
            auto idx = indices{i, j}; 
            int flatIndex = get_og_index(idx, grid);
            
            if (grid.data[flatIndex] > 90) {
                auto neighbors = get_radius_neighbors(idx, grid.info.width, grid.info.height, (int)(0.4 / grid.info.resolution));
                
                for (auto n: neighbors) {
                    grid.data[get_og_index(n, grid)] = max(80, (int)grid.data[get_og_index(n, grid)]);
                }
            }
        }
    }
}

bool is_connection_valid(coord start, coord finish, nav_msgs::OccupancyGrid grid) {
    vector<int> x;
    vector<int> y;
    bresenham(get_indices(start, grid), get_indices(finish, grid), x, y);
    
    for (auto i = x.begin(); i != x.end(); i++ ) {
        int idx = get_og_index(coord {*i, y[i - x.begin()]}, grid);
        
        if (grid.data[idx] >= 10) {
            return false;
        }
    }
    return true;
}

bool is_milestone_valid(coord ms, nav_msgs::OccupancyGrid grid) {
    auto idx = get_indices(ms, grid);
    
    return grid.data[get_og_index(idx, grid)] < 10;
}

vector<int> a_star(int start, int end, vector<coord> milestones, edgeset edges) {
    auto endpoint = milestones[end];
    
    // Open and closed sets
    set<int> open;
    set<int> closed; 
    
    // Bookkeeping
    map<int, int> trajectory;
    map<int, double> cost_actual;
    map<int, double> cost_lower_bound;
    
    // Seed open set with start node
    open.insert(start);
    cost_actual[start] = 0;
    cost_lower_bound[start] = cost_actual[start] + get_distance(milestones[start], endpoint);
    
    // While the best node isn't the goal
    while (!open.empty()) {
        int current;
        double lowest = numeric_limits<double>::infinity();
        
        // Choose the node from the open set with the lowest lower-bound estimate of cost
        for (auto i: open) {
            if (cost_lower_bound[i] < lowest) {
                lowest = cost_lower_bound[i];
                current = i;
            }
        }
        
        // If we've already reached the goal, walk backwards along the trajectory 
        // to construct a vector of node indices and return it
        if (current == end) {
            vector<int> result;
            result.push_back(end);
            
            while (trajectory.count(result.back())) {
                result.push_back(trajectory[result.back()]);
            }
            
            reverse(result.begin(), result.end());
            return result;
        }
        
        // Remove current from open set and add to closed set
        open.erase(current);
        closed.insert(current);
        
        // Deal with neighbors of current cell
        for (auto kv: edges[current]) {
            auto neighbor = kv.first;
            auto distance = kv.second;
            
            // Ignore any neighbors which have already been evaluated
            if (closed.count(neighbor)) {
                continue;
            }
            
            // Insert the neighbor in the open set and work out total cost from start node
            open.insert(neighbor);
            auto cost = cost_actual[current] + distance;
            
            // If this is a newly visited node or previous visits were through suboptimal path,
            // update metrics
            if (!trajectory.count(neighbor) || cost < cost_actual[neighbor]) {
                trajectory[neighbor] = current;
                cost_actual[neighbor] = cost;
                cost_lower_bound[neighbor] = cost + get_distance(milestones[neighbor], endpoint);
            }
        }
    }
    
    // If we haven't returned by this point, there is no path
    return {};
}

tuple<vector<coord>, edgeset> prm(const int n, const int k, coord start, vector<coord> targets, nav_msgs::OccupancyGrid grid) {    
    // Figure out map width and height
    double mapWidth = grid.info.resolution * grid.info.width;
    double mapHeight = grid.info.resolution * grid.info.height;
    
    // Inflating obstacle edges
    fuck_up_occupancygrid(grid);
    /* for (auto it = grid.data.begin(); it != grid.data.end(); it++) {
        cout << (int)*it << "; ";
        if (distance(grid.data.begin(), it) % grid.info.width == 0) {
            cout << endl;
        }
    }
    cout << endl;
    cin.get(); */
    
    // Start generating graph
    vector<coord> milestones;
    edgeset edges;
    
    // Seed graph with start node and targets
    milestones.push_back(start);
    milestones.insert(milestones.end(), targets.begin(), targets.end());
    
    // Sample a collision-free set of milestones
    while (milestones.size() < n) {
        coord point;
        // Sample a new point
        do {
            point = sample_point(grid);
        } while (!is_milestone_valid(point, grid));
        
        milestones.push_back(point);
    }
    
    // For each milestone, connect all legal paths
    for (auto it = milestones.begin(); it != milestones.end(); it++) {
	int i = distance(milestones.begin(), it);
        for (auto j: find_nearest_neighbors(milestones[i], milestones, k)) {
            if (i != j && is_connection_valid(milestones[i], milestones[j], grid)) {
                edges[i][j] = get_distance(milestones[i], milestones[j]);
            }
        }
    }
    
    return tuple<vector<coord>, edgeset>{milestones, edges};
}
