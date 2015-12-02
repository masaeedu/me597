#include <Eigen/Dense>

using std;

typedef tuple<int, int> indices;
typedef tuple<double, double> coord;
map<int, map<int, double>> edgeset;

int main() {
	// Indices of targets (the last n-1 items of the n items in milestones are the targets)
	vector<int> ti(targets.size());
	iota(ti.start(), ti.end(), 1);
}


tuple<vector<coord>, edgeset> prm(int n, int k, coord start, vector<coord> targets, gazebo_msgs::OccupancyGrid grid) {	
	// Figure out map width and height
	double mapWidth = grid.info.resolution * grid.info.width;
	double mapHeight = grid.info.resolution * grid.info.height;
	
	// Start generating graph
	vector<coord> milestones;
	edgeset edges;
	
	// Seed graph with start node and targets
	milestones.push_back(start);
	milestones.insert(milestones.end(), targets.start(), targets.end());
	
	// Sample a collision-free set of milestones
	while (milestones.size() < n) {
		coords point;
		// Sample a new point
		do {
			point = sample_point()
		} while (!is_milestone_valid(point))
		
		milestones.push_back(point);
	}
	
	// For each milestone, connect all legal paths
	for (auto i: milestones) {
		for (auto j: find_nearest_neighbors(i, milestones, k)) {
			if (is_connection_valid(milestones[i], milestones[j], grid)) {
				edges[i][j] = get_distance(milestones[i], milestones[j]);
			}
		}
	}
	
	return {milestones, edges};
}

vector<int> a_star(int start, int end, vector<coord> milestones, edgeset edges) {
	auto endpoint = milestone[end];
	
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
			
			return result;
		}
		
		// Remove current from open set and add to closed set
		open.remove(current);
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

vector<int> find_nearest_neighbors(coord point, vector<coord> candidates, int k) {
	// Create result vector
	vector<int> idx(candidates.size()); 
	iota(idx.begin(), idx.end(), 0);
	
	partial_sort(idx.begin(), idx.begin() + k, idx.end(), [=](coord a, coord b) {
		return get_distance(point, candidates[a]) - get_distance(point, candidates[b]);
	});
	return vector<int>(idx.begin(), idx.begin() + k);
}

double get_distance(coord a, coord b) {
	return sqrt(pow(get<0>(a) - get<0>(b), 2) + pow(get<1>(a) - get<1>(b), 2));
}

coords sample_point(gazebo_msgs::OccupancyGrid grid) {
	thread_local static mt19937 rng(std::random_device{}());
	thread local static uniform_real_distribution<double> dist(0.0, 1.0);
		
	double mapWidth = grid.info.resolution * grid.info.width;
	double mapHeight = grid.info.resolution * grid.info.height;
	
	return {
		dist(rng) * mapWidth - grid.info.origin.position.x, 
		dist(rng) * mapHeight - grid.info.origin.position.y
	};
}

boolean is_milestone_valid(coord ms, gazebo_msgs::OccupancyGrid grid) {
	auto idx = get_og_index(get_indices(ms));
	return grid[idx] > 10;
}

boolean is_connection_valid(coord start, coord finish, gazebo_msgs::OccupancyGrid grid) {
	auto idx1 = bresenham(get_indices(start))
}

indices get_indices(coords c, gazebo_msgs::OccupancyGrid grid) {
    return {
		(int)(get<0>(c) / grid.info.resolution), 
		(int)(get<1>(c) / grid.info.resolution)
	};
}

coords get_coordinates(indices i) {
    return {
		(0.5 + get<0>(i)) * grid.info.resolution, 
		(0.5 + get<1>(i)) * grid.info.resolution 
	};
}

int get_og_index(indices idx) {
	auto x = get<0>(idx);
	auto y = get<1>(idx);
    return (x + cell_width / 2) + (y + cell_height / 2) * cell_width;
}

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