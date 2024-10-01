#include <memory>
#include <string>
#include <vector>
#include <map>
#include <queue>
#include <utility>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

class DijkstraNode : public rclcpp::Node {
public:
    DijkstraNode() : Node("dijkstra_node") {
        path_pub_ = this->create_publisher<std_msgs::msg::String>("dijkstra_path", 10);
        timer_ = this->create_wall_timer(
            std::chrono::seconds(1),
            std::bind(&DijkstraNode::calculate_path, this));
        start_ = {0, 0};
        goal_ = {5, 5};
        map_ = create_map();
    }

private:
    using Position = std::pair<int, int>;

    std::map<Position, int> map_;
    Position start_;
    Position goal_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr path_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    std::map<Position, int> create_map() {
        std::map<Position, int> grid;
        for (int x = 0; x < 10; ++x) {
            for (int y = 0; y < 10; ++y) {
                grid[{x, y}] = 1; // All cells with cost 1
            }
        }
        return grid;
    }

    void calculate_path() {
        auto path = dijkstra(start_, goal_);
        auto msg = std_msgs::msg::String();
        msg.data = vector_to_string(path);
        path_pub_->publish(msg);
        RCLCPP_INFO(this->get_logger(), "Calculated Path: %s", msg.data.c_str());
    }

    std::vector<Position> dijkstra(Position start, Position goal) {
        std::priority_queue<std::pair<int, Position>, std::vector<std::pair<int, Position>>, std::greater<>> queue;
        std::map<Position, Position> came_from;
        std::map<Position, int> cost_so_far;
        
        queue.push({0, start});
        cost_so_far[start] = 0;

        while (!queue.empty()) {
            auto current = queue.top().second;
            queue.pop();

            if (current == goal) {
                break;
            }

            for (const auto& neighbor : get_neighbors(current)) {
                int new_cost = cost_so_far[current] + map_[neighbor];
                if (cost_so_far.find(neighbor) == cost_so_far.end() || new_cost < cost_so_far[neighbor]) {
                    cost_so_far[neighbor] = new_cost;
                    queue.push({new_cost, neighbor});
                    came_from[neighbor] = current;
                }
            }
        }
        return reconstruct_path(came_from, start, goal);
    }

    std::vector<Position> get_neighbors(Position node) {
        std::vector<Position> neighbors;
        int x = node.first, y = node.second;
        for (const auto& [dx, dy] : {std::make_pair(-1, 0), std::make_pair(1, 0), std::make_pair(0, -1), std::make_pair(0, 1)}) {
            Position neighbor = {x + dx, y + dy};
            if (map_.find(neighbor) != map_.end()) {
                neighbors.push_back(neighbor);
            }
        }
        return neighbors;
    }

    std::vector<Position> reconstruct_path(const std::map<Position, Position>& came_from, Position start, Position goal) {
        std::vector<Position> path;
        Position current = goal;
        while (current != start) {
            path.push_back(current);
            current = came_from.at(current);
        }
        path.push_back(start);
        std::reverse(path.begin(), path.end());
        return path;
    }

    std::string vector_to_string(const std::vector<Position>& path) {
        std::string result;
        for (const auto& pos : path) {
            result += "(" + std::to_string(pos.first) + ", " + std::to_string(pos.second) + ") ";
        }
        return result;
    }
};

int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DijkstraNode>());
    rclcpp::shutdown();
    return 0;
}
