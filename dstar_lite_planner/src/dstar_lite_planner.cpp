#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>

#include <unordered_map>
#include <queue>
#include <vector>
#include <cmath>
#include <limits>
#include <algorithm>
#include <mutex>
#include <std_msgs/msg/bool.hpp>
#include <optional>

struct Cell {
    int row, col;
    bool operator==(const Cell& o) const { return row == o.row && col == o.col; }
};

struct CellHash {
    size_t operator()(const Cell& c) const {
        return std::hash<int>()(c.row * 100003 + c.col);
    }
};

using Key = std::pair<double, double>;

struct PQEntry {
    Key  key;
    Cell cell;
    bool operator>(const PQEntry& o) const { return key > o.key; }
};

static constexpr double INF          = std::numeric_limits<double>::infinity();
static constexpr int    LETHAL       = 100;
static constexpr double UNKNOWN_COST = 50.0;

class DStarLite {
public:
    void init(int rows, int cols,
              const std::vector<int8_t>& costmap,
              Cell start, Cell goal)
    {
        rows_    = rows;
        cols_    = cols;
        costmap_ = costmap;
        start_   = start;
        goal_    = goal;
        km_      = 0.0;
        g_.clear();
        rhs_.clear();
        entry_finder_.clear();
        while (!pq_.empty()) pq_.pop();
        rhs_[goal_] = 0.0;
        push(goal_);
    }

    void update_costmap(const std::vector<int8_t>& new_costmap)
    {
        costmap_ = new_costmap;
        init(rows_, cols_, new_costmap, start_, goal_);
        compute_shortest_path();
    }

    void set_start(Cell s)
    {
        km_        += heuristic(last_start_, start_);
        last_start_ = start_;
        start_      = s;
    }

    void compute_shortest_path()
    {
        const size_t max_iterations = static_cast<size_t>(rows_) * cols_ * 20;
        size_t iterations = 0;

        while (!pq_.empty()) {
            if (++iterations > max_iterations) break;
            if (pq_.size() > max_iterations) break;

            Key top_k = top_key();
            Key start_k = calc_key(start_);
            bool start_consistent = (get_g(start_) == get_rhs(start_));

            if (top_k >= start_k && start_consistent) break;

            Cell u;
            Key  k_old;
            if (!pop_valid(k_old, u)) break;

            Key k_new = calc_key(u);
            if (k_old < k_new) {
                push(u);
            } else if (get_g(u) > get_rhs(u)) {
                g_[u] = get_rhs(u);
                for (auto& pred : neighbors(u)) update_vertex(pred);
            } else {
                g_[u] = INF;
                update_vertex(u);
                for (auto& pred : neighbors(u)) update_vertex(pred);
            }
        }
    }

    std::vector<Cell> extract_path()
    {
        std::vector<Cell> path;
        Cell cur = start_;
        for (int iter = 0; iter < rows_ * cols_; ++iter) {
            path.push_back(cur);
            if (cur == goal_) break;
            double best_cost = INF;
            Cell   best_next = cur;
            for (auto& nb : neighbors(cur)) {
                double c = edge_cost(cur, nb) + get_g(nb);
                if (c < best_cost) { best_cost = c; best_next = nb; }
            }
            if (best_next == cur || best_cost >= INF) { path.clear(); break; }
            cur = best_next;
        }
        return path;
    }

    bool has_path() { return get_g(start_) < INF; }

private:
    int rows_, cols_;
    std::vector<int8_t> costmap_;
    Cell   start_, goal_;
    Cell   last_start_ = {0, 0};
    double km_ = 0.0;

    std::unordered_map<Cell, double, CellHash> g_, rhs_;
    std::unordered_map<Cell, Key, CellHash>    entry_finder_;
    std::priority_queue<PQEntry, std::vector<PQEntry>, std::greater<PQEntry>> pq_;

    double get_g(const Cell& c)   const { auto it = g_.find(c);   return it == g_.end()   ? INF : it->second; }
    double get_rhs(const Cell& c) const { auto it = rhs_.find(c); return it == rhs_.end() ? INF : it->second; }

    double heuristic(const Cell& a, const Cell& b) const {
        double dr = std::abs(a.row - b.row);
        double dc = std::abs(a.col - b.col);
        return std::max(dr, dc) + (std::sqrt(2.0) - 1.0) * std::min(dr, dc);
    }

    Key calc_key(const Cell& c) const {
        double g   = get_g(c);
        double rhs = get_rhs(c);
        double mn  = std::min(g, rhs);
        return {mn + heuristic(start_, c) + km_, mn};
    }

    void push(const Cell& c) {
        Key k = calc_key(c);
        entry_finder_[c] = k;
        pq_.push({k, c});
    }

    bool pop_valid(Key& k_out, Cell& c_out) {
        while (!pq_.empty()) {
            PQEntry top = pq_.top();
            pq_.pop();
            auto it = entry_finder_.find(top.cell);
            if (it != entry_finder_.end() && it->second == top.key) {
                entry_finder_.erase(it);
                k_out = top.key;
                c_out = top.cell;
                return true;
            }
        }
        return false;
    }

    Key top_key() {
        while (!pq_.empty()) {
            PQEntry top = pq_.top();
            auto it = entry_finder_.find(top.cell);
            if (it != entry_finder_.end() && it->second == top.key) {
                return top.key;
            }
            pq_.pop();
        }
        return {INF, INF};
    }

    double cell_cost(const Cell& c) const {
        if (c.row < 0 || c.row >= rows_ || c.col < 0 || c.col >= cols_) return INF;
        int8_t v = costmap_[c.row * cols_ + c.col];
        if (v == LETHAL) return INF;
        if (v == -1)     return UNKNOWN_COST;
        return 1.0 + static_cast<double>(v) / 10.0;
    }

    double edge_cost(const Cell& a, const Cell& b) const {
        double base = cell_cost(b);
        if (base >= INF) return INF;
        bool diag = (a.row != b.row && a.col != b.col);
        return diag ? base * std::sqrt(2.0) : base;
    }

    std::vector<Cell> neighbors(const Cell& c) const {
        std::vector<Cell> nb;
        nb.reserve(8);
        for (int dr = -1; dr <= 1; ++dr)
            for (int dc = -1; dc <= 1; ++dc) {
                if (dr == 0 && dc == 0) continue;
                Cell n{c.row + dr, c.col + dc};
                if (n.row >= 0 && n.row < rows_ && n.col >= 0 && n.col < cols_)
                    nb.push_back(n);
            }
        return nb;
    }

    void update_vertex(const Cell& u)
    {
        if (!(u == goal_)) {
            double min_rhs = INF;
            for (auto& nb : neighbors(u)) {
                double c = edge_cost(u, nb) + get_g(nb);
                if (c < min_rhs) min_rhs = c;
            }
            rhs_[u] = min_rhs;
        }

        entry_finder_.erase(u);

        if (get_g(u) != get_rhs(u)) push(u);
    }
};

class DStarLitePlannerNode : public rclcpp::Node {
public:
    DStarLitePlannerNode() : Node("dstar_lite_planner")
    {
        costmap_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
            "/costmap", 10,
            [this](nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
                std::lock_guard<std::mutex> lock(mutex_);
                costmap_msg_     = msg;
                costmap_updated_ = true;
            });

        goal_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
            "/goal_pose", 10,
            [this](geometry_msgs::msg::PoseStamped::SharedPtr msg) {
                std::lock_guard<std::mutex> lock(mutex_);
                goal_msg_ = msg;
                new_goal_ = true;
                RCLCPP_INFO(get_logger(), "New goal: (%.2f, %.2f)",
                    msg->pose.position.x, msg->pose.position.y);
            });

        odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 10,
            [this](nav_msgs::msg::Odometry::SharedPtr msg) {
                std::lock_guard<std::mutex> lock(mutex_);
                odom_msg_ = msg;
            });

        cmd_vel_pub_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel_raw", 10);
        path_pub_    = create_publisher<nav_msgs::msg::Path>("/planned_path", 10);

        timer_ = create_wall_timer(
            std::chrono::milliseconds(200),
            [this]() { plan_and_control(); });

        RCLCPP_INFO(get_logger(), "D* Lite planner ready.");
    }

private:
    nav_msgs::msg::OccupancyGrid::SharedPtr    costmap_msg_;
    geometry_msgs::msg::PoseStamped::SharedPtr goal_msg_;
    nav_msgs::msg::Odometry::SharedPtr         odom_msg_;

    bool costmap_updated_ = false;
    bool obstacle_blocked_ = false;
    bool new_goal_        = false;
    bool planning_active_ = false;
    bool recovering_      = false;
    int  recovery_cycles_ = 0;
    static constexpr int RECOVERY_DURATION = 10;

    std::vector<Cell> current_path_;
    int               path_index_ = 0;
    DStarLite         dstar_;
    std::mutex        mutex_;

    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr    costmap_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr              blocked_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr         odom_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr          cmd_vel_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr                path_pub_;
    rclcpp::TimerBase::SharedPtr                                     timer_;

    Cell world_to_cell(double x, double y,
                       const nav_msgs::msg::MapMetaData& info) const
    {
        return {
            static_cast<int>((y - info.origin.position.y) / info.resolution),
            static_cast<int>((x - info.origin.position.x) / info.resolution)
        };
    }

    std::pair<double, double> cell_to_world(const Cell& c,
                       const nav_msgs::msg::MapMetaData& info) const
    {
        double x = info.origin.position.x + (c.col + 0.5) * info.resolution;
        double y = info.origin.position.y + (c.row + 0.5) * info.resolution;
        return {x, y};
    }

    void stop_robot()
    {
        geometry_msgs::msg::Twist t;
        cmd_vel_pub_->publish(t);
    }

    void plan_and_control()
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!costmap_msg_ || !odom_msg_) return;

        auto& info = costmap_msg_->info;
        double rx  = odom_msg_->pose.pose.position.x;
        double ry  = odom_msg_->pose.pose.position.y;
        Cell robot_cell = world_to_cell(rx, ry, info);

        if (new_goal_ && goal_msg_) {
            Cell goal_cell = world_to_cell(
                goal_msg_->pose.position.x,
                goal_msg_->pose.position.y, info);

            RCLCPP_INFO(get_logger(), "Planning (%d,%d) -> (%d,%d)",
                robot_cell.row, robot_cell.col,
                goal_cell.row,  goal_cell.col);

            dstar_.init(info.height, info.width,
                        costmap_msg_->data, robot_cell, goal_cell);
            dstar_.compute_shortest_path();
            current_path_    = dstar_.extract_path();
            path_index_      = 0;
            new_goal_        = false;
            planning_active_ = !current_path_.empty();

            if (!planning_active_) {
                RCLCPP_WARN(get_logger(), "No path found - starting recovery backup.");
                recovering_ = true;
                recovery_cycles_ = 0;
            }

            publish_path(info);

        } else if (costmap_updated_ && planning_active_) {
            dstar_.set_start(robot_cell);
            dstar_.update_costmap(costmap_msg_->data);
            current_path_    = dstar_.extract_path();
            path_index_      = 0;
            costmap_updated_ = false;
            planning_active_ = !current_path_.empty();
            publish_path(info);
        }

        // Recovery behavior: back up if no path found
        if (recovering_) {
            geometry_msgs::msg::Twist cmd;
            cmd.linear.x = -0.05;
            cmd_vel_pub_->publish(cmd);
            recovery_cycles_++;
            if (recovery_cycles_ >= RECOVERY_DURATION) {
                recovering_ = false;
                recovery_cycles_ = 0;
                costmap_updated_ = true;
                RCLCPP_INFO(get_logger(), "Recovery done - replanning.");
            }
            return;
        }

        if (!planning_active_ || current_path_.empty()) {
            stop_robot();
            return;
        }

        while (path_index_ < (int)current_path_.size() - 1) {
            auto [wx, wy] = cell_to_world(current_path_[path_index_], info);
            double dist   = std::hypot(rx - wx, ry - wy);
            if (dist < info.resolution * 1.5) ++path_index_;
            else break;
        }

        if (path_index_ >= (int)current_path_.size() - 1) {
            RCLCPP_INFO(get_logger(), "Goal reached.");
            stop_robot();
            planning_active_ = false;
            return;
        }

        // Safety check: look ahead 5 waypoints for lethal or high cost cells
        int lookahead = std::min(path_index_ + 5, (int)current_path_.size());
        for (int i = path_index_; i < lookahead; ++i) {
            const Cell& c = current_path_[i];
            if (c.row >= 0 && c.row < (int)info.height &&
                c.col >= 0 && c.col < (int)info.width) {
                int8_t cost = costmap_msg_->data[c.row * info.width + c.col];
                if (cost >= 80 || cost == -1) {
                    RCLCPP_WARN(get_logger(), "Obstacle detected ahead - stopping and replanning.");
                    stop_robot();
                    costmap_updated_ = true;
                    path_index_ = 0;
                    return;
                }
            }
        }

        auto [tx, ty]    = cell_to_world(current_path_[path_index_], info);
        double dx        = tx - rx;
        double dy        = ty - ry;
        double target_heading = std::atan2(dy, dx);

        auto& q = odom_msg_->pose.pose.orientation;
        double robot_yaw = std::atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z));

        double heading_error = target_heading - robot_yaw;
        while (heading_error >  M_PI) heading_error -= 2.0 * M_PI;
        while (heading_error < -M_PI) heading_error += 2.0 * M_PI;

        geometry_msgs::msg::Twist cmd;
        double turn_factor = std::max(0.0, 1.0 - std::abs(heading_error) / M_PI);
        cmd.linear.x  = 0.2 * turn_factor;
        cmd.angular.z = std::clamp(1.5 * heading_error, -1.0, 1.0);
        cmd_vel_pub_->publish(cmd);
    }

    void publish_path(const nav_msgs::msg::MapMetaData& info)
    {
        nav_msgs::msg::Path path_msg;
        path_msg.header.stamp    = get_clock()->now();
        path_msg.header.frame_id = "map";
        for (auto& cell : current_path_) {
            geometry_msgs::msg::PoseStamped ps;
            ps.header = path_msg.header;
            auto [x, y] = cell_to_world(cell, info);
            ps.pose.position.x  = x;
            ps.pose.position.y  = y;
            ps.pose.orientation.w = 1.0;
            path_msg.poses.push_back(ps);
        }
        path_pub_->publish(path_msg);
    }
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DStarLitePlannerNode>());
    rclcpp::shutdown();
    return 0;
}
