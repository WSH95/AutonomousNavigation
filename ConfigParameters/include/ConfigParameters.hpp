//
// Created by wsh on 2021/8/25.
//

#ifndef AUTONOMOUS_NAVIGATION_CONFIGPARAMETERS_HPP
#define AUTONOMOUS_NAVIGATION_CONFIGPARAMETERS_HPP

#include "yaml-cpp/yaml.h"
#include <string>
#include <vector>
#include <mutex>
#include <atomic>
#include "AStar.hpp"

class ConfigParameters
{
public:
    ConfigParameters()
    {
        _map_cfg.map_grid_resolution = 0.0;
        _map_cfg.add_border_line = false;
        _map_cfg.init_direction = (std::vector<int>) {0, 0};

        _robot_cfg.lidar_pos_bias = (std::vector<float>) {0.0, 0.0};
        _robot_cfg.self_block_radius = 0.0;

        _planning_cfg.default_forward_distance = 0.0;
        _planning_cfg.default_curve_radius = 0.0;
        _planning_cfg.target_distance_threshold = 0.0;

        _task_cfg.start_planning = false;
        _task_cfg.target_follow = false;

        _command_cfg.default_vel = 0.0;
        _command_cfg.max_angular_vel = 0.0;
        _command_cfg.udp_client.use_udp = false;
        _command_cfg.udp_client.ip_addr = "0.0.0.0";
        _command_cfg.udp_client.port = 0;

        // public
        show_lines = false;
        line_k_max = 1000;

        show_map_image = false;

        // public
        astar_cost_factors.weight_g = 1.2;
        astar_cost_factors.weight_f = 0.8;
        astar_cost_factors.turn = 1.5;
        astar_cost_factors.move_diag = 1.414;
        astar_cost_factors.weight_distant = 20.0;
        astar_cost_factors.max_distant_grid = 10;

    }

    explicit ConfigParameters(const std::string &cfgPath)
    {
        _cfg = YAML::LoadFile(cfgPath);

        _map_cfg.map_grid_resolution = _cfg["map"]["map_grid_resolution"].as<float>();
        _map_cfg.add_border_line = _cfg["map"]["add_border_line"].as<bool>();
        _map_cfg.init_direction = _cfg["map"]["init_direction"].as<std::vector<int>>();

        _robot_cfg.lidar_pos_bias = _cfg["robot"]["lidar_pos_bias"].as<std::vector<float>>();
        _robot_cfg.self_block_radius = _cfg["robot"]["self_block_radius"].as<float>();

        _planning_cfg.default_forward_distance = _cfg["planning"]["default_forward_distance"].as<float>();
        _planning_cfg.default_curve_radius = _cfg["planning"]["default_curve_radius"].as<float>();
        _planning_cfg.target_distance_threshold = _cfg["planning"]["target_distance_threshold"].as<float>();

        _task_cfg.start_planning = _cfg["task"]["start_planning"].as<bool>();
        _task_cfg.target_follow = _cfg["task"]["target_follow"].as<bool>();

        _command_cfg.default_vel = _cfg["command"]["default_vel"].as<float>();
        _command_cfg.max_angular_vel = _cfg["command"]["max_angular_vel"].as<float>();
        _command_cfg.udp_client.use_udp = _cfg["command"]["udp_client"]["use_udp"].as<bool>();
        _command_cfg.udp_client.ip_addr = _cfg["command"]["udp_client"]["ip_addr"].as<std::string>();
        _command_cfg.udp_client.port = _cfg["command"]["udp_client"]["port"].as<int>();
        _command_cfg.udp_client.send_period = _cfg["command"]["udp_client"]["send_period"].as<float>();

        // public
        show_lines = _cfg["show"]["lines"].as<bool>();
        line_k_max = _cfg["lines"]["k_max"].as<float>();

        show_map_image = _cfg["show"]["map_image"].as<bool>();

        //public
        astar_cost_factors.weight_g = _cfg["planning"]["astar_cost_factors"]["weight_g"].as<float>();
        astar_cost_factors.weight_f = _cfg["planning"]["astar_cost_factors"]["weight_f"].as<float>();
        astar_cost_factors.turn = _cfg["planning"]["astar_cost_factors"]["turn"].as<float>();
        astar_cost_factors.move_diag = _cfg["planning"]["astar_cost_factors"]["move_diag"].as<float>();
        astar_cost_factors.weight_distant = _cfg["planning"]["astar_cost_factors"]["weight_distant"].as<float>();
        astar_cost_factors.max_distant_grid = _cfg["planning"]["astar_cost_factors"]["max_distant_grid"].as<int>();
    }

    ~ConfigParameters() = default;

    float get_map_grid_resolution() const
    {
        return _map_cfg.map_grid_resolution;
    }

    bool get_add_border_line() const
    {
        return _map_cfg.add_border_line;
    }

    std::vector<int> get_init_direction() const
    {
        return _map_cfg.init_direction;
    }

    std::vector<float> get_lidar_pos_bias() const
    {
        return _robot_cfg.lidar_pos_bias;
    }

    std::vector<int> get_lidar_pos_bias_grid() const
    {
        return {(int) (_robot_cfg.lidar_pos_bias[0] / _map_cfg.map_grid_resolution),
                (int) (_robot_cfg.lidar_pos_bias[1] / _map_cfg.map_grid_resolution)};
    }

    float get_self_block_radius() const
    {
        return _robot_cfg.self_block_radius;
    }

    int get_self_block_radius_grid() const
    {
        return (int) (_robot_cfg.self_block_radius / _map_cfg.map_grid_resolution);
    }

    float get_default_forward_distance()
    {
        std::lock_guard<std::mutex> lk(_lock_default_forward_distance);
        return _planning_cfg.default_forward_distance;
    }

    int get_default_forward_distance_grid()
    {
        std::lock_guard<std::mutex> lk(_lock_default_forward_distance);
        return (int) (_planning_cfg.default_forward_distance / _map_cfg.map_grid_resolution);
    }

    float get_default_curve_radius()
    {
        std::lock_guard<std::mutex> lk(_lock_default_curve_radius);
        return _planning_cfg.default_curve_radius;
    }

    float get_default_curve_radius_grid()
    {
        std::lock_guard<std::mutex> lk(_lock_default_curve_radius);
        return _planning_cfg.default_curve_radius / _map_cfg.map_grid_resolution;
    }

    float get_target_distance_threshold_grid()
    {
        return _planning_cfg.target_distance_threshold / _map_cfg.map_grid_resolution;
    }

    bool get_start_planning() const
    {
        return _task_cfg.start_planning;
    }

    bool get_target_follow() const
    {
        return _task_cfg.target_follow;
    }

    float get_default_vel()
    {
        std::lock_guard<std::mutex> lk(_lock_default_vel);
        return _command_cfg.default_vel;
    }

    float get_max_angular_vel()
    {
        std::lock_guard<std::mutex> lk(_lock_max_angular_vel);
        return _command_cfg.max_angular_vel;
    }

    bool get_use_udp() const
    {
        return _command_cfg.udp_client.use_udp;
    }

    std::string get_ip_addr() const
    {
        return _command_cfg.udp_client.ip_addr;
    }

    int get_port() const
    {
        return _command_cfg.udp_client.port;
    }

    float get_send_period() const
    {
        return _command_cfg.udp_client.send_period;
    }

    void set_add_border_line(bool state_)
    {
        _map_cfg.add_border_line.exchange(state_);
    }

    void set_default_forward_distance(float value_)
    {
        std::lock_guard<std::mutex> lk(_lock_default_forward_distance);
        _planning_cfg.default_forward_distance = value_;
    }

    void set_default_curve_radius(float value_)
    {
        std::lock_guard<std::mutex> lk(_lock_default_curve_radius);
        _planning_cfg.default_curve_radius = value_;
    }

    void set_start_planning(bool state_)
    {
        _task_cfg.start_planning.exchange(state_);
    }

    void set_target_follow(bool state_)
    {
        _task_cfg.target_follow.exchange(state_);
    }

    void set_default_vel(float value_)
    {
        std::lock_guard<std::mutex> lk(_lock_default_vel);
        _command_cfg.default_vel = value_;
    }

    void set_max_angular_vel(float value_)
    {
        std::lock_guard<std::mutex> lk(_lock_max_angular_vel);
        _command_cfg.max_angular_vel = value_;
    }

    void set_use_udp(bool state_)
    {
        _command_cfg.udp_client.use_udp.exchange(state_);
    }

private:
    YAML::Node _cfg;

    struct map_parameters
    {
        float map_grid_resolution;
        std::atomic<bool> add_border_line;
        std::vector<int> init_direction;
    };
    struct robot_parameters
    {
        std::vector<float> lidar_pos_bias;
        float self_block_radius;
    };

    struct planning_parameters
    {
        float default_forward_distance;
        float default_curve_radius;
        float target_distance_threshold;
    };
    struct task_parameters
    {
        std::atomic<bool> start_planning;
        std::atomic<bool> target_follow;
    };
    struct command_parameters
    {
        float default_vel;
        float max_angular_vel;
        struct udp_parameters
        {
            std::atomic<bool> use_udp;
            std::string ip_addr;
            int port;
            float send_period;
        };
        udp_parameters udp_client;
    };

    map_parameters _map_cfg;
    robot_parameters _robot_cfg;
    planning_parameters _planning_cfg;
    task_parameters _task_cfg;
    command_parameters _command_cfg;

    std::mutex _lock_default_forward_distance, _lock_default_curve_radius, _lock_default_vel, _lock_max_angular_vel;

public:
    bool show_lines;
    float line_k_max;

    bool show_map_image;

    AStar::Planning::astar_cost astar_cost_factors;
};

#endif //AUTONOMOUS_NAVIGATION_CONFIGPARAMETERS_HPP
