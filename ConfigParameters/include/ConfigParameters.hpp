//
// Created by wsh on 2021/8/25.
//

#ifndef AUTONOMOUS_NAVIGATION_CONFIGPARAMETERS_HPP
#define AUTONOMOUS_NAVIGATION_CONFIGPARAMETERS_HPP

#include <iostream>
#include "yaml-cpp/yaml.h"
#include <string>
#include <vector>
#include <mutex>
#include <atomic>
#include "AStar.hpp"

enum class Follow_task : int
{
    disable = 0,
    target_follow,
    route_follow,
    cin_points_follow
};

enum class Self_localization_mode : int
{
    only_lidar = 0,
    only_SE, // use only state estimation
    replace_lidarXY_SE, // use x, y given by state estimation, and theta by lidar
    replace_lidarTheta_SE, // use theta given by state estimation, and x, y by lidar
    replace_lidarTheta_HWT,
    xy_SE_theta_HWT,
};

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
        _planning_cfg.planning_period = 0.1;

        _task_cfg.start_planning = false;
        _task_cfg.target_follow = false;
        _task_cfg.route_follow = false;
        _task_cfg.cin_points_follow = false;

        _command_cfg.default_vel = 0.0;
        _command_cfg.max_angular_vel = 0.0;
        _command_cfg.udp_client.use_udp = false;
        _command_cfg.udp_client.ip_addr = "0.0.0.0";
        _command_cfg.udp_client.port = 0;
        _command_cfg.udp_client.send_period = 0;

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

        follow_task = Follow_task::disable;
        // route follow
        route_points.push_back({0, 0});
        route_distance_threshold = 0.0;
//        self_localization_with_lidar = false;
        self_localization_WO_lidar_period = 0;
//        receive_xy_from_NUC = false;
//        get_theta_from_HWT101 = false;
        self_localization_mode = Self_localization_mode::only_lidar;

        // monitor
        monitor_use_lcm = false;
        monitor_save_txt = false;

        no_block_map = true;
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
        _planning_cfg.planning_period = _cfg["planning"]["planning_period"].as<float>();

        _task_cfg.start_planning = _cfg["task"]["start_planning"].as<bool>();
        _task_cfg.target_follow = _cfg["task"]["target_follow"].as<bool>();
        _task_cfg.route_follow = _cfg["task"]["route_follow"]["enable"].as<bool>();
        _task_cfg.cin_points_follow = _cfg["task"]["cin_points_follow"].as<bool>();

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

        // public
        astar_cost_factors.weight_g = _cfg["planning"]["astar_cost_factors"]["weight_g"].as<float>();
        astar_cost_factors.weight_f = _cfg["planning"]["astar_cost_factors"]["weight_f"].as<float>();
        astar_cost_factors.turn = _cfg["planning"]["astar_cost_factors"]["turn"].as<float>();
        astar_cost_factors.move_diag = _cfg["planning"]["astar_cost_factors"]["move_diag"].as<float>();
        astar_cost_factors.weight_distant = _cfg["planning"]["astar_cost_factors"]["weight_distant"].as<float>();
        astar_cost_factors.max_distant_grid = _cfg["planning"]["astar_cost_factors"]["max_distant_grid"].as<int>();

        // route follow
        route_points = _cfg["task"]["route_follow"]["points"].as<std::vector<std::vector<float>>>();
        route_distance_threshold = _cfg["task"]["route_follow"]["distance_threshold"].as<float>();
//        self_localization_with_lidar = _cfg["task"]["route_follow"]["self_localization_with_lidar"].as<bool>();
        self_localization_WO_lidar_period = _cfg["task"]["route_follow"]["self_localization_WO_lidar_period"].as<float>();

        set_follow_task_with_cfg();

        set_selfLocalizationMode_with_cfg();

        // monitor
        monitor_use_lcm = _cfg["monitor"]["use_lcm"].as<bool>();
        monitor_save_txt = _cfg["monitor"]["save_txt"].as<bool>();

        no_block_map = _cfg["task"]["route_follow"]["no_block_map"].as<bool>();

//        receive_xy_from_NUC = _cfg["task"]["route_follow"]["receive_xy_from_NUC"].as<bool>();
//        get_theta_from_HWT101 = _cfg["task"]["route_follow"]["get_theta_from_HWT101"].as<bool>();
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

    float get_target_distance_threshold()
    {
        return _planning_cfg.target_distance_threshold;
    }

    float get_planning_period()
    {
        return _planning_cfg.planning_period;
    }

    bool get_start_planning() const
    {
        return _task_cfg.start_planning;
    }

    bool get_target_follow() const
    {
        return _task_cfg.target_follow;
    }

    bool get_route_follow() const
    {
        return _task_cfg.route_follow;
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

    void set_follow_task(Follow_task task_)
    {
        follow_task = task_;
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
        float planning_period;
        float default_forward_distance;
        float default_curve_radius;
        float target_distance_threshold;
    };
    struct task_parameters
    {
        std::atomic<bool> start_planning;
        std::atomic<bool> target_follow;
        std::atomic<bool> route_follow;
        std::atomic<bool> cin_points_follow;
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

    void set_follow_task_with_cfg()
    {
        if (_task_cfg.target_follow && (!_task_cfg.route_follow) && (!_task_cfg.cin_points_follow))
            follow_task = Follow_task::target_follow;
        else if ((!_task_cfg.target_follow) && _task_cfg.route_follow && (!_task_cfg.cin_points_follow))
            follow_task = Follow_task::route_follow;
        else if ((!_task_cfg.target_follow) && (!_task_cfg.route_follow) && _task_cfg.cin_points_follow)
            follow_task = Follow_task::cin_points_follow;
        else if ((!_task_cfg.target_follow) && (!_task_cfg.route_follow) && (!_task_cfg.cin_points_follow))
            follow_task = Follow_task::disable;
        else
        {
            std::cout
                    << "[ERROR] Following task config wrong in path_planning_config.yaml, only one follow_task can be true"
                    << std::endl;
            exit(-1);
        }
    }

    void set_selfLocalizationMode_with_cfg()
    {
        int mode = _cfg["task"]["route_follow"]["self_localization_mode"].as<int>();
        if (mode >= 0 && mode < 6)
            self_localization_mode = static_cast<Self_localization_mode>(mode);
        else
        {
            std::cout << "[ERROR] Self-localization mode config wrong in path_planning_config.yaml, (0 1 2 3 4 5)"
                      << std::endl;
            exit(-1);
        }
    }

public:
    bool show_lines;
    float line_k_max;

    bool show_map_image;

    AStar::Planning::astar_cost astar_cost_factors;

    Follow_task follow_task;
    std::vector<std::vector<float>> route_points;
    float route_distance_threshold;
//    bool self_localization_with_lidar;
    float self_localization_WO_lidar_period;
//    bool receive_xy_from_NUC;
//    bool get_theta_from_HWT101;
    Self_localization_mode self_localization_mode;

    bool monitor_use_lcm;
    bool monitor_save_txt;

    bool no_block_map;
};

#endif //AUTONOMOUS_NAVIGATION_CONFIGPARAMETERS_HPP
