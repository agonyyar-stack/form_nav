#ifndef FORM_NAV_FORMATION_CONFIG_H
#define FORM_NAV_FORMATION_CONFIG_H

#include <map>
#include <string>
#include <vector>

#include <ros/node_handle.h>

struct FormationConfig {
    std::map<int, std::vector<double>> offsets;
    std::vector<int> neighbor_ids;

    std::string formation_mode = "offset";
    std::vector<int> leader_ids;
    std::map<int, std::map<int, double>> weights;

    bool isLeader(int id) const;
    bool isFollower(int id) const;
    bool hasWeights(int id) const;

    static FormationConfig load(const ros::NodeHandle& node_handle,
                                const std::string& config_namespace,
                                int my_id);
};

#endif
