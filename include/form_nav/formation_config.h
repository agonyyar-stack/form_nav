#ifndef FORM_NAV_FORMATION_CONFIG_H
#define FORM_NAV_FORMATION_CONFIG_H

#include <map>
#include <string>
#include <vector>

#include <ros/node_handle.h>

struct FormationConfig {
    std::map<int, std::vector<double>> offsets;
    std::vector<int> neighbor_ids;

    static FormationConfig load(const ros::NodeHandle& node_handle,
                                const std::string& config_namespace,
                                int my_id);
};

#endif
