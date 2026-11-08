#include "form_nav/formation_config.h"

#include <algorithm>
#include <cmath>
#include <set>
#include <stdexcept>

#include <xmlrpcpp/XmlRpcValue.h>

namespace {

std::string parameterPath(const std::string& config_namespace,
                          const std::string& field) {
    std::string root = config_namespace.empty() ? "/formation_config" : config_namespace;
    if (root.front() != '/') root.insert(root.begin(), '/');
    while (root.size() > 1 && root.back() == '/') root.pop_back();
    return root + "/" + field;
}

int parseAgentId(const std::string& key, const std::string& parameter_path) {
    std::string id_text = key;
    if (id_text.rfind("uav", 0) == 0) id_text.erase(0, 3);

    size_t parsed_chars = 0;
    int id = -1;
    try {
        id = std::stoi(id_text, &parsed_chars);
    } catch (const std::exception&) {
        throw std::runtime_error(parameter_path + ": invalid UAV key '" + key + "'");
    }
    if (id < 0 || parsed_chars != id_text.size()) {
        throw std::runtime_error(parameter_path + ": invalid UAV key '" + key + "'");
    }
    return id;
}

double parseNumber(XmlRpc::XmlRpcValue& value, const std::string& context) {
    if (value.getType() == XmlRpc::XmlRpcValue::TypeInt) {
        return static_cast<int>(value);
    }
    if (value.getType() == XmlRpc::XmlRpcValue::TypeDouble) {
        return static_cast<double>(value);
    }
    throw std::runtime_error(context + ": expected a numeric value");
}

}  // namespace

bool FormationConfig::isLeader(int id) const {
    return std::find(leader_ids.begin(), leader_ids.end(), id) !=
           leader_ids.end();
}

bool FormationConfig::isFollower(int id) const {
    return !isLeader(id) && hasWeights(id);
}

bool FormationConfig::hasWeights(int id) const {
    return weights.count(id) != 0;
}

FormationConfig FormationConfig::load(const ros::NodeHandle& node_handle,
                                      const std::string& config_namespace,
                                      int my_id) {
    FormationConfig config;

    const std::string offsets_path = parameterPath(config_namespace, "offsets");
    XmlRpc::XmlRpcValue offsets_value;
    if (!node_handle.getParam(offsets_path, offsets_value)) {
        throw std::runtime_error("missing formation parameter: " + offsets_path);
    }
    if (offsets_value.getType() != XmlRpc::XmlRpcValue::TypeStruct) {
        throw std::runtime_error(offsets_path + ": expected a UAV-to-offset mapping");
    }

    for (auto it = offsets_value.begin(); it != offsets_value.end(); ++it) {
        const int id = parseAgentId(it->first, offsets_path);
        XmlRpc::XmlRpcValue& offset_value = it->second;
        if (offset_value.getType() != XmlRpc::XmlRpcValue::TypeArray ||
            offset_value.size() != 4) {
            throw std::runtime_error(offsets_path + "/" + it->first +
                                     ": expected [x, y, z, yaw]");
        }
        if (config.offsets.count(id) != 0) {
            throw std::runtime_error(offsets_path + ": duplicate UAV id " +
                                     std::to_string(id));
        }

        std::vector<double> offset;
        offset.reserve(4);
        for (int index = 0; index < 4; ++index) {
            offset.push_back(parseNumber(
                offset_value[index], offsets_path + "/" + it->first));
        }
        config.offsets.emplace(id, std::move(offset));
    }

    if (config.offsets.count(my_id) == 0) {
        throw std::runtime_error(offsets_path + ": no offset configured for uav" +
                                 std::to_string(my_id));
    }

    const std::string agents_path = parameterPath(config_namespace, "agents");
    XmlRpc::XmlRpcValue agents_value;
    if (!node_handle.getParam(agents_path, agents_value)) {
        throw std::runtime_error("missing formation parameter: " + agents_path);
    }
    if (agents_value.getType() != XmlRpc::XmlRpcValue::TypeStruct) {
        throw std::runtime_error(agents_path + ": expected a UAV configuration mapping");
    }

    XmlRpc::XmlRpcValue* own_agent = nullptr;
    for (auto it = agents_value.begin(); it != agents_value.end(); ++it) {
        if (parseAgentId(it->first, agents_path) == my_id) {
            own_agent = &it->second;
            break;
        }
    }
    if (own_agent == nullptr) {
        throw std::runtime_error(agents_path + ": no configuration for uav" +
                                 std::to_string(my_id));
    }
    if (own_agent->getType() != XmlRpc::XmlRpcValue::TypeStruct ||
        !own_agent->hasMember("neighbors")) {
        throw std::runtime_error(agents_path + "/uav" + std::to_string(my_id) +
                                 ": neighbors is required");
    }

    XmlRpc::XmlRpcValue& neighbors = (*own_agent)["neighbors"];
    if (neighbors.getType() != XmlRpc::XmlRpcValue::TypeArray) {
        throw std::runtime_error(agents_path + "/uav" + std::to_string(my_id) +
                                 "/neighbors: expected an array");
    }
    std::set<int> unique_neighbors;
    for (int index = 0; index < neighbors.size(); ++index) {
        if (neighbors[index].getType() != XmlRpc::XmlRpcValue::TypeInt) {
            throw std::runtime_error(agents_path + "/uav" + std::to_string(my_id) +
                                     "/neighbors: IDs must be integers");
        }
        const int neighbor_id = static_cast<int>(neighbors[index]);
        if (neighbor_id < 0 || neighbor_id == my_id) {
            throw std::runtime_error(agents_path + "/uav" + std::to_string(my_id) +
                                     "/neighbors: invalid neighbor id " +
                                     std::to_string(neighbor_id));
        }
        if (config.offsets.count(neighbor_id) == 0) {
            throw std::runtime_error(offsets_path + ": no offset configured for neighbor uav" +
                                     std::to_string(neighbor_id));
        }
        if (!unique_neighbors.insert(neighbor_id).second) {
            throw std::runtime_error(agents_path + "/uav" + std::to_string(my_id) +
                                     "/neighbors: duplicate id " +
                                     std::to_string(neighbor_id));
        }
        config.neighbor_ids.push_back(neighbor_id);
    }

    const std::string mode_path = parameterPath(config_namespace, "formation_mode");
    if (!node_handle.getParam(mode_path, config.formation_mode)) {
        config.formation_mode = "offset";
    }
    if (config.formation_mode != "offset" &&
        config.formation_mode != "barycentric") {
        throw std::runtime_error(
            mode_path + ": expected 'offset' or 'barycentric'");
    }

    if (config.formation_mode == "offset") {
        return config;
    }

    const std::string leaders_path = parameterPath(config_namespace, "leaders");
    XmlRpc::XmlRpcValue leaders_value;
    if (!node_handle.getParam(leaders_path, leaders_value) ||
        leaders_value.getType() != XmlRpc::XmlRpcValue::TypeArray) {
        throw std::runtime_error(
            leaders_path + ": barycentric mode requires an integer array");
    }

    std::set<int> unique_leaders;
    for (int index = 0; index < leaders_value.size(); ++index) {
        if (leaders_value[index].getType() != XmlRpc::XmlRpcValue::TypeInt) {
            throw std::runtime_error(leaders_path + ": IDs must be integers");
        }
        const int leader_id = static_cast<int>(leaders_value[index]);
        if (config.offsets.count(leader_id) == 0) {
            throw std::runtime_error(
                leaders_path + ": no offset configured for leader uav" +
                std::to_string(leader_id));
        }
        if (!unique_leaders.insert(leader_id).second) {
            throw std::runtime_error(
                leaders_path + ": duplicate id " + std::to_string(leader_id));
        }
        config.leader_ids.push_back(leader_id);
    }
    if (config.leader_ids.size() != 3) {
        throw std::runtime_error(
            leaders_path + ": barycentric mode requires exactly three leaders");
    }

    const std::vector<double>& p0 = config.offsets.at(config.leader_ids[0]);
    const std::vector<double>& p1 = config.offsets.at(config.leader_ids[1]);
    const std::vector<double>& p2 = config.offsets.at(config.leader_ids[2]);
    const double triangle_cross =
        (p1[0] - p0[0]) * (p2[1] - p0[1]) -
        (p1[1] - p0[1]) * (p2[0] - p0[0]);
    if (std::abs(triangle_cross) < 1e-6) {
        throw std::runtime_error(
            leaders_path + ": leader triangle is degenerate in the xy plane");
    }

    const std::string weights_path = parameterPath(config_namespace, "weights");
    XmlRpc::XmlRpcValue weights_value;
    if (!node_handle.getParam(weights_path, weights_value) ||
        weights_value.getType() != XmlRpc::XmlRpcValue::TypeStruct) {
        throw std::runtime_error(
            weights_path + ": barycentric mode requires a UAV-to-weight mapping");
    }

    for (auto it = weights_value.begin(); it != weights_value.end(); ++it) {
        const int follower_id = parseAgentId(it->first, weights_path);
        if (config.offsets.count(follower_id) == 0) {
            throw std::runtime_error(
                weights_path + "/" + it->first + ": no matching offset");
        }
        if (unique_leaders.count(follower_id) != 0) {
            throw std::runtime_error(
                weights_path + "/" + it->first +
                ": a leader cannot have follower weights");
        }
        if (it->second.getType() != XmlRpc::XmlRpcValue::TypeStruct) {
            throw std::runtime_error(
                weights_path + "/" + it->first +
                ": expected a leader-to-weight mapping");
        }

        std::map<int, double> follower_weights;
        std::set<int> configured_leaders;
        double weight_sum = 0.0;
        for (auto jt = it->second.begin(); jt != it->second.end(); ++jt) {
            const int leader_id =
                parseAgentId(jt->first, weights_path + "/" + it->first);
            if (unique_leaders.count(leader_id) == 0) {
                throw std::runtime_error(
                    weights_path + "/" + it->first + "/" + jt->first +
                    ": key is not a configured leader");
            }
            const double weight = parseNumber(
                jt->second, weights_path + "/" + it->first + "/" + jt->first);
            if (!std::isfinite(weight) || weight < 0.0) {
                throw std::runtime_error(
                    weights_path + "/" + it->first + "/" + jt->first +
                    ": weight must be finite and non-negative");
            }
            follower_weights[leader_id] = weight;
            configured_leaders.insert(leader_id);
            weight_sum += weight;
        }

        if (configured_leaders != unique_leaders) {
            throw std::runtime_error(
                weights_path + "/" + it->first +
                ": every leader must have exactly one weight");
        }
        if (std::abs(weight_sum - 1.0) > 1e-6) {
            throw std::runtime_error(
                weights_path + "/" + it->first +
                ": weights must sum to one");
        }

        const std::vector<double>& follower_offset =
            config.offsets.at(follower_id);
        for (int axis = 0; axis < 3; ++axis) {
            double reconstructed = 0.0;
            for (int leader_id : config.leader_ids) {
                reconstructed +=
                    follower_weights.at(leader_id) *
                    config.offsets.at(leader_id)[axis];
            }
            if (std::abs(reconstructed - follower_offset[axis]) > 1e-4) {
                throw std::runtime_error(
                    weights_path + "/" + it->first +
                    ": weights do not reconstruct the nominal offset");
            }
        }

        config.weights[follower_id] = std::move(follower_weights);
    }

    for (const auto& [id, offset] : config.offsets) {
        (void)offset;
        if (unique_leaders.count(id) == 0 && config.weights.count(id) == 0) {
            throw std::runtime_error(
                weights_path + ": missing weights for follower uav" +
                std::to_string(id));
        }
    }

    return config;
}
