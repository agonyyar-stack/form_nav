#include "form_nav/formation_config.h"

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
        !own_agent->hasMember("neighbors") || !own_agent->hasMember("leader_access")) {
        throw std::runtime_error(agents_path + "/uav" + std::to_string(my_id) +
                                 ": neighbors and leader_access are required");
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

    XmlRpc::XmlRpcValue& leader_access = (*own_agent)["leader_access"];
    if (leader_access.getType() != XmlRpc::XmlRpcValue::TypeBoolean) {
        throw std::runtime_error(agents_path + "/uav" + std::to_string(my_id) +
                                 "/leader_access: expected true or false");
    }
    config.leader_access = static_cast<bool>(leader_access);
    return config;
}
