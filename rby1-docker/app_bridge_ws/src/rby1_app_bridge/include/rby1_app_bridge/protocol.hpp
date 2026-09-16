#pragma once

#include <string>

#include <nlohmann/json.hpp>

#include "rby1_app_bridge/component_state.hpp"

namespace rby1_app_bridge
{

using Json = nlohmann::json;

inline Json component_status_json(
  const ComponentSnapshot &component)
{
  Json result = {
    {"known", component.known()},
    {"enabled", nullptr},
    {"pending", component.pending},
    {"source", component.source}
  };

  if (component.known())
  {
    result["enabled"] = component.enabled();
  }

  return result;
}

inline Json components_status_json(
  const ComponentStateSnapshot &components)
{
  return {
    {"power", component_status_json(components.power)},
    {"servo", component_status_json(components.servo)},
    {"stream", component_status_json(components.stream)}
  };
}

inline bool legacy_component_enabled(
  const ComponentSnapshot &component)
{
  return component.known() && component.enabled();
}

inline Json parse_ndjson_request(
  const std::string &line)
{
  return Json::parse(line);
}

inline std::string encode_ndjson_response(
  const Json &response)
{
  return response.dump() + "\n";
}

}  // namespace rby1_app_bridge
