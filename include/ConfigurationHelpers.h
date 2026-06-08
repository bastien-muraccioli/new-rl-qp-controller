#pragma once

#include <mc_rtc/Configuration.h>
#include <mc_rtc/logging.h>

#include <map>
#include <string>
#include <vector>

namespace rlqp
{

namespace config
{

/**
 * @brief Join two path fragments with one '/'.
 */
std::string joinPath(const std::string & lhs, const std::string & rhs);

/**
 * @brief Return the final path component without its extension.
 */
std::string basenameWithoutExtension(const std::string & path);

/**
 * @brief True if the path exists and is a regular readable file.
 */
bool fileExists(const std::string & path);

/**
 * @brief True if the path exists and is a directory.
 */
bool isDirectory(const std::string & path);

/**
 * @brief Discover immediate child folders containing all required files.
 *
 * Used by PolicyManager to find policies/<policy>/ folders without requiring
 * a manually duplicated policy_names list.
 */
std::vector<std::string> listPolicies(const std::string & root,
                                                   const std::vector<std::string> & requiredFiles);

/**
 * @brief Load the conventions root configuration.
 *
 * Preferred layout:
 *   policies_root/conventions.yaml
 *
 * Compatibility layout:
 *   controllerConfig directly contains a "conventions" block.
 */
mc_rtc::Configuration loadConventionRoot(const mc_rtc::Configuration & controllerConfig);

/**
 * @brief Read a required configuration value with a domain-specific error.
 */
template<typename T>
T readRequired(const mc_rtc::Configuration & cfg,
               const std::string & key,
               const std::string & owner)
{
  if(!cfg.has(key))
  {
    mc_rtc::log::error_and_throw("[{}] Missing required field '{}'", owner, key);
  }
  return cfg(key, T());
}

/**
 * @brief Read an optional configuration value, returning fallback if absent.
 */
template<typename T>
T readOr(const mc_rtc::Configuration & cfg,
         const std::string & key,
         const T & fallback)
{
  if(!cfg.has(key))
  {
    return fallback;
  }
  return cfg(key, fallback);
}

} // namespace config

} // namespace rlqp
