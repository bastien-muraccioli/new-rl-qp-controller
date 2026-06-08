#include "ConfigurationHelpers.h"

#include <algorithm>
#include <dirent.h>
#include <fstream>
#include <sys/stat.h>

namespace rlqp
{

namespace config
{

std::string joinPath(const std::string & lhs, const std::string & rhs)
{
  if(lhs.empty())
  {
    return rhs;
  }

  if(lhs[lhs.size() - 1] == '/')
  {
    return lhs + rhs;
  }

  return lhs + "/" + rhs;
}

std::string basenameWithoutExtension(const std::string & path)
{
  std::string base = path;

  const size_t slash = base.find_last_of('/');
  if(slash != std::string::npos)
  {
    base = base.substr(slash + 1);
  }

  const size_t dot = base.find_last_of('.');
  if(dot != std::string::npos)
  {
    base = base.substr(0, dot);
  }

  return base;
}

bool fileExists(const std::string & path)
{
  std::ifstream file(path.c_str());
  return file.good();
}

bool isDirectory(const std::string & path)
{
  struct stat status;
  if(stat(path.c_str(), &status) != 0)
  {
    return false;
  }

  return S_ISDIR(status.st_mode);
}

std::vector<std::string> listPolicies(const std::string & root,
                                                   const std::vector<std::string> & requiredFiles)
{
  std::vector<std::string> folders;

  DIR * dir = opendir(root.c_str());
  if(!dir)
  {
    mc_rtc::log::error_and_throw("[ConfigurationHelpers] Could not open directory '{}'", root);
  }

  struct dirent * entry = nullptr;
  while((entry = readdir(dir)) != nullptr)
  {
    const std::string name = entry->d_name;
    if(name == "." || name == "..")
    {
      continue;
    }

    const std::string folder = joinPath(root, name);
    if(!isDirectory(folder))
    {
      continue;
    }

    bool complete = true;
    for(size_t i = 0; i < requiredFiles.size(); ++i)
    {
      if(!fileExists(joinPath(folder, requiredFiles[i])))
      {
        complete = false;
        break;
      }
    }

    if(complete)
    {
      folders.push_back(folder);
    }
  }

  closedir(dir);
  std::sort(folders.begin(), folders.end());
  return folders;
}

mc_rtc::Configuration loadConventionRoot(const mc_rtc::Configuration & controllerConfig)
{
  if(controllerConfig.has("conventions"))
  {
    return controllerConfig;
  }

  const std::string policiesRoot = config::readOr<std::string>(controllerConfig, "policies_root", "policies");
  const std::string conventionsPath = joinPath(policiesRoot, "conventions.yaml");

  mc_rtc::Configuration conventionsConfig;
  conventionsConfig.load(conventionsPath);
  return conventionsConfig;
}

} // namespace config

} // namespace rlqp
