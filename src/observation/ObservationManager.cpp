#include "observation/ObservationManager.h"

#include <mc_rtc/logging.h>

namespace rlqp
{

namespace
{

mc_rtc::Configuration emptyConfiguration()
{
  return mc_rtc::Configuration();
}

} // namespace

ObservationManager::ObservationManager()
{
}

void ObservationManager::load(const mc_rtc::Configuration & observationsConfig,
                              const mc_rtc::Configuration & controllerConfig,
                              const ObservationRegistry & registry)
{
  entries_.clear();
  size_ = 0;

  const int defaultHistory = observationsConfig("history", 1);
  const std::string conventionName = observationsConfig("training_convention", std::string("mjlab"));

  convention_ = ObservationConvention::fromConfig(controllerConfig, conventionName);

  if(!observationsConfig.has("observations"))
  {
    mc_rtc::log::error_and_throw("[ObservationManager] observations.yaml has no required 'observations' list");
  }

  mc_rtc::Configuration observations = observationsConfig("observations");

  for(size_t i = 0; i < observations.size(); ++i)
  {
    ObservationConfig config = parseObservationConfig(observations[i], defaultHistory);
    config.type = convention_.resolveType(config.requestedType);

    Entry entry;
    entry.observation = registry.create(config, convention_);
    entries_.push_back(entry);
  }

  if(entries_.empty())
  {
    mc_rtc::log::error_and_throw("[ObservationManager] observations.yaml defines an empty observation list");
  }

  mc_rtc::log::success(
    "[ObservationManager] Loaded {} observations with '{}' convention",
    entries_.size(),
    convention_.name);
}

void ObservationManager::configure(const ObservationContext & context)
{
  size_ = 0;

  for(size_t i = 0; i < entries_.size(); ++i)
  {
    entries_[i].observation->configure(context);
    size_ += entries_[i].observation->history() * entries_[i].observation->size();
  }
}

void ObservationManager::updateHistory(const ObservationContext & context)
{
  for(size_t i = 0; i < entries_.size(); ++i)
  {
    Entry & entry = entries_[i];

    entry.historyBuffer.clear();

    Eigen::VectorXd current = Eigen::VectorXd::Zero(entry.observation->size());
    entry.observation->compute(context, current);

    for(int h = 0; h < entry.observation->history(); ++h)
    {
      entry.historyBuffer.push_back(current);
    }
  }
}

Eigen::VectorXd ObservationManager::compute(const ObservationContext & context)
{
  Eigen::VectorXd out = Eigen::VectorXd::Zero(size_);
  int offset = 0;

  for(size_t i = 0; i < entries_.size(); ++i)
  {
    Entry & entry = entries_[i];

    Eigen::VectorXd current = Eigen::VectorXd::Zero(entry.observation->size());
    entry.observation->compute(context, current);

    if(!entry.historyBuffer.empty() && entry.historyBuffer.front().size() != current.size())
    {
      mc_rtc::log::error_and_throw(
        "[ObservationManager] Observation '{}' changed dimension from {} to {}",
        entry.observation->name(),
        entry.historyBuffer.front().size(),
        current.size());
    }

    entry.historyBuffer.push_front(current);

    while(static_cast<int>(entry.historyBuffer.size()) > entry.observation->history())
    {
      entry.historyBuffer.pop_back();
    }

    Eigen::VectorXd stacked = flattenHistory(entry);

    out.segment(offset, stacked.size()) = stacked;
    offset += static_cast<int>(stacked.size());
  }

  return out;
}

int ObservationManager::size() const { return size_; }

const std::string & ObservationManager::conventionName() const { return convention_.name; }

ObservationConfig ObservationManager::parseObservationConfig(const mc_rtc::Configuration & config,
                                                            int defaultHistory) const
{
  ObservationConfig out;

  out.requestedType = config("type", std::string(""));

  if(out.requestedType.empty())
  {
    mc_rtc::log::error_and_throw("[ObservationManager] Observation entry is missing required field 'type'");
  }

  out.type = out.requestedType;
  out.name = out.requestedType;
  config("name", out.name);

  out.history = defaultHistory;
  config("history", out.history);

  if(out.history <= 0)
  {
    mc_rtc::log::error_and_throw(
      "[ObservationManager] Observation '{}' has invalid history {}",
      out.name,
      out.history);
  }

  if(config.has("parameters"))
  {
    out.parameters = config("parameters");
  }
  else
  {
    out.parameters = emptyConfiguration();
  }

  return out;
}

Eigen::VectorXd ObservationManager::flattenHistory(const Entry & entry) const
{
  int total = 0;

  for(size_t i = 0; i < entry.historyBuffer.size(); ++i)
  {
    total += static_cast<int>(entry.historyBuffer[i].size());
  }

  Eigen::VectorXd out = Eigen::VectorXd::Zero(total);
  int offset = 0;

  for(std::deque<Eigen::VectorXd>::const_reverse_iterator it = entry.historyBuffer.rbegin();
      it != entry.historyBuffer.rend();
      ++it)
  {
    out.segment(offset, it->size()) = *it;
    offset += static_cast<int>(it->size());
  }

  return out;
}

} // namespace rlqp
