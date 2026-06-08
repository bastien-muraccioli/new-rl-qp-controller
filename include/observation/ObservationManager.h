#pragma once

#include "Observation.h"

#include <Eigen/Core>

#include <mc_rtc/Configuration.h>

#include <deque>
#include <memory>
#include <string>
#include <vector>

namespace rlqp
{

/**
 * @brief Owns the active policy observation list and per-observation history buffers.
 */
class ObservationManager
{
public:
  ObservationManager();

  /**
   * @brief Load observation declarations from observations.yaml.
   *
   * Call configure() after the runtime has a valid ObservationContext.
   */
  void load(const mc_rtc::Configuration & observationsConfig,
            const mc_rtc::Configuration & controllerConfig,
            const ObservationRegistry & registry);

  /** @brief Resolve body/joint indices and compute the final vector size. */
  void configure(const ObservationContext & context);

  /** @brief Fill all history buffers with the current observation values. */
  void updateHistory(const ObservationContext & context);

  /** @brief Compute the full flattened observation vector. */
  Eigen::VectorXd compute(const ObservationContext & context);

  /** @brief Full flattened observation size */
  int size() const;

  /** @brief Active convention name. */
  const std::string & conventionName() const;

private:
  struct Entry
  {
    std::shared_ptr<Observation> observation;
    std::deque<Eigen::VectorXd> historyBuffer;
  };

private:
  ObservationConfig parseObservationConfig(const mc_rtc::Configuration & config,
                                           int defaultHistory) const;

  Eigen::VectorXd flattenHistory(const Entry & entry) const;

private:
  std::vector<Entry> entries_;
  ObservationConvention convention_;
  int size_ = 0;
};

} // namespace rlqp
