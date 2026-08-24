#pragma once

#include <string>
#include <vector>

namespace BallisticApp {

/**
 * @brief Status enumeration representing network transaction execution states.
 */
enum class PublishStatus {
  Success,      // POST request complete and GET verification confirmed payload on the server
  Unverified,   // POST request complete, but GET verification failed to locate payload
  Rejected,     // Server returned 400 or 401 due to configuration issues or invalid credentials
  Failed,       // Exceeded the maximum allowance of processing attempts due to timeouts/503 errors
  FileNotFound  // The designated local mission execution summary JSON package is missing
};

/**
 * @brief Combined analytical model mapping a specific execution case id to its transmission context.
 */
struct PublishResult {
  std::string testId;
  PublishStatus status = PublishStatus::FileNotFound;
  int attempts = 0;
  std::string message;
};

/**
 * @brief Clean abstract architectural boundary interface defining remote mission report exporters.
 */
class IResultPublisher {
public:
  virtual ~IResultPublisher() = default;

  /**
   * @brief Formulates and transmits local tracking simulations to the target evaluation system.
   * @param testIds Collection containing the alphanumeric identifiers of the scenario targets.
   */
  virtual std::vector<PublishResult> publish(const std::vector<std::string>& testIds) = 0;
};

}  // namespace BallisticApp
