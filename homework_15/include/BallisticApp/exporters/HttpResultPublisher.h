#pragma once

#include "BallisticApp/interfaces/IResultPublisher.h"
#include <string>

namespace BallisticApp {

/**
 * @brief Solid configuration model aggregating network parameters according to technical specifications.
 */
struct HttpConfig {
  std::string baseUrl = "http://cppmiltech.com.ua";
  std::string apiPath = "/api/dz12/results";
  std::string apiKey = "dz12-vX7mK4qT9r2w";
  std::string studentId = "1088";
  std::string baseDataDir = "data";

  int maxAttempts = 5;
  int retryDelaySec = 1;
  int timeoutSec = 2;
};

/**
 * @brief Professional HTTP network driver engine responsible for publishing local simulations.
 */
class HttpResultPublisher : public IResultPublisher {
public:
  /**
   * @brief Instantiates the network component with specified core configuration.
   * @param config Operational parameters context block.
   */
  explicit HttpResultPublisher(HttpConfig config);

  /**
   * @brief Transmits a collection of local telemetry logs sequentially to the endpoint database.
   * @param testIds Alphanumeric identifiers matching local scenario directories.
   */
  std::vector<PublishResult> publish(const std::vector<std::string>& testIds) override;

private:
  PublishResult publishSingleTest(const std::string& testId);
  bool verifyOnServer(const std::string& testId);

  HttpConfig config_;
};

/**
 * @brief Generates and prints a clean analytical execution dashboard overview to stdout.
 * @param results Collection containing finalized network data status rows.
 */
void printSummaryReport(const std::vector<PublishResult>& results);

}  // namespace BallisticApp
