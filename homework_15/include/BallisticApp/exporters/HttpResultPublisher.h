#pragma once

#include "BallisticApp/interfaces/IResultPublisher.h"
#include <string>

namespace BallisticApp {

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

class HttpResultPublisher : public IResultPublisher {
public:
  explicit HttpResultPublisher(HttpConfig config);

  std::vector<PublishResult> publish(const std::vector<std::string>& testIds) override;

private:
  PublishResult publishSingleTest(const std::string& testId);
  bool verifyOnServer(const std::string& testId);

  HttpConfig config_;
};

void printSummaryReport(const std::vector<PublishResult>& results);

}  // namespace BallisticApp
