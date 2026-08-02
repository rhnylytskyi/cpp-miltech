#include "BallisticApp/exporters/HttpResultPublisher.h"
#include "BallisticApp/utils/Logger.h"
#include <httplib.h>
#include <fstream>
#include <sstream>
#include <iostream>
#include <thread>
#include <chrono>
#include <iomanip>
#include <nlohmann/json.hpp>

namespace BallisticApp {

using json = nlohmann::json;

HttpResultPublisher::HttpResultPublisher(HttpConfig config)
  : config_(std::move(config))
{
}

bool HttpResultPublisher::verifyOnServer(const std::string& testId)
{
  httplib::Client client(config_.baseUrl);
  client.set_connection_timeout(config_.timeoutSec, 0);
  client.set_read_timeout(config_.timeoutSec, 0);

  httplib::Headers headers = {{"x-api-key", config_.apiKey}};
  std::string url = config_.apiPath + "/" + testId + "/" + config_.studentId;

  auto res = client.Get(url.c_str(), headers);
  if (!res || res->status != 200) {
    return false;
  }

  try {
    auto body = json::parse(res->body);
    return body.value("found", false);
  }
  catch (...) {
    return false;
  }
}

PublishResult HttpResultPublisher::publishSingleTest(const std::string& testId)
{
  PublishResult result;
  result.testId = testId;

  std::string fullPath = config_.baseDataDir + "/" + testId + "/simulation.json";
  std::ifstream file(fullPath);
  if (!file.is_open()) {
    result.status = PublishStatus::FileNotFound;
    result.message = "File is missing at path: " + fullPath;
    return result;
  }

  std::stringstream buffer;
  buffer << file.rdbuf();
  file.close();

  json simulationContent;
  try {
    simulationContent = json::parse(buffer.str());
  }
  catch (const std::exception& e) {
    result.status = PublishStatus::Rejected;
    result.message = std::string("Malformed local JSON format: ") + e.what();
    return result;
  }

  json rootPayload;
  rootPayload["studentId"] = config_.studentId;
  rootPayload["testId"] = testId;
  rootPayload["simulation"] = simulationContent;
  std::string jsonStr = rootPayload.dump();

  httplib::Headers headers = {{"x-api-key", config_.apiKey}};

  for (int attempt = 1; attempt <= config_.maxAttempts; ++attempt) {
    result.attempts = attempt;

    httplib::Client client(config_.baseUrl);
    client.set_connection_timeout(config_.timeoutSec, 0);
    client.set_read_timeout(config_.timeoutSec, 0);
    client.set_write_timeout(config_.timeoutSec, 0);

    auto res = client.Post(config_.apiPath, headers, jsonStr, "application/json");

    if (!res) {
      result.message = "Network timeout or connection error on attempt " + std::to_string(attempt);
      if (attempt < config_.maxAttempts) {
        std::this_thread::sleep_for(std::chrono::seconds(config_.retryDelaySec));
        continue;
      }
      result.status = PublishStatus::Failed;
      return result;
    }

    if (res->status == 200 || res->status == 201) {
      bool isVerified = verifyOnServer(testId);
      result.status = isVerified ? PublishStatus::Success : PublishStatus::Unverified;
      result.message = "HTTP " + std::to_string(res->status);
      return result;
    }

    if (res->status == 400 || res->status == 401) {
      result.status = PublishStatus::Rejected;
      result.message = "HTTP " + std::to_string(res->status) + ": " + res->body;
      return result;
    }

    result.message = "HTTP " + std::to_string(res->status) + " (Server busy/unavailable)";
    if (attempt < config_.maxAttempts) {
      std::this_thread::sleep_for(std::chrono::seconds(config_.retryDelaySec));
      continue;
    }
    result.status = PublishStatus::Failed;
  }

  return result;
}

std::vector<PublishResult> HttpResultPublisher::publish(const std::vector<std::string>& testIds)
{
  std::vector<PublishResult> reports;
  reports.reserve(testIds.size());

  for (const auto& testId : testIds) {
    APP_LOG_MOD("Network", "Submitting telemetry package for target: {}", testId);
    auto res = publishSingleTest(testId);

    std::string statusLabel = "UNKNOWN";
    if (res.status == PublishStatus::Success)
      statusLabel = "SUCCESS (OK)";
    if (res.status == PublishStatus::Unverified)
      statusLabel = "OK (GET unverified)";
    if (res.status == PublishStatus::Rejected)
      statusLabel = "REJECTED (4xx)";
    if (res.status == PublishStatus::Failed)
      statusLabel = "FAILED ATTEMPTS (5xx/Timeout)";
    if (res.status == PublishStatus::FileNotFound)
      statusLabel = "FILE NOT FOUND";

    APP_LOG_MOD("Network", "Transaction status for {}: {} (Attempts: {})", testId, statusLabel, res.attempts);
    reports.push_back(res);
  }

  return reports;
}

void printSummaryReport(const std::vector<PublishResult>& results)
{
  std::cout << "\n========================================\n";
  std::cout << "          GLOBAL HTTP METRICS REPORT     \n";
  std::cout << "========================================\n";
  std::cout << std::left << std::setw(8) << "Test" << std::setw(28) << "Status" << "Processing Attempts\n";
  std::cout << "----------------------------------------\n";

  for (const auto& r : results) {
    std::string statusStr;
    switch (r.status) {
      case PublishStatus::Success:
        statusStr = "SUCCESS";
        break;
      case PublishStatus::Unverified:
        statusStr = "SUCCESS (NO GET CONFIRMATION)";
        break;
      case PublishStatus::Rejected:
        statusStr = "REJECTED BY REMOTE ENDPOINT";
        break;
      case PublishStatus::Failed:
        statusStr = "FAILED (TIMEOUT/503 LIMIT)";
        break;
      case PublishStatus::FileNotFound:
        statusStr = "SKIPPED (LOCAL FILE MISSING)";
        break;
    }

    std::cout << std::left << std::setw(8) << r.testId << std::setw(28) << statusStr << r.attempts << "\n";
  }
  std::cout << "========================================\n";
}

}  // namespace BallisticApp
