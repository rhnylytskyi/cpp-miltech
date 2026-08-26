#pragma once

#include "BallisticApp/exporters/SimStep.h"
#include <string>
#include <vector>

namespace BallisticApp {

class JsonExporter {
public:
  JsonExporter() = default;
  ~JsonExporter() noexcept = default;

  JsonExporter(const JsonExporter&) = delete;
  JsonExporter& operator=(const JsonExporter&) = delete;

  void record(const SimStep& step);
  bool save(const std::string& path) const;

private:
  std::vector<SimStep> m_steps;
};

}  // namespace BallisticApp
