#pragma once

#include "BallisticApp/interfaces/IBallisticSolver.h"
#include <vector>
#include <filesystem>

namespace BallisticApp::solvers {

class TableSolver : public IBallisticSolver {
public:
  struct BallisticTable {
    std::vector<float> axisZ0;
    std::vector<float> axisV0;
    std::vector<float> axisM;
    std::vector<float> axisD;
    std::vector<float> axisL;
    std::vector<Result> data;

    size_t index(int iz, int iv, int im, int id, int il) const
    {
      return ((((size_t)iz * axisV0.size() + iv) * axisM.size() + im) * axisD.size() + id) * axisL.size() + il;
    }

    const Result& at(int iz, int iv, int im, int id, int il) const { return data[index(iz, iv, im, id, il)]; }

    bool load(const std::filesystem::path& path);
    Result lookup(float Z0, float V0, float m, float d, float l) const;
  };

  TableSolver() = default;

  bool initialize(const std::filesystem::path& tablePath) override;
  Result calculate(float altitude, float speed, const AmmoParams& ammo) override;

private:
  struct Interp {
    int lo;
    float frac;
  };

  static Interp findInterp(float val, const std::vector<float>& axis);
  static Result lerp(const Result& a, const Result& b, float t);

  BallisticTable m_table;
  bool m_isTableLoaded = false;
};

}  // namespace BallisticApp::solvers
