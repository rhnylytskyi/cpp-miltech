#include "BallisticApp/solvers/TableSolver.h"
#include "BallisticApp/utils/Logger.h"
#include "BallisticApp/config/AmmoParams.h"
#include <fstream>
#include <algorithm>
#include <array>
#include <cmath>

namespace BallisticApp {

bool TableSolver::BallisticTable::load(const std::filesystem::path& path)
{
  std::ifstream f(path);
  if (!f.is_open()) {
    APP_LOG_MOD("Ballistics", "{:.<15} {}", "FILE_ERROR", "Failed to open ballistic table file!");
    return false;
  }

  int nZ = 0, nV = 0, nM = 0, nD = 0, nL = 0;
  if (!(f >> nZ >> nV >> nM >> nD >> nL)) {
    APP_LOG_MOD("Ballistics", "{:.<15} {}", "HEADER_ERROR", "Invalid ballistic table file header format.");
    return false;
  }

  if (nZ <= 0 || nV <= 0 || nM <= 0 || nD <= 0 || nL <= 0) {
    APP_LOG_MOD("Ballistics", "{:.<15} Got non-positive axis: Z={}, V={}, m={}, d={}, l={}", "AXIS_ERROR", nZ, nV, nM, nD, nL);
    return false;
  }

  size_t total = (size_t)nZ * nV * nM * nD * nL;
  constexpr size_t MAX_SAFE_ELEMENTS = 50'000'000;
  if (total > MAX_SAFE_ELEMENTS) {
    APP_LOG_MOD("Ballistics", "{:.<15} Grid size {} exceeds maximum safe limit of {} pts", "SIZE_ERROR", total, MAX_SAFE_ELEMENTS);
    return false;
  }

  axisZ0.resize(nZ);
  for (auto& v : axisZ0)
    if (!(f >> v))
      return false;
  axisV0.resize(nV);
  for (auto& v : axisV0)
    if (!(f >> v))
      return false;
  axisM.resize(nM);
  for (auto& v : axisM)
    if (!(f >> v))
      return false;
  axisD.resize(nD);
  for (auto& v : axisD)
    if (!(f >> v))
      return false;
  axisL.resize(nL);
  for (auto& v : axisL)
    if (!(f >> v))
      return false;

  data.resize(total);

  for (size_t i = 0; i < total; i++) {
    if (!(f >> data[i].flightTime >> data[i].hDistance)) {
      APP_LOG_MOD("Ballistics", "{:.<15} {}/{}", "GRID_ERROR", "Corrupted element at index", i);
      data.clear();
      return false;
    }
  }

  // APP_LOG_MOD("Ballistics", "{:.<15} [{}x{}x{}x{}x{}] ({} pts)", "GRID_SIZE", nZ, nV, nM, nD, nL, total);
  return f.good() || f.eof();
}

TableSolver::Interp TableSolver::findInterp(float val, const std::vector<float>& axis)
{
  if (axis.empty())
    return {0, 0.0f};
  if (val <= axis.front())
    return {0, 0.0f};
  if (val >= axis.back())
    return {static_cast<int>(axis.size()) - 2, 1.0f};

  auto it = std::lower_bound(axis.begin(), axis.end(), val);
  int i = static_cast<int>(it - axis.begin()) - 1;
  if (i < 0)
    i = 0;

  float frac = (val - axis[i]) / (axis[i + 1] - axis[i]);
  return {i, frac};
}

TableSolver::Result TableSolver::lerp(const Result& a, const Result& b, float t)
{
  return {a.flightTime + (b.flightTime - a.flightTime) * t, a.hDistance + (b.hDistance - a.hDistance) * t};
}

TableSolver::Result TableSolver::BallisticTable::lookup(float Z0, float V0, float m, float d, float l) const
{
  Interp iz = findInterp(Z0, axisZ0);
  Interp iv = findInterp(V0, axisV0);
  Interp im = findInterp(m, axisM);
  Interp id = findInterp(d, axisD);
  Interp il = findInterp(l, axisL);

  std::array<Result, 16> v;
  for (int a = 0; a < 2; a++) {
    for (int b = 0; b < 2; b++) {
      for (int c = 0; c < 2; c++) {
        for (int e = 0; e < 2; e++) {
          const auto& lo = at(iz.lo + a, iv.lo + b, im.lo + c, id.lo + e, il.lo);
          const auto& hi = at(iz.lo + a, iv.lo + b, im.lo + c, id.lo + e, il.lo + 1);
          v[a * 8 + b * 4 + c * 2 + e] = lerp(lo, hi, il.frac);
        }
      }
    }
  }

  std::array<Result, 8> w;
  for (int a = 0; a < 2; a++) {
    for (int b = 0; b < 2; b++) {
      for (int c = 0; c < 2; c++) {
        w[a * 4 + b * 2 + c] = lerp(v[a * 8 + b * 4 + c * 2], v[a * 8 + b * 4 + c * 2 + 1], id.frac);
      }
    }
  }

  std::array<Result, 4> u;
  for (int a = 0; a < 2; a++) {
    for (int b = 0; b < 2; b++) {
      u[a * 2 + b] = lerp(w[a * 4 + b * 2], w[a * 4 + b * 2 + 1], im.frac);
    }
  }

  std::array<Result, 2> s;
  for (int a = 0; a < 2; a++) {
    s[a] = lerp(u[a * 2], u[a * 2 + 1], iv.frac);
  }

  return lerp(s[0], s[1], iz.frac);
}

bool TableSolver::initialize(const std::filesystem::path& tablePath)
{
  m_isTableLoaded = m_table.load(tablePath);
  return m_isTableLoaded;
}

IBallisticSolver::Result TableSolver::calculate(float altitude, float speed, const AmmoParams& ammo)
{
  if (!m_isTableLoaded)
    return {0.0f, 0.0f};
  return m_table.lookup(altitude, speed, ammo.mass, ammo.drag, ammo.lift);
}

}  // namespace BallisticApp
