#pragma once

#include "ballistic_app/interfaces/IConfigLoader.hpp"
#include "ballistic_app/interfaces/ITargetProvider.hpp"
#include "ballistic_app/interfaces/IBallisticSolver.hpp"
#include "ballistic_app/dto/SimStep.hpp"

namespace BallisticApp {

enum class DroneState { STOPPED = 0, ACCELERATING = 1, DECELERATING = 2, TURNING = 3, MOVING = 4 };
const int MAX_STEPS = 10000;

class MissionProcessor {
public:
  MissionProcessor(IConfigLoader* l, ITargetProvider* p, IBallisticSolver* s);
  ~MissionProcessor();

  void init(const char* configSource, const char* ammoSource);
  bool hasNext();
  SimStep step();
  void reset();
  void changeSolver(IBallisticSolver* s);
  int getTotalSteps() const;
  const SimStep* getStepsHistory() const;

private:
  Coord interpolateTarget(int targetIdx, float t);
  Coord extrapolateTarget(int targetIdx, float time, float dt);

private:
  IConfigLoader* loader;
  ITargetProvider* provider;
  IBallisticSolver* solver;

  DroneConfig config;
  AmmoParams ammo;

  Coord dronePos;
  float direction;
  float speed;
  DroneState state;
  float currentTime;

  SimStep* steps;
  int totalSteps;
  bool isMissionFinished;
};

}  // namespace BallisticApp
