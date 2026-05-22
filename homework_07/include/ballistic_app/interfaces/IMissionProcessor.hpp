#pragma once
#include "ballistic_app/dto/SimStep.hpp"

namespace BallisticApp {

    class IBallisticSolver;

    class IMissionProcessor {
    public:
        virtual ~IMissionProcessor() = default;

        virtual void init(const char* configSource, const char* ammoSource) = 0;

        virtual bool hasNext() = 0;
        virtual SimStep step() = 0;
        virtual void reset() = 0;
        virtual void changeSolver(IBallisticSolver* s) = 0;

        virtual int getTotalSteps() const = 0;
        virtual const SimStep* getStepsHistory() const = 0;
    };

}  // namespace BallisticApp
