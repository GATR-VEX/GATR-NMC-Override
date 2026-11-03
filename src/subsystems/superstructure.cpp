
#include "subsystems/superstructure.hpp"

#include "constants.hpp"

#include <algorithm>

namespace superstructure {
    namespace {
        GoalType selected_goal = GoalType::ALLIANCE;
        int stack_count = 0;

        double get_base_height() {
            switch (selected_goal) {
                case GoalType::ALLIANCE:
                    return constants::superstructure::ALLIANCE_BASE_HEIGHT_IN;
                case GoalType::SHORT:
                    return constants::superstructure::SHORT_BASE_HEIGHT_IN;
                case GoalType::TALL:
                    return constants::superstructure::TALL_BASE_HEIGHT_IN;
            }

            return 0.0;
        }
    }

    void initialize() {
        selected_goal = GoalType::ALLIANCE;
        stack_count = 0;
    }

    void next_goal() {
        switch (selected_goal) {
            case GoalType::ALLIANCE:
                selected_goal = GoalType::SHORT;
                break;
            case GoalType::SHORT:
                selected_goal = GoalType::TALL;
                break;
            case GoalType::TALL:
                selected_goal = GoalType::ALLIANCE;
                break;
        }
    }

    void previous_goal() {
        switch (selected_goal) {
            case GoalType::ALLIANCE:
                selected_goal = GoalType::TALL;
                break;
            case GoalType::SHORT:
                selected_goal = GoalType::ALLIANCE;
                break;
            case GoalType::TALL:
                selected_goal = GoalType::SHORT;
                break;
        }
    }

    void increase_stack() {
        stack_count = std::min(stack_count + 1, constants::superstructure::MAX_STACK_COUNT);
    }

    void decrease_stack() {
        stack_count = std::max(stack_count - 1, 0);
    }

    GoalType get_goal_type() {
        return selected_goal;
    }

    int get_stack_count() {
        return stack_count;
    }

    const char* get_goal_name() {
        switch (selected_goal) {
            case GoalType::ALLIANCE:
                return "ALLIANCE";
            case GoalType::SHORT:
                return "SHORT";
            case GoalType::TALL:
                return "TALL";
        }

        return "UNKNOWN";
    }

    double get_scoring_height() {
        return get_base_height() + stack_count * constants::superstructure::STACK_INCREMENT_IN;
    }
}
