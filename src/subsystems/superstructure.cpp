#include "subsystems/superstructure.hpp"
#include "subsystems/lift.hpp"
#include "utilities/logger.hpp"
#include "constants.hpp"

#include <algorithm>

using utilities::Logger;

namespace superstructure {
    namespace {
        namespace k = constants::superstructure;

        GoalType selected_goal = GoalType::ALLIANCE;
        GamePieceType selected_piece = GamePieceType::PIN;
        int stack_count = 0;

        double get_base_height() {
            switch (selected_goal) {
                case GoalType::ALLIANCE: 
                    return k::ALLIANCE_BASE_HEIGHT_IN;
                case GoalType::SHORT:
                    return k::SHORT_BASE_HEIGHT_IN;
                case GoalType::TALL:
                    return k::TALL_BASE_HEIGHT_IN;
            }

            return 0.0;
        }

        double get_piece_offset() {
            switch (selected_piece) {
                case GamePieceType::PIN:
                    return k::PIN_OFFSET_IN;
                case GamePieceType::CUP:
                    return k::CUP_OFFSET_IN;
                case GamePieceType::BOTH:
                    return k::BOTH_OFFSET_IN;
            }

            return 0.0;
        }
    }

    void initialize() {
        selected_goal = GoalType::ALLIANCE;
        selected_piece = GamePieceType::PIN;
        stack_count = 0;

        const char* outputs[] = {
            "Superstructure/GoalType",
            "Superstructure/PieceType",
            "Superstructure/StackCount",
            "Superstructure/ScoringHeightInches",
        };

        for (const char* name : outputs) {
            Logger::registerOutput(name);
        }
    }

    void periodic() {
        Logger::recordOutput("Superstructure/GoalType", static_cast<int>(selected_goal));
        Logger::recordOutput("Superstructure/PieceType", static_cast<int>(selected_piece));
        Logger::recordOutput("Superstructure/StackCount", stack_count);
        Logger::recordOutput("Superstructure/ScoringHeightInches", get_scoring_height());
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

    void next_game_piece() {
        switch (selected_piece) {
            case GamePieceType::PIN:         
                selected_piece = GamePieceType::CUP; 
                break;
            case GamePieceType::CUP:         
                selected_piece = GamePieceType::BOTH; 
                break;
            case GamePieceType::BOTH: 
                selected_piece = GamePieceType::PIN; 
                break;
        }
    }

    void previous_game_piece() {
        switch (selected_piece) {
            case GamePieceType::PIN:         
                selected_piece = GamePieceType::BOTH; 
                break;
            case GamePieceType::CUP:         
                selected_piece = GamePieceType::PIN; 
                break;
            case GamePieceType::BOTH: 
                selected_piece = GamePieceType::CUP; 
                break;
        }
    }

    void increase_stack() {
        stack_count = std::min(stack_count + 1, k::MAX_STACK_COUNT);
    }

    void decrease_stack() {
        stack_count = std::max(stack_count - 1, 0);
    }

    GoalType get_goal_type() {
        return selected_goal;
    }

    GamePieceType get_game_piece_type() {
        return selected_piece;
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

        return "?";
    }

    const char* get_game_piece_name() {
        switch (selected_piece) {
            case GamePieceType::PIN:
                return "PIN";
            case GamePieceType::CUP:
                return "CUP";
            case GamePieceType::BOTH: 
                return "PIN & CUP";
        }

        return "?";
    }

    double get_scoring_height() {
        return get_base_height() + get_piece_offset() + stack_count * k::STACK_INCREMENT_IN;
    }

    void write_display(utilities::Controller& controller) {
        controller.set_line(0, "%-8s %s", get_goal_name(), get_game_piece_name());
        controller.set_line(1, "Stack %d  %5.1f in", stack_count, get_scoring_height());
        controller.set_line(2, "Lift %5.1f in", lift::get_current_height());
    }
}