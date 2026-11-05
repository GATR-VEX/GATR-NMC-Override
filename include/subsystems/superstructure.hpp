#pragma once

#include "utilities/bindings.hpp"

namespace superstructure {
    enum class GoalType {
        ALLIANCE,
        SHORT,
        TALL
    };

    enum class GamePieceType {
		PIN,
		CUP,
		BOTH
    };

    void initialize();
	void periodic();

    void next_goal();
    void previous_goal();

	void next_game_piece();
	void previous_game_piece();

    void increase_stack();
    void decrease_stack();

    GoalType get_goal_type();
	GamePieceType get_game_piece_type();
    int get_stack_count();

    const char* get_goal_name();
	const char* get_game_piece_name();

    double get_scoring_height();
	void write_display(utilities::Controller &controller);
}
