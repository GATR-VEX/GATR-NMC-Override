#pragma once

#include "utilities/bindings.hpp"

namespace superstructure {
    enum class Node {
        ALLIANCE,
		NEUTRAL,
		CENTER
    };

    void initialize();
	void periodic();

    void next_node();
    void previous_node();

    void increase_stack();
    void decrease_stack();

	Node get_node();
    const char* get_node_name();

	double get_stack_height();
	double get_scoring_height();

	bool has_scoring_height();

	void write_display(utilities::Controller &controller, int seconds_left);
}
