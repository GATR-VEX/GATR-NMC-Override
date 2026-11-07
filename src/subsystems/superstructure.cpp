#include "subsystems/superstructure.hpp"
#include "subsystems/lift.hpp"
#include "utilities/logger.hpp"
#include "constants.hpp"

#include <array>
#include <cstddef> 
#include <cstdio>

using utilities::Logger;

namespace superstructure {
	namespace {
		namespace k = constants::superstructure;

		Node selected_node = Node::ALLIANCE;

		std::size_t pieces_on_node = 0;

		const std::array<double, k::STACK_POSITIONS>& heights_for(Node node) {
			switch (node) {
				case Node::ALLIANCE:
					return k::ALLIANCE_NODE_HEIGHTS;
				case Node::NEUTRAL:
					return k::NEUTRAL_NODE_HEIGHTS;
				case Node::CENTER:
					return k::CENTER_NODE_HEIGHTS;
			}

			return k::ALLIANCE_NODE_HEIGHTS;
		}
	}

	void initialize() {
		selected_node = Node::ALLIANCE;
		pieces_on_node = 0;

		const char* outputs[] = {
			"Superstructure/Node",
			"Superstructure/StackHeight",
			"Superstructure/ScoringHeightInches"
		};

		for (const char* output : outputs) {
			Logger::registerOutput(output);
		}
	}

	void periodic() {
		Logger::recordOutput("Superstructure/Node", static_cast<int>(selected_node));
		Logger::recordOutput("Superstructure/StackHeight", get_stack_height());
		Logger::recordOutput("Superstructure/ScoringHeightInches", get_scoring_height());
	}

	void next_node() {
		switch (selected_node) {
			case Node::ALLIANCE:
				selected_node = Node::NEUTRAL;
			case Node::NEUTRAL:
				selected_node = Node::CENTER;
			case Node::CENTER:
				selected_node = Node::ALLIANCE;
		}
	}

	void previous_node() {
		switch (selected_node) {
			case Node::ALLIANCE:
				selected_node = Node::CENTER;
			case Node::NEUTRAL:
				selected_node = Node::ALLIANCE;
			case Node::CENTER:
				selected_node = Node::NEUTRAL;
		}
	}

	void increase_stack() {
		if (pieces_on_node + 1 < k::STACK_POSITIONS) {
			++pieces_on_node;
		}
	}

	void decrease_stack() {
		if (pieces_on_node > 0) {
			--pieces_on_node;
		}
	}

	Node get_node() {
		return selected_node;
	}

	const char* get_node_name() {
		switch (selected_node) {
			case Node::ALLIANCE:
				return "Alliance";
			case Node::NEUTRAL:
				return "Neutral";
			case Node::CENTER:
				return "Center";
		}

		return "N/A";
	}

	double get_stack_height() {
		return pieces_on_node * 0.5;
	}

	double get_scoring_height() {
		return heights_for(selected_node)[pieces_on_node];
	}

	bool has_scoring_height() {
		return get_scoring_height() > 0;
	}

	void write_display(utilities::Controller& controller, int seconds_left) {
        char node[16];
        std::snprintf(node, sizeof(node), "Node: %s", get_node_name());

        controller.set_line(0, "%-15s %3d", node, seconds_left);
        controller.set_line(1, "Stack height: %.1f", get_stack_height());
        controller.set_line(2, "Lift height: %5.1f", lift::get_current_height());
    }
}
