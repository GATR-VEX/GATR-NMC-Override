#include "utilities/bindings.hpp"
#include "pros/rtos.hpp"

#include <cmath>
#include <memory>
#include <utility>
#include <cstdarg>
#include <cstdio>

namespace utilities {
    void EventLoop::bind(std::function<void()> action) {
        actions.push_back(std::move(action));
    }

    void EventLoop::poll() {
        for (const auto &action : actions) {
            action();
        }
    }

    void EventLoop::clear() {
        actions.clear();
    }

    Trigger::Trigger(EventLoop &loop, std::function<bool()> condition): p_event_loop(&loop), condition(std::move(condition)) {}

    void Trigger::bind_edges(std::function<void(bool previous, bool current)> edge_action) {
        p_event_loop->bind([condition = condition, edge_action = std::move(edge_action), initialized = false, previous = false]() mutable {
            const bool current = condition();

            if (!initialized) {
                initialized = true;
                previous = current;
                return;
            }

            edge_action(previous, current);
            previous = current;
        });
    }

    Trigger &Trigger::on_true(std::function<void()> action) {
        bind_edges([action = std::move(action)](bool previous, bool current) {
            if (!previous && current) action();
        });

        return *this;
    }

    Trigger &Trigger::on_false(std::function<void()> action) {
        bind_edges([action = std::move(action)](bool previous, bool current) {
            if (previous && !current) action();
        });

        return *this;
    }

    Trigger &Trigger::while_true(std::function<void()> action, std::function<void()> on_exit) {
        bind_edges([action = std::move(action), on_exit = std::move(on_exit)](bool previous, bool current) {
            if (current) {
                action();
            } else if (previous && on_exit) {
                on_exit();
            }
        });

        return *this;
    }

    Trigger & Trigger::while_false(std::function<void()> action, std::function<void()> on_exit) {
        bind_edges([action = std::move(action), on_exit = std::move(on_exit)](bool previous, bool current) {
            if (!current) {
                action();
            } else if (!previous && on_exit) {
                on_exit();
            }
        });

        return *this;
    }

    Trigger& Trigger::toggle_on_true(std::function<void()> on_enable, std::function<void()> on_disable) {
        bind_edges([on_enable = std::move(on_enable), on_disable = std::move(on_disable), enabled = false](bool previous, bool current) mutable {
            if (previous || !current) return;

            enabled = !enabled;

            if (enabled) {
                if (on_enable) on_enable();
            } else {
                if (on_disable) on_disable();
            }
        });

        return *this;
    }

    bool Trigger::get() const {
        return condition();
    }

    Trigger Trigger::operator&&(const Trigger &other) const {
        return Trigger(*p_event_loop, [a = condition, b = other.condition]() {
            return a() && b();
        });
    }

    Trigger Trigger::operator||(const Trigger &other) const {
        return Trigger(*p_event_loop, [a = condition, b = other.condition]() {
            return a() || b();
        });
    }

    Trigger Trigger::operator!() const {
        return Trigger(*p_event_loop, [a = condition]() {
            return !a();
        });
    }
    
    Controller::Controller(pros::controller_id_e_t id, EventLoop &loop): controller(id), p_event_loop(&loop) {
		for (auto &line : wanted_lines) {
			line.assign(SCREEN_WIDTH, ' ');
		}
    }

    Trigger Controller::button(pros::controller_digital_e_t button) {
        return Trigger(*p_event_loop, [this, button]() {
            return controller.get_digital(button) == 1;
        });
    }

    Trigger Controller::l1() {
        return button(pros::controller_digital_e_t::E_CONTROLLER_DIGITAL_L1);
    }

    Trigger Controller::l2() {
        return button(pros::controller_digital_e_t::E_CONTROLLER_DIGITAL_L2);
    }

    Trigger Controller::r1() {
        return button(pros::controller_digital_e_t::E_CONTROLLER_DIGITAL_R1);
    }

    Trigger Controller::r2() {
        return button(pros::controller_digital_e_t::E_CONTROLLER_DIGITAL_R2);
    }

    Trigger Controller::up() {
        return button(pros::controller_digital_e_t::E_CONTROLLER_DIGITAL_UP);
    }

    Trigger Controller::down() {
        return button(pros::controller_digital_e_t::E_CONTROLLER_DIGITAL_DOWN);
    }

    Trigger Controller::left() {
        return button(pros::controller_digital_e_t::E_CONTROLLER_DIGITAL_LEFT);
    }

    Trigger Controller::right() {
        return button(pros::controller_digital_e_t::E_CONTROLLER_DIGITAL_RIGHT);
    }

    Trigger Controller::x() {
        return button(pros::controller_digital_e_t::E_CONTROLLER_DIGITAL_X);
    }

    Trigger Controller::y() {
        return button(pros::controller_digital_e_t::E_CONTROLLER_DIGITAL_Y);
    }

    Trigger Controller::b() {
        return button(pros::controller_digital_e_t::E_CONTROLLER_DIGITAL_B);
    }

    Trigger Controller::a() {
        return button(pros::controller_digital_e_t::E_CONTROLLER_DIGITAL_A);
    }

    Trigger Controller::magnitude_greater_than(pros::controller_analog_e_t axis, double threshold) {
        return Trigger(*p_event_loop, [this, axis, threshold]() { 
            return std::abs(get_axis(axis)) > threshold; 
        });
    }

    double Controller::get_axis(pros::controller_analog_e_t axis) {
        return controller.get_analog(axis) / 127.0;
    }

    std::int32_t Controller::rumble(const char *p_pattern) {
        return controller.rumble(p_pattern);
    }

	void Controller::set_line(std::size_t line, const char* format, ...) {
		if (line >= SCREEN_LINES || format == nullptr) return;

		char buffer[SCREEN_WIDTH + 1] = {};

		va_list args;
		va_start(args, format);
		std::vsnprintf(buffer, sizeof(buffer), format, args);
		va_end(args);

		std::string text(buffer);
		text.resize(SCREEN_WIDTH, ' ');
		wanted_lines[line] = text;
	}

	void Controller::update_display() {
		const std::uint32_t now = pros::millis();
		if (now - last_display_ms < SCREEN_UPDATE_MS) return;

		if (clear_pending) {
			last_display_ms = now;

			if (controller.clear() == 1) {
				clear_pending = false;
				line_sent.fill(false);
				next_line = 0;
			}

			return;
		}
        
		std::size_t line_to_send = SCREEN_LINES;

		for (std::size_t i = 0; i < SCREEN_LINES; ++i) {
			const std::size_t line = (next_line + i) % SCREEN_LINES;

			if (!line_sent[line] || sent_lines[line] != wanted_lines[line]) {
				line_to_send = line;
				break;
			}
		}

		const bool refreshing = (line_to_send == SCREEN_LINES);

		if (refreshing) {
			if (now - last_refresh_ms < SCREEN_REFRESH_MS) return;
			line_to_send = next_line;
		}

		last_display_ms = now;

		if (controller.set_text(static_cast<std::uint8_t>(line_to_send), 0, wanted_lines[line_to_send]) != 1) {
			return;
		}

		sent_lines[line_to_send] = wanted_lines[line_to_send];
		line_sent[line_to_send] = true;
		next_line = (line_to_send + 1) % SCREEN_LINES;

		if (refreshing) last_refresh_ms = now;
	}

	void Controller::redraw_display() {
		clear_pending = true;
	}

    pros::Controller &Controller::raw() {
        return controller;
    }
}
