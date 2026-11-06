#pragma once

#include "pros/misc.hpp"
#include <array>
#include <cstdint>
#include <functional>
#include <string>
#include <vector>

namespace utilities {
    class EventLoop {
        public:
            void bind(std::function<void()> action);
            void poll();
            void clear();
        private:
            std::vector<std::function<void()>> actions;
    };

    class Trigger {
        public:
            Trigger(EventLoop &loop, std::function<bool()> condition);

            Trigger &on_true(std::function<void()> action);
            Trigger &on_false(std::function<void()> action);
            Trigger &while_true(std::function<void()> action, std::function<void()> on_exit = {});
            Trigger &while_false(std::function<void()> action, std::function<void()> on_exit = {});
            Trigger &toggle_on_true(std::function<void()> on_enable, std::function<void()> on_disable);

            bool get() const;

            Trigger operator&&(const Trigger &other) const;
            Trigger operator||(const Trigger &other) const;
            Trigger operator!() const;

        private:
            EventLoop *p_event_loop;
            std::function<bool()> condition;

            void bind_edges(std::function<void(bool previous, bool current)> edge_action);
    };

    class Controller {
        public:
			static constexpr std::size_t SCREEN_LINES = 3;
			static constexpr std::size_t SCREEN_WIDTH = 19;
			static constexpr std::uint32_t SCREEN_UPDATE_MS = 50;
            static constexpr std::uint32_t SCREEN_REFRESH_MS = 250;

            Controller(pros::controller_id_e_t id, EventLoop &loop);

            Trigger button(pros::controller_digital_e_t button);

            Trigger l1();
            Trigger l2();
            Trigger r1();
            Trigger r2();
            Trigger up();
            Trigger down();
            Trigger left();
            Trigger right();
            Trigger x();
            Trigger y();
            Trigger b();
            Trigger a();

            Trigger magnitude_greater_than(pros::controller_analog_e_t axis, double threshold);

            double get_axis(pros::controller_analog_e_t axis);

            std::int32_t rumble(const char *p_pattern);

	    	void set_line(std::size_t line, const char* format, ...);
	    	void update_display();
	    	void redraw_display();

            pros::Controller &raw();
        private:
            pros::Controller controller;
            EventLoop *p_event_loop;

			std::array<std::string, SCREEN_LINES> wanted_lines{};
			std::array<std::string, SCREEN_LINES> sent_lines{};
			std::array<bool, SCREEN_LINES> line_sent{};
			std::size_t next_line = 0;
			std::uint32_t last_display_ms = 0;
			std::uint32_t last_refresh_ms = 0;
    };
}
