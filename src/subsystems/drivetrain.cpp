#include "subsystems/drivetrain.hpp"

#include "globals.hpp"

namespace drivetrain {
    namespace {
        Behavior currentBehavior = Behavior::SPLIT_ARCADE;
    }

    void initialize() {
        globals::chassis.opcontrol_curve_buttons_toggle(true);
        globals::chassis.opcontrol_drive_activebrake_set(0.0);
        globals::chassis.opcontrol_curve_default_set(0.0, 0.0);
        globals::chassis.opcontrol_drive_reverse_set(true);
    }

    void setBehavior(Behavior behavior) {
        currentBehavior = behavior;
    }

    Behavior getBehavior() {
        return currentBehavior;
    }

    void teleop() {
        switch (currentBehavior) {
            case Behavior::TANK:
                globals::chassis.opcontrol_tank();
                break;
            case Behavior::SPLIT_ARCADE:
                globals::chassis.opcontrol_arcade_standard(ez::SPLIT);
                break;
            case Behavior::SINGLE_ARCADE:
                globals::chassis.opcontrol_arcade_standard(ez::SINGLE);
                break;
            case Behavior::FLIPPED_SPLIT_ARCADE:
                globals::chassis.opcontrol_arcade_flipped(ez::SPLIT);
                break;
            case Behavior::FLIPPED_SINGLE_ARCADE:
                globals::chassis.opcontrol_arcade_flipped(ez::SINGLE);
                break;
        }
    }
}