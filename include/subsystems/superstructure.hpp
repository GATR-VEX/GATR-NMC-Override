namespace superstructure {
    enum class GoalType {
        ALLIANCE,
        SHORT,
        TALL
    };

    void initialize();

    void next_goal();
    void previous_goal();

    void increase_stack();
    void decrease_stack();

    GoalType get_goal_type();
    int get_stack_count();

    const char* get_goal_name();
    double get_scoring_height();
}
