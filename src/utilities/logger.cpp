
#include "utilities/logger.hpp"
#include "api.h"

#include <array>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <ctime>
#include <cstdint>

namespace utilities {
    namespace {

        constexpr std::size_t NAME_SIZE = 48;
        constexpr std::size_t LABEL_SIZE = 24;
        constexpr std::size_t PATH_SIZE = 128;

        constexpr std::uint32_t SAMPLE_PERIOD_MS = 50;
        constexpr std::uint32_t FLUSH_PERIOD_MS = 500;

        struct Output {
            char name[NAME_SIZE] = {};
            double value = 0.0;
            bool published = false;
        };

        std::array<Output, Logger::MAX_OUTPUTS> outputs{};
        std::size_t output_count = 0;

        // Protects the collection of telemetry values.
        pros::Mutex data_mutex;

        // Protects the open file and all file operations.
        pros::Mutex file_mutex;

        FILE* log_file = nullptr;

        bool initialized = false;

        std::uint32_t recording_start_ms = 0;
        std::uint32_t last_flush_ms = 0;

        int find_output(const char* name) {
            for (std::size_t i = 0; i < output_count; ++i) {
                if (std::strcmp(outputs[i].name, name) == 0) {
                    return static_cast<int>(i);
                }
            }

            return -1;
        }

        // Reject names that would break a CSV header.
        bool valid_name(const char* name) {
            if (!name || !name[0]) {
                return false;
            }

            const std::size_t length = std::strlen(name);

            if (length >= NAME_SIZE) {
                return false;
            }

            for (std::size_t i = 0; i < length; ++i) {
                if (name[i] == ',' || name[i] == '"' || name[i] == '\n' || name[i] == '\r') {
                    return false;
                }
            }

            return true;
        }

        // Convert a session name into a safe filename component.
        void safe_label(const char* source, char (&label)[LABEL_SIZE]) {
            if (!source || !source[0]) {
                source = "practice";
            }

            std::size_t i = 0;

            for (; i < LABEL_SIZE - 1 && source[i]; ++i) {
                const char c = source[i];
                const bool allowed = (c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z') || (c >= '0' && c <= '9') || c == '-' || c == '_';
                label[i] = allowed ? c : '_';
            }

            label[i] = '\0';
        }

        // Generate a unique filename and open the CSV.
        FILE* create_file(const char* label, char (&path)[PATH_SIZE]) {
            char timestamp[32] = "unknown-date";

            const std::time_t now = std::time(nullptr);
            const std::tm* calendar = std::localtime(&now);

            if (calendar && calendar->tm_year + 1900 >= 2020 && calendar->tm_year + 1900 <= 2100) {
                std::strftime(timestamp, sizeof(timestamp), "%Y-%m-%d_%H-%M-%S", calendar);
            }

            for (unsigned n = 1; n <= 9999; ++n) {
                const int count = std::snprintf(path, PATH_SIZE, "/usd/%s_%s_%04u.csv", timestamp, label, n);

                if (count < 0 ||
                    count >= static_cast<int>(PATH_SIZE)) {
                    return nullptr;
                }

                // Do not overwrite an existing recording.
                FILE* existing = std::fopen(path, "r");

                if (existing) {
                    std::fclose(existing);
                    continue;
                }

                return std::fopen(path, "w");
            }

            return nullptr;
        }

        // Executes periodically in a separate PROS task.
        void writer_task(void*) {
            while (true) {
                pros::delay(SAMPLE_PERIOD_MS);

                file_mutex.take();

                // No active recording.
                if (!log_file) {
                    file_mutex.give();
                    continue;
                }

                std::array<double, Logger::MAX_OUTPUTS> values{};
                std::array<bool, Logger::MAX_OUTPUTS> published{};

                std::size_t count = 0;

                // Copy the latest telemetry values.
                data_mutex.take();
                count = output_count;

                for (std::size_t i = 0; i < count; ++i) {
                    values[i] = outputs[i].value;
                    published[i] = outputs[i].published;
                }

                data_mutex.give();

                // Everything below operates on the local snapshot.
                const std::uint32_t now = pros::millis();

                // First CSV column: elapsed time in seconds.
                std::fprintf(log_file, "%.3f", (now - recording_start_ms) / 1000.0);

                // Remaining columns: telemetry measurements.
                for (std::size_t i = 0; i < count; ++i) {
                    if (published[i]) {
                        std::fprintf(log_file, ",%.6f", values[i]);
                    } else {
                        std::fputc(',', log_file); // Leave empty until published.
                    }
                }

                std::fputc('\n', log_file);

                bool failed = std::ferror(log_file) != 0;

                // Periodically flush buffered data to the SD card.
                if (now - last_flush_ms >= FLUSH_PERIOD_MS) {
                    failed = (std::fflush(log_file) != 0) || failed;
                    last_flush_ms = now;
                }

                if (failed) {
                    std::fclose(log_file);
                    log_file = nullptr;
                }

                file_mutex.give();

                if (failed) {
                    std::printf("Logger: SD write failed\n");
                }
            }
        }
    }

    void Logger::initialize() {
        file_mutex.take();

        if (initialized) {
            file_mutex.give();
            return;
        }

        initialized = true;
        file_mutex.give();

        // Function-local static: remains alive after initialize().
        static pros::Task task(writer_task, nullptr, TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "Logger");
    }

    bool Logger::registerOutput(const char* name) {
        if (!valid_name(name)) {
            return false;
        }

        file_mutex.take();

        // CSV columns cannot change during a recording.
        if (!initialized || log_file) {
            file_mutex.give();
            return false;
        }

        data_mutex.take();

        if (find_output(name) >= 0) {
            data_mutex.give();
            file_mutex.give();
            return true;
        }

        if (output_count >= MAX_OUTPUTS) {
            data_mutex.give();
            file_mutex.give();
            return false;
        }

        std::strcpy(outputs[output_count].name, name);
        ++output_count;

        data_mutex.give();
        file_mutex.give();

        return true;
    }

    bool Logger::recordOutput(const char* name, double value) {
        if (!name || !std::isfinite(value)) {
            return false;
        }

        // Do not delay the control loop waiting for telemetry.
        if (!data_mutex.take(0)) {
            return false;
        }

        const int index = find_output(name);

        if (index >= 0) {
            outputs[index].value = value;
            outputs[index].published = true;
        }

        data_mutex.give();

        return index >= 0;
    }

    bool Logger::getOutput(const char* name, double& value) {
        if (!name || !data_mutex.take(0)) {
            return false;
        }

        const int index = find_output(name);

        const bool found = index >= 0 && outputs[index].published;

        if (found) {
            value = outputs[index].value;
        }

        data_mutex.give();

        return found;
    }

    bool Logger::start(const char* sessionName) {
        file_mutex.take();

        if (!initialized || log_file) {
            file_mutex.give();
            return false;
        }

        // Copy registered names for the CSV header.
        std::array< std::array<char, NAME_SIZE>, MAX_OUTPUTS> names{};
        std::size_t count = 0;

        data_mutex.take();
        count = output_count;

        for (std::size_t i = 0; i < count; ++i) {
            std::strcpy(names[i].data(), outputs[i].name);
        }

        data_mutex.give();

        char label[LABEL_SIZE] = {};
        safe_label(sessionName, label);
        char path[PATH_SIZE] = {};

        FILE* opened = create_file(label, path);

        if (!opened) {
            file_mutex.give();
            std::printf("Logger: Could not create file on SD card\n");

            return false;
        }

        std::fprintf(opened, "time_seconds");

        for (std::size_t i = 0; i < count; ++i) {
            std::fprintf(opened, ",%s", names[i].data());
        }

        std::fputc('\n', opened);

        if (std::fflush(opened) != 0 ||
            std::ferror(opened)) {

            std::fclose(opened);
            file_mutex.give();

            std::printf("Logger: Could not write CSV header\n");

            return false;
        }

        recording_start_ms = pros::millis();
        last_flush_ms = recording_start_ms;

        // Makes the new file visible to writer_task().
        log_file = opened;
        file_mutex.give();

        std::printf("Logger: Recording %s\n", path);

        return true;
    }

    bool Logger::stop() {
        file_mutex.take();

        if (!log_file) {
            file_mutex.give();
            return true;
        }

        // fclose() flushes and closes the recording.
        const int result = std::fclose(log_file);

        log_file = nullptr;
        file_mutex.give();

        if (result != 0) {
            std::printf("Logger: Could not close CSV, ended with errors\n");
        }

        return result == 0;
    }
}
