#include "AppController.hpp"

#include <exception>
#include <iostream>
#include <string>

namespace {

void printUsage(const char* program_name) {
    std::cout
        << "Usage:\n"
        << "  " << program_name << "                Run Guardian FSM scenario demo\n"
        << "  " << program_name << " --motion-smoke-test\n"
        << "  " << program_name << " --live-test [options]\n"
        << "\nOptions:\n"
        << "  --motion-smoke-test       Run the motion-only servo smoke test\n"
        << "\nOptions for --live-test:\n"
        << "  --camera-index <n>        Camera index (default: 0)\n"
        << "  --expected-marker-id <n>  Expected ArUco marker ID (default: 23)\n"
        << "  --auto-ack                Auto-send operator acknowledge when frozen\n"
        << "  --help                    Show this message\n";
}

bool parseIntArg(const char* text, int& value) {
    try {
        const std::string raw(text);
        std::size_t parsed_count = 0;
        const int parsed = std::stoi(raw, &parsed_count);
        if (parsed_count != raw.size()) {
            return false;
        }
        value = parsed;
        return true;
    } catch (const std::exception&) {
        return false;
    }
}

}  // namespace

int main(int argc, char* argv[]) {
    bool run_live_test = false;
    bool run_motion_smoke_test = false;
    AppController::LiveTestOptions options;

    for (int i = 1; i < argc; ++i) {
        const std::string arg(argv[i]);

        if (arg == "--help") {
            printUsage(argv[0]);
            return 0;
        }

        if (arg == "--live-test") {
            run_live_test = true;
            continue;
        }

        if (arg == "--motion-smoke-test") {
            run_motion_smoke_test = true;
            continue;
        }

        if (arg == "--auto-ack") {
            options.auto_ack = true;
            continue;
        }

        if ((arg == "--camera-index" || arg == "--expected-marker-id") &&
            i + 1 < argc) {
            int parsed_value = 0;
            if (!parseIntArg(argv[i + 1], parsed_value)) {
                std::cerr << "Invalid numeric value for " << arg << ": "
                          << argv[i + 1] << std::endl;
                return 1;
            }

            if (arg == "--camera-index") {
                options.camera_index = parsed_value;
            } else {
                options.expected_marker_id = parsed_value;
            }

            ++i;
            continue;
        }

        std::cerr << "Unknown or incomplete argument: " << arg << std::endl;
        printUsage(argv[0]);
        return 1;
    }

    if (run_live_test && run_motion_smoke_test) {
        std::cerr << "Choose exactly one mode: --live-test or --motion-smoke-test."
                  << std::endl;
        printUsage(argv[0]);
        return 1;
    }

    AppController app_controller;
    if (run_motion_smoke_test) {
        return app_controller.runMotionSmokeTest();
    }

    if (run_live_test) {
        return app_controller.runLiveMarkerTest(options);
    }

    return app_controller.runGuardianScenarioDemo();
}
