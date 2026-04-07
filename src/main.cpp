#include "AppController.hpp"

#include <exception>
#include <iostream>
#include <string>

namespace {

void printUsage(const char* program_name) {
    std::cout
        << "Usage:\n"
        << "  " << program_name << "                Run Guardian FSM scenario demo\n"
        << "  " << program_name << " --button-test\n"
        << "  " << program_name << " --full-demo [options]\n"
        << "  " << program_name << " --motion-smoke-test [options]\n"
        << "  " << program_name << " --live-test [options]\n"
        << "\nOptions:\n"
        << "  --button-test             Run a GPIO button diagnostic loop\n"
        << "  --full-demo               Run the full pipeline demo (camera + vision + guardian + interlock + motion)\n"
        << "  --motion-smoke-test       Run the motion-only servo smoke test (all joints by default)\n"
        << "  --base                    Smoke-test only the base servo\n"
        << "  --lower                   Smoke-test only the lower servo\n"
        << "  --upper                   Smoke-test only the upper servo\n"
        << "  --grip                    Smoke-test only the grip servo\n"
        << "\nOptions for --full-demo and --live-test:\n"
        << "  --camera-index <n>        Camera index (default: 0)\n"
        << "  --expected-marker-id <n>  Expected ArUco marker ID (default: 23)\n"
        << "\nOptions for --live-test:\n"
        << "  --auto-ack                Auto-send operator acknowledge when frozen\n"
        << "  --help                    Show this message\n";
}

bool parseSmokeJointFlag(const std::string& arg, AppController::SmokeJoint& joint) {
    if (arg == "--base") {
        joint = AppController::SmokeJoint::Base;
        return true;
    }
    if (arg == "--lower") {
        joint = AppController::SmokeJoint::Lower;
        return true;
    }
    if (arg == "--upper") {
        joint = AppController::SmokeJoint::Upper;
        return true;
    }
    if (arg == "--grip") {
        joint = AppController::SmokeJoint::Grip;
        return true;
    }
    return false;
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
    bool run_button_test = false;
    bool run_live_test = false;
    bool run_full_demo = false;
    bool run_motion_smoke_test = false;
    AppController::LiveTestOptions options;
    AppController::MotionSmokeTestOptions smoke_options;
    bool smoke_joint_selected = false;

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

        if (arg == "--button-test") {
            run_button_test = true;
            continue;
        }

        if (arg == "--full-demo") {
            run_full_demo = true;
            continue;
        }

        if (arg == "--motion-smoke-test") {
            run_motion_smoke_test = true;
            continue;
        }

        if (parseSmokeJointFlag(arg, smoke_options.joint)) {
            if (smoke_joint_selected) {
                std::cerr << "Choose one smoke-test joint flag at a time." << std::endl;
                printUsage(argv[0]);
                return 1;
            }
            run_motion_smoke_test = true;
            smoke_joint_selected = true;
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

    if ((run_button_test && run_live_test) ||
        (run_button_test && run_full_demo) ||
        (run_button_test && run_motion_smoke_test) ||
        (run_live_test && run_full_demo) ||
        (run_live_test && run_motion_smoke_test) ||
        (run_full_demo && run_motion_smoke_test)) {
        std::cerr << "Choose exactly one mode: --button-test, --full-demo, --live-test, or --motion-smoke-test."
                  << std::endl;
        printUsage(argv[0]);
        return 1;
    }

    if (run_full_demo && options.auto_ack) {
        std::cerr << "--auto-ack is not supported in --full-demo mode."
                  << std::endl;
        printUsage(argv[0]);
        return 1;
    }

    AppController app_controller;
    if (run_button_test) {
        return app_controller.runButtonTest();
    }

    if (run_motion_smoke_test) {
        return app_controller.runMotionSmokeTest(smoke_options);
    }

    if (run_full_demo) {
        return app_controller.runFullPipelineDemo(options);
    }

    if (run_live_test) {
        return app_controller.runLiveMarkerTest(options);
    }

    return app_controller.runGuardianScenarioDemo();
}
