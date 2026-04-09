/**
 * @file MetricsLogger.cpp
 * @brief Implementation of MetricsLogger class
 */

#include "MetricsLogger.hpp"
#include <iostream>

MetricsLogger::MetricsLogger() : running(true) {
    // Open log file
    logFile.open("guardian_log.txt", std::ios::out | std::ios::app);
    
    if (!logFile.is_open()) {
        std::cerr << "Warning: Could not open log file, logging to console only\n";
    }

    // Start background logging thread
    logThread = std::thread(&MetricsLogger::processLogs, this);
}

MetricsLogger::~MetricsLogger() {
    // Signal thread to stop
    running = false;
    queueCV.notify_one();

    // Wait for thread to finish
    if (logThread.joinable()) {
        logThread.join();
    }

    // Close log file
    if (logFile.is_open()) {
        logFile.close();
    }
}

MetricsLogger& MetricsLogger::getInstance() {
    static MetricsLogger instance;
    return instance;
}

std::string MetricsLogger::getCurrentTimestamp() {
    auto now = std::chrono::system_clock::now();
    auto time_t_now = std::chrono::system_clock::to_time_t(now);
    auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(
        now.time_since_epoch()) % 1000;

    std::stringstream ss;
    ss << std::put_time(std::localtime(&time_t_now), "%Y-%m-%d %H:%M:%S");
    ss << '.' << std::setfill('0') << std::setw(3) << ms.count();
    return ss.str();
}

void MetricsLogger::logEvent(const std::string& category, const std::string& message) {
    LogEntry entry;
    entry.timestamp = getCurrentTimestamp();
    entry.category = category;
    entry.message = message;

    // Lock only for queue insertion (minimal critical section)
    {
        std::lock_guard<std::mutex> lock(queueMutex);
        logQueue.push(entry);
    }

    // Notify background thread
    queueCV.notify_one();
}

void MetricsLogger::processLogs() {
    while (running) {
        std::unique_lock<std::mutex> lock(queueMutex);

        // Wait for logs or shutdown signal
        queueCV.wait(lock, [this] { return !logQueue.empty() || !running; });

        // Process all queued logs
        while (!logQueue.empty()) {
            LogEntry entry = logQueue.front();
            logQueue.pop();

            // Unlock while doing I/O (don't block producers)
            lock.unlock();

            // Format and write log
            std::string logLine = "[" + entry.timestamp + "] " + 
                                 entry.category + ": " + entry.message;

            // Write to console
            std::cout << logLine << '\n';

            // Write to file
            if (logFile.is_open()) {
                logFile << logLine << '\n';
            }

            // Re-lock for next iteration
            lock.lock();
        }

        // Flush file periodically
        if (logFile.is_open()) {
            logFile.flush();
        }
    }
}

void MetricsLogger::flush() {
    // Wait for queue to empty
    while (true) {
        std::lock_guard<std::mutex> lock(queueMutex);
        if (logQueue.empty()) break;
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    if (logFile.is_open()) {
        logFile.flush();
    }
}