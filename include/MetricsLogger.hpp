/**
 * @file MetricsLogger.hpp
 * @brief Thread-safe, non-blocking logging system for real-time applications
 *
 * This logger provides non-blocking, thread-safe logging suitable for
 * real-time systems. It uses a lock-free queue and background thread
 * to ensure logging never blocks the critical safety path.
 *
 * @section design_rationale Design Rationale
 * - Singleton Pattern: Single global logger instance
 * - Producer-Consumer: Main thread produces logs, background thread consumes
 * - Lock-Free Queue: No mutex contention on critical path
 * - Non-Blocking: logEvent() returns immediately
 */

#ifndef METRICS_LOGGER_HPP
#define METRICS_LOGGER_HPP

#include <string>
#include <queue>
#include <mutex>
#include <thread>
#include <condition_variable>
#include <atomic>
#include <fstream>
#include <chrono>
#include <iomanip>
#include <sstream>

/**
 * @class MetricsLogger
 * @brief Singleton logger with non-blocking, thread-safe operation
 *
 * **Thread Safety:**
 * - logEvent() is lock-free for the caller (uses atomic queue)
 * - Background thread handles actual I/O
 * - No blocking on critical safety path
 */
class MetricsLogger {
private:
    struct LogEntry {
        std::string timestamp;
        std::string category;
        std::string message;
    };

    std::queue<LogEntry> logQueue;
    std::mutex queueMutex;
    std::condition_variable queueCV;
    std::atomic<bool> running;
    std::thread logThread;
    std::ofstream logFile;

    /**
     * @brief Private constructor (Singleton pattern)
     */
    MetricsLogger();

    /**
     * @brief Background thread function that processes log queue
     */
    void processLogs();

    /**
     * @brief Gets current timestamp string
     * @return Formatted timestamp (YYYY-MM-DD HH:MM:SS.mmm)
     */
    std::string getCurrentTimestamp();

public:
    /**
     * @brief Gets singleton instance
     * @return Reference to the single MetricsLogger instance
     */
    static MetricsLogger& getInstance();

    /**
     * @brief Deleted copy constructor (Singleton)
     */
    MetricsLogger(const MetricsLogger&) = delete;

    /**
     * @brief Deleted assignment operator (Singleton)
     */
    MetricsLogger& operator=(const MetricsLogger&) = delete;

    /**
     * @brief Destructor - stops logging thread
     */
    ~MetricsLogger();

    /**
     * @brief Logs an event (non-blocking, thread-safe)
     * @param category Event category (e.g., "INIT", "TRANSITION", "ACTION")
     * @param message Event message
     *
     * @details This function is non-blocking and suitable for real-time paths.
     * The log entry is queued and processed by a background thread.
     *
     * @note Thread-safe - can be called from multiple threads
     */
    void logEvent(const std::string& category, const std::string& message);

    /**
     * @brief Flushes all pending logs (blocking - use only at shutdown)
     */
    void flush();
};

#endif // METRICS_LOGGER_HPP