// SPDX-FileCopyrightText: 2026 Sergiu Deitsch <sergiu.deitsch@gmail.com>
//
// SPDX-License-Identifier: GPL-3.0-or-later

#ifndef ORB_SLAM_TESTS_LOG_CAPTURE_H
#define ORB_SLAM_TESTS_LOG_CAPTURE_H

#include <algorithm>
#include <mutex>
#include <string>
#include <string_view>
#include <vector>

#include <ng-log/experimental/setup.h>
#include <ng-log/experimental/sink.h>

namespace ORB_SLAM::tests {

//! Message that the library logged.
struct LoggedMessage
{
    nglog::LogSeverity severity;
    std::string text;
};

/**
 * Records the messages logged while it exists, from any thread.
 */
class LogCapture final : public nglog::experimental::Sink
{
public:
    LogCapture()
    {
        nglog::experimental::AddSink(*this);
    }

    ~LogCapture()
    {
        nglog::experimental::RemoveSink(*this);
    }

    LogCapture(const LogCapture&) = delete;
    LogCapture& operator=(const LogCapture&) = delete;

    void Send(const nglog::experimental::Record& record) override
    {
        std::scoped_lock lock{mutex_};
        messages_.push_back({record.severity(), std::string{record.text()}});
    }

    //! Returns the messages recorded so far.
    std::vector<LoggedMessage> messages() const
    {
        std::scoped_lock lock{mutex_};
        return messages_;
    }

    //! Returns whether a message of @p severity contains @p text.
    bool contains(nglog::LogSeverity severity, std::string_view text) const
    {
        std::scoped_lock lock{mutex_};
        return std::ranges::any_of(messages_, [severity, text](const LoggedMessage& message) {
            return message.severity == severity && message.text.contains(text);
        });
    }

private:
    mutable std::mutex mutex_;
    std::vector<LoggedMessage> messages_;
};

} // namespace ORB_SLAM::tests

#endif // ORB_SLAM_TESTS_LOG_CAPTURE_H
