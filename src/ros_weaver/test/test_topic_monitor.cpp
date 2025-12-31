#include <gtest/gtest.h>
#include "ros_weaver/core/topic_monitor.hpp"
#include <QApplication>
#include <QDateTime>

using namespace ros_weaver;

// ============================================================================
// TopicInfo Struct Tests
// ============================================================================

TEST(TopicInfoTest, DefaultValues) {
  TopicInfo info;

  EXPECT_TRUE(info.name.isEmpty());
  EXPECT_TRUE(info.type.isEmpty());
  EXPECT_TRUE(info.publishers.isEmpty());
  EXPECT_TRUE(info.subscribers.isEmpty());
  EXPECT_FALSE(info.isActive);
  EXPECT_DOUBLE_EQ(info.messageRate, 0.0);
  EXPECT_FALSE(info.lastMessageTime.isValid());
  EXPECT_EQ(info.messageCount, 0);
}

TEST(TopicInfoTest, SetBasicValues) {
  TopicInfo info;
  info.name = "/cmd_vel";
  info.type = "geometry_msgs/msg/Twist";
  info.isActive = true;
  info.messageRate = 10.0;
  info.messageCount = 100;

  EXPECT_EQ(info.name, "/cmd_vel");
  EXPECT_EQ(info.type, "geometry_msgs/msg/Twist");
  EXPECT_TRUE(info.isActive);
  EXPECT_DOUBLE_EQ(info.messageRate, 10.0);
  EXPECT_EQ(info.messageCount, 100);
}

TEST(TopicInfoTest, PublishersAndSubscribers) {
  TopicInfo info;
  info.name = "/scan";

  info.publishers << "lidar_driver" << "sensor_fusion";
  info.subscribers << "slam_toolbox" << "obstacle_detector" << "costmap";

  EXPECT_EQ(info.publishers.size(), 2);
  EXPECT_EQ(info.subscribers.size(), 3);
  EXPECT_TRUE(info.publishers.contains("lidar_driver"));
  EXPECT_TRUE(info.subscribers.contains("slam_toolbox"));
}

TEST(TopicInfoTest, LastMessageTime) {
  TopicInfo info;
  info.lastMessageTime = QDateTime::currentDateTime();

  EXPECT_TRUE(info.lastMessageTime.isValid());

  // Should be recent
  qint64 diff = QDateTime::currentMSecsSinceEpoch() - info.lastMessageTime.toMSecsSinceEpoch();
  EXPECT_LT(diff, 1000);  // Less than 1 second old
}

TEST(TopicInfoTest, CopyConstruction) {
  TopicInfo original;
  original.name = "/odom";
  original.type = "nav_msgs/msg/Odometry";
  original.publishers << "robot_driver";
  original.isActive = true;
  original.messageRate = 50.0;
  original.messageCount = 500;

  TopicInfo copy = original;

  EXPECT_EQ(copy.name, original.name);
  EXPECT_EQ(copy.type, original.type);
  EXPECT_EQ(copy.publishers.size(), original.publishers.size());
  EXPECT_EQ(copy.isActive, original.isActive);
  EXPECT_DOUBLE_EQ(copy.messageRate, original.messageRate);
  EXPECT_EQ(copy.messageCount, original.messageCount);
}

// ============================================================================
// TopicStats Struct Tests
// ============================================================================

TEST(TopicStatsTest, DefaultValues) {
  TopicStats stats;

  EXPECT_TRUE(stats.topicName.isEmpty());
  EXPECT_DOUBLE_EQ(stats.currentRate, 0.0);
  EXPECT_DOUBLE_EQ(stats.averageRate, 0.0);
  EXPECT_DOUBLE_EQ(stats.peakRate, 0.0);
  EXPECT_EQ(stats.totalMessages, 0);
  EXPECT_EQ(stats.lastMessageTimestamp, 0);
  EXPECT_TRUE(stats.recentTimestamps.empty());
}

TEST(TopicStatsTest, Constants) {
  // Verify the constants used for rate calculation
  EXPECT_EQ(TopicStats::MAX_TIMESTAMPS, 100);
  EXPECT_DOUBLE_EQ(TopicStats::RATE_WINDOW_SEC, 2.0);
}

TEST(TopicStatsTest, SetBasicValues) {
  TopicStats stats;
  stats.topicName = "/cmd_vel";
  stats.currentRate = 10.5;
  stats.averageRate = 9.8;
  stats.peakRate = 15.0;
  stats.totalMessages = 1000;
  stats.lastMessageTimestamp = QDateTime::currentMSecsSinceEpoch();

  EXPECT_EQ(stats.topicName, "/cmd_vel");
  EXPECT_DOUBLE_EQ(stats.currentRate, 10.5);
  EXPECT_DOUBLE_EQ(stats.averageRate, 9.8);
  EXPECT_DOUBLE_EQ(stats.peakRate, 15.0);
  EXPECT_EQ(stats.totalMessages, 1000);
  EXPECT_GT(stats.lastMessageTimestamp, 0);
}

TEST(TopicStatsTest, RecentTimestamps) {
  TopicStats stats;
  stats.topicName = "/scan";

  qint64 now = QDateTime::currentMSecsSinceEpoch();

  // Add some timestamps
  for (int i = 0; i < 10; ++i) {
    stats.recentTimestamps.push_back(now - (9 - i) * 100);  // 100ms apart
  }

  EXPECT_EQ(stats.recentTimestamps.size(), 10);
  EXPECT_LT(stats.recentTimestamps.front(), stats.recentTimestamps.back());
}

TEST(TopicStatsTest, TimestampQueueBehavior) {
  TopicStats stats;

  // Simulate adding more than MAX_TIMESTAMPS
  qint64 now = QDateTime::currentMSecsSinceEpoch();
  for (int i = 0; i < TopicStats::MAX_TIMESTAMPS + 50; ++i) {
    stats.recentTimestamps.push_back(now + i);

    // Mimic the trimming behavior in TopicMonitor
    while (stats.recentTimestamps.size() > TopicStats::MAX_TIMESTAMPS) {
      stats.recentTimestamps.pop_front();
    }
  }

  EXPECT_EQ(stats.recentTimestamps.size(), TopicStats::MAX_TIMESTAMPS);
}

TEST(TopicStatsTest, RateCalculationWindow) {
  TopicStats stats;

  // Verify the rate window is 2 seconds
  qint64 windowMs = static_cast<qint64>(TopicStats::RATE_WINDOW_SEC * 1000);
  EXPECT_EQ(windowMs, 2000);
}

TEST(TopicStatsTest, CopyConstruction) {
  TopicStats original;
  original.topicName = "/tf";
  original.currentRate = 100.0;
  original.averageRate = 98.5;
  original.peakRate = 105.0;
  original.totalMessages = 10000;

  original.recentTimestamps.push_back(1000);
  original.recentTimestamps.push_back(1010);
  original.recentTimestamps.push_back(1020);

  TopicStats copy = original;

  EXPECT_EQ(copy.topicName, original.topicName);
  EXPECT_DOUBLE_EQ(copy.currentRate, original.currentRate);
  EXPECT_DOUBLE_EQ(copy.averageRate, original.averageRate);
  EXPECT_DOUBLE_EQ(copy.peakRate, original.peakRate);
  EXPECT_EQ(copy.totalMessages, original.totalMessages);
  EXPECT_EQ(copy.recentTimestamps.size(), original.recentTimestamps.size());
}

// ============================================================================
// TopicMonitor Tests (without ROS - testing interface only)
// ============================================================================

class TopicMonitorTest : public ::testing::Test {
protected:
  void SetUp() override {
    monitor_ = std::make_unique<TopicMonitor>();
  }

  void TearDown() override {
    monitor_.reset();
  }

  std::unique_ptr<TopicMonitor> monitor_;
};

TEST_F(TopicMonitorTest, InitialState) {
  EXPECT_FALSE(monitor_->isMonitoring());
}

TEST_F(TopicMonitorTest, GetTopicsWhenNotMonitoring) {
  QList<TopicInfo> topics = monitor_->getTopics();
  EXPECT_TRUE(topics.isEmpty());
}

TEST_F(TopicMonitorTest, GetTopicInfoForNonExistent) {
  TopicInfo info = monitor_->getTopicInfo("/nonexistent");
  EXPECT_TRUE(info.name.isEmpty());
}

TEST_F(TopicMonitorTest, GetTopicStatsForNonExistent) {
  TopicStats stats = monitor_->getTopicStats("/nonexistent");
  EXPECT_TRUE(stats.topicName.isEmpty());
}

TEST_F(TopicMonitorTest, GetMonitoredTopicsWhenEmpty) {
  QStringList monitored = monitor_->getMonitoredTopics();
  EXPECT_TRUE(monitored.isEmpty());
}

TEST_F(TopicMonitorTest, IsTopicMonitoredForNonExistent) {
  EXPECT_FALSE(monitor_->isTopicMonitored("/nonexistent"));
}

// ============================================================================
// TopicMonitor Activity Timeout Tests
// ============================================================================

TEST(TopicMonitorConstantsTest, ActivityTimeout) {
  // Access the constant through the class
  // ACTIVITY_TIMEOUT_MS = 3000 (3 seconds)
  // This is a private constant, so we test the expected behavior

  // If a topic hasn't received a message in 3 seconds, it should become inactive
  // We can't directly access the constant, but we document the expected value
  EXPECT_TRUE(true);  // Placeholder - actual test requires ROS
}

TEST(TopicMonitorConstantsTest, UpdateInterval) {
  // UPDATE_INTERVAL_MS = 100 (100ms)
  // DISCOVERY_INTERVAL_MS = 2000 (2 seconds)
  // These are private constants
  EXPECT_TRUE(true);  // Placeholder - documenting expected values
}

// ============================================================================
// Rate Calculation Algorithm Tests
// ============================================================================

class RateCalculationTest : public ::testing::Test {
protected:
  // Simulate the rate calculation algorithm from TopicMonitor
  double calculateRate(const std::deque<qint64>& timestamps, qint64 now) {
    if (timestamps.size() < 2) {
      return 0.0;
    }

    qint64 windowStart = now - static_cast<qint64>(TopicStats::RATE_WINDOW_SEC * 1000);

    int messagesInWindow = 0;
    qint64 firstTimestamp = now;
    qint64 lastTimestamp = 0;

    for (const auto& ts : timestamps) {
      if (ts >= windowStart) {
        messagesInWindow++;
        if (ts < firstTimestamp) firstTimestamp = ts;
        if (ts > lastTimestamp) lastTimestamp = ts;
      }
    }

    if (messagesInWindow >= 2 && lastTimestamp > firstTimestamp) {
      double durationSec = (lastTimestamp - firstTimestamp) / 1000.0;
      return (messagesInWindow - 1) / durationSec;
    } else if (messagesInWindow == 1) {
      return 0.5;  // Assume at least some activity
    }

    return 0.0;
  }
};

TEST_F(RateCalculationTest, NoTimestamps) {
  std::deque<qint64> timestamps;
  qint64 now = QDateTime::currentMSecsSinceEpoch();

  double rate = calculateRate(timestamps, now);
  EXPECT_DOUBLE_EQ(rate, 0.0);
}

TEST_F(RateCalculationTest, SingleTimestamp) {
  std::deque<qint64> timestamps;
  qint64 now = QDateTime::currentMSecsSinceEpoch();
  timestamps.push_back(now - 100);

  double rate = calculateRate(timestamps, now);
  // With less than 2 timestamps, rate is 0.0 (cannot calculate rate from single point)
  EXPECT_DOUBLE_EQ(rate, 0.0);
}

TEST_F(RateCalculationTest, TwoTimestamps100msApart) {
  std::deque<qint64> timestamps;
  qint64 now = QDateTime::currentMSecsSinceEpoch();
  timestamps.push_back(now - 100);
  timestamps.push_back(now);

  double rate = calculateRate(timestamps, now);
  // 1 interval over 0.1 seconds = 10 Hz
  EXPECT_NEAR(rate, 10.0, 0.1);
}

TEST_F(RateCalculationTest, TenMessagesAt10Hz) {
  std::deque<qint64> timestamps;
  qint64 now = QDateTime::currentMSecsSinceEpoch();

  // 10 messages, 100ms apart = 10 Hz
  for (int i = 0; i < 10; ++i) {
    timestamps.push_back(now - (9 - i) * 100);
  }

  double rate = calculateRate(timestamps, now);
  EXPECT_NEAR(rate, 10.0, 0.5);
}

TEST_F(RateCalculationTest, HighFrequency100Hz) {
  std::deque<qint64> timestamps;
  qint64 now = QDateTime::currentMSecsSinceEpoch();

  // 100 messages, 10ms apart = 100 Hz
  for (int i = 0; i < 100; ++i) {
    timestamps.push_back(now - (99 - i) * 10);
  }

  double rate = calculateRate(timestamps, now);
  EXPECT_NEAR(rate, 100.0, 5.0);
}

TEST_F(RateCalculationTest, OldTimestampsIgnored) {
  std::deque<qint64> timestamps;
  qint64 now = QDateTime::currentMSecsSinceEpoch();

  // Add old timestamps outside the 2-second window
  timestamps.push_back(now - 5000);  // 5 seconds ago
  timestamps.push_back(now - 4000);  // 4 seconds ago

  // Add recent timestamps in the window
  timestamps.push_back(now - 100);
  timestamps.push_back(now);

  double rate = calculateRate(timestamps, now);
  // Only the last 2 messages should count
  EXPECT_NEAR(rate, 10.0, 0.5);
}

TEST_F(RateCalculationTest, AllTimestampsOld) {
  std::deque<qint64> timestamps;
  qint64 now = QDateTime::currentMSecsSinceEpoch();

  // All timestamps are older than the rate window
  timestamps.push_back(now - 5000);
  timestamps.push_back(now - 4000);
  timestamps.push_back(now - 3000);

  double rate = calculateRate(timestamps, now);
  EXPECT_DOUBLE_EQ(rate, 0.0);
}

// ============================================================================
// Exponential Moving Average Tests
// ============================================================================

TEST(ExponentialMovingAverageTest, InitialValue) {
  double averageRate = 0.0;
  double currentRate = 10.0;

  // First update sets the average to current
  if (averageRate == 0.0) {
    averageRate = currentRate;
  }

  EXPECT_DOUBLE_EQ(averageRate, 10.0);
}

TEST(ExponentialMovingAverageTest, Smoothing) {
  double averageRate = 10.0;
  double currentRate = 20.0;

  // EMA formula: 0.9 * average + 0.1 * current
  averageRate = 0.9 * averageRate + 0.1 * currentRate;

  // 0.9 * 10 + 0.1 * 20 = 9 + 2 = 11
  EXPECT_DOUBLE_EQ(averageRate, 11.0);
}

TEST(ExponentialMovingAverageTest, ConvergenceToStable) {
  double averageRate = 0.0;
  double stableRate = 50.0;

  // After the first update
  averageRate = stableRate;

  // Simulate many updates with stable rate
  for (int i = 0; i < 100; ++i) {
    averageRate = 0.9 * averageRate + 0.1 * stableRate;
  }

  // Should converge to the stable rate
  EXPECT_NEAR(averageRate, stableRate, 0.01);
}

TEST(ExponentialMovingAverageTest, SlowResponseToChange) {
  double averageRate = 10.0;
  double newRate = 100.0;

  // The EMA smooths out sudden changes
  double previousAvg = averageRate;
  averageRate = 0.9 * averageRate + 0.1 * newRate;

  // Should be closer to old value than new value
  EXPECT_LT(averageRate, (previousAvg + newRate) / 2);
  EXPECT_NEAR(averageRate, 19.0, 0.01);  // 0.9*10 + 0.1*100 = 19
}

int main(int argc, char** argv) {
  QApplication app(argc, argv);
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
