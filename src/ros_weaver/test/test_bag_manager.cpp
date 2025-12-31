#include <gtest/gtest.h>
#include "ros_weaver/core/bag_manager.hpp"
#include <QApplication>
#include <QSignalSpy>
#include <QTemporaryDir>
#include <QFile>

using namespace ros_weaver;

// ============================================================================
// BagTopicInfo Struct Tests
// ============================================================================

TEST(BagTopicInfoTest, DefaultValues) {
  BagTopicInfo info;

  EXPECT_TRUE(info.name.isEmpty());
  EXPECT_TRUE(info.messageType.isEmpty());
  EXPECT_TRUE(info.serialization_format.isEmpty());
  EXPECT_EQ(info.messageCount, 0);
  EXPECT_FALSE(info.durabilityTransient);
  EXPECT_TRUE(info.reliabilityReliable);  // Default is reliable
}

TEST(BagTopicInfoTest, SetValues) {
  BagTopicInfo info;
  info.name = "/scan";
  info.messageType = "sensor_msgs/msg/LaserScan";
  info.serialization_format = "cdr";
  info.messageCount = 10000;
  info.durabilityTransient = false;
  info.reliabilityReliable = true;

  EXPECT_EQ(info.name, "/scan");
  EXPECT_EQ(info.messageType, "sensor_msgs/msg/LaserScan");
  EXPECT_EQ(info.serialization_format, "cdr");
  EXPECT_EQ(info.messageCount, 10000);
  EXPECT_FALSE(info.durabilityTransient);
  EXPECT_TRUE(info.reliabilityReliable);
}

TEST(BagTopicInfoTest, TransientDurability) {
  BagTopicInfo info;
  info.name = "/tf_static";
  info.durabilityTransient = true;

  EXPECT_TRUE(info.durabilityTransient);
}

TEST(BagTopicInfoTest, BestEffortReliability) {
  BagTopicInfo info;
  info.name = "/camera/image_raw";
  info.reliabilityReliable = false;

  EXPECT_FALSE(info.reliabilityReliable);
}

TEST(BagTopicInfoTest, CopyConstruction) {
  BagTopicInfo original;
  original.name = "/odom";
  original.messageType = "nav_msgs/msg/Odometry";
  original.messageCount = 5000;

  BagTopicInfo copy = original;

  EXPECT_EQ(copy.name, original.name);
  EXPECT_EQ(copy.messageType, original.messageType);
  EXPECT_EQ(copy.messageCount, original.messageCount);
}

// ============================================================================
// TopicConfig Struct Tests
// ============================================================================

TEST(TopicConfigTest, DefaultValues) {
  TopicConfig config;

  EXPECT_TRUE(config.originalName.isEmpty());
  EXPECT_TRUE(config.remappedName.isEmpty());
  EXPECT_TRUE(config.messageType.isEmpty());
  EXPECT_TRUE(config.enabled);
  EXPECT_EQ(config.qosHistoryDepth, 10);
  EXPECT_TRUE(config.qosReliable);
  EXPECT_FALSE(config.qosDurabilityTransient);
  EXPECT_EQ(config.messageCount, 0);
}

TEST(TopicConfigTest, SetValues) {
  TopicConfig config;
  config.originalName = "/cmd_vel";
  config.remappedName = "/robot/cmd_vel";
  config.messageType = "geometry_msgs/msg/Twist";
  config.enabled = true;
  config.qosHistoryDepth = 20;
  config.qosReliable = true;
  config.qosDurabilityTransient = false;
  config.messageCount = 1000;

  EXPECT_EQ(config.originalName, "/cmd_vel");
  EXPECT_EQ(config.remappedName, "/robot/cmd_vel");
  EXPECT_EQ(config.messageType, "geometry_msgs/msg/Twist");
  EXPECT_TRUE(config.enabled);
  EXPECT_EQ(config.qosHistoryDepth, 20);
  EXPECT_TRUE(config.qosReliable);
  EXPECT_FALSE(config.qosDurabilityTransient);
  EXPECT_EQ(config.messageCount, 1000);
}

TEST(TopicConfigTest, DisabledTopic) {
  TopicConfig config;
  config.originalName = "/debug_topic";
  config.enabled = false;

  EXPECT_FALSE(config.enabled);
}

TEST(TopicConfigTest, RemappedTopic) {
  TopicConfig config;
  config.originalName = "/scan";
  config.remappedName = "/lidar/scan";

  EXPECT_NE(config.originalName, config.remappedName);
  EXPECT_EQ(config.remappedName, "/lidar/scan");
}

TEST(TopicConfigTest, CopyConstruction) {
  TopicConfig original;
  original.originalName = "/tf";
  original.remappedName = "/robot/tf";
  original.enabled = true;
  original.qosHistoryDepth = 100;

  TopicConfig copy = original;

  EXPECT_EQ(copy.originalName, original.originalName);
  EXPECT_EQ(copy.remappedName, original.remappedName);
  EXPECT_EQ(copy.enabled, original.enabled);
  EXPECT_EQ(copy.qosHistoryDepth, original.qosHistoryDepth);
}

// ============================================================================
// BagMetadata Struct Tests
// ============================================================================

TEST(BagMetadataTest, DefaultValues) {
  BagMetadata meta;

  EXPECT_TRUE(meta.path.isEmpty());
  EXPECT_TRUE(meta.storageId.isEmpty());
  EXPECT_TRUE(meta.serializationFormat.isEmpty());
  EXPECT_EQ(meta.startTimeNs, 0);
  EXPECT_EQ(meta.endTimeNs, 0);
  EXPECT_EQ(meta.durationNs, 0);
  EXPECT_EQ(meta.messageCount, 0);
  EXPECT_EQ(meta.fileSizeBytes, 0);
  EXPECT_EQ(meta.topicCount, 0);
}

TEST(BagMetadataTest, SetValues) {
  BagMetadata meta;
  meta.path = "/home/user/bags/robot_bag";
  meta.storageId = "sqlite3";
  meta.serializationFormat = "cdr";
  meta.startTimeNs = 1000000000;    // 1 second
  meta.endTimeNs = 61000000000;     // 61 seconds
  meta.durationNs = 60000000000;    // 60 seconds
  meta.messageCount = 100000;
  meta.fileSizeBytes = 1024 * 1024 * 50;  // 50 MB
  meta.topicCount = 15;

  EXPECT_EQ(meta.path, "/home/user/bags/robot_bag");
  EXPECT_EQ(meta.storageId, "sqlite3");
  EXPECT_EQ(meta.serializationFormat, "cdr");
  EXPECT_EQ(meta.startTimeNs, 1000000000);
  EXPECT_EQ(meta.endTimeNs, 61000000000);
  EXPECT_EQ(meta.durationNs, 60000000000);
  EXPECT_EQ(meta.messageCount, 100000);
  EXPECT_EQ(meta.fileSizeBytes, 1024 * 1024 * 50);
  EXPECT_EQ(meta.topicCount, 15);
}

TEST(BagMetadataTest, HelperMethods_StartTime) {
  BagMetadata meta;
  meta.startTimeNs = 5000000000;  // 5 seconds

  rclcpp::Time startTime = meta.startTime();
  EXPECT_EQ(startTime.nanoseconds(), 5000000000);
}

TEST(BagMetadataTest, HelperMethods_EndTime) {
  BagMetadata meta;
  meta.endTimeNs = 10000000000;  // 10 seconds

  rclcpp::Time endTime = meta.endTime();
  EXPECT_EQ(endTime.nanoseconds(), 10000000000);
}

TEST(BagMetadataTest, HelperMethods_Duration) {
  BagMetadata meta;
  meta.durationNs = 30000000000;  // 30 seconds

  rclcpp::Duration duration = meta.duration();
  EXPECT_EQ(duration.nanoseconds(), 30000000000);
}

TEST(BagMetadataTest, HelperMethods_Consistency) {
  BagMetadata meta;
  meta.startTimeNs = 1000000000;  // 1 second
  meta.endTimeNs = 11000000000;   // 11 seconds
  meta.durationNs = 10000000000;  // 10 seconds

  // Duration should be consistent with end - start
  EXPECT_EQ(meta.endTimeNs - meta.startTimeNs, meta.durationNs);

  // Helper method results should be consistent
  EXPECT_EQ(meta.endTime().nanoseconds() - meta.startTime().nanoseconds(),
            meta.duration().nanoseconds());
}

TEST(BagMetadataTest, StorageFormats) {
  BagMetadata meta1;
  meta1.storageId = "sqlite3";
  EXPECT_EQ(meta1.storageId, "sqlite3");

  BagMetadata meta2;
  meta2.storageId = "mcap";
  EXPECT_EQ(meta2.storageId, "mcap");
}

// ============================================================================
// BagManager Static Methods Tests
// ============================================================================

TEST(BagManagerStaticTest, DetectStorageFormat_McapFile) {
  QString format = BagManager::detectStorageFormat("/path/to/bag.mcap");
  EXPECT_EQ(format, "mcap");
}

TEST(BagManagerStaticTest, DetectStorageFormat_Db3File) {
  QString format = BagManager::detectStorageFormat("/path/to/bag.db3");
  EXPECT_EQ(format, "sqlite3");
}

TEST(BagManagerStaticTest, DetectStorageFormat_McapFileUpperCase) {
  QString format = BagManager::detectStorageFormat("/path/to/bag.MCAP");
  EXPECT_EQ(format, "mcap");
}

TEST(BagManagerStaticTest, DetectStorageFormat_Db3FileUpperCase) {
  QString format = BagManager::detectStorageFormat("/path/to/bag.DB3");
  EXPECT_EQ(format, "sqlite3");
}

TEST(BagManagerStaticTest, DetectStorageFormat_UnknownFile) {
  QString format = BagManager::detectStorageFormat("/path/to/unknown.txt");
  EXPECT_EQ(format, "sqlite3");  // Default fallback
}

TEST(BagManagerStaticTest, IsValidBagPath_NonExistent) {
  bool valid = BagManager::isValidBagPath("/nonexistent/path/to/bag.db3");
  EXPECT_FALSE(valid);
}

TEST(BagManagerStaticTest, DetectStorageFormat_EmptyPath) {
  QString format = BagManager::detectStorageFormat("");
  EXPECT_EQ(format, "sqlite3");  // Default fallback
}

// Tests with temporary directories
class BagManagerPathTest : public ::testing::Test {
protected:
  void SetUp() override {
    tempDir_ = std::make_unique<QTemporaryDir>();
    ASSERT_TRUE(tempDir_->isValid());
  }

  void TearDown() override {
    tempDir_.reset();
  }

  std::unique_ptr<QTemporaryDir> tempDir_;
};

TEST_F(BagManagerPathTest, IsValidBagPath_DirectoryWithMetadata) {
  QString bagPath = tempDir_->path() + "/test_bag";
  QDir().mkpath(bagPath);

  // Create metadata.yaml
  QFile metadataFile(bagPath + "/metadata.yaml");
  metadataFile.open(QIODevice::WriteOnly);
  metadataFile.write("rosbag2_bagfile_information:\n  version: 6\n");
  metadataFile.close();

  bool valid = BagManager::isValidBagPath(bagPath);
  EXPECT_TRUE(valid);
}

TEST_F(BagManagerPathTest, IsValidBagPath_DirectoryWithDb3) {
  QString bagPath = tempDir_->path() + "/test_bag";
  QDir().mkpath(bagPath);

  // Create a .db3 file
  QFile db3File(bagPath + "/test.db3");
  db3File.open(QIODevice::WriteOnly);
  db3File.write("dummy");
  db3File.close();

  bool valid = BagManager::isValidBagPath(bagPath);
  EXPECT_TRUE(valid);
}

TEST_F(BagManagerPathTest, IsValidBagPath_DirectoryWithMcap) {
  QString bagPath = tempDir_->path() + "/test_bag";
  QDir().mkpath(bagPath);

  // Create a .mcap file
  QFile mcapFile(bagPath + "/test.mcap");
  mcapFile.open(QIODevice::WriteOnly);
  mcapFile.write("dummy");
  mcapFile.close();

  bool valid = BagManager::isValidBagPath(bagPath);
  EXPECT_TRUE(valid);
}

TEST_F(BagManagerPathTest, IsValidBagPath_EmptyDirectory) {
  QString bagPath = tempDir_->path() + "/empty_bag";
  QDir().mkpath(bagPath);

  bool valid = BagManager::isValidBagPath(bagPath);
  EXPECT_FALSE(valid);
}

TEST_F(BagManagerPathTest, DetectStorageFormat_DirectoryWithMcap) {
  QString bagPath = tempDir_->path() + "/mcap_bag";
  QDir().mkpath(bagPath);

  QFile mcapFile(bagPath + "/data.mcap");
  mcapFile.open(QIODevice::WriteOnly);
  mcapFile.write("dummy");
  mcapFile.close();

  QString format = BagManager::detectStorageFormat(bagPath);
  EXPECT_EQ(format, "mcap");
}

TEST_F(BagManagerPathTest, DetectStorageFormat_DirectoryWithDb3) {
  QString bagPath = tempDir_->path() + "/sqlite_bag";
  QDir().mkpath(bagPath);

  QFile db3File(bagPath + "/data.db3");
  db3File.open(QIODevice::WriteOnly);
  db3File.write("dummy");
  db3File.close();

  QString format = BagManager::detectStorageFormat(bagPath);
  EXPECT_EQ(format, "sqlite3");
}

TEST_F(BagManagerPathTest, DetectStorageFormat_DirectoryWithBoth) {
  QString bagPath = tempDir_->path() + "/mixed_bag";
  QDir().mkpath(bagPath);

  // Create both types - MCAP should take priority
  QFile mcapFile(bagPath + "/data.mcap");
  mcapFile.open(QIODevice::WriteOnly);
  mcapFile.write("dummy");
  mcapFile.close();

  QFile db3File(bagPath + "/data.db3");
  db3File.open(QIODevice::WriteOnly);
  db3File.write("dummy");
  db3File.close();

  QString format = BagManager::detectStorageFormat(bagPath);
  EXPECT_EQ(format, "mcap");  // MCAP takes priority
}

// ============================================================================
// BagManager Instance Tests
// ============================================================================

class BagManagerTest : public ::testing::Test {
protected:
  void SetUp() override {
    manager_ = std::make_unique<BagManager>();
  }

  void TearDown() override {
    manager_.reset();
  }

  std::unique_ptr<BagManager> manager_;
};

TEST_F(BagManagerTest, InitialState) {
  EXPECT_FALSE(manager_->isOpen());
}

TEST_F(BagManagerTest, MetadataWhenClosed) {
  BagMetadata meta = manager_->metadata();
  EXPECT_TRUE(meta.path.isEmpty());
  EXPECT_EQ(meta.messageCount, 0);
}

TEST_F(BagManagerTest, TopicsWhenClosed) {
  QList<BagTopicInfo> topics = manager_->topics();
  EXPECT_TRUE(topics.isEmpty());
}

TEST_F(BagManagerTest, TopicConfigsWhenClosed) {
  QList<TopicConfig> configs = manager_->topicConfigs();
  EXPECT_TRUE(configs.isEmpty());
}

TEST_F(BagManagerTest, EnabledTopicsWhenClosed) {
  QStringList enabled = manager_->enabledTopics();
  EXPECT_TRUE(enabled.isEmpty());
}

TEST_F(BagManagerTest, HasNextWhenClosed) {
  EXPECT_FALSE(manager_->hasNext());
}

TEST_F(BagManagerTest, ReadNextWhenClosed) {
  auto msg = manager_->readNext();
  EXPECT_EQ(msg, nullptr);
}

TEST_F(BagManagerTest, OpenNonExistentBag) {
  bool result = manager_->openBag("/nonexistent/path/to/bag");
  EXPECT_FALSE(result);
  EXPECT_FALSE(manager_->isOpen());
}

TEST_F(BagManagerTest, OpenNonExistentBagEmitsError) {
  QSignalSpy spy(manager_.get(), &BagManager::errorOccurred);

  manager_->openBag("/nonexistent/path/to/bag");

  EXPECT_GE(spy.count(), 1);
}

TEST_F(BagManagerTest, CloseBagWhenNotOpen) {
  // Should not crash
  manager_->closeBag();
  EXPECT_FALSE(manager_->isOpen());
}

TEST_F(BagManagerTest, CloseBagEmitsSignal) {
  QSignalSpy spy(manager_.get(), &BagManager::bagClosed);

  manager_->closeBag();

  // Should emit even when not open
  EXPECT_EQ(spy.count(), 1);
}

TEST_F(BagManagerTest, SeekWhenClosed) {
  rclcpp::Time time(1000000000);
  bool result = manager_->seekToTime(time);
  EXPECT_FALSE(result);
}

TEST_F(BagManagerTest, SeekToStartWhenClosed) {
  bool result = manager_->seekToStart();
  EXPECT_FALSE(result);
}

TEST_F(BagManagerTest, CurrentReadTimeWhenClosed) {
  rclcpp::Time time = manager_->currentReadTime();
  EXPECT_EQ(time.nanoseconds(), 0);
}

TEST_F(BagManagerTest, TopicConfigForNonExistent) {
  TopicConfig config = manager_->topicConfig("/nonexistent_topic");
  EXPECT_TRUE(config.originalName.isEmpty());
}

TEST_F(BagManagerTest, SetTopicEnabledWhenClosed) {
  // Should not crash, just do nothing
  manager_->setTopicEnabled("/topic", false);
}

TEST_F(BagManagerTest, SetTopicRemapWhenClosed) {
  // Should not crash, just do nothing
  manager_->setTopicRemap("/original", "/remapped");
}

TEST_F(BagManagerTest, SetTopicQosWhenClosed) {
  // Should not crash, just do nothing
  manager_->setTopicQos("/topic", 100, true, false);
}

// ============================================================================
// Thread Safety Tests
// ============================================================================

TEST_F(BagManagerTest, ConcurrentAccess) {
  std::vector<std::thread> threads;

  // Multiple readers
  for (int i = 0; i < 5; ++i) {
    threads.emplace_back([this]() {
      for (int j = 0; j < 100; ++j) {
        manager_->isOpen();
        manager_->metadata();
        manager_->topics();
        manager_->topicConfigs();
        manager_->enabledTopics();
        manager_->hasNext();
      }
    });
  }

  for (auto& t : threads) {
    t.join();
  }

  // If we get here without deadlock or crash, test passes
  EXPECT_TRUE(true);
}

// ============================================================================
// Time Conversion Tests
// ============================================================================

TEST(TimeConversionTest, NanosecondsToSeconds) {
  int64_t nanoseconds = 5000000000;  // 5 seconds
  double seconds = nanoseconds / 1e9;
  EXPECT_DOUBLE_EQ(seconds, 5.0);
}

TEST(TimeConversionTest, SecondsToNanoseconds) {
  double seconds = 2.5;
  int64_t nanoseconds = static_cast<int64_t>(seconds * 1e9);
  EXPECT_EQ(nanoseconds, 2500000000);
}

TEST(TimeConversionTest, DurationCalculation) {
  int64_t startNs = 1000000000;  // 1 second
  int64_t endNs = 61000000000;   // 61 seconds
  int64_t durationNs = endNs - startNs;

  EXPECT_EQ(durationNs, 60000000000);  // 60 seconds

  double durationSeconds = durationNs / 1e9;
  EXPECT_DOUBLE_EQ(durationSeconds, 60.0);
}

int main(int argc, char** argv) {
  QApplication app(argc, argv);
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
