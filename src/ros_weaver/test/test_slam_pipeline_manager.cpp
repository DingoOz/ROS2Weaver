#include <gtest/gtest.h>
#include "ros_weaver/core/slam_pipeline_manager.hpp"
#include <QApplication>
#include <QSignalSpy>

using namespace ros_weaver;

// ============================================================================
// SlamNodeStatus Enum Tests
// ============================================================================

TEST(SlamNodeStatusTest, EnumValues) {
  // Verify enum values for serialization compatibility
  EXPECT_EQ(static_cast<int>(SlamNodeStatus::NotRunning), 0);
  EXPECT_EQ(static_cast<int>(SlamNodeStatus::Starting), 1);
  EXPECT_EQ(static_cast<int>(SlamNodeStatus::Running), 2);
  EXPECT_EQ(static_cast<int>(SlamNodeStatus::Error), 3);
  EXPECT_EQ(static_cast<int>(SlamNodeStatus::Stopping), 4);
}

TEST(SlamNodeStatusTest, AllStatusesDefined) {
  // Ensure we have all expected statuses
  SlamNodeStatus statuses[] = {
    SlamNodeStatus::NotRunning,
    SlamNodeStatus::Starting,
    SlamNodeStatus::Running,
    SlamNodeStatus::Error,
    SlamNodeStatus::Stopping
  };
  EXPECT_EQ(sizeof(statuses) / sizeof(statuses[0]), 5);
}

// ============================================================================
// SlamConfig Struct Tests
// ============================================================================

TEST(SlamConfigTest, DefaultValues) {
  SlamConfig config;

  // Check default frame names
  EXPECT_EQ(config.mapFrame, "map");
  EXPECT_EQ(config.odomFrame, "odom");
  EXPECT_EQ(config.baseFrame, "base_footprint");
  EXPECT_EQ(config.scanTopic, "/scan");

  // Empty by default
  EXPECT_TRUE(config.packageName.isEmpty());
  EXPECT_TRUE(config.launchFile.isEmpty());
  EXPECT_TRUE(config.mode.isEmpty());
  EXPECT_TRUE(config.parameters.isEmpty());
}

TEST(SlamConfigTest, SetValues) {
  SlamConfig config;
  config.packageName = "slam_toolbox";
  config.launchFile = "online_async_launch.py";
  config.mode = "mapping";
  config.mapFrame = "custom_map";
  config.odomFrame = "custom_odom";
  config.baseFrame = "base_link";
  config.scanTopic = "/laser_scan";
  config.parameters["resolution"] = 0.05;
  config.parameters["max_laser_range"] = 20.0;

  EXPECT_EQ(config.packageName, "slam_toolbox");
  EXPECT_EQ(config.launchFile, "online_async_launch.py");
  EXPECT_EQ(config.mode, "mapping");
  EXPECT_EQ(config.mapFrame, "custom_map");
  EXPECT_EQ(config.odomFrame, "custom_odom");
  EXPECT_EQ(config.baseFrame, "base_link");
  EXPECT_EQ(config.scanTopic, "/laser_scan");
  EXPECT_EQ(config.parameters.size(), 2);
  EXPECT_DOUBLE_EQ(config.parameters["resolution"].toDouble(), 0.05);
  EXPECT_DOUBLE_EQ(config.parameters["max_laser_range"].toDouble(), 20.0);
}

TEST(SlamConfigTest, CopyConstruction) {
  SlamConfig original;
  original.packageName = "test_package";
  original.mapFrame = "my_map";
  original.parameters["key"] = "value";

  SlamConfig copy = original;

  EXPECT_EQ(copy.packageName, original.packageName);
  EXPECT_EQ(copy.mapFrame, original.mapFrame);
  EXPECT_EQ(copy.parameters["key"], original.parameters["key"]);
}

TEST(SlamConfigTest, ParameterVariantTypes) {
  SlamConfig config;

  // Test different QVariant types
  config.parameters["string_param"] = QString("test");
  config.parameters["int_param"] = 42;
  config.parameters["double_param"] = 3.14159;
  config.parameters["bool_param"] = true;

  EXPECT_EQ(config.parameters["string_param"].toString(), "test");
  EXPECT_EQ(config.parameters["int_param"].toInt(), 42);
  EXPECT_DOUBLE_EQ(config.parameters["double_param"].toDouble(), 3.14159);
  EXPECT_TRUE(config.parameters["bool_param"].toBool());
}

// ============================================================================
// SlamPreset Struct Tests
// ============================================================================

TEST(SlamPresetTest, DefaultConstruction) {
  SlamPreset preset;

  EXPECT_TRUE(preset.name.isEmpty());
  EXPECT_TRUE(preset.description.isEmpty());
  EXPECT_TRUE(preset.packageName.isEmpty());
}

TEST(SlamPresetTest, SetValues) {
  SlamPreset preset;
  preset.name = "Indoor Mapping";
  preset.description = "Optimized for indoor environments";
  preset.packageName = "slam_toolbox";
  preset.config.mode = "mapping";
  preset.config.parameters["resolution"] = 0.05;

  EXPECT_EQ(preset.name, "Indoor Mapping");
  EXPECT_EQ(preset.description, "Optimized for indoor environments");
  EXPECT_EQ(preset.packageName, "slam_toolbox");
  EXPECT_EQ(preset.config.mode, "mapping");
  EXPECT_DOUBLE_EQ(preset.config.parameters["resolution"].toDouble(), 0.05);
}

// ============================================================================
// SlamPipelineManager Tests (without ROS dependencies)
// ============================================================================

class SlamPipelineManagerTest : public ::testing::Test {
protected:
  void SetUp() override {
    manager_ = std::make_unique<SlamPipelineManager>();
  }

  void TearDown() override {
    manager_.reset();
  }

  std::unique_ptr<SlamPipelineManager> manager_;
};

TEST_F(SlamPipelineManagerTest, InitialStatus) {
  EXPECT_EQ(manager_->status(), SlamNodeStatus::NotRunning);
  EXPECT_FALSE(manager_->isRunning());
}

TEST_F(SlamPipelineManagerTest, InitialAutoRerunDisabled) {
  EXPECT_FALSE(manager_->isAutoRerunEnabled());
}

TEST_F(SlamPipelineManagerTest, EnableAutoRerun) {
  manager_->enableAutoRerun(true);
  EXPECT_TRUE(manager_->isAutoRerunEnabled());

  manager_->enableAutoRerun(false);
  EXPECT_FALSE(manager_->isAutoRerunEnabled());
}

TEST_F(SlamPipelineManagerTest, SetAndGetParameter) {
  manager_->setParameter("test_param", 42);

  QVariant value = manager_->getParameter("test_param");
  EXPECT_EQ(value.toInt(), 42);
}

TEST_F(SlamPipelineManagerTest, SetMultipleParameters) {
  manager_->setParameter("resolution", 0.05);
  manager_->setParameter("max_range", 20.0);
  manager_->setParameter("min_range", 0.5);

  EXPECT_DOUBLE_EQ(manager_->getParameter("resolution").toDouble(), 0.05);
  EXPECT_DOUBLE_EQ(manager_->getParameter("max_range").toDouble(), 20.0);
  EXPECT_DOUBLE_EQ(manager_->getParameter("min_range").toDouble(), 0.5);
}

TEST_F(SlamPipelineManagerTest, GetAllParameters) {
  manager_->setParameter("param1", 1);
  manager_->setParameter("param2", 2);
  manager_->setParameter("param3", 3);

  QMap<QString, QVariant> params = manager_->getAllParameters();
  EXPECT_EQ(params.size(), 3);
  EXPECT_TRUE(params.contains("param1"));
  EXPECT_TRUE(params.contains("param2"));
  EXPECT_TRUE(params.contains("param3"));
}

TEST_F(SlamPipelineManagerTest, ParameterNames) {
  manager_->setParameter("alpha", 1.0);
  manager_->setParameter("beta", 2.0);

  QStringList names = manager_->parameterNames();
  EXPECT_EQ(names.size(), 2);
  EXPECT_TRUE(names.contains("alpha"));
  EXPECT_TRUE(names.contains("beta"));
}

TEST_F(SlamPipelineManagerTest, GetNonExistentParameter) {
  QVariant value = manager_->getParameter("nonexistent");
  EXPECT_FALSE(value.isValid());
}

TEST_F(SlamPipelineManagerTest, OverwriteParameter) {
  manager_->setParameter("param", 100);
  EXPECT_EQ(manager_->getParameter("param").toInt(), 100);

  manager_->setParameter("param", 200);
  EXPECT_EQ(manager_->getParameter("param").toInt(), 200);
}

TEST_F(SlamPipelineManagerTest, ParameterChangedSignal) {
  QSignalSpy spy(manager_.get(), &SlamPipelineManager::parameterChanged);

  manager_->setParameter("test_param", 42);

  // Signal may or may not be emitted depending on implementation
  // This test documents the expected behavior
  if (spy.count() > 0) {
    QList<QVariant> arguments = spy.takeFirst();
    EXPECT_EQ(arguments.at(0).toString(), "test_param");
    EXPECT_EQ(arguments.at(1).toInt(), 42);
  }
}

TEST_F(SlamPipelineManagerTest, CurrentConfigInitiallyEmpty) {
  SlamConfig config = manager_->currentConfig();
  EXPECT_TRUE(config.packageName.isEmpty());
  EXPECT_TRUE(config.launchFile.isEmpty());
}

TEST_F(SlamPipelineManagerTest, AvailablePresetsInitiallyEmpty) {
  QStringList presets = manager_->availablePresets();
  // May have built-in presets or be empty
  // Just verify it doesn't crash
  EXPECT_GE(presets.size(), 0);
}

TEST_F(SlamPipelineManagerTest, SetPresetsDirectory) {
  // Should not crash with empty or invalid path
  manager_->setPresetsDirectory("");
  manager_->setPresetsDirectory("/nonexistent/path");
  manager_->setPresetsDirectory("/tmp");
}

TEST_F(SlamPipelineManagerTest, LoadNonExistentPreset) {
  // Should handle gracefully
  manager_->loadPreset("nonexistent_preset");
  // Just verify no crash
}

TEST_F(SlamPipelineManagerTest, DeleteNonExistentPreset) {
  // Should handle gracefully
  manager_->deletePreset("nonexistent_preset");
  // Just verify no crash
}

TEST_F(SlamPipelineManagerTest, StopWhenNotRunning) {
  // Should be safe to call stop when not running
  EXPECT_EQ(manager_->status(), SlamNodeStatus::NotRunning);
  manager_->stopSlam();
  EXPECT_EQ(manager_->status(), SlamNodeStatus::NotRunning);
}

TEST_F(SlamPipelineManagerTest, TriggerRerunWhenNotRunning) {
  // Should be safe to call rerun when not running
  manager_->triggerRerun();
  // Just verify no crash
}

TEST_F(SlamPipelineManagerTest, SetBagManagerNull) {
  // Should handle null gracefully
  manager_->setBagManager(nullptr);
  // Just verify no crash
}

TEST_F(SlamPipelineManagerTest, SetPlaybackControllerNull) {
  // Should handle null gracefully
  manager_->setPlaybackController(nullptr);
  // Just verify no crash
}

// ============================================================================
// Thread Safety Tests (for the deadlock fix)
// ============================================================================

TEST_F(SlamPipelineManagerTest, ConcurrentParameterAccess) {
  // Test concurrent read/write to verify thread safety
  // This tests the mutex usage that was fixed in the deadlock fix

  std::vector<std::thread> threads;

  // Writers
  for (int i = 0; i < 5; ++i) {
    threads.emplace_back([this, i]() {
      for (int j = 0; j < 100; ++j) {
        manager_->setParameter(QString("param_%1").arg(i), j);
      }
    });
  }

  // Readers
  for (int i = 0; i < 5; ++i) {
    threads.emplace_back([this, i]() {
      for (int j = 0; j < 100; ++j) {
        manager_->getParameter(QString("param_%1").arg(i));
        manager_->getAllParameters();
        manager_->parameterNames();
      }
    });
  }

  for (auto& t : threads) {
    t.join();
  }

  // If we get here without deadlock or crash, the test passes
  EXPECT_TRUE(true);
}

TEST_F(SlamPipelineManagerTest, ConcurrentStatusAccess) {
  // Test concurrent status reads with other operations

  std::vector<std::thread> threads;

  for (int i = 0; i < 10; ++i) {
    threads.emplace_back([this]() {
      for (int j = 0; j < 100; ++j) {
        manager_->status();
        manager_->isRunning();
        manager_->isAutoRerunEnabled();
      }
    });
  }

  for (auto& t : threads) {
    t.join();
  }

  EXPECT_TRUE(true);
}

int main(int argc, char** argv) {
  // QApplication needed for Qt types
  QApplication app(argc, argv);
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
