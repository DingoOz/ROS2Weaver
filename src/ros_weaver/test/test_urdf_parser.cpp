#include <gtest/gtest.h>
#include <QCoreApplication>
#include <QVector3D>
#include <QQuaternion>

#include "ros_weaver/core/urdf_parser.hpp"

using namespace ros_weaver;

// Fixture path defined by CMake
#ifndef TEST_FIXTURES_DIR
#define TEST_FIXTURES_DIR "test/fixtures"
#endif

static const QString TEST_URDF_PATH = QString(TEST_FIXTURES_DIR) + "/simple_robot.urdf";

class URDFParserTest : public ::testing::Test {
protected:
  void SetUp() override {
    parser_ = new URDFParser();
  }

  void TearDown() override {
    delete parser_;
  }

  URDFParser* parser_;
};

TEST_F(URDFParserTest, LoadValidURDF) {
  bool success = parser_->loadFromFile(TEST_URDF_PATH);
  EXPECT_TRUE(success);
  EXPECT_TRUE(parser_->isLoaded());
  EXPECT_EQ(parser_->getModel().name, "simple_robot");
}

TEST_F(URDFParserTest, FailOnInvalidFile) {
  bool success = parser_->loadFromFile("nonexistent_file.urdf");
  EXPECT_FALSE(success);
  EXPECT_FALSE(parser_->isLoaded());
}

TEST_F(URDFParserTest, ParseLinks) {
  parser_->loadFromFile(TEST_URDF_PATH);

  const URDFModel& model = parser_->getModel();
  EXPECT_EQ(model.links.size(), 5);

  // Check root link
  EXPECT_EQ(model.rootLink, "base_link");

  // Check link names
  QStringList linkNames = parser_->getLinkNames();
  EXPECT_TRUE(linkNames.contains("base_link"));
  EXPECT_TRUE(linkNames.contains("shoulder_link"));
  EXPECT_TRUE(linkNames.contains("upper_arm_link"));
  EXPECT_TRUE(linkNames.contains("forearm_link"));
  EXPECT_TRUE(linkNames.contains("end_effector_link"));
}

TEST_F(URDFParserTest, ParseJoints) {
  parser_->loadFromFile(TEST_URDF_PATH);

  const URDFModel& model = parser_->getModel();
  EXPECT_EQ(model.joints.size(), 4);

  // Check joint names
  QStringList jointNames = parser_->getJointNames();
  EXPECT_TRUE(jointNames.contains("base_to_shoulder"));
  EXPECT_TRUE(jointNames.contains("shoulder_to_upper_arm"));
  EXPECT_TRUE(jointNames.contains("upper_arm_to_forearm"));
  EXPECT_TRUE(jointNames.contains("forearm_to_end_effector"));
}

TEST_F(URDFParserTest, GetJointByName) {
  parser_->loadFromFile(TEST_URDF_PATH);

  URDFJoint* joint = parser_->getJoint("base_to_shoulder");
  ASSERT_NE(joint, nullptr);
  EXPECT_EQ(joint->name, "base_to_shoulder");
  EXPECT_EQ(joint->type, "revolute");
  EXPECT_EQ(joint->parentLink, "base_link");
  EXPECT_EQ(joint->childLink, "shoulder_link");
}

TEST_F(URDFParserTest, GetLinkByName) {
  parser_->loadFromFile(TEST_URDF_PATH);

  URDFLink* link = parser_->getLink("shoulder_link");
  ASSERT_NE(link, nullptr);
  EXPECT_EQ(link->name, "shoulder_link");
}

TEST_F(URDFParserTest, JointOrientationModification) {
  parser_->loadFromFile(TEST_URDF_PATH);

  QQuaternion rot90Z = QQuaternion::fromAxisAndAngle(0, 0, 1, 90);
  parser_->setJointOrientation("base_to_shoulder", rot90Z);

  URDFJoint* joint = parser_->getJoint("base_to_shoulder");
  ASSERT_NE(joint, nullptr);

  // Check that orientation was set (allowing for floating point tolerance)
  EXPECT_NEAR(joint->orientation.x(), rot90Z.x(), 0.001f);
  EXPECT_NEAR(joint->orientation.y(), rot90Z.y(), 0.001f);
  EXPECT_NEAR(joint->orientation.z(), rot90Z.z(), 0.001f);
  EXPECT_NEAR(joint->orientation.scalar(), rot90Z.scalar(), 0.001f);
}

TEST_F(URDFParserTest, RotateJointBy) {
  parser_->loadFromFile(TEST_URDF_PATH);

  parser_->rotateJointBy("base_to_shoulder", QVector3D(0, 0, 1), 45.0);

  URDFJoint* joint = parser_->getJoint("base_to_shoulder");
  ASSERT_NE(joint, nullptr);

  // Check that orientation is not identity after rotation
  EXPECT_FALSE(joint->orientation.isIdentity());
}

TEST_F(URDFParserTest, ExportToString) {
  parser_->loadFromFile(TEST_URDF_PATH);

  QString exported = parser_->exportToString();
  EXPECT_FALSE(exported.isEmpty());
  EXPECT_TRUE(exported.contains("simple_robot"));
  EXPECT_TRUE(exported.contains("base_link"));
  EXPECT_TRUE(exported.contains("revolute"));
}

TEST_F(URDFParserTest, ExportModifiedURDF) {
  parser_->loadFromFile(TEST_URDF_PATH);
  parser_->rotateJointBy("base_to_shoulder", QVector3D(0, 0, 1), 90);

  QString exported = parser_->exportToString();
  // Check that orientation is preserved in export (rpy attribute)
  EXPECT_TRUE(exported.contains("rpy=") || exported.contains("origin"));
}

TEST_F(URDFParserTest, GetChildJoints) {
  parser_->loadFromFile(TEST_URDF_PATH);

  QStringList children = parser_->getChildJoints("base_link");
  EXPECT_EQ(children.size(), 1);
  EXPECT_TRUE(children.contains("base_to_shoulder"));

  children = parser_->getChildJoints("shoulder_link");
  EXPECT_EQ(children.size(), 1);
  EXPECT_TRUE(children.contains("shoulder_to_upper_arm"));
}

TEST_F(URDFParserTest, GetParentChain) {
  parser_->loadFromFile(TEST_URDF_PATH);

  QStringList chain = parser_->getParentChain("end_effector_link");
  EXPECT_EQ(chain.size(), 4);  // 4 joints from end_effector back to base
  EXPECT_TRUE(chain.contains("forearm_to_end_effector"));
  EXPECT_TRUE(chain.contains("upper_arm_to_forearm"));
  EXPECT_TRUE(chain.contains("shoulder_to_upper_arm"));
  EXPECT_TRUE(chain.contains("base_to_shoulder"));
}

TEST_F(URDFParserTest, Clear) {
  parser_->loadFromFile(TEST_URDF_PATH);
  EXPECT_TRUE(parser_->isLoaded());

  parser_->clear();
  EXPECT_FALSE(parser_->isLoaded());
  EXPECT_EQ(parser_->getModel().links.size(), 0);
  EXPECT_EQ(parser_->getModel().joints.size(), 0);
}

TEST_F(URDFParserTest, ParsePrimitiveGeometries) {
  // Test that primitive geometries (box, cylinder, sphere) are parsed correctly
  parser_->loadFromFile(TEST_URDF_PATH);

  // base_link has a box
  URDFLink* baseLink = parser_->getLink("base_link");
  ASSERT_NE(baseLink, nullptr);
  EXPECT_TRUE(baseLink->meshPath.contains("primitive://box"));

  // shoulder_link has a cylinder
  URDFLink* shoulderLink = parser_->getLink("shoulder_link");
  ASSERT_NE(shoulderLink, nullptr);
  EXPECT_TRUE(shoulderLink->meshPath.contains("primitive://cylinder"));

  // end_effector_link has a sphere
  URDFLink* endEffectorLink = parser_->getLink("end_effector_link");
  ASSERT_NE(endEffectorLink, nullptr);
  EXPECT_TRUE(endEffectorLink->meshPath.contains("primitive://sphere"));
}

TEST_F(URDFParserTest, ParseJointLimits) {
  parser_->loadFromFile(TEST_URDF_PATH);

  URDFJoint* joint = parser_->getJoint("base_to_shoulder");
  ASSERT_NE(joint, nullptr);

  EXPECT_NEAR(joint->lowerLimit, -3.14159, 0.001);
  EXPECT_NEAR(joint->upperLimit, 3.14159, 0.001);
  EXPECT_NEAR(joint->effort, 100.0, 0.001);
  EXPECT_NEAR(joint->velocity, 1.0, 0.001);
}

TEST_F(URDFParserTest, ParseMaterial) {
  parser_->loadFromFile(TEST_URDF_PATH);

  URDFLink* baseLink = parser_->getLink("base_link");
  ASSERT_NE(baseLink, nullptr);

  // base_link should have blue color
  EXPECT_EQ(baseLink->color.red(), 0);
  EXPECT_EQ(baseLink->color.green(), 0);
  EXPECT_NEAR(baseLink->color.blue(), 204, 2);  // 0.8 * 255 ~ 204
}

int main(int argc, char** argv) {
  // Initialize Qt (required for Qt types)
  QCoreApplication app(argc, argv);

  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
