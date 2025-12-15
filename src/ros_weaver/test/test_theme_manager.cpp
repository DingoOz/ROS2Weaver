#include <gtest/gtest.h>
#include "ros_weaver/core/theme_manager.hpp"
#include <QApplication>

using namespace ros_weaver;

// Note: ThemeManager is a singleton, so we test its methods
// rather than creating separate instances

class ThemeManagerTest : public ::testing::Test {
protected:
  void SetUp() override {
    // Store the initial theme to restore after tests
    initialTheme_ = ThemeManager::instance().currentTheme();
  }

  void TearDown() override {
    // Restore initial theme
    ThemeManager::instance().setTheme(initialTheme_);
  }

  Theme initialTheme_;
};

// ============================================================================
// Theme Name Tests
// ============================================================================

TEST(ThemeNameTest, DarkThemeName) {
  EXPECT_EQ(ThemeManager::themeName(Theme::Dark), "Dark");
}

TEST(ThemeNameTest, LightThemeName) {
  EXPECT_EQ(ThemeManager::themeName(Theme::Light), "Light");
}

// ============================================================================
// Palette Tests
// ============================================================================

TEST_F(ThemeManagerTest, DarkPalette_WindowColor) {
  QPalette palette = ThemeManager::instance().darkPalette();
  QColor windowColor = palette.color(QPalette::Window);

  // Dark theme should have dark window color
  EXPECT_LT(windowColor.lightness(), 100);
  EXPECT_EQ(windowColor, QColor(53, 53, 53));
}

TEST_F(ThemeManagerTest, DarkPalette_TextColor) {
  QPalette palette = ThemeManager::instance().darkPalette();
  QColor textColor = palette.color(QPalette::Text);

  // Dark theme should have light text
  EXPECT_EQ(textColor, Qt::white);
}

TEST_F(ThemeManagerTest, DarkPalette_HighlightColor) {
  QPalette palette = ThemeManager::instance().darkPalette();
  QColor highlightColor = palette.color(QPalette::Highlight);

  // Should be a blue accent color
  EXPECT_EQ(highlightColor, QColor(42, 130, 218));
}

TEST_F(ThemeManagerTest, LightPalette_WindowColor) {
  QPalette palette = ThemeManager::instance().lightPalette();
  QColor windowColor = palette.color(QPalette::Window);

  // Light theme should have light window color
  EXPECT_GT(windowColor.lightness(), 200);
  EXPECT_EQ(windowColor, QColor(240, 240, 240));
}

TEST_F(ThemeManagerTest, LightPalette_TextColor) {
  QPalette palette = ThemeManager::instance().lightPalette();
  QColor textColor = palette.color(QPalette::Text);

  // Light theme should have dark text
  EXPECT_EQ(textColor, QColor(30, 30, 30));
}

TEST_F(ThemeManagerTest, LightPalette_BaseColor) {
  QPalette palette = ThemeManager::instance().lightPalette();
  QColor baseColor = palette.color(QPalette::Base);

  // Light theme base should be white
  EXPECT_EQ(baseColor, QColor(255, 255, 255));
}

TEST_F(ThemeManagerTest, DarkPalette_DisabledTextColor) {
  QPalette palette = ThemeManager::instance().darkPalette();
  QColor disabledText = palette.color(QPalette::Disabled, QPalette::Text);

  // Disabled text should be grayed out
  EXPECT_EQ(disabledText, QColor(127, 127, 127));
}

TEST_F(ThemeManagerTest, LightPalette_DisabledTextColor) {
  QPalette palette = ThemeManager::instance().lightPalette();
  QColor disabledText = palette.color(QPalette::Disabled, QPalette::Text);

  // Disabled text should be grayed out
  EXPECT_EQ(disabledText, QColor(160, 160, 160));
}

// ============================================================================
// Theme Switching Tests
// ============================================================================

TEST_F(ThemeManagerTest, SetTheme_Dark) {
  ThemeManager::instance().setTheme(Theme::Dark);
  EXPECT_EQ(ThemeManager::instance().currentTheme(), Theme::Dark);
}

TEST_F(ThemeManagerTest, SetTheme_Light) {
  ThemeManager::instance().setTheme(Theme::Light);
  EXPECT_EQ(ThemeManager::instance().currentTheme(), Theme::Light);
}

TEST_F(ThemeManagerTest, CurrentPalette_MatchesTheme_Dark) {
  ThemeManager::instance().setTheme(Theme::Dark);
  QPalette currentPalette = ThemeManager::instance().currentPalette();
  QPalette darkPalette = ThemeManager::instance().darkPalette();

  EXPECT_EQ(currentPalette.color(QPalette::Window),
            darkPalette.color(QPalette::Window));
  EXPECT_EQ(currentPalette.color(QPalette::Text),
            darkPalette.color(QPalette::Text));
}

TEST_F(ThemeManagerTest, CurrentPalette_MatchesTheme_Light) {
  ThemeManager::instance().setTheme(Theme::Light);
  QPalette currentPalette = ThemeManager::instance().currentPalette();
  QPalette lightPalette = ThemeManager::instance().lightPalette();

  EXPECT_EQ(currentPalette.color(QPalette::Window),
            lightPalette.color(QPalette::Window));
  EXPECT_EQ(currentPalette.color(QPalette::Text),
            lightPalette.color(QPalette::Text));
}

// ============================================================================
// Stylesheet Tests
// ============================================================================

TEST_F(ThemeManagerTest, DarkStylesheet_ContainsTooltipStyles) {
  ThemeManager::instance().setTheme(Theme::Dark);
  QString stylesheet = ThemeManager::instance().currentStyleSheet();

  EXPECT_TRUE(stylesheet.contains("QToolTip"));
  EXPECT_TRUE(stylesheet.contains("#ffffff"));  // White text
}

TEST_F(ThemeManagerTest, DarkStylesheet_ContainsMenuStyles) {
  ThemeManager::instance().setTheme(Theme::Dark);
  QString stylesheet = ThemeManager::instance().currentStyleSheet();

  EXPECT_TRUE(stylesheet.contains("QMenu"));
  EXPECT_TRUE(stylesheet.contains("QMenuBar"));
  EXPECT_TRUE(stylesheet.contains("#353535"));  // Dark background
}

TEST_F(ThemeManagerTest, LightStylesheet_ContainsTooltipStyles) {
  ThemeManager::instance().setTheme(Theme::Light);
  QString stylesheet = ThemeManager::instance().currentStyleSheet();

  EXPECT_TRUE(stylesheet.contains("QToolTip"));
  EXPECT_TRUE(stylesheet.contains("#1e1e1e"));  // Dark text
}

TEST_F(ThemeManagerTest, LightStylesheet_ContainsMenuStyles) {
  ThemeManager::instance().setTheme(Theme::Light);
  QString stylesheet = ThemeManager::instance().currentStyleSheet();

  EXPECT_TRUE(stylesheet.contains("QMenu"));
  EXPECT_TRUE(stylesheet.contains("#ffffff"));  // White background
}

// ============================================================================
// Color Contrast Tests (Accessibility)
// ============================================================================

TEST_F(ThemeManagerTest, DarkTheme_HasSufficientContrast) {
  QPalette palette = ThemeManager::instance().darkPalette();
  QColor windowColor = palette.color(QPalette::Window);
  QColor textColor = palette.color(QPalette::Text);

  // Calculate luminance difference (simplified)
  int windowLightness = windowColor.lightness();
  int textLightness = textColor.lightness();
  int contrast = qAbs(textLightness - windowLightness);

  // Expect at least 100 lightness difference for readability
  EXPECT_GT(contrast, 100);
}

TEST_F(ThemeManagerTest, LightTheme_HasSufficientContrast) {
  QPalette palette = ThemeManager::instance().lightPalette();
  QColor windowColor = palette.color(QPalette::Window);
  QColor textColor = palette.color(QPalette::Text);

  int windowLightness = windowColor.lightness();
  int textLightness = textColor.lightness();
  int contrast = qAbs(textLightness - windowLightness);

  EXPECT_GT(contrast, 100);
}

// ============================================================================
// Theme Enum Tests
// ============================================================================

TEST(ThemeEnumTest, DarkEnumValue) {
  EXPECT_EQ(static_cast<int>(Theme::Dark), 0);
}

TEST(ThemeEnumTest, LightEnumValue) {
  EXPECT_EQ(static_cast<int>(Theme::Light), 1);
}

int main(int argc, char** argv) {
  // QApplication is needed for QPalette
  QApplication app(argc, argv);
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
