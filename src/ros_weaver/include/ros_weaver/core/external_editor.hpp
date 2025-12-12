#ifndef ROS_WEAVER_CORE_EXTERNAL_EDITOR_HPP
#define ROS_WEAVER_CORE_EXTERNAL_EDITOR_HPP

#include <QObject>
#include <QString>
#include <QStringList>

namespace ros_weaver {

/**
 * @brief Utility class for opening files and folders in external editors (VS Code, etc.)
 *
 * Provides cross-platform support for launching external editors with various options
 * like opening at specific lines, reusing windows, etc.
 */
class ExternalEditor : public QObject {
  Q_OBJECT

public:
  explicit ExternalEditor(QObject* parent = nullptr);
  ~ExternalEditor() override = default;

  /**
   * @brief Check if VS Code is available on the system
   * @return true if the 'code' command is found in PATH
   */
  static bool isVSCodeAvailable();

  /**
   * @brief Get the VS Code command for the current platform
   * @return The command to invoke VS Code (e.g., "code" on Linux)
   */
  static QString getVSCodeCommand();

  /**
   * @brief Open a folder in VS Code
   * @param folderPath Path to the folder to open
   * @param reuseWindow If true, reuse existing VS Code window
   * @return true if successfully launched
   */
  bool openFolderInVSCode(const QString& folderPath, bool reuseWindow = false);

  /**
   * @brief Open a file in VS Code
   * @param filePath Path to the file to open
   * @param reuseWindow If true, reuse existing VS Code window
   * @return true if successfully launched
   */
  bool openFileInVSCode(const QString& filePath, bool reuseWindow = false);

  /**
   * @brief Open a file in VS Code at a specific line
   * @param filePath Path to the file to open
   * @param lineNumber Line number to go to (1-based)
   * @param reuseWindow If true, reuse existing VS Code window
   * @return true if successfully launched
   */
  bool openFileInVSCodeAtLine(const QString& filePath, int lineNumber, bool reuseWindow = false);

  /**
   * @brief Open a file or folder using the system default application
   * @param path Path to open
   * @return true if successfully launched
   */
  static bool openWithSystemDefault(const QString& path);

  /**
   * @brief Open a file in the system's default text editor
   * @param filePath Path to the file to open
   * @return true if successfully launched
   */
  static bool openInDefaultEditor(const QString& filePath);

  /**
   * @brief Get the last error message
   * @return Error message from the last failed operation
   */
  QString lastError() const { return lastError_; }

  // Settings
  void setEditorCommand(const QString& command) { editorCommand_ = command; }
  QString editorCommand() const { return editorCommand_; }

  void setAutoOpenAfterGeneration(bool value) { autoOpenAfterGeneration_ = value; }
  bool autoOpenAfterGeneration() const { return autoOpenAfterGeneration_; }

  void setReuseWindow(bool value) { reuseWindow_ = value; }
  bool reuseWindow() const { return reuseWindow_; }

signals:
  void editorLaunched(const QString& path);
  void editorLaunchFailed(const QString& path, const QString& error);

private:
  bool launchVSCode(const QStringList& arguments);

  QString lastError_;
  QString editorCommand_;  // Custom editor command, empty means use default
  bool autoOpenAfterGeneration_ = false;
  bool reuseWindow_ = true;
};

}  // namespace ros_weaver

#endif  // ROS_WEAVER_CORE_EXTERNAL_EDITOR_HPP
