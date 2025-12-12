#include "ros_weaver/core/external_editor.hpp"

#include <QProcess>
#include <QFileInfo>
#include <QDir>
#include <QDesktopServices>
#include <QUrl>
#include <QStandardPaths>

namespace ros_weaver {

ExternalEditor::ExternalEditor(QObject* parent)
  : QObject(parent)
{
}

bool ExternalEditor::isVSCodeAvailable() {
  QString command = getVSCodeCommand();
  if (command.isEmpty()) {
    return false;
  }

  QProcess process;
  process.start(command, QStringList() << "--version");
  process.waitForFinished(3000);  // 3 second timeout

  return process.exitCode() == 0;
}

QString ExternalEditor::getVSCodeCommand() {
#ifdef Q_OS_WIN
  // Windows: Try 'code.cmd' first, then check common install locations
  QString codePath = QStandardPaths::findExecutable("code.cmd");
  if (!codePath.isEmpty()) {
    return codePath;
  }
  codePath = QStandardPaths::findExecutable("code");
  if (!codePath.isEmpty()) {
    return codePath;
  }
  // Check common Windows install locations
  QStringList commonPaths = {
    QDir::homePath() + "/AppData/Local/Programs/Microsoft VS Code/bin/code.cmd",
    "C:/Program Files/Microsoft VS Code/bin/code.cmd",
    "C:/Program Files (x86)/Microsoft VS Code/bin/code.cmd"
  };
  for (const QString& path : commonPaths) {
    if (QFileInfo::exists(path)) {
      return path;
    }
  }
#elif defined(Q_OS_MACOS)
  // macOS: Check PATH first, then common locations
  QString codePath = QStandardPaths::findExecutable("code");
  if (!codePath.isEmpty()) {
    return codePath;
  }
  // Check the standard macOS VS Code location
  QString macPath = "/Applications/Visual Studio Code.app/Contents/Resources/app/bin/code";
  if (QFileInfo::exists(macPath)) {
    return macPath;
  }
#else
  // Linux and other Unix-like systems
  QString codePath = QStandardPaths::findExecutable("code");
  if (!codePath.isEmpty()) {
    return codePath;
  }
  // Check snap and flatpak locations
  QStringList linuxPaths = {
    "/snap/bin/code",
    "/var/lib/flatpak/exports/bin/com.visualstudio.code"
  };
  for (const QString& path : linuxPaths) {
    if (QFileInfo::exists(path)) {
      return path;
    }
  }
#endif

  return QString();  // Not found
}

bool ExternalEditor::openFolderInVSCode(const QString& folderPath, bool reuseWindow) {
  QFileInfo info(folderPath);
  if (!info.exists() || !info.isDir()) {
    lastError_ = QString("Folder does not exist: %1").arg(folderPath);
    emit editorLaunchFailed(folderPath, lastError_);
    return false;
  }

  QStringList args;
  if (reuseWindow || reuseWindow_) {
    args << "-r";
  }
  args << folderPath;

  return launchVSCode(args);
}

bool ExternalEditor::openFileInVSCode(const QString& filePath, bool reuseWindow) {
  QFileInfo info(filePath);
  if (!info.exists() || !info.isFile()) {
    lastError_ = QString("File does not exist: %1").arg(filePath);
    emit editorLaunchFailed(filePath, lastError_);
    return false;
  }

  QStringList args;
  if (reuseWindow || reuseWindow_) {
    args << "-r";
  }
  args << filePath;

  return launchVSCode(args);
}

bool ExternalEditor::openFileInVSCodeAtLine(const QString& filePath, int lineNumber, bool reuseWindow) {
  QFileInfo info(filePath);
  if (!info.exists() || !info.isFile()) {
    lastError_ = QString("File does not exist: %1").arg(filePath);
    emit editorLaunchFailed(filePath, lastError_);
    return false;
  }

  QStringList args;
  if (reuseWindow || reuseWindow_) {
    args << "-r";
  }
  // Use -g flag for goto line
  args << "-g" << QString("%1:%2").arg(filePath).arg(lineNumber);

  return launchVSCode(args);
}

bool ExternalEditor::openWithSystemDefault(const QString& path) {
  QUrl url = QUrl::fromLocalFile(path);
  return QDesktopServices::openUrl(url);
}

bool ExternalEditor::openInDefaultEditor(const QString& filePath) {
  QFileInfo info(filePath);
  if (!info.exists()) {
    return false;
  }

  // Try to open with the system's default text editor
  return QDesktopServices::openUrl(QUrl::fromLocalFile(filePath));
}

bool ExternalEditor::launchVSCode(const QStringList& arguments) {
  QString command = editorCommand_.isEmpty() ? getVSCodeCommand() : editorCommand_;

  if (command.isEmpty()) {
    lastError_ = "VS Code not found. Please install VS Code or configure a custom editor command.";
    if (!arguments.isEmpty()) {
      emit editorLaunchFailed(arguments.last(), lastError_);
    }
    return false;
  }

  QProcess* process = new QProcess(this);
  process->setProgram(command);
  process->setArguments(arguments);

  // Start detached so VS Code runs independently
  qint64 pid;
  bool success = process->startDetached(&pid);

  if (!success) {
    lastError_ = QString("Failed to launch VS Code: %1").arg(process->errorString());
    if (!arguments.isEmpty()) {
      emit editorLaunchFailed(arguments.last(), lastError_);
    }
    delete process;
    return false;
  }

  // Clean up the process object (VS Code is running detached)
  delete process;

  if (!arguments.isEmpty()) {
    emit editorLaunched(arguments.last());
  }

  return true;
}

}  // namespace ros_weaver
