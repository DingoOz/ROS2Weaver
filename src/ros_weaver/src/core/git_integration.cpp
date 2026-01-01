#include "ros_weaver/core/git_integration.hpp"

#include <QProcess>
#include <QDir>
#include <QFileInfo>
#include <QRegularExpression>

namespace ros_weaver {

GitIntegration& GitIntegration::instance() {
  static GitIntegration instance;
  return instance;
}

GitIntegration::GitIntegration()
    : QObject(nullptr)
    , gitAvailable_(false)
{
  // Check if git is available
  QProcess process;
  process.start("git", {"--version"});
  if (process.waitForFinished(3000)) {
    gitAvailable_ = (process.exitCode() == 0);
  }
}

bool GitIntegration::isGitAvailable() const {
  return gitAvailable_;
}

void GitIntegration::setWorkingDirectory(const QString& path) {
  workingDir_ = path;
}

QString GitIntegration::runGitCommand(const QStringList& args, bool* success) {
  if (!gitAvailable_) {
    if (success) *success = false;
    return "";
  }

  QProcess process;
  if (!workingDir_.isEmpty()) {
    process.setWorkingDirectory(workingDir_);
  }

  process.start("git", args);
  if (!process.waitForFinished(10000)) {
    if (success) *success = false;
    return "";
  }

  if (success) *success = (process.exitCode() == 0);

  return QString::fromUtf8(process.readAllStandardOutput());
}

bool GitIntegration::isGitRepository(const QString& path) const {
  if (!gitAvailable_) return false;

  QProcess process;
  process.setWorkingDirectory(path);
  process.start("git", {"rev-parse", "--is-inside-work-tree"});
  if (!process.waitForFinished(3000)) {
    return false;
  }

  return process.exitCode() == 0;
}

GitRepoStatus GitIntegration::getStatus(const QString& path) {
  GitRepoStatus status;

  if (!isGitRepository(path)) {
    return status;
  }

  QString oldWorkingDir = workingDir_;
  workingDir_ = path;

  status.isRepo = true;
  status.repoPath = path;

  // Get current branch
  bool success;
  QString branch = runGitCommand({"rev-parse", "--abbrev-ref", "HEAD"}, &success);
  if (success) {
    status.branch = branch.trimmed();
  }

  // Get upstream tracking
  QString upstream = runGitCommand({"rev-parse", "--abbrev-ref", "@{upstream}"}, &success);
  if (success) {
    status.upstream = upstream.trimmed();

    // Get ahead/behind counts
    QString counts = runGitCommand({"rev-list", "--left-right", "--count",
                                    status.branch + "..." + status.upstream}, &success);
    if (success) {
      QStringList parts = counts.trimmed().split('\t');
      if (parts.size() == 2) {
        status.aheadCount = parts[0].toInt();
        status.behindCount = parts[1].toInt();
      }
    }
  }

  // Get file status
  QString statusOutput = runGitCommand({"status", "--porcelain", "-uall"}, &success);
  if (success) {
    status.files = parseStatusOutput(statusOutput);

    for (const GitFileInfo& file : status.files) {
      if (file.status == GitFileStatus::Untracked) {
        status.hasUntrackedFiles = true;
      } else if (file.isStaged) {
        status.hasStagedChanges = true;
      } else if (file.status != GitFileStatus::Unmodified) {
        status.hasUncommittedChanges = true;
      }
    }
  }

  workingDir_ = oldWorkingDir;
  emit statusChanged(status);

  return status;
}

QList<GitFileInfo> GitIntegration::parseStatusOutput(const QString& output) {
  QList<GitFileInfo> files;

  QStringList lines = output.split('\n', Qt::SkipEmptyParts);
  for (const QString& line : lines) {
    if (line.length() < 4) continue;

    GitFileInfo info;
    info.path = line.mid(3);

    QChar indexStatus = line[0];
    QChar workTreeStatus = line[1];

    // Parse index (staging) status
    info.isStaged = (indexStatus != ' ' && indexStatus != '?');

    // Parse work tree status
    if (workTreeStatus == '?' || indexStatus == '?') {
      info.status = GitFileStatus::Untracked;
    } else if (workTreeStatus == 'M' || indexStatus == 'M') {
      info.status = GitFileStatus::Modified;
    } else if (workTreeStatus == 'D' || indexStatus == 'D') {
      info.status = GitFileStatus::Deleted;
    } else if (workTreeStatus == 'R' || indexStatus == 'R') {
      info.status = GitFileStatus::Renamed;
    } else if (workTreeStatus == 'C' || indexStatus == 'C') {
      info.status = GitFileStatus::Copied;
    } else if (workTreeStatus == 'A' || indexStatus == 'A') {
      info.status = GitFileStatus::Staged;
    } else if (workTreeStatus == '!' || indexStatus == '!') {
      info.status = GitFileStatus::Ignored;
    } else {
      info.status = GitFileStatus::Unmodified;
    }

    files.append(info);
  }

  return files;
}

GitFileStatus GitIntegration::getFileStatus(const QString& filePath) {
  bool success;
  QString output = runGitCommand({"status", "--porcelain", filePath}, &success);

  if (!success || output.isEmpty()) {
    return GitFileStatus::Unmodified;
  }

  QList<GitFileInfo> files = parseStatusOutput(output);
  if (!files.isEmpty()) {
    return files.first().status;
  }

  return GitFileStatus::Unmodified;
}

bool GitIntegration::stageFile(const QString& filePath) {
  bool success;
  runGitCommand({"add", filePath}, &success);
  return success;
}

bool GitIntegration::unstageFile(const QString& filePath) {
  bool success;
  runGitCommand({"reset", "HEAD", filePath}, &success);
  return success;
}

bool GitIntegration::commit(const QString& message) {
  emit operationStarted("commit");

  bool success;
  QString output = runGitCommand({"commit", "-m", message}, &success);

  if (success) {
    // Get the commit hash
    QString hash = runGitCommand({"rev-parse", "HEAD"}, &success);
    if (success) {
      emit commitCreated(hash.trimmed());
    }
    emit operationCompleted("commit");
  } else {
    emit operationFailed("commit", output);
  }

  return success;
}

QList<GitCommitInfo> GitIntegration::getHistory(int limit) {
  bool success;
  QString format = "%H%n%h%n%an%n%ae%n%aI%n%s%n%b%n--END--";
  QString output = runGitCommand({"log", QString("--max-count=%1").arg(limit),
                                  QString("--format=%1").arg(format)}, &success);

  if (!success) {
    return {};
  }

  return parseLogOutput(output);
}

QList<GitCommitInfo> GitIntegration::parseLogOutput(const QString& output) {
  QList<GitCommitInfo> commits;

  QStringList entries = output.split("--END--", Qt::SkipEmptyParts);
  for (const QString& entry : entries) {
    QStringList lines = entry.trimmed().split('\n');
    if (lines.size() < 6) continue;

    GitCommitInfo info;
    info.hash = lines[0];
    info.shortHash = lines[1];
    info.author = lines[2];
    info.email = lines[3];
    info.date = QDateTime::fromString(lines[4], Qt::ISODate);
    info.summary = lines[5];

    // Rest is the body
    if (lines.size() > 6) {
      QStringList bodyLines;
      for (int i = 6; i < lines.size(); ++i) {
        bodyLines.append(lines[i]);
      }
      info.message = info.summary + "\n" + bodyLines.join("\n");
    } else {
      info.message = info.summary;
    }

    commits.append(info);
  }

  return commits;
}

QList<GitBranchInfo> GitIntegration::getBranches() {
  bool success;
  QString output = runGitCommand({"branch", "-a", "-v"}, &success);

  if (!success) {
    return {};
  }

  return parseBranchOutput(output);
}

QList<GitBranchInfo> GitIntegration::parseBranchOutput(const QString& output) {
  QList<GitBranchInfo> branches;

  QStringList lines = output.split('\n', Qt::SkipEmptyParts);
  for (const QString& line : lines) {
    if (line.length() < 3) continue;

    GitBranchInfo info;
    info.isCurrent = (line[0] == '*');

    QString branchLine = line.mid(2).trimmed();
    QStringList parts = branchLine.split(QRegularExpression("\\s+"));

    if (parts.isEmpty()) continue;

    info.name = parts[0];
    info.isRemote = info.name.startsWith("remotes/");

    if (info.isRemote) {
      info.name = info.name.mid(8);  // Remove "remotes/"
    }

    branches.append(info);
  }

  return branches;
}

bool GitIntegration::checkout(const QString& branch) {
  emit operationStarted("checkout");

  bool success;
  QString output = runGitCommand({"checkout", branch}, &success);

  if (success) {
    emit operationCompleted("checkout");
  } else {
    emit operationFailed("checkout", output);
  }

  return success;
}

bool GitIntegration::createBranch(const QString& name, bool doCheckout) {
  emit operationStarted("create branch");

  bool success;
  QStringList args = {"branch", name};
  QString output = runGitCommand(args, &success);

  if (success && doCheckout) {
    success = checkout(name);
  }

  if (success) {
    emit operationCompleted("create branch");
  } else {
    emit operationFailed("create branch", output);
  }

  return success;
}

QString GitIntegration::getFileDiff(const QString& filePath) {
  bool success;
  return runGitCommand({"diff", filePath}, &success);
}

QString GitIntegration::getDiff(const QString& fromCommit, const QString& toCommit) {
  bool success;
  QStringList args = {"diff", fromCommit};
  if (!toCommit.isEmpty()) {
    args.append(toCommit);
  }
  return runGitCommand(args, &success);
}

bool GitIntegration::pull() {
  emit operationStarted("pull");

  bool success;
  QString output = runGitCommand({"pull"}, &success);

  if (success) {
    emit operationCompleted("pull");
  } else {
    emit operationFailed("pull", output);
  }

  return success;
}

bool GitIntegration::push() {
  emit operationStarted("push");

  bool success;
  QString output = runGitCommand({"push"}, &success);

  if (success) {
    emit operationCompleted("push");
  } else {
    emit operationFailed("push", output);
  }

  return success;
}

}  // namespace ros_weaver
