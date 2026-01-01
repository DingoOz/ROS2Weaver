#ifndef ROS_WEAVER_CORE_GIT_INTEGRATION_HPP
#define ROS_WEAVER_CORE_GIT_INTEGRATION_HPP

#include <QObject>
#include <QString>
#include <QStringList>
#include <QDateTime>
#include <QMap>

namespace ros_weaver {

// Git file status
enum class GitFileStatus {
  Untracked,
  Modified,
  Staged,
  Unmodified,
  Deleted,
  Renamed,
  Copied,
  Ignored
};

// Information about a file in git
struct GitFileInfo {
  QString path;
  GitFileStatus status;
  bool isStaged = false;
};

// Information about a commit
struct GitCommitInfo {
  QString hash;
  QString shortHash;
  QString author;
  QString email;
  QDateTime date;
  QString message;
  QString summary;
  int filesChanged = 0;
  int insertions = 0;
  int deletions = 0;
};

// Information about a branch
struct GitBranchInfo {
  QString name;
  bool isCurrent = false;
  bool isRemote = false;
  QString upstream;
  int aheadCount = 0;
  int behindCount = 0;
};

// Overall repository status
struct GitRepoStatus {
  bool isRepo = false;
  QString repoPath;
  QString branch;
  QString upstream;
  bool hasUncommittedChanges = false;
  bool hasUntrackedFiles = false;
  bool hasStagedChanges = false;
  int aheadCount = 0;
  int behindCount = 0;
  QList<GitFileInfo> files;
};

// Git integration manager
class GitIntegration : public QObject {
  Q_OBJECT

public:
  static GitIntegration& instance();

  // Prevent copying
  GitIntegration(const GitIntegration&) = delete;
  GitIntegration& operator=(const GitIntegration&) = delete;

  // Check if git is available
  bool isGitAvailable() const;

  // Check if a directory is a git repository
  bool isGitRepository(const QString& path) const;

  // Get repository status
  GitRepoStatus getStatus(const QString& path);

  // Get file status
  GitFileStatus getFileStatus(const QString& filePath);

  // Stage a file
  bool stageFile(const QString& filePath);

  // Unstage a file
  bool unstageFile(const QString& filePath);

  // Commit staged changes
  bool commit(const QString& message);

  // Get commit history
  QList<GitCommitInfo> getHistory(int limit = 50);

  // Get branches
  QList<GitBranchInfo> getBranches();

  // Checkout a branch
  bool checkout(const QString& branch);

  // Create a new branch
  bool createBranch(const QString& name, bool checkout = true);

  // Get diff for a file
  QString getFileDiff(const QString& filePath);

  // Get diff between commits
  QString getDiff(const QString& fromCommit, const QString& toCommit = "");

  // Pull from remote
  bool pull();

  // Push to remote
  bool push();

  // Set working directory
  void setWorkingDirectory(const QString& path);
  QString workingDirectory() const { return workingDir_; }

signals:
  void statusChanged(const GitRepoStatus& status);
  void commitCreated(const QString& hash);
  void operationFailed(const QString& operation, const QString& error);
  void operationStarted(const QString& operation);
  void operationCompleted(const QString& operation);

private:
  GitIntegration();
  ~GitIntegration() override = default;

  // Run a git command and return output
  QString runGitCommand(const QStringList& args, bool* success = nullptr);

  // Parse status output
  QList<GitFileInfo> parseStatusOutput(const QString& output);

  // Parse log output
  QList<GitCommitInfo> parseLogOutput(const QString& output);

  // Parse branch output
  QList<GitBranchInfo> parseBranchOutput(const QString& output);

  QString workingDir_;
  bool gitAvailable_;
};

}  // namespace ros_weaver

#endif  // ROS_WEAVER_CORE_GIT_INTEGRATION_HPP
