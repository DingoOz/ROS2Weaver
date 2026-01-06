#include "ros_weaver/widgets/workspace_browser_panel.hpp"

#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QHeaderView>
#include <QFileDialog>
#include <QMenu>
#include <QAction>
#include <QProcess>
#include <QDesktopServices>
#include <QUrl>
#include <QXmlStreamReader>
#include <QDirIterator>
#include <QMessageBox>
#include <QApplication>
#include <QStandardPaths>
#include <QTimer>
#include <QClipboard>

namespace ros_weaver {

WorkspaceBrowserPanel::WorkspaceBrowserPanel(QWidget* parent)
  : QWidget(parent)
  , fileWatcher_(new QFileSystemWatcher(this))
{
  setupUi();

  // Connect file watcher
  connect(fileWatcher_, &QFileSystemWatcher::directoryChanged,
          this, &WorkspaceBrowserPanel::onFileChanged);
  connect(fileWatcher_, &QFileSystemWatcher::fileChanged,
          this, &WorkspaceBrowserPanel::onFileChanged);

  // Auto-detect current workspace from environment
  QString ros_ws = qgetenv("COLCON_PREFIX_PATH");
  if (!ros_ws.isEmpty()) {
    // Extract workspace root from prefix path
    QDir prefixDir(ros_ws.split(':').first());
    if (prefixDir.cdUp()) {
      setWorkspacePath(prefixDir.absolutePath());
    }
  } else {
    // Try common workspace locations
    QDir homeDir(QDir::homePath());
    QStringList commonWorkspaces = {
      "ros2_ws", "colcon_ws", "dev_ws", "catkin_ws/src"
    };
    for (const auto& ws : commonWorkspaces) {
      QDir wsDir(homeDir.filePath(ws));
      if (wsDir.exists()) {
        setWorkspacePath(wsDir.absolutePath());
        break;
      }
    }
  }
}

void WorkspaceBrowserPanel::setupUi() {
  QVBoxLayout* mainLayout = new QVBoxLayout(this);
  mainLayout->setContentsMargins(4, 4, 4, 4);
  mainLayout->setSpacing(4);

  // Workspace selector row
  QHBoxLayout* workspaceLayout = new QHBoxLayout();
  workspaceLayout->setSpacing(4);

  workspaceCombo_ = new QComboBox(this);
  workspaceCombo_->setMinimumWidth(100);
  workspaceCombo_->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
  workspaceCombo_->setToolTip(tr("Select workspace to browse"));
  connect(workspaceCombo_, QOverload<int>::of(&QComboBox::currentIndexChanged),
          this, &WorkspaceBrowserPanel::onWorkspaceChanged);
  workspaceLayout->addWidget(workspaceCombo_);

  addWorkspaceButton_ = new QPushButton("+", this);
  addWorkspaceButton_->setFixedWidth(30);
  addWorkspaceButton_->setToolTip(tr("Add workspace"));
  connect(addWorkspaceButton_, &QPushButton::clicked,
          this, &WorkspaceBrowserPanel::onAddWorkspaceClicked);
  workspaceLayout->addWidget(addWorkspaceButton_);

  removeWorkspaceButton_ = new QPushButton("-", this);
  removeWorkspaceButton_->setFixedWidth(30);
  removeWorkspaceButton_->setToolTip(tr("Remove workspace"));
  connect(removeWorkspaceButton_, &QPushButton::clicked,
          this, &WorkspaceBrowserPanel::onRemoveWorkspaceClicked);
  workspaceLayout->addWidget(removeWorkspaceButton_);

  refreshButton_ = new QPushButton(tr("Refresh"), this);
  refreshButton_->setToolTip(tr("Refresh workspace contents"));
  connect(refreshButton_, &QPushButton::clicked,
          this, &WorkspaceBrowserPanel::onRefreshClicked);
  workspaceLayout->addWidget(refreshButton_);

  mainLayout->addLayout(workspaceLayout);

  // Search bar
  searchEdit_ = new QLineEdit(this);
  searchEdit_->setPlaceholderText(tr("Search packages and files..."));
  searchEdit_->setClearButtonEnabled(true);
  connect(searchEdit_, &QLineEdit::textChanged,
          this, &WorkspaceBrowserPanel::onSearchTextChanged);
  mainLayout->addWidget(searchEdit_);

  // Tree view
  treeWidget_ = new QTreeWidget(this);
  treeWidget_->setHeaderLabels(QStringList{tr("Name"), tr("Type"), tr("Path")});
  treeWidget_->setColumnWidth(0, 200);
  treeWidget_->setColumnWidth(1, 80);
  treeWidget_->setRootIsDecorated(true);
  treeWidget_->setSelectionMode(QAbstractItemView::SingleSelection);
  treeWidget_->setContextMenuPolicy(Qt::CustomContextMenu);
  treeWidget_->setAnimated(true);
  treeWidget_->setStyleSheet(R"(
    QTreeWidget {
      border: 1px solid #ccc;
      border-radius: 4px;
    }
    QTreeWidget::item {
      padding: 2px;
    }
    QTreeWidget::item:hover {
      background-color: #e8f0fe;
    }
    QTreeWidget::item:selected {
      background-color: #1a73e8;
      color: white;
    }
  )");

  connect(treeWidget_, &QTreeWidget::itemClicked,
          this, &WorkspaceBrowserPanel::onItemClicked);
  connect(treeWidget_, &QTreeWidget::itemDoubleClicked,
          this, &WorkspaceBrowserPanel::onItemDoubleClicked);
  connect(treeWidget_, &QTreeWidget::customContextMenuRequested,
          this, &WorkspaceBrowserPanel::onContextMenu);

  mainLayout->addWidget(treeWidget_, 1);

  // Status label
  statusLabel_ = new QLabel(tr("No workspace selected"), this);
  statusLabel_->setStyleSheet("color: #666; font-style: italic;");
  mainLayout->addWidget(statusLabel_);

  setMinimumWidth(280);
}

void WorkspaceBrowserPanel::setWorkspacePath(const QString& path) {
  if (path.isEmpty() || !QDir(path).exists()) {
    return;
  }

  // Add to list if not already present
  if (!workspacePaths_.contains(path)) {
    workspacePaths_.append(path);
    workspaceCombo_->addItem(QDir(path).dirName(), path);
  }

  // Select this workspace
  int index = workspacePaths_.indexOf(path);
  if (index >= 0) {
    workspaceCombo_->setCurrentIndex(index);
  }

  currentWorkspace_ = path;
  scanWorkspace(path);
  populateTree();
}

void WorkspaceBrowserPanel::refresh() {
  if (!currentWorkspace_.isEmpty()) {
    scanWorkspace(currentWorkspace_);
    populateTree();
  }
}

void WorkspaceBrowserPanel::setFilterText(const QString& text) {
  searchEdit_->setText(text);
}

void WorkspaceBrowserPanel::addWorkspace(const QString& path) {
  if (!path.isEmpty() && QDir(path).exists() && !workspacePaths_.contains(path)) {
    setWorkspacePath(path);
  }
}

void WorkspaceBrowserPanel::removeWorkspace(const QString& path) {
  int index = workspacePaths_.indexOf(path);
  if (index >= 0) {
    workspacePaths_.removeAt(index);
    workspacePackages_.remove(path);
    workspaceCombo_->removeItem(index);

    if (currentWorkspace_ == path) {
      if (!workspacePaths_.isEmpty()) {
        setWorkspacePath(workspacePaths_.first());
      } else {
        currentWorkspace_.clear();
        treeWidget_->clear();
        statusLabel_->setText(tr("No workspace selected"));
      }
    }
  }
}

void WorkspaceBrowserPanel::onSearchTextChanged(const QString& text) {
  populateWithFilter(text);
}

void WorkspaceBrowserPanel::onWorkspaceChanged(int index) {
  if (index >= 0 && index < workspacePaths_.size()) {
    currentWorkspace_ = workspacePaths_.at(index);
    if (!workspacePackages_.contains(currentWorkspace_)) {
      scanWorkspace(currentWorkspace_);
    }
    populateTree();
  }
}

void WorkspaceBrowserPanel::onItemClicked(QTreeWidgetItem* item, int column) {
  Q_UNUSED(column)

  QString filePath = item->data(0, FilePathRole).toString();
  QString fileType = item->data(0, FileTypeRole).toString();

  if (fileType == "package") {
    QString pkgName = item->data(0, PackageNameRole).toString();
    for (const auto& pkg : workspacePackages_[currentWorkspace_]) {
      if (pkg.name == pkgName) {
        emit packageSelected(pkg);
        break;
      }
    }
  }
}

void WorkspaceBrowserPanel::onItemDoubleClicked(QTreeWidgetItem* item, int column) {
  Q_UNUSED(column)

  QString filePath = item->data(0, FilePathRole).toString();
  QString fileType = item->data(0, FileTypeRole).toString();

  if (!filePath.isEmpty() && QFile::exists(filePath)) {
    if (fileType == "launch") {
      emit launchFileSelected(filePath);
    } else if (fileType == "config") {
      emit configFileSelected(filePath);
    } else {
      emit openInEditorRequested(filePath);
    }
  }
}

void WorkspaceBrowserPanel::onRefreshClicked() {
  refresh();
}

void WorkspaceBrowserPanel::onAddWorkspaceClicked() {
  QString path = QFileDialog::getExistingDirectory(
    this,
    tr("Select ROS2 Workspace"),
    QDir::homePath(),
    QFileDialog::ShowDirsOnly
  );

  if (!path.isEmpty()) {
    // Check if it looks like a workspace (has src folder or package.xml files)
    QDir dir(path);
    bool isWorkspace = dir.exists("src") ||
                       !QDirIterator(path, QStringList{"package.xml"},
                         QDir::Files, QDirIterator::Subdirectories).hasNext();

    if (isWorkspace || QMessageBox::question(this, tr("Add Workspace"),
        tr("This directory doesn't appear to be a ROS2 workspace. Add it anyway?"))
        == QMessageBox::Yes) {
      addWorkspace(path);
    }
  }
}

void WorkspaceBrowserPanel::onRemoveWorkspaceClicked() {
  if (!currentWorkspace_.isEmpty()) {
    removeWorkspace(currentWorkspace_);
  }
}

void WorkspaceBrowserPanel::onFileChanged(const QString& path) {
  Q_UNUSED(path)
  // Debounce and refresh
  QTimer::singleShot(500, this, &WorkspaceBrowserPanel::refresh);
}

void WorkspaceBrowserPanel::onContextMenu(const QPoint& pos) {
  QTreeWidgetItem* item = treeWidget_->itemAt(pos);
  if (!item) return;

  QString filePath = item->data(0, FilePathRole).toString();
  QString fileType = item->data(0, FileTypeRole).toString();

  QMenu menu;

  if (!filePath.isEmpty() && QFile::exists(filePath)) {
    QAction* openAction = menu.addAction(tr("Open in Editor"));
    connect(openAction, &QAction::triggered, this, [this, filePath]() {
      emit openInEditorRequested(filePath);
    });

    QAction* revealAction = menu.addAction(tr("Reveal in File Manager"));
    connect(revealAction, &QAction::triggered, this, [filePath]() {
      QFileInfo fi(filePath);
      QDesktopServices::openUrl(QUrl::fromLocalFile(fi.absolutePath()));
    });

    menu.addSeparator();

    QAction* copyPathAction = menu.addAction(tr("Copy Path"));
    connect(copyPathAction, &QAction::triggered, this, [filePath]() {
      QApplication::clipboard()->setText(filePath);
    });

    if (fileType == "launch") {
      menu.addSeparator();
      QAction* runLaunchAction = menu.addAction(tr("Run Launch File"));
      connect(runLaunchAction, &QAction::triggered, this, [filePath]() {
        // Launch the file using ros2 launch
        QProcess::startDetached("ros2", QStringList{"launch", filePath});
      });
    }
  }

  if (fileType == "package") {
    QAction* buildAction = menu.addAction(tr("Build Package"));
    QString pkgName = item->data(0, PackageNameRole).toString();
    connect(buildAction, &QAction::triggered, this, [pkgName]() {
      QProcess::startDetached("gnome-terminal",
        QStringList{"--", "bash", "-c",
          QString("colcon build --packages-select %1; read -p 'Press Enter to close...'").arg(pkgName)});
    });
  }

  if (!menu.isEmpty()) {
    menu.exec(treeWidget_->mapToGlobal(pos));
  }
}

void WorkspaceBrowserPanel::scanWorkspace(const QString& path) {
  QList<WorkspacePackageInfo> packages;
  QDir wsDir(path);

  // Look in src folder first
  QDir srcDir(wsDir.filePath("src"));
  if (!srcDir.exists()) {
    srcDir = wsDir;  // Use workspace root if no src folder
  }

  // Find all package.xml files
  QDirIterator it(srcDir.absolutePath(), QStringList{"package.xml"},
                  QDir::Files, QDirIterator::Subdirectories);

  while (it.hasNext()) {
    QString packageXmlPath = it.next();
    QFileInfo fi(packageXmlPath);
    QString packagePath = fi.absolutePath();

    WorkspacePackageInfo pkg = parsePackageXml(packagePath);
    if (!pkg.name.isEmpty()) {
      pkg.path = packagePath;
      pkg.launchFiles = findLaunchFiles(packagePath);
      pkg.configFiles = findConfigFiles(packagePath);
      pkg.sourceFiles = findSourceFiles(packagePath);
      packages.append(pkg);
    }
  }

  // Sort by name
  std::sort(packages.begin(), packages.end(),
            [](const WorkspacePackageInfo& a, const WorkspacePackageInfo& b) {
    return a.name.toLower() < b.name.toLower();
  });

  workspacePackages_[path] = packages;

  // Update file watcher
  fileWatcher_->addPath(srcDir.absolutePath());

  statusLabel_->setText(tr("%1 packages found").arg(packages.size()));
}

void WorkspaceBrowserPanel::populateTree() {
  treeWidget_->clear();

  if (currentWorkspace_.isEmpty() || !workspacePackages_.contains(currentWorkspace_)) {
    return;
  }

  for (const auto& pkg : workspacePackages_[currentWorkspace_]) {
    addPackageToTree(pkg);
  }

  treeWidget_->expandAll();
}

void WorkspaceBrowserPanel::populateWithFilter(const QString& filter) {
  treeWidget_->clear();

  if (currentWorkspace_.isEmpty() || !workspacePackages_.contains(currentWorkspace_)) {
    return;
  }

  QString lowerFilter = filter.toLower();

  for (const auto& pkg : workspacePackages_[currentWorkspace_]) {
    bool matchPackage = filter.isEmpty() || pkg.name.toLower().contains(lowerFilter);

    // Also check files within package
    bool matchFiles = false;
    if (!filter.isEmpty()) {
      for (const auto& f : pkg.launchFiles) {
        if (QFileInfo(f).fileName().toLower().contains(lowerFilter)) {
          matchFiles = true;
          break;
        }
      }
      if (!matchFiles) {
        for (const auto& f : pkg.configFiles) {
          if (QFileInfo(f).fileName().toLower().contains(lowerFilter)) {
            matchFiles = true;
            break;
          }
        }
      }
    }

    if (matchPackage || matchFiles) {
      addPackageToTree(pkg);
    }
  }

  treeWidget_->expandAll();
}

void WorkspaceBrowserPanel::addPackageToTree(const WorkspacePackageInfo& pkg) {
  QTreeWidgetItem* pkgItem = new QTreeWidgetItem(treeWidget_);
  pkgItem->setText(0, QString::fromUtf8("\xF0\x9F\x93\xA6 %1").arg(pkg.name));  // Package icon
  pkgItem->setText(1, pkg.buildType.isEmpty() ? "package" : pkg.buildType);
  pkgItem->setText(2, pkg.path);
  pkgItem->setData(0, FilePathRole, pkg.path);
  pkgItem->setData(0, FileTypeRole, "package");
  pkgItem->setData(0, PackageNameRole, pkg.name);
  pkgItem->setToolTip(0, pkg.description);

  // Launch files
  if (!pkg.launchFiles.isEmpty()) {
    QTreeWidgetItem* launchFolder = new QTreeWidgetItem(pkgItem);
    launchFolder->setText(0, QString::fromUtf8("\xF0\x9F\x9A\x80 Launch (%1)").arg(pkg.launchFiles.size()));
    launchFolder->setText(1, "folder");

    for (const auto& file : pkg.launchFiles) {
      QTreeWidgetItem* item = createFileItem(file, "launch");
      launchFolder->addChild(item);
    }
  }

  // Config files
  if (!pkg.configFiles.isEmpty()) {
    QTreeWidgetItem* configFolder = new QTreeWidgetItem(pkgItem);
    configFolder->setText(0, QString::fromUtf8("\xE2\x9A\x99 Config (%1)").arg(pkg.configFiles.size()));
    configFolder->setText(1, "folder");

    for (const auto& file : pkg.configFiles) {
      QTreeWidgetItem* item = createFileItem(file, "config");
      configFolder->addChild(item);
    }
  }

  // Source files (limited to first few)
  if (!pkg.sourceFiles.isEmpty()) {
    QTreeWidgetItem* srcFolder = new QTreeWidgetItem(pkgItem);
    srcFolder->setText(0, QString::fromUtf8("\xF0\x9F\x93\x84 Source (%1)").arg(pkg.sourceFiles.size()));
    srcFolder->setText(1, "folder");

    int maxFiles = 10;
    for (int i = 0; i < qMin(pkg.sourceFiles.size(), maxFiles); ++i) {
      QTreeWidgetItem* item = createFileItem(pkg.sourceFiles[i], "source");
      srcFolder->addChild(item);
    }

    if (pkg.sourceFiles.size() > maxFiles) {
      QTreeWidgetItem* moreItem = new QTreeWidgetItem(srcFolder);
      moreItem->setText(0, tr("... and %1 more").arg(pkg.sourceFiles.size() - maxFiles));
      moreItem->setForeground(0, QColor(128, 128, 128));
    }
  }
}

WorkspacePackageInfo WorkspaceBrowserPanel::parsePackageXml(const QString& packagePath) {
  WorkspacePackageInfo info;

  QFile file(QDir(packagePath).filePath("package.xml"));
  if (!file.open(QIODevice::ReadOnly)) {
    return info;
  }

  QXmlStreamReader xml(&file);

  while (!xml.atEnd() && !xml.hasError()) {
    xml.readNext();

    if (xml.isStartElement()) {
      QString name = xml.name().toString();

      if (name == "name") {
        info.name = xml.readElementText();
      } else if (name == "version") {
        info.version = xml.readElementText();
      } else if (name == "description") {
        info.description = xml.readElementText().simplified();
      } else if (name == "maintainer") {
        info.maintainer = xml.readElementText();
      } else if (name == "license") {
        info.license = xml.readElementText();
      } else if (name == "build_depend" || name == "buildtool_depend") {
        info.buildDepends.append(xml.readElementText());
      } else if (name == "exec_depend") {
        info.execDepends.append(xml.readElementText());
      } else if (name == "build_type") {
        info.buildType = xml.readElementText();
      }
    }
  }

  file.close();

  // Detect build type from CMakeLists.txt if not in package.xml
  if (info.buildType.isEmpty()) {
    QFile cmakeFile(QDir(packagePath).filePath("CMakeLists.txt"));
    if (cmakeFile.open(QIODevice::ReadOnly)) {
      QString content = QString::fromUtf8(cmakeFile.readAll());
      if (content.contains("ament_cmake")) {
        info.buildType = "ament_cmake";
      } else if (content.contains("catkin")) {
        info.buildType = "catkin";
      }
      cmakeFile.close();
    } else if (QFile::exists(QDir(packagePath).filePath("setup.py"))) {
      info.buildType = "ament_python";
    }
  }

  return info;
}

QStringList WorkspaceBrowserPanel::findLaunchFiles(const QString& packagePath) {
  QStringList files;
  QDir dir(packagePath);

  // Common launch file locations
  QStringList searchDirs = {"launch", ".", "bringup"};
  QStringList patterns = {"*.launch.py", "*.launch.xml", "*.launch", "*.launch.yaml"};

  for (const auto& subdir : searchDirs) {
    QDir searchDir(dir.filePath(subdir));
    if (searchDir.exists()) {
      for (const auto& pattern : patterns) {
        QDirIterator it(searchDir.absolutePath(), QStringList{pattern},
                        QDir::Files, QDirIterator::Subdirectories);
        while (it.hasNext()) {
          files.append(it.next());
        }
      }
    }
  }

  files.removeDuplicates();
  return files;
}

QStringList WorkspaceBrowserPanel::findConfigFiles(const QString& packagePath) {
  QStringList files;
  QDir dir(packagePath);

  // Common config file locations
  QStringList searchDirs = {"config", "params", "cfg", "."};
  QStringList patterns = {"*.yaml", "*.yml", "*.json", "*.xml"};

  for (const auto& subdir : searchDirs) {
    QDir searchDir(dir.filePath(subdir));
    if (searchDir.exists()) {
      for (const auto& pattern : patterns) {
        QDirIterator it(searchDir.absolutePath(), QStringList{pattern},
                        QDir::Files);  // Don't recurse into subdirs
        while (it.hasNext()) {
          QString path = it.next();
          // Exclude package.xml and other non-config files
          if (!path.endsWith("package.xml") && !path.contains("CMakeLists")) {
            files.append(path);
          }
        }
      }
    }
  }

  files.removeDuplicates();
  return files;
}

QStringList WorkspaceBrowserPanel::findSourceFiles(const QString& packagePath) {
  QStringList files;
  QDir dir(packagePath);

  // Source file patterns
  QStringList patterns = {"*.cpp", "*.hpp", "*.h", "*.c", "*.py"};

  QDirIterator it(dir.absolutePath(), patterns,
                  QDir::Files, QDirIterator::Subdirectories);

  while (it.hasNext()) {
    QString path = it.next();
    // Exclude build directories
    if (!path.contains("/build/") && !path.contains("/install/") &&
        !path.contains("/__pycache__/")) {
      files.append(path);
    }
  }

  return files;
}

QTreeWidgetItem* WorkspaceBrowserPanel::createFileItem(const QString& filePath, const QString& fileType) {
  QTreeWidgetItem* item = new QTreeWidgetItem();
  QFileInfo fi(filePath);

  QString icon = getFileIcon(fi.suffix());
  item->setText(0, QString("%1 %2").arg(icon).arg(fi.fileName()));
  item->setText(1, fileType);
  item->setText(2, filePath);
  item->setData(0, FilePathRole, filePath);
  item->setData(0, FileTypeRole, fileType);
  item->setToolTip(0, filePath);

  return item;
}

QString WorkspaceBrowserPanel::getFileIcon(const QString& extension) {
  QString ext = extension.toLower();

  if (ext == "py") {
    return QString::fromUtf8("\xF0\x9F\x90\x8D");  // Snake for Python
  } else if (ext == "cpp" || ext == "hpp" || ext == "h" || ext == "c") {
    return QString::fromUtf8("\xE2\x9A\x99");  // Gear for C++
  } else if (ext == "yaml" || ext == "yml") {
    return QString::fromUtf8("\xF0\x9F\x93\x8B");  // Clipboard
  } else if (ext == "xml") {
    return QString::fromUtf8("\xF0\x9F\x93\x84");  // Document
  } else if (ext == "json") {
    return QString::fromUtf8("\xF0\x9F\x93\x8A");  // Chart
  } else if (ext == "launch") {
    return QString::fromUtf8("\xF0\x9F\x9A\x80");  // Rocket
  }

  return QString::fromUtf8("\xF0\x9F\x93\x84");  // Default document
}

}  // namespace ros_weaver
