#include "ros_weaver/widgets/readme_preview_panel.hpp"
#include "ros_weaver/core/ros_docs_provider.hpp"

#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QFile>
#include <QTextStream>
#include <QDesktopServices>
#include <QUrl>
#include <QStyle>
#include <QRegularExpression>

namespace ros_weaver {

ReadmePreviewPanel::ReadmePreviewPanel(QWidget* parent)
    : QWidget(parent)
    , titleLabel_(nullptr)
    , viewModeCombo_(nullptr)
    , openExternalButton_(nullptr)
    , refreshButton_(nullptr)
    , contentBrowser_(nullptr)
    , statusLabel_(nullptr)
{
  setupUi();
}

void ReadmePreviewPanel::setupUi() {
  QVBoxLayout* mainLayout = new QVBoxLayout(this);
  mainLayout->setContentsMargins(8, 8, 8, 8);
  mainLayout->setSpacing(8);

  // Header bar
  QHBoxLayout* headerLayout = new QHBoxLayout();

  titleLabel_ = new QLabel(tr("Package Documentation"));
  titleLabel_->setStyleSheet("font-weight: bold; font-size: 14px;");

  viewModeCombo_ = new QComboBox();
  viewModeCombo_->addItem(tr("README"), static_cast<int>(ViewMode::Readme));
  viewModeCombo_->addItem(tr("package.xml"), static_cast<int>(ViewMode::PackageXml));
  viewModeCombo_->addItem(tr("Both"), static_cast<int>(ViewMode::Both));
  viewModeCombo_->setMinimumWidth(100);
  connect(viewModeCombo_, QOverload<int>::of(&QComboBox::currentIndexChanged),
          this, &ReadmePreviewPanel::onViewModeChanged);

  openExternalButton_ = new QPushButton();
  openExternalButton_->setIcon(style()->standardIcon(QStyle::SP_DialogOpenButton));
  openExternalButton_->setToolTip(tr("Open in external application"));
  openExternalButton_->setFixedWidth(32);
  openExternalButton_->setEnabled(false);
  connect(openExternalButton_, &QPushButton::clicked,
          this, &ReadmePreviewPanel::onOpenExternalClicked);

  refreshButton_ = new QPushButton();
  refreshButton_->setIcon(style()->standardIcon(QStyle::SP_BrowserReload));
  refreshButton_->setToolTip(tr("Refresh"));
  refreshButton_->setFixedWidth(32);
  connect(refreshButton_, &QPushButton::clicked,
          this, &ReadmePreviewPanel::onRefreshClicked);

  headerLayout->addWidget(titleLabel_, 1);
  headerLayout->addWidget(viewModeCombo_);
  headerLayout->addWidget(openExternalButton_);
  headerLayout->addWidget(refreshButton_);
  mainLayout->addLayout(headerLayout);

  // Content browser
  contentBrowser_ = new QTextBrowser();
  contentBrowser_->setOpenExternalLinks(true);
  contentBrowser_->setStyleSheet(R"(
    QTextBrowser {
      background-color: #ffffff;
      border: 1px solid #ddd;
      border-radius: 4px;
      padding: 8px;
    }
  )");
  connect(contentBrowser_, &QTextBrowser::anchorClicked, this, [this](const QUrl& url) {
    emit linkClicked(url.toString());
  });
  mainLayout->addWidget(contentBrowser_, 1);

  // Status bar
  statusLabel_ = new QLabel();
  statusLabel_->setStyleSheet("color: #666; font-size: 11px;");
  mainLayout->addWidget(statusLabel_);

  // Show placeholder
  contentBrowser_->setHtml(R"(
    <div style='text-align: center; padding: 40px; color: #888;'>
      <p style='font-size: 16px;'>📄 Package Documentation</p>
      <p>Select a package to view its README and documentation.</p>
    </div>
  )");
}

void ReadmePreviewPanel::setPackage(const QString& packageName) {
  currentPackage_ = packageName;
  titleLabel_->setText(tr("Package: %1").arg(packageName));

  loadPackageInfo();
}

void ReadmePreviewPanel::clear() {
  currentPackage_.clear();
  readmePath_.clear();
  packageXmlPath_.clear();
  titleLabel_->setText(tr("Package Documentation"));
  contentBrowser_->clear();
  statusLabel_->clear();
  openExternalButton_->setEnabled(false);
}

void ReadmePreviewPanel::loadPackageInfo() {
  if (currentPackage_.isEmpty()) {
    return;
  }

  RosDocsProvider& provider = RosDocsProvider::instance();
  PackageDoc doc = provider.getPackageDoc(currentPackage_);

  readmePath_ = doc.readmePath;
  packageXmlPath_ = doc.xmlPath;

  openExternalButton_->setEnabled(!readmePath_.isEmpty() || !packageXmlPath_.isEmpty());

  // Update view based on mode
  onViewModeChanged(viewModeCombo_->currentIndex());
}

void ReadmePreviewPanel::loadReadme() {
  if (currentPackage_.isEmpty()) {
    return;
  }

  QString html;
  ViewMode mode = static_cast<ViewMode>(viewModeCombo_->currentData().toInt());

  // Get package documentation
  RosDocsProvider& provider = RosDocsProvider::instance();
  PackageDoc doc = provider.getPackageDoc(currentPackage_);

  if (mode == ViewMode::PackageXml || mode == ViewMode::Both) {
    if (!packageXmlPath_.isEmpty()) {
      html += provider.formatPackageAsHtml(doc);
    } else {
      html += QString("<p><i>No package.xml found for %1</i></p>").arg(currentPackage_);
    }

    if (mode == ViewMode::Both) {
      html += "<hr style='margin: 20px 0;'/>";
    }
  }

  if (mode == ViewMode::Readme || mode == ViewMode::Both) {
    if (!readmePath_.isEmpty() && QFile::exists(readmePath_)) {
      QFile file(readmePath_);
      if (file.open(QIODevice::ReadOnly | QIODevice::Text)) {
        QTextStream in(&file);
        QString content = in.readAll();
        file.close();

        // Convert markdown to HTML
        if (readmePath_.endsWith(".md", Qt::CaseInsensitive)) {
          html += markdownToHtml(content);
        } else {
          html += QString("<pre style='white-space: pre-wrap;'>%1</pre>")
                      .arg(content.toHtmlEscaped());
        }
        statusLabel_->setText(tr("Loaded: %1").arg(readmePath_));
      }
    } else if (mode == ViewMode::Readme) {
      html += QString("<p><i>No README found for %1</i></p>").arg(currentPackage_);
    }
  }

  if (html.isEmpty()) {
    html = QString("<p><i>No documentation available for %1</i></p>").arg(currentPackage_);
  }

  contentBrowser_->setHtml(html);
}

QString ReadmePreviewPanel::markdownToHtml(const QString& markdown) const {
  QString html = markdown;

  // Basic markdown to HTML conversion
  // Headers
  static QRegularExpression h1Pattern("^# (.+)$", QRegularExpression::MultilineOption);
  static QRegularExpression h2Pattern("^## (.+)$", QRegularExpression::MultilineOption);
  static QRegularExpression h3Pattern("^### (.+)$", QRegularExpression::MultilineOption);
  static QRegularExpression h4Pattern("^#### (.+)$", QRegularExpression::MultilineOption);

  html.replace(h4Pattern, "<h4>\\1</h4>");
  html.replace(h3Pattern, "<h3>\\1</h3>");
  html.replace(h2Pattern, "<h2>\\1</h2>");
  html.replace(h1Pattern, "<h1>\\1</h1>");

  // Bold and italic
  static QRegularExpression boldPattern("\\*\\*(.+?)\\*\\*");
  static QRegularExpression italicPattern("\\*(.+?)\\*");
  html.replace(boldPattern, "<b>\\1</b>");
  html.replace(italicPattern, "<i>\\1</i>");

  // Code blocks (fenced)
  static QRegularExpression codeBlockPattern("```[\\w]*\\n([\\s\\S]*?)```");
  html.replace(codeBlockPattern,
               "<pre style='background-color: #f5f5f5; padding: 8px; "
               "border-radius: 4px; font-family: monospace;'>\\1</pre>");

  // Inline code
  static QRegularExpression inlineCodePattern("`([^`]+)`");
  html.replace(inlineCodePattern,
               "<code style='background-color: #f0f0f0; padding: 2px 4px; "
               "border-radius: 3px; font-family: monospace;'>\\1</code>");

  // Links
  static QRegularExpression linkPattern("\\[([^\\]]+)\\]\\(([^)]+)\\)");
  html.replace(linkPattern, "<a href='\\2'>\\1</a>");

  // Unordered lists
  static QRegularExpression ulItemPattern("^[\\*\\-] (.+)$", QRegularExpression::MultilineOption);
  html.replace(ulItemPattern, "<li>\\1</li>");

  // Ordered lists
  static QRegularExpression olItemPattern("^\\d+\\. (.+)$", QRegularExpression::MultilineOption);
  html.replace(olItemPattern, "<li>\\1</li>");

  // Horizontal rules
  static QRegularExpression hrPattern("^---+$", QRegularExpression::MultilineOption);
  html.replace(hrPattern, "<hr/>");

  // Paragraphs (double newlines)
  html.replace("\n\n", "</p><p>");

  // Single newlines to breaks (within paragraphs)
  html.replace("\n", "<br/>");

  // Wrap in paragraph
  html = "<p>" + html + "</p>";

  // Fix consecutive list items
  html.replace("</li><br/><li>", "</li><li>");
  html.replace("<p><li>", "<ul><li>");
  html.replace("</li></p>", "</li></ul>");

  return html;
}

void ReadmePreviewPanel::onViewModeChanged(int /*index*/) {
  loadReadme();
}

void ReadmePreviewPanel::onOpenExternalClicked() {
  ViewMode mode = static_cast<ViewMode>(viewModeCombo_->currentData().toInt());

  QString pathToOpen;
  if ((mode == ViewMode::Readme || mode == ViewMode::Both) && !readmePath_.isEmpty()) {
    pathToOpen = readmePath_;
  } else if (!packageXmlPath_.isEmpty()) {
    pathToOpen = packageXmlPath_;
  }

  if (!pathToOpen.isEmpty()) {
    QDesktopServices::openUrl(QUrl::fromLocalFile(pathToOpen));
  }
}

void ReadmePreviewPanel::onRefreshClicked() {
  if (!currentPackage_.isEmpty()) {
    RosDocsProvider::instance().refreshPackageDoc(currentPackage_);
    loadPackageInfo();
  }
}

}  // namespace ros_weaver
