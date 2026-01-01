#include "ros_weaver/core/ros_docs_provider.hpp"

#include <QProcess>
#include <QFile>
#include <QDir>
#include <QXmlStreamReader>
#include <QRegularExpression>
#include <rclcpp/rclcpp.hpp>

namespace ros_weaver {

RosDocsProvider& RosDocsProvider::instance() {
  static RosDocsProvider instance;
  return instance;
}

RosDocsProvider::RosDocsProvider() : QObject(nullptr) {
}

InterfaceDoc RosDocsProvider::getInterfaceDoc(const QString& fullName) {
  QMutexLocker locker(&mutex_);

  // Check cache
  if (interfaceCache_.contains(fullName)) {
    const InterfaceDoc& cached = interfaceCache_[fullName];
    if (!isCacheExpired(cached.fetchedAt)) {
      return cached;
    }
  }

  // Fetch and cache
  locker.unlock();
  InterfaceDoc doc = fetchInterfaceDoc(fullName);
  locker.relock();
  interfaceCache_[fullName] = doc;
  return doc;
}

PackageDoc RosDocsProvider::getPackageDoc(const QString& packageName) {
  QMutexLocker locker(&mutex_);

  // Check cache
  if (packageCache_.contains(packageName)) {
    const PackageDoc& cached = packageCache_[packageName];
    if (!isCacheExpired(cached.fetchedAt)) {
      return cached;
    }
  }

  // Fetch and cache
  locker.unlock();
  PackageDoc doc = fetchPackageDoc(packageName);
  locker.relock();
  packageCache_[packageName] = doc;
  return doc;
}

bool RosDocsProvider::hasInterfaceDoc(const QString& fullName) const {
  QMutexLocker locker(&mutex_);
  return interfaceCache_.contains(fullName) &&
         !isCacheExpired(interfaceCache_[fullName].fetchedAt);
}

bool RosDocsProvider::hasPackageDoc(const QString& packageName) const {
  QMutexLocker locker(&mutex_);
  return packageCache_.contains(packageName) &&
         !isCacheExpired(packageCache_[packageName].fetchedAt);
}

void RosDocsProvider::clearCache() {
  QMutexLocker locker(&mutex_);
  interfaceCache_.clear();
  packageCache_.clear();
  availableMessages_.clear();
  availableServices_.clear();
  availableActions_.clear();
  availablePackages_.clear();
}

void RosDocsProvider::setCacheExpirationMinutes(int minutes) {
  cacheExpirationMinutes_ = qMax(1, minutes);
}

InterfaceDoc RosDocsProvider::refreshInterfaceDoc(const QString& fullName) {
  QMutexLocker locker(&mutex_);
  interfaceCache_.remove(fullName);
  locker.unlock();
  return getInterfaceDoc(fullName);
}

PackageDoc RosDocsProvider::refreshPackageDoc(const QString& packageName) {
  QMutexLocker locker(&mutex_);
  packageCache_.remove(packageName);
  locker.unlock();
  return getPackageDoc(packageName);
}

QStringList RosDocsProvider::getAvailableMessages() {
  QMutexLocker locker(&mutex_);

  if (!availableMessages_.isEmpty() && !isCacheExpired(lastInterfaceListRefresh_)) {
    return availableMessages_;
  }

  locker.unlock();
  QString output = executeRos2Command({"interface", "list", "-m"});
  locker.relock();

  availableMessages_ = output.split('\n', Qt::SkipEmptyParts);
  for (QString& msg : availableMessages_) {
    msg = msg.trimmed();
  }
  lastInterfaceListRefresh_ = QDateTime::currentDateTime();
  return availableMessages_;
}

QStringList RosDocsProvider::getAvailableServices() {
  QMutexLocker locker(&mutex_);

  if (!availableServices_.isEmpty() && !isCacheExpired(lastInterfaceListRefresh_)) {
    return availableServices_;
  }

  locker.unlock();
  QString output = executeRos2Command({"interface", "list", "-s"});
  locker.relock();

  availableServices_ = output.split('\n', Qt::SkipEmptyParts);
  for (QString& srv : availableServices_) {
    srv = srv.trimmed();
  }
  lastInterfaceListRefresh_ = QDateTime::currentDateTime();
  return availableServices_;
}

QStringList RosDocsProvider::getAvailableActions() {
  QMutexLocker locker(&mutex_);

  if (!availableActions_.isEmpty() && !isCacheExpired(lastInterfaceListRefresh_)) {
    return availableActions_;
  }

  locker.unlock();
  QString output = executeRos2Command({"interface", "list", "-a"});
  locker.relock();

  availableActions_ = output.split('\n', Qt::SkipEmptyParts);
  for (QString& action : availableActions_) {
    action = action.trimmed();
  }
  lastInterfaceListRefresh_ = QDateTime::currentDateTime();
  return availableActions_;
}

QStringList RosDocsProvider::getAvailablePackages() {
  QMutexLocker locker(&mutex_);

  if (!availablePackages_.isEmpty() && !isCacheExpired(lastPackageListRefresh_)) {
    return availablePackages_;
  }

  locker.unlock();
  QString output = executeRos2Command({"pkg", "list"});
  locker.relock();

  availablePackages_ = output.split('\n', Qt::SkipEmptyParts);
  for (QString& pkg : availablePackages_) {
    pkg = pkg.trimmed();
  }
  lastPackageListRefresh_ = QDateTime::currentDateTime();
  return availablePackages_;
}

InterfaceDoc RosDocsProvider::fetchInterfaceDoc(const QString& fullName) {
  InterfaceDoc doc;
  doc.fullName = fullName;
  doc.fetchedAt = QDateTime::currentDateTime();

  // Parse the full name (e.g., "std_msgs/msg/String")
  QStringList parts = fullName.split('/');
  if (parts.size() >= 3) {
    doc.packageName = parts[0];
    QString interfaceTypeStr = parts[1];
    doc.typeName = parts.mid(2).join('/');

    if (interfaceTypeStr == "msg") {
      doc.interfaceType = InterfaceDoc::Type::Message;
    } else if (interfaceTypeStr == "srv") {
      doc.interfaceType = InterfaceDoc::Type::Service;
    } else if (interfaceTypeStr == "action") {
      doc.interfaceType = InterfaceDoc::Type::Action;
    }
  } else if (parts.size() == 2) {
    // Handle shorthand like "std_msgs/String"
    doc.packageName = parts[0];
    doc.typeName = parts[1];
    doc.interfaceType = InterfaceDoc::Type::Message;  // Assume message
  }

  // Get interface definition
  QString output = executeRos2Command({"interface", "show", fullName});

  if (output.isEmpty() || output.contains("Unknown")) {
    RCLCPP_WARN(rclcpp::get_logger("ros_weaver"),
                "Could not fetch interface documentation for: %s",
                fullName.toStdString().c_str());
    return doc;
  }

  doc.rawDefinition = output;
  doc.fields = parseInterfaceDefinition(output);
  doc.isValid = true;

  emit interfaceDocFetched(fullName, doc);
  return doc;
}

PackageDoc RosDocsProvider::fetchPackageDoc(const QString& packageName) {
  PackageDoc doc;
  doc.name = packageName;
  doc.fetchedAt = QDateTime::currentDateTime();

  // Get package prefix path
  QString prefixOutput = executeRos2Command({"pkg", "prefix", packageName});
  QString prefixPath = prefixOutput.trimmed();

  if (prefixPath.isEmpty() || prefixPath.contains("not found")) {
    RCLCPP_WARN(rclcpp::get_logger("ros_weaver"),
                "Could not find package: %s",
                packageName.toStdString().c_str());
    return doc;
  }

  // Find package.xml (could be in share or src)
  QString sharePath = prefixPath + "/share/" + packageName;
  QString packageXmlPath = sharePath + "/package.xml";

  if (!QFile::exists(packageXmlPath)) {
    // Try alternative locations
    QString altPath = prefixPath + "/" + packageName + "/package.xml";
    if (QFile::exists(altPath)) {
      packageXmlPath = altPath;
    }
  }

  if (QFile::exists(packageXmlPath)) {
    doc.xmlPath = packageXmlPath;
    doc = parsePackageXml(packageXmlPath);
    doc.name = packageName;
    doc.fetchedAt = QDateTime::currentDateTime();
  }

  // Check for README
  QStringList readmeNames = {"README.md", "README.rst", "README.txt", "README"};
  for (const QString& readmeName : readmeNames) {
    QString readmePath = sharePath + "/" + readmeName;
    if (QFile::exists(readmePath)) {
      doc.readmePath = readmePath;
      break;
    }
  }

  doc.isValid = !doc.description.isEmpty() || !doc.version.isEmpty();

  emit packageDocFetched(packageName, doc);
  return doc;
}

QList<FieldInfo> RosDocsProvider::parseInterfaceDefinition(const QString& definition) {
  QList<FieldInfo> fields;

  QStringList lines = definition.split('\n');
  int currentDepth = 0;
  bool inServiceResponse = false;
  bool inActionResult = false;
  bool inActionFeedback = false;

  static QRegularExpression fieldPattern(
      R"(^(\s*)(\w+(?:/\w+)*(?:\[\d*\])?)\s+(\w+)(?:\s*=\s*(.+?))?(?:\s*#\s*(.*))?$)");

  static QRegularExpression separatorPattern(R"(^---\s*$)");

  for (const QString& line : lines) {
    QString trimmedLine = line.trimmed();

    // Handle service/action separators
    if (separatorPattern.match(trimmedLine).hasMatch()) {
      if (inServiceResponse || inActionResult) {
        inActionFeedback = true;
        inActionResult = false;
      } else if (inActionFeedback) {
        // After feedback section
      } else {
        inServiceResponse = true;
        inActionResult = true;
      }
      continue;
    }

    // Skip empty lines and comments
    if (trimmedLine.isEmpty() || trimmedLine.startsWith('#')) {
      continue;
    }

    // Calculate depth from leading whitespace
    int leadingSpaces = line.length() - line.trimmed().length();
    currentDepth = leadingSpaces / 2;

    QRegularExpressionMatch match = fieldPattern.match(line);
    if (match.hasMatch()) {
      FieldInfo field;
      field.type = match.captured(2);
      field.name = match.captured(3);
      field.defaultValue = match.captured(4).trimmed();
      field.comment = match.captured(5).trimmed();
      field.depth = currentDepth;
      fields.append(field);
    }
  }

  return fields;
}

PackageDoc RosDocsProvider::parsePackageXml(const QString& xmlPath) {
  PackageDoc doc;

  QFile file(xmlPath);
  if (!file.open(QIODevice::ReadOnly | QIODevice::Text)) {
    return doc;
  }

  QXmlStreamReader xml(&file);

  while (!xml.atEnd() && !xml.hasError()) {
    QXmlStreamReader::TokenType token = xml.readNext();

    if (token == QXmlStreamReader::StartElement) {
      QString elementName = xml.name().toString();

      if (elementName == "name") {
        doc.name = xml.readElementText();
      } else if (elementName == "version") {
        doc.version = xml.readElementText();
      } else if (elementName == "description") {
        doc.description = xml.readElementText().trimmed();
      } else if (elementName == "maintainer") {
        QString email = xml.attributes().value("email").toString();
        QString name = xml.readElementText();
        doc.maintainer = email.isEmpty() ? name : QString("%1 <%2>").arg(name, email);
      } else if (elementName == "license") {
        doc.license = xml.readElementText();
      } else if (elementName == "buildtool_depend" || elementName == "build_type") {
        if (elementName == "build_type") {
          doc.buildType = xml.readElementText();
        }
      } else if (elementName == "depend" || elementName == "exec_depend" ||
                 elementName == "build_depend") {
        doc.dependencies.append(xml.readElementText());
      }
    }
  }

  file.close();
  doc.xmlPath = xmlPath;
  return doc;
}

QString RosDocsProvider::executeRos2Command(const QStringList& args, int timeoutMs) {
  QProcess process;
  process.setProgram("ros2");
  process.setArguments(args);

  // Source ROS2 environment
  QProcessEnvironment env = QProcessEnvironment::systemEnvironment();
  process.setProcessEnvironment(env);

  process.start();

  if (!process.waitForStarted(3000)) {
    emit errorOccurred(QString("Failed to start ros2 command: %1").arg(args.join(" ")));
    return QString();
  }

  if (!process.waitForFinished(timeoutMs)) {
    process.kill();
    emit errorOccurred(QString("ros2 command timed out: %1").arg(args.join(" ")));
    return QString();
  }

  if (process.exitCode() != 0) {
    QString errorOutput = QString::fromUtf8(process.readAllStandardError());
    if (!errorOutput.isEmpty()) {
      RCLCPP_DEBUG(rclcpp::get_logger("ros_weaver"),
                   "ros2 command error: %s",
                   errorOutput.toStdString().c_str());
    }
  }

  return QString::fromUtf8(process.readAllStandardOutput());
}

bool RosDocsProvider::isCacheExpired(const QDateTime& fetchedAt) const {
  if (!fetchedAt.isValid()) {
    return true;
  }
  qint64 elapsedMins = fetchedAt.secsTo(QDateTime::currentDateTime()) / 60;
  return elapsedMins >= cacheExpirationMinutes_;
}

QString RosDocsProvider::formatInterfaceAsHtml(const InterfaceDoc& doc) const {
  if (!doc.isValid) {
    return QString("<p><i>Documentation not available for %1</i></p>").arg(doc.fullName);
  }

  QString html;
  html += QString("<h3>%1</h3>").arg(doc.fullName);

  QString typeStr;
  switch (doc.interfaceType) {
    case InterfaceDoc::Type::Message: typeStr = "Message"; break;
    case InterfaceDoc::Type::Service: typeStr = "Service"; break;
    case InterfaceDoc::Type::Action: typeStr = "Action"; break;
    default: typeStr = "Interface"; break;
  }
  html += QString("<p><b>Type:</b> %1</p>").arg(typeStr);
  html += QString("<p><b>Package:</b> %1</p>").arg(doc.packageName);

  if (!doc.fields.isEmpty()) {
    html += "<h4>Fields:</h4>";
    html += "<table style='border-collapse: collapse; width: 100%;'>";
    html += "<tr style='background-color: #f0f0f0;'>"
            "<th style='padding: 4px; text-align: left;'>Type</th>"
            "<th style='padding: 4px; text-align: left;'>Name</th>"
            "<th style='padding: 4px; text-align: left;'>Default</th>"
            "</tr>";

    for (const FieldInfo& field : doc.fields) {
      QString indent = QString("&nbsp;").repeated(field.depth * 4);
      html += QString("<tr>"
                      "<td style='padding: 4px; font-family: monospace;'>%1%2</td>"
                      "<td style='padding: 4px; font-family: monospace;'>%3</td>"
                      "<td style='padding: 4px; color: #666;'>%4</td>"
                      "</tr>")
                  .arg(indent, field.type, field.name, field.defaultValue);
    }
    html += "</table>";
  }

  if (!doc.rawDefinition.isEmpty()) {
    html += "<h4>Raw Definition:</h4>";
    html += QString("<pre style='background-color: #f5f5f5; padding: 8px; "
                    "font-family: monospace; font-size: 11px;'>%1</pre>")
                .arg(doc.rawDefinition.toHtmlEscaped());
  }

  return html;
}

QString RosDocsProvider::formatPackageAsHtml(const PackageDoc& doc) const {
  if (!doc.isValid) {
    return QString("<p><i>Documentation not available for package %1</i></p>").arg(doc.name);
  }

  QString html;
  html += QString("<h3>%1</h3>").arg(doc.name);

  if (!doc.version.isEmpty()) {
    html += QString("<p><b>Version:</b> %1</p>").arg(doc.version);
  }

  if (!doc.description.isEmpty()) {
    html += QString("<p>%1</p>").arg(doc.description.toHtmlEscaped());
  }

  if (!doc.maintainer.isEmpty()) {
    html += QString("<p><b>Maintainer:</b> %1</p>").arg(doc.maintainer.toHtmlEscaped());
  }

  if (!doc.license.isEmpty()) {
    html += QString("<p><b>License:</b> %1</p>").arg(doc.license);
  }

  if (!doc.buildType.isEmpty()) {
    html += QString("<p><b>Build Type:</b> %1</p>").arg(doc.buildType);
  }

  if (!doc.dependencies.isEmpty()) {
    html += "<h4>Dependencies:</h4><ul>";
    for (const QString& dep : doc.dependencies) {
      html += QString("<li>%1</li>").arg(dep);
    }
    html += "</ul>";
  }

  return html;
}

}  // namespace ros_weaver
