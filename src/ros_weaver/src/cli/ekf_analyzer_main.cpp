#include "ros_weaver/cli/ekf_analyzer_cli.hpp"

#include <QCoreApplication>
#include <QCommandLineParser>
#include <QFileInfo>

#include <iostream>

using namespace ros_weaver;

/**
 * @brief Parse sweep parameter specification
 *
 * Format: "name:min:max:steps[:log]"
 * Examples:
 *   - "processNoiseCovariance.0:0.01:0.5:5"
 *   - "processNoiseCovariance.6:0.001:0.1:10:log"
 */
bool parseSweepParam(const QString& spec, EKFSweepParameter& param) {
  QStringList parts = spec.split(':');
  if (parts.size() < 4 || parts.size() > 5) {
    std::cerr << "Error: Invalid sweep parameter format: " << spec.toStdString() << std::endl;
    std::cerr << "Expected: name:min:max:steps[:log]" << std::endl;
    return false;
  }

  param.name = parts[0];

  bool ok;
  param.minValue = parts[1].toDouble(&ok);
  if (!ok) {
    std::cerr << "Error: Invalid min value: " << parts[1].toStdString() << std::endl;
    return false;
  }

  param.maxValue = parts[2].toDouble(&ok);
  if (!ok) {
    std::cerr << "Error: Invalid max value: " << parts[2].toStdString() << std::endl;
    return false;
  }

  param.steps = parts[3].toInt(&ok);
  if (!ok || param.steps < 1) {
    std::cerr << "Error: Invalid steps value: " << parts[3].toStdString() << std::endl;
    return false;
  }

  param.logScale = false;
  if (parts.size() == 5) {
    if (parts[4].toLower() == "log") {
      param.logScale = true;
    } else {
      std::cerr << "Error: Unknown option: " << parts[4].toStdString() << std::endl;
      return false;
    }
  }

  return true;
}

int main(int argc, char* argv[]) {
  // Use QCoreApplication for headless operation (no GUI)
  QCoreApplication app(argc, argv);
  QCoreApplication::setApplicationName("ekf_analyzer");
  QCoreApplication::setApplicationVersion("1.0.0");

  // Set up command line parser
  QCommandLineParser parser;
  parser.setApplicationDescription(
      "EKF Analyzer - Analyze robot_localization EKF performance on rosbags\n\n"
      "Examples:\n"
      "  ros2 run ros_weaver ekf_analyzer /path/to/bag.mcap -c ekf_config.yaml\n"
      "  ros2 run ros_weaver ekf_analyzer bag.mcap -c config.yaml -o results.json -f json\n"
      "  ros2 run ros_weaver ekf_analyzer bag.mcap -c config.yaml --sweep \\\n"
      "      --sweep-param \"processNoiseCovariance.0:0.01:0.5:5\" -o sweep.csv -f csv");
  parser.addHelpOption();
  parser.addVersionOption();

  // Positional argument: input bag
  parser.addPositionalArgument("bag", "Input rosbag2 file (.mcap or .db3)");

  // Options
  QCommandLineOption configOption(QStringList() << "c" << "config",
                                   "EKF YAML config file (robot_localization format)",
                                   "file");
  parser.addOption(configOption);

  QCommandLineOption outputOption(QStringList() << "o" << "output",
                                   "Output file path (stdout if omitted)",
                                   "file");
  parser.addOption(outputOption);

  QCommandLineOption formatOption(QStringList() << "f" << "format",
                                   "Output format: text, json, csv (default: text)",
                                   "format", "text");
  parser.addOption(formatOption);

  QCommandLineOption groundTruthOption(QStringList() << "g" << "ground-truth",
                                        "Ground truth topic (default: /ground_truth/odom)",
                                        "topic", "/ground_truth/odom");
  parser.addOption(groundTruthOption);

  QCommandLineOption rateOption(QStringList() << "r" << "rate",
                                 "Playback rate, 0 = max speed (default: 0)",
                                 "rate", "0");
  parser.addOption(rateOption);

  QCommandLineOption sweepOption("sweep", "Enable parameter sweep mode");
  parser.addOption(sweepOption);

  QCommandLineOption sweepParamOption("sweep-param",
                                       "Sweep parameter spec: name:min:max:steps[:log]",
                                       "spec");
  parser.addOption(sweepParamOption);

  QCommandLineOption verboseOption("verbose",
                                    "Verbose output with progress");
  parser.addOption(verboseOption);

  // Process arguments
  parser.process(app);

  // Get positional arguments
  QStringList args = parser.positionalArguments();
  if (args.isEmpty()) {
    std::cerr << "Error: No input bag specified" << std::endl;
    std::cerr << std::endl;
    parser.showHelp(static_cast<int>(ExitCode::InvalidArguments));
  }

  QString bagPath = args.first();

  // Validate bag path
  QFileInfo bagInfo(bagPath);
  if (!bagInfo.exists()) {
    std::cerr << "Error: Input bag not found: " << bagPath.toStdString() << std::endl;
    return static_cast<int>(ExitCode::InputBagError);
  }

  // Config is required
  if (!parser.isSet(configOption)) {
    std::cerr << "Error: Config file is required (-c/--config)" << std::endl;
    return static_cast<int>(ExitCode::InvalidArguments);
  }

  QString configPath = parser.value(configOption);
  QFileInfo configInfo(configPath);
  if (!configInfo.exists()) {
    std::cerr << "Error: Config file not found: " << configPath.toStdString() << std::endl;
    return static_cast<int>(ExitCode::ConfigFileError);
  }

  // Parse output format
  OutputFormat format = OutputFormat::Text;
  QString formatStr = parser.value(formatOption).toLower();
  if (formatStr == "text") {
    format = OutputFormat::Text;
  } else if (formatStr == "json") {
    format = OutputFormat::Json;
  } else if (formatStr == "csv") {
    format = OutputFormat::Csv;
  } else {
    std::cerr << "Error: Invalid output format: " << formatStr.toStdString() << std::endl;
    std::cerr << "Valid formats: text, json, csv" << std::endl;
    return static_cast<int>(ExitCode::InvalidArguments);
  }

  // Parse playback rate
  bool ok;
  double rate = parser.value(rateOption).toDouble(&ok);
  if (!ok || rate < 0) {
    std::cerr << "Error: Invalid playback rate" << std::endl;
    return static_cast<int>(ExitCode::InvalidArguments);
  }

  // Create CLI runner
  EKFAnalyzerCLI cli;
  cli.setInputBagPath(bagPath);
  cli.setOutputPath(parser.value(outputOption));
  cli.setOutputFormat(format);
  cli.setGroundTruthTopic(parser.value(groundTruthOption));
  cli.setPlaybackRate(rate);
  cli.setVerbose(parser.isSet(verboseOption));

  // Load config
  if (!cli.loadConfig(configPath)) {
    std::cerr << "Error: Failed to load config file" << std::endl;
    return static_cast<int>(ExitCode::ConfigFileError);
  }

  // Handle sweep mode
  if (parser.isSet(sweepOption)) {
    cli.enableSweepMode(true);

    QStringList sweepParams = parser.values(sweepParamOption);
    if (sweepParams.isEmpty()) {
      std::cerr << "Error: --sweep requires at least one --sweep-param" << std::endl;
      return static_cast<int>(ExitCode::InvalidArguments);
    }

    for (const QString& paramSpec : sweepParams) {
      EKFSweepParameter param;
      if (!parseSweepParam(paramSpec, param)) {
        return static_cast<int>(ExitCode::InvalidArguments);
      }
      cli.addSweepParameter(param);
    }
  }

  // Run analysis
  ExitCode result = cli.run();
  return static_cast<int>(result);
}
