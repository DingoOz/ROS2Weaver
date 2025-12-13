"""
PDF report generator for UI regression test results.

Generates a professional PDF report showing screenshots before each test
action along with pass/fail status and any error details.
"""

import os
from pathlib import Path
from datetime import datetime
from dataclasses import dataclass, field
from typing import List, Optional
from enum import Enum

from reportlab.lib import colors
from reportlab.lib.pagesizes import A4
from reportlab.lib.styles import getSampleStyleSheet, ParagraphStyle
from reportlab.lib.units import mm, cm
from reportlab.platypus import (
    SimpleDocTemplate, Paragraph, Spacer, Image, Table, TableStyle,
    PageBreak, KeepTogether
)
from reportlab.lib.enums import TA_CENTER, TA_LEFT


class TestStatus(Enum):
    """Test result status."""
    PASSED = 'PASSED'
    FAILED = 'FAILED'
    SKIPPED = 'SKIPPED'
    ERROR = 'ERROR'


@dataclass
class TestResult:
    """Result of a single UI test."""
    test_name: str
    widget_name: str
    widget_type: str
    status: TestStatus
    screenshot_path: Optional[Path] = None
    error_message: Optional[str] = None
    duration_seconds: float = 0.0
    timestamp: datetime = field(default_factory=datetime.now)
    
    @property
    def passed(self) -> bool:
        return self.status == TestStatus.PASSED


@dataclass
class TestSuite:
    """Collection of test results for a test suite."""
    name: str
    results: List[TestResult] = field(default_factory=list)
    start_time: datetime = field(default_factory=datetime.now)
    end_time: Optional[datetime] = None
    
    def add_result(self, result: TestResult):
        self.results.append(result)
    
    @property
    def total_tests(self) -> int:
        return len(self.results)
    
    @property
    def passed_tests(self) -> int:
        return sum(1 for r in self.results if r.status == TestStatus.PASSED)
    
    @property
    def failed_tests(self) -> int:
        return sum(1 for r in self.results if r.status == TestStatus.FAILED)
    
    @property
    def error_tests(self) -> int:
        return sum(1 for r in self.results if r.status == TestStatus.ERROR)
    
    @property
    def skipped_tests(self) -> int:
        return sum(1 for r in self.results if r.status == TestStatus.SKIPPED)
    
    @property
    def pass_rate(self) -> float:
        if self.total_tests == 0:
            return 0.0
        return (self.passed_tests / self.total_tests) * 100


class ReportGenerator:
    """Generates PDF reports for UI test results."""
    
    # Page dimensions
    PAGE_WIDTH, PAGE_HEIGHT = A4
    MARGIN = 2 * cm
    
    # Colours
    COLOUR_PASS = colors.HexColor('#28a745')
    COLOUR_FAIL = colors.HexColor('#dc3545')
    COLOUR_SKIP = colors.HexColor('#6c757d')
    COLOUR_ERROR = colors.HexColor('#fd7e14')
    COLOUR_HEADER = colors.HexColor('#343a40')
    COLOUR_LIGHT_BG = colors.HexColor('#f8f9fa')
    
    def __init__(self, output_path: Path):
        """
        Initialise the report generator.
        
        Args:
            output_path: Path for the output PDF file.
        """
        self.output_path = Path(output_path)
        self.output_path.parent.mkdir(parents=True, exist_ok=True)
        
        self.styles = getSampleStyleSheet()
        self._setup_custom_styles()
        
        self.suites: List[TestSuite] = []
    
    def _setup_custom_styles(self):
        """Configure custom paragraph styles."""
        self.styles.add(ParagraphStyle(
            name='ReportTitle',
            parent=self.styles['Heading1'],
            fontSize=24,
            spaceAfter=30,
            alignment=TA_CENTER,
            textColor=self.COLOUR_HEADER
        ))
        
        self.styles.add(ParagraphStyle(
            name='SuiteTitle',
            parent=self.styles['Heading2'],
            fontSize=16,
            spaceBefore=20,
            spaceAfter=10,
            textColor=self.COLOUR_HEADER
        ))
        
        self.styles.add(ParagraphStyle(
            name='TestName',
            parent=self.styles['Heading3'],
            fontSize=12,
            spaceBefore=10,
            spaceAfter=5
        ))
        
        self.styles.add(ParagraphStyle(
            name='StatusPass',
            parent=self.styles['Normal'],
            fontSize=11,
            textColor=self.COLOUR_PASS,
            fontName='Helvetica-Bold'
        ))
        
        self.styles.add(ParagraphStyle(
            name='StatusFail',
            parent=self.styles['Normal'],
            fontSize=11,
            textColor=self.COLOUR_FAIL,
            fontName='Helvetica-Bold'
        ))
        
        self.styles.add(ParagraphStyle(
            name='ErrorText',
            parent=self.styles['Normal'],
            fontSize=9,
            textColor=self.COLOUR_FAIL,
            leftIndent=10,
            fontName='Courier'
        ))
        
        self.styles.add(ParagraphStyle(
            name='InfoText',
            parent=self.styles['Normal'],
            fontSize=10,
            textColor=colors.gray
        ))
    
    def add_suite(self, suite: TestSuite):
        """Add a test suite to the report."""
        self.suites.append(suite)
    
    def _get_status_colour(self, status: TestStatus) -> colors.Color:
        """Get the colour for a test status."""
        return {
            TestStatus.PASSED: self.COLOUR_PASS,
            TestStatus.FAILED: self.COLOUR_FAIL,
            TestStatus.SKIPPED: self.COLOUR_SKIP,
            TestStatus.ERROR: self.COLOUR_ERROR
        }.get(status, colors.black)
    
    def _build_title_page(self) -> List:
        """Build the title page elements."""
        elements = []
        
        # Title
        elements.append(Spacer(1, 3 * cm))
        elements.append(Paragraph(
            'ROS2Weaver UI Regression Test Report',
            self.styles['ReportTitle']
        ))
        
        # Timestamp
        elements.append(Spacer(1, 1 * cm))
        timestamp = datetime.now().strftime('%Y-%m-%d %H:%M:%S')
        elements.append(Paragraph(
            f'Generated: {timestamp}',
            self.styles['InfoText']
        ))
        
        # Summary statistics
        elements.append(Spacer(1, 2 * cm))
        
        total_tests = sum(s.total_tests for s in self.suites)
        total_passed = sum(s.passed_tests for s in self.suites)
        total_failed = sum(s.failed_tests for s in self.suites)
        total_errors = sum(s.error_tests for s in self.suites)
        total_skipped = sum(s.skipped_tests for s in self.suites)
        
        pass_rate = (total_passed / total_tests * 100) if total_tests > 0 else 0
        
        summary_data = [
            ['Total Tests', str(total_tests)],
            ['Passed', str(total_passed)],
            ['Failed', str(total_failed)],
            ['Errors', str(total_errors)],
            ['Skipped', str(total_skipped)],
            ['Pass Rate', f'{pass_rate:.1f}%']
        ]
        
        summary_table = Table(summary_data, colWidths=[6 * cm, 4 * cm])
        summary_table.setStyle(TableStyle([
            ('BACKGROUND', (0, 0), (0, -1), self.COLOUR_LIGHT_BG),
            ('TEXTCOLOR', (0, 0), (-1, -1), self.COLOUR_HEADER),
            ('ALIGN', (0, 0), (-1, -1), 'CENTER'),
            ('FONTNAME', (0, 0), (-1, -1), 'Helvetica'),
            ('FONTSIZE', (0, 0), (-1, -1), 12),
            ('BOTTOMPADDING', (0, 0), (-1, -1), 8),
            ('TOPPADDING', (0, 0), (-1, -1), 8),
            ('GRID', (0, 0), (-1, -1), 0.5, colors.gray)
        ]))
        
        elements.append(summary_table)
        
        # Overall status
        elements.append(Spacer(1, 1 * cm))
        if total_failed == 0 and total_errors == 0:
            status_text = 'ALL TESTS PASSED'
            status_colour = self.COLOUR_PASS
        else:
            status_text = f'{total_failed + total_errors} TESTS FAILED'
            status_colour = self.COLOUR_FAIL
        
        elements.append(Paragraph(
            f'<font color="{status_colour.hexval()}">{status_text}</font>',
            ParagraphStyle(
                'OverallStatus',
                parent=self.styles['Normal'],
                fontSize=18,
                alignment=TA_CENTER,
                fontName='Helvetica-Bold'
            )
        ))
        
        elements.append(PageBreak())
        
        return elements
    
    def _build_suite_section(self, suite: TestSuite) -> List:
        """Build the section for a single test suite."""
        elements = []
        
        # Suite header
        elements.append(Paragraph(
            f'Test Suite: {suite.name}',
            self.styles['SuiteTitle']
        ))
        
        # Suite summary
        summary_text = (
            f'{suite.total_tests} tests | '
            f'{suite.passed_tests} passed | '
            f'{suite.failed_tests} failed | '
            f'{suite.pass_rate:.1f}% pass rate'
        )
        elements.append(Paragraph(summary_text, self.styles['InfoText']))
        elements.append(Spacer(1, 0.5 * cm))
        
        # Individual test results
        for result in suite.results:
            elements.extend(self._build_test_result(result))
        
        elements.append(PageBreak())
        
        return elements
    
    def _build_test_result(self, result: TestResult) -> List:
        """Build the section for a single test result."""
        elements = []
        
        # Create a keep-together block for each test
        test_elements = []
        
        # Test header with status
        status_colour = self._get_status_colour(result.status)
        header_text = (
            f'<font color="{self.COLOUR_HEADER.hexval()}">'
            f'{result.widget_type}: {result.widget_name}</font> '
            f'<font color="{status_colour.hexval()}">[{result.status.value}]</font>'
        )
        test_elements.append(Paragraph(header_text, self.styles['TestName']))
        
        # Test info
        info_text = f'Duration: {result.duration_seconds:.2f}s'
        test_elements.append(Paragraph(info_text, self.styles['InfoText']))
        
        # Screenshot if available
        if result.screenshot_path and result.screenshot_path.exists():
            test_elements.append(Spacer(1, 0.3 * cm))
            
            # Calculate image size to fit page width while maintaining aspect ratio
            max_width = self.PAGE_WIDTH - 2 * self.MARGIN - 2 * cm
            max_height = 8 * cm
            
            try:
                img = Image(str(result.screenshot_path))
                
                # Scale to fit
                aspect = img.imageWidth / img.imageHeight
                if img.imageWidth > max_width:
                    img.drawWidth = max_width
                    img.drawHeight = max_width / aspect
                
                if img.drawHeight > max_height:
                    img.drawHeight = max_height
                    img.drawWidth = max_height * aspect
                
                test_elements.append(img)
            except Exception as e:
                test_elements.append(Paragraph(
                    f'[Screenshot unavailable: {e}]',
                    self.styles['InfoText']
                ))
        
        # Error message if failed
        if result.error_message:
            test_elements.append(Spacer(1, 0.2 * cm))
            # Truncate very long error messages
            error_msg = result.error_message
            if len(error_msg) > 500:
                error_msg = error_msg[:500] + '...'
            test_elements.append(Paragraph(
                f'Error: {error_msg}',
                self.styles['ErrorText']
            ))
        
        test_elements.append(Spacer(1, 0.5 * cm))
        
        # Try to keep test result together on one page
        elements.append(KeepTogether(test_elements))
        
        return elements
    
    def generate(self) -> Path:
        """
        Generate the PDF report.
        
        Returns:
            Path to the generated PDF file.
        """
        doc = SimpleDocTemplate(
            str(self.output_path),
            pagesize=A4,
            leftMargin=self.MARGIN,
            rightMargin=self.MARGIN,
            topMargin=self.MARGIN,
            bottomMargin=self.MARGIN
        )
        
        elements = []
        
        # Title page
        elements.extend(self._build_title_page())
        
        # Test suite sections
        for suite in self.suites:
            elements.extend(self._build_suite_section(suite))
        
        # Build PDF
        doc.build(elements)
        
        return self.output_path


class TestResultCollector:
    """
    Collects test results during test execution.
    
    Used as a pytest fixture to gather results for report generation.
    """
    
    def __init__(self, report_name: str = 'ui_test_report'):
        self.report_name = report_name
        self.suites: dict[str, TestSuite] = {}
        self.current_suite: Optional[str] = None
        self.run_dir: Optional[Path] = None
    
    def set_run_dir(self, run_dir: Path):
        """Set the directory for this test run."""
        self.run_dir = run_dir
    
    def start_suite(self, name: str):
        """Start collecting results for a new test suite."""
        if name not in self.suites:
            self.suites[name] = TestSuite(name=name)
        self.current_suite = name
    
    def add_result(
        self,
        test_name: str,
        widget_name: str,
        widget_type: str,
        passed: bool,
        screenshot_path: Optional[Path] = None,
        error_message: Optional[str] = None,
        duration: float = 0.0
    ):
        """Add a test result to the current suite."""
        if self.current_suite is None:
            self.start_suite('Default')
        
        result = TestResult(
            test_name=test_name,
            widget_name=widget_name,
            widget_type=widget_type,
            status=TestStatus.PASSED if passed else TestStatus.FAILED,
            screenshot_path=screenshot_path,
            error_message=error_message,
            duration_seconds=duration
        )
        
        self.suites[self.current_suite].add_result(result)
    
    def generate_report(self, output_dir: Optional[Path] = None) -> Path:
        """Generate the PDF report from collected results."""
        if output_dir is None:
            output_dir = self.run_dir or Path.cwd()
        
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        output_path = output_dir / f'{self.report_name}_{timestamp}.pdf'
        
        generator = ReportGenerator(output_path)
        
        for suite in self.suites.values():
            suite.end_time = datetime.now()
            generator.add_suite(suite)
        
        return generator.generate()
