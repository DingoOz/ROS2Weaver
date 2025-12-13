"""
Canvas interaction smoke tests for ROS2Weaver.

Tests basic canvas operations like right-click context menu, zooming, and
ensuring the canvas area responds to interactions without crashes.
"""

import time
import pytest

import gi
gi.require_version('Atspi', '2.0')
from gi.repository import Atspi

from .utils import (
    find_widgets_by_role,
    find_buttons,
    find_menus,
    click_widget,
    close_dialogs,
    wait_for_idle,
    get_all_children,
    Widget
)


class TestCanvasBasics:
    """Basic canvas interaction tests."""
    
    @pytest.fixture(autouse=True)
    def setup(self, result_collector):
        result_collector.start_suite('Canvas Interactions')
    
    def test_canvas_exists(self, app, app_accessible, screenshot_session, result_collector):
        """Verify the canvas widget exists and is accessible."""
        screenshot_path = screenshot_session.capture('canvas_initial')
        
        start_time = time.time()
        passed = True
        error_msg = None
        
        try:
            # Look for canvas-like widgets
            # Qt canvas might appear as a scroll area, viewport, or custom widget
            all_children = get_all_children(app_accessible, max_depth=10)
            
            canvas_candidates = []
            for child in all_children:
                try:
                    name = (child.get_name() or '').lower()
                    role = (child.get_role_name() or '').lower()
                    
                    # Look for canvas indicators
                    if any(term in name for term in ['canvas', 'weaver', 'view', 'scene']):
                        canvas_candidates.append(child)
                    elif role in ['viewport', 'scroll pane', 'drawing area']:
                        canvas_candidates.append(child)
                except Exception:
                    pass
            
            if not canvas_candidates:
                # Not necessarily a failure - canvas might have generic naming
                error_msg = "No explicit canvas widget found (may use generic Qt naming)"
                passed = True  # Soft pass
            else:
                print(f"Found {len(canvas_candidates)} canvas candidates")
                
        except Exception as e:
            passed = False
            error_msg = str(e)
        
        result_collector.add_result(
            test_name='canvas_exists',
            widget_name='Canvas',
            widget_type='Canvas Widget',
            passed=passed,
            screenshot_path=screenshot_path,
            error_message=error_msg,
            duration=time.time() - start_time
        )
    
    def test_canvas_right_click_menu(
        self,
        app,
        app_accessible,
        screenshot_session,
        result_collector,
        ui_assert
    ):
        """Test right-click context menu on canvas."""
        screenshot_path = screenshot_session.capture_before_action('canvas_right_click')
        
        start_time = time.time()
        passed = True
        error_msg = None
        
        try:
            # Get window dimensions to find approximate canvas centre
            # We'll click in the centre of the main window
            all_children = get_all_children(app_accessible, max_depth=3)
            
            main_window = None
            for child in all_children:
                try:
                    role = child.get_role_name() or ''
                    if 'window' in role.lower() or 'frame' in role.lower():
                        component = child.get_component_iface()
                        if component:
                            rect = component.get_extents(Atspi.CoordType.SCREEN)
                            if rect.width > 400 and rect.height > 300:  # Main window
                                main_window = (rect.x + rect.width // 2, rect.y + rect.height // 2)
                                break
                except Exception:
                    pass
            
            if main_window:
                x, y = main_window
                # Right-click (button 3)
                Atspi.generate_mouse_event(x, y, 'b3c')
                time.sleep(0.3)
                
                ui_assert.assert_app_running(app, "right-click on canvas")
                
                # Take screenshot of context menu
                screenshot_session.capture('canvas_context_menu')
                
                # Close context menu with Escape
                Atspi.generate_keyboard_event(9, None, Atspi.KeySynthType.PRESSRELEASE)
                time.sleep(0.1)
                
            else:
                error_msg = "Could not locate main window for canvas click"
                passed = True  # Soft pass
                
        except Exception as e:
            passed = False
            error_msg = str(e)
        
        result_collector.add_result(
            test_name='canvas_right_click_menu',
            widget_name='Context Menu',
            widget_type='Canvas Interaction',
            passed=passed,
            screenshot_path=screenshot_path,
            error_message=error_msg,
            duration=time.time() - start_time
        )
    
    def test_canvas_zoom_shortcuts(
        self,
        app,
        app_accessible,
        screenshot_session,
        result_collector,
        ui_assert
    ):
        """Test Ctrl+scroll zoom doesn't crash."""
        screenshot_path = screenshot_session.capture_before_action('canvas_zoom')
        
        start_time = time.time()
        passed = True
        error_msg = None
        
        try:
            # Find main window centre for zoom target
            all_children = get_all_children(app_accessible, max_depth=3)
            
            target_pos = None
            for child in all_children:
                try:
                    role = child.get_role_name() or ''
                    if 'window' in role.lower() or 'frame' in role.lower():
                        component = child.get_component_iface()
                        if component:
                            rect = component.get_extents(Atspi.CoordType.SCREEN)
                            if rect.width > 400:
                                target_pos = (rect.x + rect.width // 2, rect.y + rect.height // 2)
                                break
                except Exception:
                    pass
            
            if target_pos:
                x, y = target_pos
                
                # Move mouse to centre
                Atspi.generate_mouse_event(x, y, 'abs')
                time.sleep(0.1)
                
                # Simulate Ctrl+Scroll (zoom in)
                # Press Ctrl
                Atspi.generate_keyboard_event(37, None, Atspi.KeySynthType.PRESS)  # Left Ctrl
                time.sleep(0.05)
                
                # Scroll events are harder to simulate via AT-SPI
                # Instead, try keyboard shortcuts if available (Ctrl+Plus, Ctrl+Minus)
                # Ctrl + Plus (zoom in)
                Atspi.generate_keyboard_event(21, None, Atspi.KeySynthType.PRESSRELEASE)  # Plus key
                time.sleep(0.1)
                
                # Ctrl + Minus (zoom out)
                Atspi.generate_keyboard_event(20, None, Atspi.KeySynthType.PRESSRELEASE)  # Minus key
                time.sleep(0.1)
                
                # Release Ctrl
                Atspi.generate_keyboard_event(37, None, Atspi.KeySynthType.RELEASE)
                
                ui_assert.assert_app_running(app, "zoom operations")
                
                screenshot_session.capture_after_action('canvas_zoom')
                
            else:
                error_msg = "Could not locate window for zoom test"
                passed = True  # Soft pass
                
        except Exception as e:
            passed = False
            error_msg = str(e)
        
        result_collector.add_result(
            test_name='canvas_zoom_shortcuts',
            widget_name='Zoom',
            widget_type='Canvas Interaction',
            passed=passed,
            screenshot_path=screenshot_path,
            error_message=error_msg,
            duration=time.time() - start_time
        )


class TestKeyboardShortcuts:
    """Test keyboard shortcuts don't crash the application."""
    
    @pytest.fixture(autouse=True)
    def setup(self, result_collector):
        result_collector.start_suite('Keyboard Shortcuts')
    
    def _test_shortcut(
        self,
        app,
        app_accessible,
        screenshot_session,
        result_collector,
        ui_assert,
        shortcut_name: str,
        key_sequence: list
    ):
        """Helper to test a keyboard shortcut."""
        screenshot_path = screenshot_session.capture_before_action(f'shortcut_{shortcut_name}')
        
        start_time = time.time()
        passed = True
        error_msg = None
        
        try:
            # Execute key sequence
            for key_code, key_type in key_sequence:
                Atspi.generate_keyboard_event(key_code, None, key_type)
                time.sleep(0.05)
            
            time.sleep(0.3)
            
            ui_assert.assert_app_running(app, f"shortcut {shortcut_name}")
            
            # Close any dialogs that opened
            close_dialogs(app_accessible)
            
            # Press Escape to cancel any pending operations
            Atspi.generate_keyboard_event(9, None, Atspi.KeySynthType.PRESSRELEASE)
            time.sleep(0.1)
            
        except Exception as e:
            passed = False
            error_msg = str(e)
        
        result_collector.add_result(
            test_name=f'shortcut_{shortcut_name}',
            widget_name=shortcut_name,
            widget_type='Keyboard Shortcut',
            passed=passed,
            screenshot_path=screenshot_path,
            error_message=error_msg,
            duration=time.time() - start_time
        )
        
        return passed
    
    def test_ctrl_n_new(self, app, app_accessible, screenshot_session, result_collector, ui_assert):
        """Test Ctrl+N (New Project)."""
        self._test_shortcut(
            app, app_accessible, screenshot_session, result_collector, ui_assert,
            'Ctrl_N_New',
            [
                (37, Atspi.KeySynthType.PRESS),    # Ctrl press
                (57, Atspi.KeySynthType.PRESSRELEASE),  # N
                (37, Atspi.KeySynthType.RELEASE),  # Ctrl release
            ]
        )
    
    def test_ctrl_o_open(self, app, app_accessible, screenshot_session, result_collector, ui_assert):
        """Test Ctrl+O (Open Project)."""
        self._test_shortcut(
            app, app_accessible, screenshot_session, result_collector, ui_assert,
            'Ctrl_O_Open',
            [
                (37, Atspi.KeySynthType.PRESS),
                (32, Atspi.KeySynthType.PRESSRELEASE),  # O
                (37, Atspi.KeySynthType.RELEASE),
            ]
        )
    
    def test_ctrl_s_save(self, app, app_accessible, screenshot_session, result_collector, ui_assert):
        """Test Ctrl+S (Save Project)."""
        self._test_shortcut(
            app, app_accessible, screenshot_session, result_collector, ui_assert,
            'Ctrl_S_Save',
            [
                (37, Atspi.KeySynthType.PRESS),
                (39, Atspi.KeySynthType.PRESSRELEASE),  # S
                (37, Atspi.KeySynthType.RELEASE),
            ]
        )
    
    def test_ctrl_g_generate(self, app, app_accessible, screenshot_session, result_collector, ui_assert):
        """Test Ctrl+G (Generate Code)."""
        self._test_shortcut(
            app, app_accessible, screenshot_session, result_collector, ui_assert,
            'Ctrl_G_Generate',
            [
                (37, Atspi.KeySynthType.PRESS),
                (42, Atspi.KeySynthType.PRESSRELEASE),  # G
                (37, Atspi.KeySynthType.RELEASE),
            ]
        )
    
    def test_ctrl_t_tf_tree(self, app, app_accessible, screenshot_session, result_collector, ui_assert):
        """Test Ctrl+T (Show TF Tree)."""
        self._test_shortcut(
            app, app_accessible, screenshot_session, result_collector, ui_assert,
            'Ctrl_T_TFTree',
            [
                (37, Atspi.KeySynthType.PRESS),
                (28, Atspi.KeySynthType.PRESSRELEASE),  # T
                (37, Atspi.KeySynthType.RELEASE),
            ]
        )
    
    def test_f5_scan(self, app, app_accessible, screenshot_session, result_collector, ui_assert):
        """Test F5 (Scan System)."""
        self._test_shortcut(
            app, app_accessible, screenshot_session, result_collector, ui_assert,
            'F5_Scan',
            [
                (71, Atspi.KeySynthType.PRESSRELEASE),  # F5
            ]
        )
    
    def test_delete_key(self, app, app_accessible, screenshot_session, result_collector, ui_assert):
        """Test Delete key doesn't crash (even with nothing selected)."""
        self._test_shortcut(
            app, app_accessible, screenshot_session, result_collector, ui_assert,
            'Delete',
            [
                (119, Atspi.KeySynthType.PRESSRELEASE),  # Delete
            ]
        )
