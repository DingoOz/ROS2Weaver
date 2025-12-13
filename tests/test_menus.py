"""
Menu interaction smoke tests for ROS2Weaver.

Tests that all menu items can be clicked without causing crashes or hangs.
Each menu item is clicked, and the test verifies:
1. The application doesn't crash
2. The application responds within the timeout
3. Any dialogs that open can be dismissed

Screenshots are captured before each interaction for the test report.
"""

import time
import pytest

from .utils import (
    find_menu_bars,
    find_menus,
    click_widget,
    expand_menu,
    close_dialogs,
    wait_for_idle,
    get_all_children,
    find_widgets_by_role,
    Widget,
    AccessibilityError
)


class TestMenuInteractions:
    """Test suite for menu interactions."""
    
    @pytest.fixture(autouse=True)
    def setup(self, result_collector):
        """Setup test suite in result collector."""
        result_collector.start_suite('Menu Interactions')
    
    def test_discover_menus(self, app, app_accessible, screenshot_session, result_collector):
        """Discover all menu items in the application."""
        # Take initial screenshot
        screenshot_session.capture('initial_state')
        
        # Find top-level menu bar items
        menu_bars = find_menu_bars(app_accessible)
        
        assert len(menu_bars) > 0, "No menus found in application"
        
        menu_names = [m.name for m in menu_bars if m.name]
        print(f"Found menus: {menu_names}")
        
        result_collector.add_result(
            test_name='discover_menus',
            widget_name='Menu Bar',
            widget_type='Discovery',
            passed=True,
            screenshot_path=screenshot_session.screenshots[-1]['path'] if screenshot_session.screenshots else None,
            duration=0.0
        )
    
    def test_click_all_menu_items(
        self,
        app,
        app_accessible,
        screenshot_session,
        result_collector,
        ui_assert,
        interaction_timeout
    ):
        """Click each menu item and verify no crash or hang."""
        # Find top-level menus
        menu_bars = find_menu_bars(app_accessible)
        
        if not menu_bars:
            pytest.skip("No menus found")
        
        tested_count = 0
        
        for menu in menu_bars:
            if not menu.name:
                continue
            
            print(f"\nTesting menu: {menu.name}")
            
            # Take screenshot before expanding menu
            screenshot_path = screenshot_session.capture_before_action(f'menu_{menu.name}')
            
            start_time = time.time()
            passed = True
            error_msg = None
            
            try:
                # Expand the menu
                expand_menu(menu)
                time.sleep(0.3)  # Wait for menu to appear
                
                # Find menu items within this menu
                menu_items = find_menus(app_accessible)
                related_items = [
                    item for item in menu_items 
                    if item.name and menu.name.lower() in ' > '.join(item.path).lower()
                ]
                
                print(f"  Found {len(related_items)} items in {menu.name}")
                
                # Click each menu item
                for item in related_items:
                    if not item.name:
                        continue
                    
                    item_screenshot = screenshot_session.capture_before_action(
                        f'menuitem_{menu.name}_{item.name}'
                    )
                    
                    item_start = time.time()
                    item_passed = True
                    item_error = None
                    
                    try:
                        print(f"    Clicking: {item.name}")
                        click_widget(item)
                        
                        # Wait for response
                        time.sleep(0.2)
                        
                        # Check for crash
                        ui_assert.assert_app_running(app, f"clicking menu item '{item.name}'")
                        
                        # Wait for app to be responsive
                        ui_assert.assert_responsive(app, interaction_timeout)
                        
                        # Close any dialogs that opened
                        close_dialogs(app_accessible)
                        time.sleep(0.1)
                        
                    except Exception as e:
                        item_passed = False
                        item_error = str(e)
                        print(f"      FAILED: {e}")
                    
                    result_collector.add_result(
                        test_name=f'click_menu_{menu.name}_{item.name}',
                        widget_name=item.name,
                        widget_type='Menu Item',
                        passed=item_passed,
                        screenshot_path=item_screenshot,
                        error_message=item_error,
                        duration=time.time() - item_start
                    )
                    
                    tested_count += 1
                
                # Close the menu by pressing Escape or clicking elsewhere
                try:
                    import gi
                    gi.require_version('Atspi', '2.0')
                    from gi.repository import Atspi
                    Atspi.generate_keyboard_event(9, None, Atspi.KeySynthType.PRESSRELEASE)
                    time.sleep(0.1)
                except Exception:
                    pass
                
            except Exception as e:
                passed = False
                error_msg = str(e)
                print(f"  Menu test failed: {e}")
            
            duration = time.time() - start_time
            
            result_collector.add_result(
                test_name=f'expand_menu_{menu.name}',
                widget_name=menu.name,
                widget_type='Menu',
                passed=passed,
                screenshot_path=screenshot_path,
                error_message=error_msg,
                duration=duration
            )
        
        print(f"\nTested {tested_count} menu items")
        assert tested_count > 0, "No menu items were tested"


class TestFileMenu:
    """Specific tests for File menu operations."""
    
    @pytest.fixture(autouse=True)
    def setup(self, result_collector):
        result_collector.start_suite('File Menu')
    
    def test_file_menu_new(self, app, app_accessible, screenshot_session, result_collector, ui_assert):
        """Test File > New action."""
        screenshot_path = screenshot_session.capture('before_file_new')
        
        start_time = time.time()
        passed = True
        error_msg = None
        
        try:
            # Find and click File menu
            menus = find_menu_bars(app_accessible)
            file_menu = next((m for m in menus if m.name and 'file' in m.name.lower()), None)
            
            if not file_menu:
                pytest.skip("File menu not found")
            
            expand_menu(file_menu)
            time.sleep(0.2)
            
            # Find New menu item
            menu_items = find_menus(app_accessible)
            new_item = next((m for m in menu_items if m.name and 'new' in m.name.lower()), None)
            
            if new_item:
                click_widget(new_item)
                time.sleep(0.3)
                
                ui_assert.assert_app_running(app, "File > New")
                close_dialogs(app_accessible)
            else:
                pytest.skip("New menu item not found")
                
        except Exception as e:
            passed = False
            error_msg = str(e)
        
        result_collector.add_result(
            test_name='file_menu_new',
            widget_name='New',
            widget_type='Menu Item',
            passed=passed,
            screenshot_path=screenshot_path,
            error_message=error_msg,
            duration=time.time() - start_time
        )
        
        assert passed, error_msg


class TestEditMenu:
    """Specific tests for Edit menu operations."""
    
    @pytest.fixture(autouse=True)
    def setup(self, result_collector):
        result_collector.start_suite('Edit Menu')
    
    def test_edit_menu_exists(self, app, app_accessible, screenshot_session, result_collector):
        """Verify Edit menu exists and can be expanded."""
        screenshot_path = screenshot_session.capture('before_edit_menu')
        
        start_time = time.time()
        passed = True
        error_msg = None
        
        try:
            menus = find_menu_bars(app_accessible)
            edit_menu = next((m for m in menus if m.name and 'edit' in m.name.lower()), None)
            
            if edit_menu:
                expand_menu(edit_menu)
                time.sleep(0.2)
                
                # Press Escape to close
                import gi
                gi.require_version('Atspi', '2.0')
                from gi.repository import Atspi
                Atspi.generate_keyboard_event(9, None, Atspi.KeySynthType.PRESSRELEASE)
            else:
                # Not a failure - Edit menu may not exist
                passed = True
                error_msg = "Edit menu not present (not required)"
                
        except Exception as e:
            passed = False
            error_msg = str(e)
        
        result_collector.add_result(
            test_name='edit_menu_exists',
            widget_name='Edit',
            widget_type='Menu',
            passed=passed,
            screenshot_path=screenshot_path,
            error_message=error_msg,
            duration=time.time() - start_time
        )


class TestROS2Menu:
    """Tests for ROS2-specific menu operations."""
    
    @pytest.fixture(autouse=True)
    def setup(self, result_collector):
        result_collector.start_suite('ROS2 Menu')
    
    def test_ros2_menu_items(self, app, app_accessible, screenshot_session, result_collector, ui_assert):
        """Test ROS2 menu items (Show TF Tree, Topic Viewer, etc.)."""
        screenshot_path = screenshot_session.capture('before_ros2_menu')
        
        start_time = time.time()
        passed = True
        error_msg = None
        
        try:
            menus = find_menu_bars(app_accessible)
            ros2_menu = next((m for m in menus if m.name and 'ros' in m.name.lower()), None)
            
            if not ros2_menu:
                pytest.skip("ROS2 menu not found")
            
            expand_menu(ros2_menu)
            time.sleep(0.2)
            
            # Find ROS2-specific items
            menu_items = find_menus(app_accessible)
            ros2_items = [
                m for m in menu_items 
                if m.name and any(
                    keyword in m.name.lower() 
                    for keyword in ['tf', 'topic', 'scan', 'monitor', 'discovery']
                )
            ]
            
            for item in ros2_items:
                item_screenshot = screenshot_session.capture_before_action(f'ros2_{item.name}')
                
                try:
                    click_widget(item)
                    time.sleep(0.5)
                    
                    ui_assert.assert_app_running(app, f"ROS2 > {item.name}")
                    
                    # Close any panels or dialogs
                    close_dialogs(app_accessible)
                    
                    result_collector.add_result(
                        test_name=f'ros2_menu_{item.name}',
                        widget_name=item.name,
                        widget_type='ROS2 Menu Item',
                        passed=True,
                        screenshot_path=item_screenshot,
                        duration=0.5
                    )
                    
                except Exception as e:
                    result_collector.add_result(
                        test_name=f'ros2_menu_{item.name}',
                        widget_name=item.name,
                        widget_type='ROS2 Menu Item',
                        passed=False,
                        screenshot_path=item_screenshot,
                        error_message=str(e)
                    )
            
            # Close menu
            import gi
            gi.require_version('Atspi', '2.0')
            from gi.repository import Atspi
            Atspi.generate_keyboard_event(9, None, Atspi.KeySynthType.PRESSRELEASE)
            
        except Exception as e:
            passed = False
            error_msg = str(e)
        
        result_collector.add_result(
            test_name='ros2_menu',
            widget_name='ROS2',
            widget_type='Menu',
            passed=passed,
            screenshot_path=screenshot_path,
            error_message=error_msg,
            duration=time.time() - start_time
        )
