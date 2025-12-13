"""
Panel and button interaction smoke tests for ROS2Weaver.

Tests that all toolbar buttons and panel controls can be clicked without
causing crashes or hangs.
"""

import time
import pytest

from .utils import (
    find_buttons,
    find_toggle_buttons,
    find_tool_buttons,
    find_panels,
    find_widgets_by_role,
    click_widget,
    close_dialogs,
    wait_for_idle,
    Widget,
    AccessibilityError
)


class TestToolbarButtons:
    """Test suite for toolbar button interactions."""
    
    @pytest.fixture(autouse=True)
    def setup(self, result_collector):
        result_collector.start_suite('Toolbar Buttons')
    
    def test_discover_buttons(self, app, app_accessible, screenshot_session, result_collector):
        """Discover all buttons in the application."""
        screenshot_path = screenshot_session.capture('initial_button_state')
        
        buttons = find_buttons(app_accessible)
        toggle_buttons = find_toggle_buttons(app_accessible)
        
        all_buttons = buttons + toggle_buttons
        button_names = [b.name for b in all_buttons if b.name]
        
        print(f"Found {len(all_buttons)} buttons: {button_names[:20]}...")  # Limit output
        
        result_collector.add_result(
            test_name='discover_buttons',
            widget_name='All Buttons',
            widget_type='Discovery',
            passed=len(all_buttons) > 0,
            screenshot_path=screenshot_path,
            error_message=None if all_buttons else "No buttons found",
            duration=0.0
        )
        
        assert len(all_buttons) > 0, "No buttons found in application"
    
    def test_click_toolbar_buttons(
        self,
        app,
        app_accessible,
        screenshot_session,
        result_collector,
        ui_assert,
        interaction_timeout
    ):
        """Click each toolbar button and verify no crash or hang."""
        # Find toolbar buttons
        tool_buttons = find_tool_buttons(app_accessible)
        
        if not tool_buttons:
            # Fall back to all buttons if no toolbar-specific buttons found
            tool_buttons = find_buttons(app_accessible)
        
        # Filter to named buttons only
        named_buttons = [b for b in tool_buttons if b.name]
        
        if not named_buttons:
            pytest.skip("No named toolbar buttons found")
        
        tested_count = 0
        failed_count = 0
        
        for button in named_buttons[:30]:  # Limit to first 30 to avoid very long tests
            print(f"Testing button: {button.name}")
            
            screenshot_path = screenshot_session.capture_before_action(f'button_{button.name}')
            
            start_time = time.time()
            passed = True
            error_msg = None
            
            try:
                click_widget(button)
                time.sleep(0.2)
                
                # Check for crash
                ui_assert.assert_app_running(app, f"clicking button '{button.name}'")
                
                # Wait for app to respond
                wait_for_idle(app_accessible, timeout=interaction_timeout)
                
                # Close any dialogs
                close_dialogs(app_accessible)
                time.sleep(0.1)
                
            except Exception as e:
                passed = False
                error_msg = str(e)
                failed_count += 1
                print(f"  FAILED: {e}")
            
            result_collector.add_result(
                test_name=f'click_button_{button.name}',
                widget_name=button.name,
                widget_type='Toolbar Button',
                passed=passed,
                screenshot_path=screenshot_path,
                error_message=error_msg,
                duration=time.time() - start_time
            )
            
            tested_count += 1
        
        print(f"\nTested {tested_count} toolbar buttons, {failed_count} failed")


class TestToggleButtons:
    """Test suite for toggle/checkbox button interactions."""
    
    @pytest.fixture(autouse=True)
    def setup(self, result_collector):
        result_collector.start_suite('Toggle Buttons')
    
    def test_toggle_buttons(
        self,
        app,
        app_accessible,
        screenshot_session,
        result_collector,
        ui_assert
    ):
        """Test toggling each toggle button twice (on and off)."""
        toggle_buttons = find_toggle_buttons(app_accessible)
        
        named_toggles = [t for t in toggle_buttons if t.name]
        
        if not named_toggles:
            pytest.skip("No toggle buttons found")
        
        for toggle in named_toggles[:15]:  # Limit to first 15
            print(f"Testing toggle: {toggle.name}")
            
            screenshot_path = screenshot_session.capture_before_action(f'toggle_{toggle.name}')
            
            start_time = time.time()
            passed = True
            error_msg = None
            
            try:
                # Click to toggle on
                click_widget(toggle)
                time.sleep(0.2)
                ui_assert.assert_app_running(app, f"toggling '{toggle.name}' on")
                
                # Click to toggle off
                click_widget(toggle)
                time.sleep(0.2)
                ui_assert.assert_app_running(app, f"toggling '{toggle.name}' off")
                
                close_dialogs(app_accessible)
                
            except Exception as e:
                passed = False
                error_msg = str(e)
            
            result_collector.add_result(
                test_name=f'toggle_{toggle.name}',
                widget_name=toggle.name,
                widget_type='Toggle Button',
                passed=passed,
                screenshot_path=screenshot_path,
                error_message=error_msg,
                duration=time.time() - start_time
            )


class TestPanelControls:
    """Test suite for panel-specific controls."""
    
    @pytest.fixture(autouse=True)
    def setup(self, result_collector):
        result_collector.start_suite('Panel Controls')
    
    def test_discover_panels(self, app, app_accessible, screenshot_session, result_collector):
        """Discover all panels in the application."""
        screenshot_path = screenshot_session.capture('panel_discovery')
        
        panels = find_panels(app_accessible)
        panel_names = [p.name for p in panels if p.name]
        
        print(f"Found {len(panels)} panels: {panel_names}")
        
        result_collector.add_result(
            test_name='discover_panels',
            widget_name='All Panels',
            widget_type='Discovery',
            passed=True,
            screenshot_path=screenshot_path,
            duration=0.0
        )
    
    def test_output_panel_buttons(
        self,
        app,
        app_accessible,
        screenshot_session,
        result_collector,
        ui_assert
    ):
        """Test buttons in the Output Panel."""
        # Find output panel
        panels = find_panels(app_accessible)
        output_panel = next(
            (p for p in panels if p.name and 'output' in p.name.lower()),
            None
        )
        
        if not output_panel:
            pytest.skip("Output panel not found")
        
        # Find buttons within output panel
        from .utils import get_all_children
        panel_children = get_all_children(output_panel.accessible, max_depth=5)
        
        panel_buttons = []
        for child in panel_children:
            try:
                role = child.get_role_name() or ''
                if 'button' in role.lower():
                    name = child.get_name() or ''
                    if name:
                        panel_buttons.append(Widget(
                            name=name,
                            role=role,
                            accessible=child,
                            path=['Output Panel', name]
                        ))
            except Exception:
                pass
        
        for button in panel_buttons:
            screenshot_path = screenshot_session.capture_before_action(
                f'output_panel_{button.name}'
            )
            
            start_time = time.time()
            passed = True
            error_msg = None
            
            try:
                click_widget(button)
                time.sleep(0.2)
                ui_assert.assert_app_running(app, f"Output Panel > {button.name}")
                close_dialogs(app_accessible)
                
            except Exception as e:
                passed = False
                error_msg = str(e)
            
            result_collector.add_result(
                test_name=f'output_panel_{button.name}',
                widget_name=button.name,
                widget_type='Output Panel Button',
                passed=passed,
                screenshot_path=screenshot_path,
                error_message=error_msg,
                duration=time.time() - start_time
            )
    
    def test_topic_viewer_buttons(
        self,
        app,
        app_accessible,
        screenshot_session,
        result_collector,
        ui_assert
    ):
        """Test buttons in the Topic Viewer panel."""
        panels = find_panels(app_accessible)
        topic_panel = next(
            (p for p in panels if p.name and 'topic' in p.name.lower()),
            None
        )
        
        if not topic_panel:
            # Topic viewer might not be visible by default - that's OK
            result_collector.add_result(
                test_name='topic_viewer_buttons',
                widget_name='Topic Viewer',
                widget_type='Panel',
                passed=True,
                screenshot_path=screenshot_session.capture('no_topic_viewer'),
                error_message="Topic Viewer not visible (may require menu action)",
                duration=0.0
            )
            return
        
        # Similar logic to output panel...
        from .utils import get_all_children
        panel_children = get_all_children(topic_panel.accessible, max_depth=5)
        
        panel_buttons = []
        for child in panel_children:
            try:
                role = child.get_role_name() or ''
                if 'button' in role.lower():
                    name = child.get_name() or ''
                    if name:
                        panel_buttons.append(Widget(
                            name=name,
                            role=role,
                            accessible=child,
                            path=['Topic Viewer', name]
                        ))
            except Exception:
                pass
        
        for button in panel_buttons:
            screenshot_path = screenshot_session.capture_before_action(
                f'topic_viewer_{button.name}'
            )
            
            start_time = time.time()
            passed = True
            error_msg = None
            
            try:
                click_widget(button)
                time.sleep(0.3)
                ui_assert.assert_app_running(app, f"Topic Viewer > {button.name}")
                close_dialogs(app_accessible)
                
            except Exception as e:
                passed = False
                error_msg = str(e)
            
            result_collector.add_result(
                test_name=f'topic_viewer_{button.name}',
                widget_name=button.name,
                widget_type='Topic Viewer Button',
                passed=passed,
                screenshot_path=screenshot_path,
                error_message=error_msg,
                duration=time.time() - start_time
            )
    
    def test_tf_tree_buttons(
        self,
        app,
        app_accessible,
        screenshot_session,
        result_collector,
        ui_assert
    ):
        """Test buttons in the TF Tree panel."""
        panels = find_panels(app_accessible)
        tf_panel = next(
            (p for p in panels if p.name and ('tf' in p.name.lower() or 'tree' in p.name.lower())),
            None
        )
        
        if not tf_panel:
            result_collector.add_result(
                test_name='tf_tree_buttons',
                widget_name='TF Tree',
                widget_type='Panel',
                passed=True,
                screenshot_path=screenshot_session.capture('no_tf_tree'),
                error_message="TF Tree not visible (may require menu action)",
                duration=0.0
            )
            return
        
        # Find and test buttons in TF tree panel
        from .utils import get_all_children
        panel_children = get_all_children(tf_panel.accessible, max_depth=5)
        
        panel_buttons = []
        for child in panel_children:
            try:
                role = child.get_role_name() or ''
                if 'button' in role.lower():
                    name = child.get_name() or ''
                    if name:
                        panel_buttons.append(Widget(
                            name=name,
                            role=role,
                            accessible=child,
                            path=['TF Tree', name]
                        ))
            except Exception:
                pass
        
        for button in panel_buttons:
            screenshot_path = screenshot_session.capture_before_action(
                f'tf_tree_{button.name}'
            )
            
            start_time = time.time()
            passed = True
            error_msg = None
            
            try:
                click_widget(button)
                time.sleep(0.3)
                ui_assert.assert_app_running(app, f"TF Tree > {button.name}")
                close_dialogs(app_accessible)
                
            except Exception as e:
                passed = False
                error_msg = str(e)
            
            result_collector.add_result(
                test_name=f'tf_tree_{button.name}',
                widget_name=button.name,
                widget_type='TF Tree Button',
                passed=passed,
                screenshot_path=screenshot_path,
                error_message=error_msg,
                duration=time.time() - start_time
            )


class TestLiveMonitoring:
    """Test suite for live monitoring features."""
    
    @pytest.fixture(autouse=True)
    def setup(self, result_collector):
        result_collector.start_suite('Live Monitoring')
    
    def test_live_button_toggle(
        self,
        app,
        app_accessible,
        screenshot_session,
        result_collector,
        ui_assert
    ):
        """Test the Live monitoring toggle button."""
        # Find Live button
        buttons = find_buttons(app_accessible)
        live_button = next(
            (b for b in buttons if b.name and 'live' in b.name.lower()),
            None
        )
        
        if not live_button:
            toggle_buttons = find_toggle_buttons(app_accessible)
            live_button = next(
                (b for b in toggle_buttons if b.name and 'live' in b.name.lower()),
                None
            )
        
        if not live_button:
            result_collector.add_result(
                test_name='live_button_toggle',
                widget_name='Live',
                widget_type='Toggle Button',
                passed=True,
                screenshot_path=screenshot_session.capture('no_live_button'),
                error_message="Live button not found (may be in hidden panel)",
                duration=0.0
            )
            return
        
        screenshot_path = screenshot_session.capture_before_action('live_toggle')
        
        start_time = time.time()
        passed = True
        error_msg = None
        
        try:
            # Toggle on
            click_widget(live_button)
            time.sleep(1.0)  # Live features may take time to initialise
            ui_assert.assert_app_running(app, "enabling Live monitoring")
            
            # Toggle off
            click_widget(live_button)
            time.sleep(0.5)
            ui_assert.assert_app_running(app, "disabling Live monitoring")
            
        except Exception as e:
            passed = False
            error_msg = str(e)
        
        result_collector.add_result(
            test_name='live_button_toggle',
            widget_name='Live',
            widget_type='Toggle Button',
            passed=passed,
            screenshot_path=screenshot_path,
            error_message=error_msg,
            duration=time.time() - start_time
        )
