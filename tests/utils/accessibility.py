"""
AT-SPI2 accessibility utilities for Qt5 widget discovery and interaction.

Provides functions to enumerate menus, buttons, and panels in the ros_weaver
application and interact with them programmatically.
"""

import gi
gi.require_version('Atspi', '2.0')
from gi.repository import Atspi
import time
from typing import Optional, List, Callable, Any
from dataclasses import dataclass


@dataclass
class Widget:
    """Represents a discovered UI widget."""
    name: str
    role: str
    accessible: Any  # Atspi.Accessible
    path: List[str]  # Hierarchy path for debugging
    
    def __str__(self) -> str:
        return f"{self.role}: {self.name} ({' > '.join(self.path)})"


class AccessibilityError(Exception):
    """Raised when accessibility operations fail."""
    pass


class AppNotFoundError(AccessibilityError):
    """Raised when the target application cannot be found."""
    pass


class WidgetNotFoundError(AccessibilityError):
    """Raised when a specific widget cannot be found."""
    pass


class InteractionTimeoutError(AccessibilityError):
    """Raised when an interaction times out (potential hang)."""
    pass


def initialise_atspi() -> None:
    """Initialise the AT-SPI2 accessibility framework."""
    Atspi.init()


def find_application(app_name: str, timeout: float = 10.0) -> Any:
    """
    Find an application by name in the accessibility tree.

    Args:
        app_name: Name of the application (e.g., 'ros_weaver')
        timeout: Maximum time to wait for application to appear

    Returns:
        Atspi.Accessible for the application

    Raises:
        AppNotFoundError: If application not found within timeout
    """
    start_time = time.time()

    while time.time() - start_time < timeout:
        desktop = Atspi.get_desktop(0)
        candidates = []
        for i in range(desktop.get_child_count()):
            app = desktop.get_child_at_index(i)
            if app and app_name.lower() in (app.get_name() or '').lower():
                candidates.append(app)

        # Prefer the app with children (the actual UI), not empty registration
        for app in candidates:
            if app.get_child_count() > 0:
                return app

        # If no app with children, return first match
        if candidates:
            return candidates[0]

        time.sleep(0.2)

    raise AppNotFoundError(f"Application '{app_name}' not found within {timeout}s")


def get_all_children(accessible: Any, max_depth: int = 20) -> List[Any]:
    """
    Recursively get all children of an accessible object.
    
    Args:
        accessible: Root Atspi.Accessible object
        max_depth: Maximum recursion depth
        
    Returns:
        Flat list of all descendant Atspi.Accessible objects
    """
    children = []
    
    def recurse(obj: Any, depth: int):
        if depth > max_depth or obj is None:
            return
        children.append(obj)
        try:
            count = obj.get_child_count()
            for i in range(count):
                child = obj.get_child_at_index(i)
                if child:
                    recurse(child, depth + 1)
        except Exception:
            pass  # Some objects may not support child enumeration
    
    recurse(accessible, 0)
    return children


def find_widgets_by_role(
    app: Any,
    role_name: str,
    name_filter: Optional[Callable[[str], bool]] = None
) -> List[Widget]:
    """
    Find all widgets matching a specific role.
    
    Args:
        app: Application Atspi.Accessible
        role_name: AT-SPI2 role name (e.g., 'push button', 'menu item')
        name_filter: Optional function to filter by widget name
        
    Returns:
        List of Widget objects matching criteria
    """
    widgets = []
    all_children = get_all_children(app)
    
    for child in all_children:
        try:
            role = child.get_role_name()
            name = child.get_name() or ''
            
            if role and role_name.lower() in role.lower():
                if name_filter is None or name_filter(name):
                    path = _get_widget_path(child)
                    widgets.append(Widget(
                        name=name,
                        role=role,
                        accessible=child,
                        path=path
                    ))
        except Exception:
            continue
    
    return widgets


def _get_widget_path(accessible: Any, max_depth: int = 10) -> List[str]:
    """Build the hierarchy path for a widget."""
    path = []
    current = accessible
    depth = 0
    
    while current and depth < max_depth:
        name = current.get_name() or current.get_role_name() or '?'
        path.insert(0, name)
        try:
            current = current.get_parent()
        except Exception:
            break
        depth += 1
    
    return path


def find_menus(app: Any) -> List[Widget]:
    """Find all menu items in the application."""
    return find_widgets_by_role(app, 'menu item')


def find_menu_bars(app: Any) -> List[Widget]:
    """Find all menu bar items (top-level menus)."""
    return find_widgets_by_role(app, 'menu')


def find_buttons(app: Any) -> List[Widget]:
    """Find all push buttons in the application."""
    return find_widgets_by_role(app, 'push button')


def find_toggle_buttons(app: Any) -> List[Widget]:
    """Find all toggle/check buttons in the application."""
    widgets = find_widgets_by_role(app, 'toggle button')
    widgets.extend(find_widgets_by_role(app, 'check box'))
    return widgets


def find_tool_buttons(app: Any) -> List[Widget]:
    """Find all toolbar buttons."""
    # Qt toolbar buttons often appear as push buttons within a tool bar
    all_buttons = find_buttons(app)
    tool_buttons = []
    
    for btn in all_buttons:
        # Check if parent is a toolbar
        try:
            parent = btn.accessible.get_parent()
            if parent and 'tool' in (parent.get_role_name() or '').lower():
                tool_buttons.append(btn)
        except Exception:
            pass
    
    return tool_buttons if tool_buttons else all_buttons


def find_panels(app: Any) -> List[Widget]:
    """Find all panel/dock widgets."""
    panels = find_widgets_by_role(app, 'panel')
    panels.extend(find_widgets_by_role(app, 'dock'))
    panels.extend(find_widgets_by_role(app, 'frame'))
    return panels


def click_widget(widget: Widget, timeout: float = 5.0) -> bool:
    """
    Click a widget using the accessibility action interface.
    
    Args:
        widget: Widget to click
        timeout: Maximum time to wait for click to complete
        
    Returns:
        True if click succeeded
        
    Raises:
        InteractionTimeoutError: If click doesn't complete within timeout
    """
    try:
        action = widget.accessible.get_action_iface()
        if action:
            # Find and execute the click/activate action
            for i in range(action.get_n_actions()):
                action_name = action.get_action_name(i)
                if action_name in ('click', 'activate', 'press', 'invoke', ''):
                    action.do_action(i)
                    time.sleep(0.1)  # Brief pause for UI to respond
                    return True
        
        # Fallback: try to get component and simulate click at centre
        component = widget.accessible.get_component_iface()
        if component:
            rect = component.get_extents(Atspi.CoordType.SCREEN)
            x = rect.x + rect.width // 2
            y = rect.y + rect.height // 2
            Atspi.generate_mouse_event(x, y, 'b1c')
            time.sleep(0.1)
            return True
            
    except Exception as e:
        raise AccessibilityError(f"Failed to click widget {widget.name}: {e}")
    
    return False


def expand_menu(widget: Widget) -> bool:
    """
    Expand a menu to show its items.
    
    Args:
        widget: Menu widget to expand
        
    Returns:
        True if menu expanded successfully
    """
    try:
        # Try selection interface first
        selection = widget.accessible.get_selection_iface()
        if selection:
            selection.select_child(0)
            time.sleep(0.2)
            return True
        
        # Fallback to click
        return click_widget(widget)
    except Exception:
        return click_widget(widget)


def close_dialogs(app: Any) -> int:
    """
    Close any open dialogs by pressing Escape or clicking close buttons.
    
    Args:
        app: Application Atspi.Accessible
        
    Returns:
        Number of dialogs closed
    """
    closed = 0
    dialogs = find_widgets_by_role(app, 'dialog')
    dialogs.extend(find_widgets_by_role(app, 'alert'))
    
    for dialog in dialogs:
        try:
            # Look for close button
            children = get_all_children(dialog.accessible)
            for child in children:
                name = (child.get_name() or '').lower()
                role = child.get_role_name() or ''
                
                if 'button' in role and name in ('close', 'cancel', 'ok', 'no', 'yes'):
                    close_widget = Widget(
                        name=child.get_name(),
                        role=role,
                        accessible=child,
                        path=[]
                    )
                    click_widget(close_widget)
                    closed += 1
                    break
            
            # Fallback: send Escape key
            Atspi.generate_keyboard_event(9, None, Atspi.KeySynthType.PRESSRELEASE)  # Escape
            time.sleep(0.1)
            
        except Exception:
            pass
    
    return closed


def get_window_state(app: Any) -> dict:
    """
    Get current window state information.
    
    Args:
        app: Application Atspi.Accessible
        
    Returns:
        Dictionary with window state info
    """
    state = {
        'window_count': 0,
        'dialog_count': 0,
        'focused_widget': None,
        'windows': []
    }
    
    try:
        all_children = get_all_children(app, max_depth=5)
        
        for child in all_children:
            role = child.get_role_name() or ''
            
            if 'window' in role.lower() or 'frame' in role.lower():
                state['window_count'] += 1
                state['windows'].append(child.get_name())
            elif 'dialog' in role.lower():
                state['dialog_count'] += 1
                
            # Check for focus
            try:
                state_set = child.get_state_set()
                if state_set.contains(Atspi.StateType.FOCUSED):
                    state['focused_widget'] = child.get_name()
            except Exception:
                pass
                
    except Exception:
        pass
    
    return state


def wait_for_idle(app: Any, timeout: float = 5.0, poll_interval: float = 0.1) -> bool:
    """
    Wait for application to become idle (no pending events).
    
    This is a heuristic based on checking if the widget tree is stable.
    
    Args:
        app: Application Atspi.Accessible
        timeout: Maximum time to wait
        poll_interval: Time between checks
        
    Returns:
        True if application appears idle, False if timeout
    """
    start_time = time.time()
    last_state = None
    stable_count = 0
    
    while time.time() - start_time < timeout:
        current_state = get_window_state(app)
        
        if current_state == last_state:
            stable_count += 1
            if stable_count >= 3:  # Stable for 3 consecutive checks
                return True
        else:
            stable_count = 0
            
        last_state = current_state
        time.sleep(poll_interval)
    
    return False
