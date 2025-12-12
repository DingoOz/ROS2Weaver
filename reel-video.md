# ROS2 Weaver Feature Demo Video Outline

A recommended sequence of feature demonstrations for creating a compelling video showcasing ROS2 Weaver's capabilities.

## Video Structure

**Target Duration**: 3-5 minutes
**Tone**: Professional, showcasing productivity gains
**Music**: Upbeat, tech-focused background track

---

## Scene 1: Opening Hook (10-15 seconds)

**Visual**: Quick montage of the finished app in action
- Drag-drop package blocks
- Animated connections being created
- Code being generated
- ROS logs streaming

**Text Overlay**: "ROS2 Weaver - Visual ROS2 Package Development"

---

## Scene 2: The Problem Statement (15-20 seconds)

**Visual**: Split screen showing:
- Left: Terminal with manual ROS2 package creation commands
- Right: Multiple text editor windows with CMakeLists.txt, package.xml, node code

**Voiceover/Text**: "Creating ROS2 packages manually means juggling multiple files, remembering syntax, and hoping you don't miss a dependency..."

---

## Scene 3: App Launch & Overview (20-30 seconds)

**Demo Sequence**:
1. Launch ROS2 Weaver from terminal (`ros2 run ros_weaver ros_weaver`)
2. Pan across the main interface showing:
   - Package Browser (left panel)
   - Central canvas workspace
   - Properties/Parameters panel (right)
   - Output panel at bottom

**Highlight**: Clean, professional Qt-based interface

---

## Scene 4: Node Templates & Drag-Drop (30-45 seconds)

**Demo Sequence**:
1. Expand "Node Templates" in Package Browser
2. Double-click "publisher_node" - watch it appear on canvas
3. Add "subscriber_node"
4. Add "relay_node"
5. Show the blocks animating into place

**Highlight**: "Create nodes in seconds, not minutes"

---

## Scene 5: Visual Connections (30-40 seconds)

**Demo Sequence**:
1. Click and drag from publisher output pin to relay input
2. Watch the curved connection line animate
3. Connect relay to subscriber
4. Show the visual data flow representation

**Highlight**: "See your ROS2 architecture at a glance"

---

## Scene 6: Node Groups - Unreal Engine Style (20-30 seconds)

**Demo Sequence**:
1. Select multiple nodes (Ctrl+click or drag selection)
2. Right-click → "Group Selected Nodes"
3. Type a group name like "Navigation Stack"
4. Show the colored comment box surrounding nodes
5. Resize and reposition the group

**Highlight**: "Organize complex systems like a pro"

---

## Scene 7: Parameter Dashboard (30-45 seconds)

**Demo Sequence**:
1. Click on a node to select it
2. Show Parameters tab populating with node parameters
3. Edit a parameter value inline
4. Show parameter groups expanding/collapsing
5. Demonstrate YAML source selector (switch between block params and YAML files)

**Highlight**: "Configure parameters visually - no more YAML typos"

---

## Scene 8: TurtleBot3 Example Project (20-30 seconds)

**Demo Sequence**:
1. File → Examples → TurtleBot3 Navigation
2. Watch the canvas populate with Nav2 stack components
3. Pan around showing:
   - AMCL
   - Map Server
   - Planner Server
   - Controller Server
   - BT Navigator
4. Show the loaded YAML parameters

**Highlight**: "Start with real-world examples"

---

## Scene 9: Code Generation (45-60 seconds)

**Demo Sequence**:
1. File → Generate ROS2 Package
2. Select output directory
3. Enter package name
4. Watch the progress bar fill
5. Show the Output panel with generation progress:
   - "Creating directory structure..."
   - "Generating CMakeLists.txt..."
   - "Generating package.xml..."
   - "Generating node source files..."
   - "Done!"
6. Quick preview of generated files in VS Code

**Highlight**: "Complete, buildable ROS2 packages in seconds"

---

## Scene 10: ROS Logger Integration (30-40 seconds)

**Demo Sequence**:
1. Switch to "ROS Logs" tab in Output panel
2. Click "Start Listening"
3. Show connection status changing to "Connected to /rosout"
4. Run a ROS2 node in background that generates logs
5. Watch logs stream in real-time with color coding:
   - Gray for DEBUG
   - White for INFO
   - Orange for WARN
   - Red for ERROR
6. Demonstrate filtering by severity level
7. Filter by node name

**Highlight**: "Debug without leaving the IDE"

---

## Scene 11: Integrated Terminal (20-30 seconds)

**Demo Sequence**:
1. Switch to "Terminal" tab
2. Type `ros2 topic list`
3. Show output appearing
4. Run `colcon build --packages-select <generated_package>`
5. Show build output streaming

**Highlight**: "Full terminal access built-in"

---

## Scene 12: Complete Workflow Demo (45-60 seconds)

**Fast-paced demo showing full workflow**:
1. Create new project
2. Add 3 nodes rapidly
3. Connect them
4. Group them
5. Configure a parameter
6. Generate code
7. Build in terminal tab
8. Show ROS logs from running the nodes

**Text Overlay**: "Design → Generate → Build → Run"

---

## Scene 13: Closing (15-20 seconds)

**Visual**: Logo animation with feature bullets appearing:
- Visual Node Editor
- One-Click Code Generation
- Real-Time ROS Logs
- Parameter Configuration
- Integrated Terminal

**Call to Action**:
- GitHub link
- "Star on GitHub"
- "Open Source - Contribute Today"

---

## Technical Notes for Recording

### Screen Resolution
- Record at 1920x1080 or 2560x1440
- Use 60fps for smooth animations

### Cursor Visibility
- Use a highlighted cursor
- Consider zoom effects on clicks

### Window Setup
- Maximize ROS Weaver to fill screen
- Close unnecessary panels initially, reveal them as features

### Hot Keys to Practice
- Ctrl+G: Generate code
- Ctrl+S: Save project
- F: Fit all nodes in view
- Delete: Remove selected items

### Sample Commands to Pre-type
```bash
# For Terminal demo
ros2 topic list
ros2 node list
colcon build --packages-select my_ros_package
ros2 run my_ros_package publisher_node
```

### ROS2 Nodes to Have Running (for logs demo)
```bash
# In separate terminal before recording
ros2 run demo_nodes_cpp talker
ros2 run demo_nodes_cpp listener
```

---

## B-Roll Suggestions

- Close-up of connection lines animating
- Parameter values being typed
- Progress bar filling during generation
- Log messages scrolling
- Code appearing in generated files
- Node blocks being dragged around canvas

---

## Alternative Short-Form Versions

### 60-Second TikTok/Reel Version
1. Hook: "Stop writing ROS2 boilerplate" (3s)
2. Drag node (5s)
3. Connect (5s)
4. Generate (10s)
5. Show code (5s)
6. Build & run (15s)
7. Logs streaming (10s)
8. CTA (7s)

### 30-Second Twitter/X Version
1. Hook (3s)
2. Speed-run: add nodes, connect, generate (15s)
3. Show output (7s)
4. Logo + CTA (5s)
