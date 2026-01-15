# Push Remote Skill

Push changes to remote repository while tracking release notes and suggesting releases when appropriate.

## Arguments

Optional: A brief description of the changes being pushed (e.g., "Add undo support for param dashboard").

## Instructions

When this skill is invoked ($ARGUMENTS):

### Step 1: Analyze Current Changes

1. Run `git status` to see all modified, added, and deleted files
2. Run `git diff --staged` and `git diff` to understand the changes
3. Run `git log --oneline -5` to see recent commit message style

### Step 2: Commit Changes

1. Stage all relevant changes with `git add`
2. Create a commit with a clear, descriptive message following the project's style
3. If $ARGUMENTS is provided, use it as the basis for the commit message
4. Do NOT include any AI attribution in the commit message (per project guidelines)

### Step 3: Update Release Notes

Read `CHANGELOG.md` and update the `## [Unreleased]` or next version section:

**Categorize changes as:**
- **Added**: New features or capabilities
- **Changed**: Changes to existing functionality
- **Fixed**: Bug fixes
- **Deprecated**: Features marked for removal
- **Removed**: Features that were removed
- **Security**: Security-related changes

Add a concise bullet point describing the change under the appropriate category.

**Example format:**
```markdown
## [1.2.0] - Unreleased

### Added
- Undo/redo support for parameter dashboard and remapping editor

### Fixed
- Parameter changes now properly integrate with undo system
```

### Step 4: Push to Remote

1. Push the commit to the remote repository: `git push`
2. If the push fails due to upstream changes, inform the user

### Step 5: Evaluate Release Readiness

Read the current `## [Unreleased]` section in CHANGELOG.md and count the items:

**Suggest a release when ANY of these conditions are met:**
- 5 or more items total across all categories
- 3 or more items in "Added" (new features)
- 1 or more items in "Security"
- A major bug fix that users need urgently

**Release suggestion output:**
```
Release Suggestion: The unreleased changes include X features, Y fixes.
Consider creating a release with the following steps:

1. Update version numbers:
   - CHANGELOG.md: Change "[Unreleased]" to "[X.Y.Z] - YYYY-MM-DD"
   - packaging/debian/changelog: Add new version entry
   - package.xml: Update <version> tag
   - CMakeLists.txt: Update VERSION if present

2. Create a git tag:
   git tag -a vX.Y.Z -m "Release X.Y.Z"
   git push origin vX.Y.Z

3. Build the .deb package:
   cd /home/dingo/Programming/ROS2Weaver
   ./build-deb.sh
   # Or manually:
   mkdir -p build-deb && cd build-deb
   cmake .. -DCMAKE_INSTALL_PREFIX=/usr -DCMAKE_BUILD_TYPE=Release
   make -j$(nproc)
   cpack -G DEB

4. The .deb will be created in the build-deb directory.
```

If not enough changes for a release, output:
```
Changes pushed. Current unreleased items: X features, Y fixes.
(Release typically suggested at 5+ total items or 3+ features)
```

### Step 6: Summary Output

Provide a summary:
- Commit hash and message
- Files changed
- Release notes entry added
- Push status
- Release recommendation (if applicable)

## Version Numbering Guidelines

Follow Semantic Versioning:
- **MAJOR** (X.0.0): Breaking changes, major architecture changes
- **MINOR** (0.X.0): New features, non-breaking enhancements
- **PATCH** (0.0.X): Bug fixes, minor improvements

## Debian Changelog Format

When suggesting a release, the debian/changelog entry should follow this format:
```
ros-weaver (X.Y.Z-1) unstable; urgency=medium

  * Feature or fix description
  * Another change
  * ...

 -- ROS Weaver Team <user@example.com>  Day, DD Mon YYYY HH:MM:SS +0000
```

## Example Usage

```
/push_remote Add undo support for parameter dashboard
```

This will:
1. Commit all staged/modified changes
2. Add "Undo/redo support for parameter dashboard" to CHANGELOG.md
3. Push to remote
4. Check if a release should be suggested
