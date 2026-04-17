# Error Pattern Log

### Stats updates gated behind display filter — 2026-04-17

- **Severity:** High
- **Category:** Logic
- **File(s):** `src/ros_weaver/src/widgets/plot_panel.cpp`
- **Pattern:** Stream statistics (message counts, rates, last-seen timestamps) updated *after* a display-filter early-return, so the filter silently corrupts the reported stream characteristics.
- **Root cause:** Sample-rate decimation was added as an early-return in `PlotPanel::onDataReceived()`, placed before the `messageCount++` / `publishRate` updates. With decimation at 10 Hz on a 100 Hz topic, the UI reported 10 Hz — matching the display rate, not the true publish rate.
- **Fix applied:** Moved `currentValue`, `messageCount`, `lastMessageTime`, and `publishRate` updates *above* the decimation check. Decimation now only gates `buffer.push_back`, not stats.
- **Prevention rule:** When adding a display-side filter (decimation, downsample, throttle) to an ingress callback, keep observability updates (counters, rate meters, last-seen timestamps) on the *ingress* side of the filter. Filters should affect what is *rendered/stored*, never what is *measured*.

### Missing input validation on paired numeric bounds — 2026-04-17

- **Severity:** Medium
- **Category:** API Misuse
- **File(s):** `src/ros_weaver/src/widgets/plot_series_config_dialog.cpp`, `src/ros_weaver/include/ros_weaver/widgets/plot_series_config_dialog.hpp`
- **Pattern:** Dialog collects a lower/upper (or min/max) pair from the user and writes it to config without enforcing `lower < upper`. Downstream code may silently degrade (e.g., alarm-predicate always true, or range becomes zero/negative and clamps).
- **Root cause:** `PlotSeriesConfigDialog::result()` returned `QDoubleSpinBox::value()` directly for threshold and gradient bounds. A swapped upper/lower made `isAlarm(y) = y > upper || y < lower` evaluate true for almost every value; swapped gradient bounds produced a flat single-color plot due to `range <= 0` clamping.
- **Fix applied:** Overrode `QDialog::accept()` to reject swapped bounds with `QMessageBox::warning` and focus the offending spinbox.
- **Prevention rule:** Any dialog exposing a `(min, max)` or `(lower, upper)` pair must validate ordering in `accept()` before calling `QDialog::accept()`. Do not rely on spinbox ranges alone — the *relationship* between paired values is not enforceable at the widget level.

### Division without range guard in geometric interpolation — 2026-04-17

- **Severity:** Low
- **Category:** Logic
- **File(s):** `src/ros_weaver/src/widgets/plot_panel.cpp`
- **Pattern:** Linear-interpolation helper computes `t = (target - a) / (b - a)` without guarding the denominator. Safe in current callers but a footgun if the helper is reused with equal endpoints.
- **Root cause:** `interpolateCrossing` in threshold rendering assumed `b.y() != a.y()` based on the caller's alarm-state-change precondition.
- **Fix applied:** Added `std::abs(dy) < 1e-12` guard returning the midpoint at the threshold line.
- **Prevention rule:** Geometric interpolation helpers should be self-contained — guard every divisor, even when current callers guarantee it's non-zero. Future callers may not.

### Silent truncation of bounded resource pool — 2026-04-17

- **Severity:** Low
- **Category:** Other
- **File(s):** `src/ros_weaver/src/widgets/plot_panel.cpp`, `src/ros_weaver/include/ros_weaver/widgets/plot_panel.hpp`
- **Pattern:** `ensureCapacity(needed)` silently caps allocation at a `MAX_POOL_SIZE` constant. Callers receive a pool smaller than requested with no diagnostic, causing trailing data to not render.
- **Root cause:** `SegmentSeriesPool::ensureCapacity` used `while (pool.size() < needed && pool.size() < MAX_POOL_SIZE)` with no warning path.
- **Fix applied:** Added a one-shot `qWarning` (gated by a `poolCapWarned` member, reset in `clearAll`) when `needed > MAX_POOL_SIZE`.
- **Prevention rule:** When a bounded pool/buffer silently truncates a requested size, emit a one-shot warning. Silence makes rendering/capacity bugs very hard to diagnose.

### Debug prints left in destructor — 2026-04-17

- **Severity:** Low
- **Category:** Convention
- **File(s):** `src/ros_weaver/src/widgets/plot_panel.cpp`
- **Pattern:** Diagnostic `std::cerr` prints left in production code fire on every object lifecycle event, polluting logs.
- **Root cause:** Tracing prints added during earlier debugging were not removed after the issue was resolved.
- **Fix applied:** Removed `std::cerr` lines from `PlotPanel::~PlotPanel()` and replaced `#include <iostream>` with `#include <QDebug>` (used by the new `qWarning` in `ensureCapacity`).
- **Prevention rule:** When closing out a debugging session, grep the touched files for `std::cerr`, `std::cout`, `printf`, and stray `qDebug()` before committing. Prefer `qDebug`/`qWarning` over raw streams so output can be filtered by Qt's logging rules.
