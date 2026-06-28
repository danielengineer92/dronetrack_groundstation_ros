# Mission Planner UI — CSS Theme Tokens & Component Styles

> **Target:** pixel-consistent dark theme extension for the existing DroneTrack dashboard.
> **Context:** `web_dashboard_node.py` embeds all HTML/CSS/JS in a `DASHBOARD_HTML` string.
> This spec covers only the **new** mission-planner editor UI components while keeping
> every existing style (`body`, `.card`, `.grid`, `.camera`, `.planner`, status badges,
> operator buttons) untouched.

---

## 1. CSS Custom Properties (Theme Tokens)

Add these inside the existing `<style>` block **after** the `:root { color-scheme: dark; }` rule.
They formalise every colour used by the new components so the whole UI can be re-themed by
editing one block.

```css
:root {
  color-scheme: dark;

  /* ── existing palette (documented, keep identical) ── */
  --bg-body:        #06101e;
  --bg-card:        #0a1528;
  --bg-panel:       #071326;
  --bg-step:        #09182d;
  --bg-input:       #09182d;
  --bg-frame:       #02060c;

  --text-primary:   #e7f4ff;
  --text-bright:    #cfe6ff;
  --text-muted:     #83a3bd;

  --border-subtle:  rgba(100,160,220,.14);
  --border-card:    rgba(100,160,220,.18);
  --border-panel:   rgba(100,160,220,.2);
  --border-input:   rgba(100,160,220,.28);

  --accent:         #1f6feb;
  --accent-text:    #ffffff;

  /* ── semantic (existing, keep identical) ── */
  --ok:             #50fa7b;
  --warn:           #ffcc66;
  --bad:            #ff5f75;

  /* ── new: verb category badge colours ──────────────────── */
  /* green — action verbs: scan, track_center, approach, orbit */
  --cat-action:            #2ea043;
  --cat-action-bg:         rgba(46, 160, 67, 0.15);
  --cat-action-border:     rgba(46, 160, 67, 0.35);

  /* blue — preflight verbs: takeoff, prime_offboard */
  --cat-preflight:         #1f6feb;
  --cat-preflight-bg:      rgba(31, 111, 235, 0.15);
  --cat-preflight-border:  rgba(31, 111, 235, 0.35);

  /* orange — motion verbs: rtl, land, hold */
  --cat-motion:            #d4782a;
  --cat-motion-bg:         rgba(212, 120, 42, 0.15);
  --cat-motion-border:     rgba(212, 120, 42, 0.35);

  /* ── new: danger / delete ──────────────────────────────── */
  --danger:         #ff5f75;
  --danger-bg:      rgba(255, 95, 117, 0.12);
  --danger-border:  rgba(255, 95, 117, 0.3);

  /* ── new: editor chrome ────────────────────────────────── */
  --toolbar-bg:     #09182d;
  --editor-bg:      #0a1528;
  --editor-border:  rgba(100,160,220,.25);
  --hover-glow:     rgba(31, 111, 235, 0.12);
}
```

> **IMPORTANT:** The fallback values in existing rules (`background:#06101e;` etc.) **must
> not change**. The `var()` tokens are for the *new* rules added by this spec. Existing
> rules keep their hard-coded values so the legacy UI cannot regress.

---

## 2. Verb Category → Colour Mapping

The user-facing category model (matches `mission_plan_model.CATEGORY_COLORS` names but
with a different verb-to-category assignment for UI purposes):

| Category       | CSS var prefix         | Verbs                                 |
|----------------|------------------------|---------------------------------------|
| `action`       | `--cat-action`         | `scan`, `track_center`, `approach`, `orbit` |
| `preflight`    | `--cat-preflight`      | `takeoff`, `prime_offboard`           |
| `motion`       | `--cat-motion`         | `rtl`, `land`, `hold`                 |

The verb→category assignment is driven by a **JavaScript object** in the client code
(not by `mission_plan_model.CATEGORY_COLORS` which maps the same three names to different
verbs). Reference:

```js
const VERB_CATEGORY = {
  // action (green)
  scan: "action", track_center: "action", approach: "action", orbit: "action",
  // preflight (blue)
  takeoff: "preflight", prime_offboard: "preflight",
  // motion (orange)
  rtl: "motion", land: "motion", hold: "motion",
};
```

The colour utility function:

```js
function catColor(cat) {
  const m = {
    action:    getComputedStyle(document.documentElement).getPropertyValue('--cat-action'),
    preflight: getComputedStyle(document.documentElement).getPropertyValue('--cat-preflight'),
    motion:    getComputedStyle(document.documentElement).getPropertyValue('--cat-motion'),
  };
  return m[cat] || 'currentColor';
}
```

> A simpler approach: define three CSS classes `.cat-action`, `.cat-preflight`,
> `.cat-motion` and apply the class to the element. This avoids runtime style lookups.

---

## 3. Step Card — Full Layout Specification

### 3.1 Existing `.step` rule — keep as-is for the *preview* mode

The current `.step` must not change; it renders read-only preview steps:

```css
.step {
  display: grid;
  grid-template-columns: 34px minmax(0, 1fr);
  gap: 10px;
  align-items: start;
  padding: 9px;
  background: #09182d;
  border: 1px solid rgba(100, 160, 220, .14);
  border-radius: 8px;
}
```

### 3.2 New editor-mode step card: `.step-editor`

The editor step card replaces `.step` when the mission planner is in *editor* mode.
It is an interactive block with hover/focus states, category colour hint, and action
buttons.

```css
/* ── Editor step container ─────────────────────────────── */
.step-editor {
  display: grid;
  grid-template-columns: 36px minmax(0, 1fr) auto;
  gap: 10px;
  align-items: start;
  padding: 10px 12px;
  background: var(--bg-step);
  border: 1px solid var(--border-subtle);
  border-left: 3px solid var(--border-subtle);  /* overridden by category class */
  border-radius: 8px;
  cursor: pointer;
  transition: background 0.15s ease, border-color 0.15s ease, box-shadow 0.15s ease;
}

.step-editor:hover {
  background: #0c1e38;                       /* slightly lighter than bg-step */
  border-color: rgba(100, 160, 220, .28);
  box-shadow: 0 0 0 1px var(--hover-glow);
}

.step-editor:focus-visible {
  outline: 2px solid var(--accent);
  outline-offset: 1px;
}

/* ── Category-coloured left border ─────────────────────── */
.step-editor.cat-action    { border-left-color: var(--cat-action); }
.step-editor.cat-preflight { border-left-color: var(--cat-preflight); }
.step-editor.cat-motion    { border-left-color: var(--cat-motion); }

/* ── Step number badge ─────────────────────────────────── */
.step-num {
  color: var(--text-muted);
  font-weight: 800;
  font-size: 13px;
  text-align: center;
  line-height: 1;
}

/* ── Step body: verb name + parameter summary ──────────── */
.step-body {
  display: grid;
  gap: 3px;
  min-width: 0;                              /* prevent overflow in grid */
}

.step-verb {
  font-weight: 800;
  font-size: 14px;
  color: var(--text-primary);
  display: flex;
  align-items: center;
  gap: 8px;
}

.step-summary {
  font-size: 11px;
  color: var(--text-muted);
  white-space: nowrap;
  overflow: hidden;
  text-overflow: ellipsis;
}

/* ── Verb category badge (inline pill) ─────────────────── */
.verb-badge {
  display: inline-block;
  font-size: 10px;
  font-weight: 700;
  text-transform: uppercase;
  letter-spacing: 0.08em;
  padding: 2px 7px;
  border-radius: 4px;
  line-height: 1.4;
}

.verb-badge.cat-action    {
  color: var(--cat-action);
  background: var(--cat-action-bg);
  border: 1px solid var(--cat-action-border);
}

.verb-badge.cat-preflight {
  color: var(--cat-preflight);
  background: var(--cat-preflight-bg);
  border: 1px solid var(--cat-preflight-border);
}

.verb-badge.cat-motion    {
  color: var(--cat-motion);
  background: var(--cat-motion-bg);
  border: 1px solid var(--cat-motion-border);
}

/* ── Step action buttons (right column) ────────────────── */
.step-actions {
  display: flex;
  gap: 4px;
  align-items: center;
  opacity: 0;
  transition: opacity 0.12s ease;
}

.step-editor:hover .step-actions { opacity: 1; }

.step-btn {
  font: inherit;
  font-size: 11px;
  font-weight: 700;
  border: 1px solid transparent;
  border-radius: 5px;
  padding: 4px 8px;
  cursor: pointer;
  background: transparent;
  color: var(--text-muted);
  transition: background 0.12s ease, color 0.12s ease, border-color 0.12s ease;
  line-height: 1.2;
}

.step-btn:hover {
  background: rgba(100, 160, 220, .12);
  color: var(--text-primary);
}

.step-btn.move-up,
.step-btn.move-down {
  font-size: 13px;
  padding: 2px 6px;
}

.step-btn.danger {
  color: var(--danger);
  border-color: transparent;
}

.step-btn.danger:hover {
  background: var(--danger-bg);
  border-color: var(--danger-border);
}
```

### 3.3 DOM structure for one editor step

```html
<div class="step-editor cat-action" data-index="0" data-type="scan">
  <div class="step-num">1</div>
  <div class="step-body">
    <div class="step-verb">
      scan
      <span class="verb-badge cat-action">action</span>
    </div>
    <div class="step-summary">Sweep ccw 180° @ 20°/s, timeout 12s</div>
  </div>
  <div class="step-actions">
    <button class="step-btn move-up"   title="Move up" aria-label="Move step up">&#9650;</button>
    <button class="step-btn move-down" title="Move down" aria-label="Move step down">&#9660;</button>
    <button class="step-btn danger"    title="Delete step" aria-label="Delete step">&#10005;</button>
  </div>
</div>
```

---

## 4. Add-Step Toolbar Button

Placed between the planner-head and the step list. A horizontal bar that holds the
**+ Add Step** button and, optionally, a **plan name** text input.

```css
/* ── Toolbar bar ───────────────────────────────────────── */
.step-toolbar {
  display: flex;
  align-items: center;
  gap: 10px;
  padding: 0 14px 10px;
}

/* ── Add Step button ───────────────────────────────────── */
.btn-add-step {
  font: inherit;
  font-size: 13px;
  font-weight: 700;
  color: var(--accent);
  background: rgba(31, 111, 235, .1);
  border: 1px dashed var(--accent);
  border-radius: 8px;
  padding: 10px 18px;
  cursor: pointer;
  transition: background 0.12s ease, box-shadow 0.12s ease;
  display: inline-flex;
  align-items: center;
  gap: 6px;
}

.btn-add-step::before {
  content: "+";
  font-size: 18px;
  font-weight: 400;
  line-height: 1;
}

.btn-add-step:hover {
  background: rgba(31, 111, 235, .18);
  box-shadow: 0 0 0 2px var(--hover-glow);
}

.btn-add-step:focus-visible {
  outline: 2px solid var(--accent);
  outline-offset: 2px;
}

.btn-add-step:active {
  background: rgba(31, 111, 235, .25);
}

/* ── Plan name input (inside toolbar, optional) ────────── */
.plan-name-input {
  font: inherit;
  font-size: 16px;
  font-weight: 700;
  color: var(--text-primary);
  background: transparent;
  border: 1px solid transparent;
  border-bottom-color: var(--border-input);
  border-radius: 0;
  padding: 4px 0;
  min-width: 180px;
  outline: none;
  transition: border-color 0.15s ease;
}

.plan-name-input:hover  { border-bottom-color: var(--accent); }
.plan-name-input:focus  { border-bottom-color: var(--accent); }

/* ── Save / Discard buttons (far right of toolbar) ─────── */
.btn-save-plan {
  font: inherit;
  font-size: 13px;
  font-weight: 700;
  color: var(--accent-text);
  background: var(--accent);
  border: 0;
  border-radius: 6px;
  padding: 8px 16px;
  cursor: pointer;
  transition: opacity 0.12s ease;
  margin-left: auto;                         /* push to right */
}

.btn-save-plan:hover  { opacity: 0.9; }
.btn-save-plan:active { opacity: 0.8; }

.btn-discard-plan {
  font: inherit;
  font-size: 13px;
  font-weight: 700;
  color: var(--text-muted);
  background: transparent;
  border: 1px solid var(--border-card);
  border-radius: 6px;
  padding: 8px 16px;
  cursor: pointer;
  transition: background 0.12s ease, color 0.12s ease;
}

.btn-discard-plan:hover {
  background: rgba(100, 160, 220, .1);
  color: var(--text-primary);
}
```

### 4.1 Add Step dropdown menu (appears below the + button)

When the **+ Add Step** button is clicked, a menu lists the 9 available verbs grouped
by category.

```css
/* ── Dropdown menu (positioned via JS, styled here) ───── */
.step-menu {
  position: absolute;
  z-index: 100;
  background: var(--bg-card);
  border: 1px solid var(--border-panel);
  border-radius: 8px;
  padding: 6px 0;
  min-width: 240px;
  box-shadow: 0 8px 24px rgba(0, 0, 0, .4);
  display: grid;
  gap: 0;
}

.step-menu-group {
  padding: 4px 0;
}

.step-menu-group + .step-menu-group {
  border-top: 1px solid var(--border-subtle);
}

.step-menu-heading {
  font-size: 9px;
  font-weight: 700;
  text-transform: uppercase;
  letter-spacing: .15em;
  color: var(--text-muted);
  padding: 6px 14px 2px;
}

.step-menu-item {
  display: flex;
  align-items: center;
  gap: 8px;
  padding: 7px 14px;
  font-size: 13px;
  font-weight: 600;
  color: var(--text-primary);
  cursor: pointer;
  border: none;
  background: none;
  width: 100%;
  text-align: left;
  transition: background 0.08s ease;
}

.step-menu-item:hover {
  background: var(--hover-glow);
}

.step-menu-item .verb-badge {
  margin-left: auto;                         /* badge to the right */
}

.step-menu-desc {
  font-size: 10px;
  font-weight: 400;
  color: var(--text-muted);
  padding: 0 14px 0 30px;                   /* indented under the verb name */
  margin-top: -2px;
  margin-bottom: 4px;
}
```

### 4.2 DOM for the Add Step menu

```html
<div class="step-menu" style="position:absolute; top:…; left:…;">
  <div class="step-menu-group">
    <div class="step-menu-heading">Preflight</div>
    <button class="step-menu-item" data-type="takeoff">
      Take Off <span class="verb-badge cat-preflight">preflight</span>
    </button>
    <button class="step-menu-item" data-type="prime_offboard">
      Prime Offboard <span class="verb-badge cat-preflight">preflight</span>
    </button>
  </div>
  <div class="step-menu-group">
    <div class="step-menu-heading">Action</div>
    <button class="step-menu-item" data-type="scan">
      Scan / Seek <span class="verb-badge cat-action">action</span>
    </button>
    <div class="step-menu-desc">Yaw sweep to find the target in place</div>
    <button class="step-menu-item" data-type="track_center">
      Track Center <span class="verb-badge cat-action">action</span>
    </button>
    <button class="step-menu-item" data-type="approach">
      Approach Target <span class="verb-badge cat-action">action</span>
    </button>
    <button class="step-menu-item" data-type="orbit">
      Orbit <span class="verb-badge cat-action">action</span>
    </button>
  </div>
  <div class="step-menu-group">
    <div class="step-menu-heading">Motion</div>
    <button class="step-menu-item" data-type="rtl">
      Return to Launch <span class="verb-badge cat-motion">motion</span>
    </button>
    <button class="step-menu-item" data-type="land">
      Land <span class="verb-badge cat-motion">motion</span>
    </button>
    <button class="step-menu-item" data-type="hold">
      Hold <span class="verb-badge cat-motion">motion</span>
    </button>
  </div>
</div>
```

---

## 5. Inline Parameter Editor Panel

When a step card is clicked in editor mode, it expands to reveal an inline parameter
form. The rest of the step list stays in place below it; no modal, no page overlay.

### 5.1 Editor panel CSS

```css
/* ── Inline parameter editor (appears inside the step card) ── */
.param-editor {
  grid-column: 1 / -1;                       /* span all three columns */
  display: grid;
  gap: 10px;
  padding: 12px;
  margin-top: 6px;
  background: var(--editor-bg);
  border: 1px solid var(--editor-border);
  border-radius: 6px;
}

.param-editor-divider {
  grid-column: 1 / -1;
  border: none;
  border-top: 1px solid var(--border-subtle);
  margin: 2px 0;
}

/* ── Field row: label + input ──────────────────────────── */
.param-field {
  display: grid;
  grid-template-columns: 130px minmax(0, 1fr);
  gap: 10px;
  align-items: center;
}

.param-label {
  font-size: 11px;
  font-weight: 600;
  text-transform: uppercase;
  letter-spacing: .06em;
  color: var(--text-muted);
}

/* ── Input controls ────────────────────────────────────── */
.param-input {
  font: inherit;
  font-size: 13px;
  color: var(--text-primary);
  background: var(--bg-input);
  border: 1px solid var(--border-input);
  border-radius: 6px;
  padding: 7px 10px;
  outline: none;
  transition: border-color 0.12s ease, box-shadow 0.12s ease;
}

.param-input:hover  { border-color: rgba(100, 160, 220, .5); }
.param-input:focus  {
  border-color: var(--accent);
  box-shadow: 0 0 0 2px var(--hover-glow);
}

/* numeric spinners — compact, integer-friendly */
.param-input[type="number"] {
  max-width: 130px;
  -moz-appearance: textfield;
}
.param-input[type="number"]::-webkit-inner-spin-button,
.param-input[type="number"]::-webkit-outer-spin-button {
  opacity: 1;
}

/* text input */
.param-input[type="text"] {
  max-width: 280px;
}

/* ── Select (enum) ─────────────────────────────────────── */
.param-select {
  font: inherit;
  font-size: 13px;
  color: var(--text-primary);
  background: var(--bg-input);
  border: 1px solid var(--border-input);
  border-radius: 6px;
  padding: 7px 10px;
  outline: none;
  max-width: 200px;
  cursor: pointer;
  transition: border-color 0.12s ease;
}

.param-select:hover { border-color: rgba(100, 160, 220, .5); }
.param-select:focus {
  border-color: var(--accent);
  box-shadow: 0 0 0 2px var(--hover-glow);
}

/* ── Unit suffix (after numeric inputs) ────────────────── */
.param-unit {
  font-size: 11px;
  color: var(--text-muted);
  margin-left: 6px;
}

/* ── Editor action buttons ─────────────────────────────── */
.param-editor-actions {
  display: flex;
  gap: 8px;
  justify-content: flex-end;
  margin-top: 4px;
}

.btn-apply {
  font: inherit;
  font-size: 12px;
  font-weight: 700;
  color: var(--accent-text);
  background: var(--accent);
  border: 0;
  border-radius: 6px;
  padding: 6px 14px;
  cursor: pointer;
  transition: opacity 0.12s ease;
}
.btn-apply:hover  { opacity: 0.9; }
.btn-apply:active { opacity: 0.8; }

.btn-cancel {
  font: inherit;
  font-size: 12px;
  font-weight: 700;
  color: var(--text-muted);
  background: transparent;
  border: 1px solid var(--border-card);
  border-radius: 6px;
  padding: 6px 14px;
  cursor: pointer;
  transition: background 0.12s ease, color 0.12s ease;
}
.btn-cancel:hover {
  background: rgba(100, 160, 220, .1);
  color: var(--text-primary);
}
```

### 5.2 Expanded editor step DOM

```html
<div class="step-editor cat-action" data-index="2" data-type="scan">
  <div class="step-num">2</div>
  <div class="step-body">
    <div class="step-verb">scan <span class="verb-badge cat-action">action</span></div>
    <div class="step-summary">Sweep ccw 180° @ 20°/s, timeout 12s</div>
  </div>
  <div class="step-actions">
    <button class="step-btn move-up">&#9650;</button>
    <button class="step-btn move-down">&#9660;</button>
    <button class="step-btn danger">&#10005;</button>
  </div>

  <!-- ── inline editor ─────────────────────────────── -->
  <div class="param-editor">
    <div class="param-field">
      <span class="param-label">Sweep Direction</span>
      <select class="param-select">
        <option value="ccw" selected>Counter-clockwise</option>
        <option value="cw">Clockwise</option>
      </select>
    </div>
    <div class="param-field">
      <span class="param-label">Sweep Angle</span>
      <span>
        <input class="param-input" type="number" value="180" min="5" max="360" step="15"/>
        <span class="param-unit">deg</span>
      </span>
    </div>
    <div class="param-field">
      <span class="param-label">Sweep Rate</span>
      <span>
        <input class="param-input" type="number" value="20" min="1" max="90" step="5"/>
        <span class="param-unit">deg/s</span>
      </span>
    </div>
    <div class="param-field">
      <span class="param-label">Exit Condition</span>
      <select class="param-select">
        <option value="locked" selected>Stop when locked</option>
        <option value="none">Run to completion</option>
      </select>
    </div>
    <div class="param-field">
      <span class="param-label">Timeout</span>
      <span>
        <input class="param-input" type="number" value="12" min="0" step="1"/>
        <span class="param-unit">s</span>
      </span>
    </div>
    <div class="param-editor-actions">
      <button class="btn-cancel">Cancel</button>
      <button class="btn-apply">Apply</button>
    </div>
  </div>
</div>
```

---

## 6. Warnings & Errors Panels

Two distinct panels in the left column of `.plan-body` (where the existing
`#plan_warnings` div lives today). Replace the single `#plan_warnings` with
two containers to separate severity.

### 6.1 CSS

```css
/* ── Warnings (amber) ──────────────────────────────────── */
.plan-warnings {
  margin-top: 10px;
  display: grid;
  gap: 6px;
}

.warning-item {
  font-size: 12px;
  color: var(--warn);
  background: rgba(255, 204, 102, .08);
  border: 1px solid rgba(255, 204, 102, .2);
  border-radius: 6px;
  padding: 8px 10px;
  display: flex;
  align-items: flex-start;
  gap: 6px;
  line-height: 1.35;
}

.warning-item::before {
  content: "\26A0";                          /* ⚠ */
  font-size: 13px;
  flex-shrink: 0;
}

/* ── Errors (red) ──────────────────────────────────────── */
.plan-errors {
  margin-top: 10px;
  display: grid;
  gap: 6px;
}

.error-item {
  font-size: 12px;
  color: var(--bad);
  background: rgba(255, 95, 117, .08);
  border: 1px solid rgba(255, 95, 117, .2);
  border-radius: 6px;
  padding: 8px 10px;
  display: flex;
  align-items: flex-start;
  gap: 6px;
  line-height: 1.35;
}

.error-item::before {
  content: "\2716";                          /* ✖ */
  font-size: 13px;
  flex-shrink: 0;
}

/* ── Plan name display above meta ──────────────────────── */
.plan-name-display {
  font-size: 18px;
  font-weight: 800;
  color: var(--text-primary);
  margin-bottom: 8px;
}

.plan-file-display {
  font-size: 11px;
  color: var(--text-muted);
  word-break: break-all;
  margin-bottom: 12px;
}

/* ── Plan metadata section heading ─────────────────────── */
.plan-section-heading {
  font-size: 10px;
  font-weight: 700;
  text-transform: uppercase;
  letter-spacing: .12em;
  color: var(--text-muted);
  margin-bottom: 6px;
}
```

### 6.2 DOM for the left column in editor mode

```html
<div class="plan-body">
  <div>
    <div class="plan-name-display" id="plan_name_display">
      <span id="plan_name">orbit_red_ball</span>
    </div>
    <div class="plan-file-display" id="plan_file">
      C:\Users\...\missions\orbit_red_ball.yaml
    </div>

    <!-- ── errors (only rendered when present) ────────── -->
    <div class="plan-section-heading">Errors</div>
    <div class="plan-errors" id="plan_errors">
      <div class="error-item">Step 3 (scan): until=none but no timeout_s — step could run forever</div>
    </div>

    <!-- ── warnings (only rendered when present) ──────── -->
    <div class="plan-section-heading">Warnings</div>
    <div class="plan-warnings" id="plan_warnings">
      <div class="warning-item">Plan contains motion steps but no prime_offboard</div>
    </div>

    <div style="margin-top:14px">
      <div class="k">PI Config</div>
      <small id="plan_hint">Set mission_plan_file on the Pi before Start Mission</small>
    </div>
  </div>
  <div class="steps" id="plan_steps">
    <!-- step list here -->
  </div>
</div>
```

---

## 7. `.danger` Class — Red / Delete

A reusable semantic class for destructive actions anywhere in the planner UI.

```css
/* ── Danger / destructive actions ──────────────────────── */
.danger {
  color: var(--danger);
}

.danger-bg {
  background: var(--danger-bg);
  border-color: var(--danger-border);
}

.danger:hover,
.danger-bg:hover,
.btn-danger:hover {
  background: rgba(255, 95, 117, .18);
  border-color: var(--danger);
}

/* ── Dedicated danger button (e.g. "Delete Plan") ─────── */
.btn-danger {
  font: inherit;
  font-size: 13px;
  font-weight: 700;
  color: var(--danger);
  background: var(--danger-bg);
  border: 1px solid var(--danger-border);
  border-radius: 6px;
  padding: 8px 16px;
  cursor: pointer;
  transition: background 0.12s ease, border-color 0.12s ease;
}

.btn-danger:hover {
  background: rgba(255, 95, 117, .18);
  border-color: var(--danger);
  color: #ff8099;                            /* slightly lighter on hover */
}

.btn-danger:focus-visible {
  outline: 2px solid var(--danger);
  outline-offset: 2px;
}
```

### 7.1 Confirmation dialog (inlined, not a browser `confirm()`)

When a delete button is clicked, show a brief inline confirmation inline within
the step card or as a small popover.

```css
/* ── Inline delete confirmation ────────────────────────── */
.delete-confirm {
  display: flex;
  align-items: center;
  gap: 8px;
  padding: 6px 10px;
  background: var(--danger-bg);
  border: 1px solid var(--danger-border);
  border-radius: 6px;
  font-size: 12px;
  color: var(--danger);
  font-weight: 600;
}

.delete-confirm .btn-confirm-yes {
  font: inherit;
  font-size: 11px;
  font-weight: 700;
  color: #fff;
  background: var(--danger);
  border: 0;
  border-radius: 4px;
  padding: 3px 10px;
  cursor: pointer;
}

.delete-confirm .btn-confirm-no {
  font: inherit;
  font-size: 11px;
  font-weight: 700;
  color: var(--text-muted);
  background: transparent;
  border: 1px solid var(--border-card);
  border-radius: 4px;
  padding: 3px 10px;
  cursor: pointer;
}
```

---

## 8. Status Bar — Persistent Footer

A new full-width status bar pinned to the bottom of the viewport. It shows:

| Zone       | Content                                    |
|------------|--------------------------------------------|
| Left       | Connection status (Link UP/DOWN + reason)  |
| Centre     | Plan dirty indicator (Unsaved changes)     |
| Right      | Last save timestamp + validation summary   |

```css
/* ── Status bar ────────────────────────────────────────── */
.status-bar {
  position: fixed;
  bottom: 0;
  left: 0;
  right: 0;
  z-index: 200;
  display: flex;
  justify-content: space-between;
  align-items: center;
  gap: 16px;
  padding: 6px 20px;
  font-size: 11px;
  background: var(--bg-card);
  border-top: 1px solid var(--border-panel);
  color: var(--text-muted);
  min-height: 32px;
}

.status-bar-left {
  display: flex;
  align-items: center;
  gap: 12px;
}

.status-bar-center {
  font-weight: 600;
}

.status-bar-right {
  display: flex;
  align-items: center;
  gap: 12px;
}

/* ── Status dot ────────────────────────────────────────── */
.status-dot {
  display: inline-block;
  width: 7px;
  height: 7px;
  border-radius: 50%;
  margin-right: 4px;
  flex-shrink: 0;
}

.status-dot.ok    { background: var(--ok); }
.status-dot.warn  { background: var(--warn); }
.status-dot.bad   { background: var(--bad); }

/* ── Dirty indicator ───────────────────────────────────── */
.status-dirty {
  color: var(--warn);
}

.status-clean {
  color: var(--ok);
}

/* Push body content up so the status bar doesn't cover it */
body {
  padding-bottom: 38px;
}
```

### 8.1 Status bar DOM

```html
<div class="status-bar">
  <div class="status-bar-left">
    <span><span class="status-dot ok"></span>Link UP</span>
    <span>Latency 12 ms</span>
  </div>
  <div class="status-bar-center">
    <span class="status-dirty" id="dirty_indicator">Unsaved changes</span>
  </div>
  <div class="status-bar-right">
    <span id="validation_summary">9 steps, 0 errors, 2 warnings</span>
    <span>Last saved 14:32:05</span>
  </div>
</div>
```

> **Note:** The existing status cards (`.grid` of `.card`) at the top of the page **do
> not change**. The status bar is additive — it provides a persistent glanceable footer
> that remains visible while the user scrolls through the mission editor.

---

## 9. Mission Planner Layout Integration

### 9.1 Where the planner section sits

The `<section class="planner">` block (line 125–137 in `web_dashboard_node.py`) stays
in its current position:

```
.wrap
  h1 + small#updated
  section.camera          (full width — camera + targets)
  div.grid                (auto-fit card grid — 11 status cards)
  section.planner  <── NEW EDITOR MODE CONTENT GOES HERE
  div.row                 (operator buttons)
```

### 9.2 Planner header — edit-mode variant

The existing `.planner-head` contains a `<select>` for picking a mission to preview.
In editor mode it is replaced by (or toggled to) a toolbar that includes:

```
.planner-head
  left:   "Mission Plans" label + "Editor" title
  right:  [Import YAML] [Export YAML] [New Plan] buttons
  (no <select> — the select is only for preview mode)
```

```css
/* ── Editor-mode planner header ────────────────────────── */
.planner-head-editor {
  display: flex;
  justify-content: space-between;
  gap: 12px;
  flex-wrap: wrap;
  padding: 14px;
  align-items: flex-end;
}

.planner-mode-toggle {
  display: inline-flex;
  gap: 4px;
  background: var(--bg-step);
  border: 1px solid var(--border-subtle);
  border-radius: 6px;
  padding: 3px;
}

.mode-btn {
  font: inherit;
  font-size: 11px;
  font-weight: 700;
  color: var(--text-muted);
  background: transparent;
  border: 0;
  border-radius: 4px;
  padding: 5px 12px;
  cursor: pointer;
  transition: background 0.1s ease, color 0.1s ease;
}

.mode-btn.active {
  background: var(--accent);
  color: var(--accent-text);
}

.mode-btn:not(.active):hover {
  background: rgba(100, 160, 220, .15);
  color: var(--text-primary);
}

/* ── Planner header action buttons ─────────────────────── */
.planner-header-actions {
  display: flex;
  gap: 6px;
  align-items: center;
}

.btn-header {
  font: inherit;
  font-size: 12px;
  font-weight: 600;
  color: var(--text-muted);
  background: transparent;
  border: 1px solid var(--border-card);
  border-radius: 6px;
  padding: 6px 12px;
  cursor: pointer;
  transition: background 0.12s ease, color 0.12s ease;
}

.btn-header:hover {
  background: rgba(100, 160, 220, .1);
  color: var(--text-primary);
}
```

### 9.3 DOM for the planner section in editor mode

```html
<section class="planner">
  <div class="planner-head-editor">
    <div>
      <div class="k">Mission Plans</div>
      <div class="camera-title">
        <span id="planner_mode_label">Editor</span>
      </div>
    </div>
    <div class="planner-mode-toggle">
      <button class="mode-btn" onclick="switchMode('preview')">Preview</button>
      <button class="mode-btn active" onclick="switchMode('editor')">Editor</button>
    </div>
    <div class="planner-header-actions">
      <button class="btn-header" onclick="importYaml()">Import YAML</button>
      <button class="btn-header" onclick="exportYaml()">Export YAML</button>
      <button class="btn-header" onclick="newPlan()">New Plan</button>
    </div>
  </div>

  <!-- ── toolbar ───────────────────────────────────── -->
  <div class="step-toolbar">
    <input class="plan-name-input" type="text" value="orbit_red_ball" placeholder="Plan name..."/>
    <button class="btn-add-step" onclick="openStepMenu()">Add Step</button>
    <button class="btn-save-plan" onclick="savePlan()">Save</button>
    <button class="btn-discard-plan" onclick="discardChanges()">Discard</button>
  </div>

  <!-- ── two-column body ───────────────────────────── -->
  <div class="plan-body">
    <div>
      <!-- plan metadata + errors + warnings (see §6.2) -->
    </div>
    <div class="steps" id="plan_steps">
      <!-- step-editor cards (see §3.3, §5.2) -->
    </div>
  </div>
</section>
```

---

## 10. Reorder Feedback (Drag Handle Visual)

Steps can be reordered. The interaction pattern is **click-to-move** (up/down arrows in
`.step-actions`) rather than drag-and-drop, so no drag ghost CSS is needed.

However, a brief "moved" animation provides feedback:

```css
/* ── Step move animation ───────────────────────────────── */
@keyframes step-flash {
  0%   { box-shadow: 0 0 0 0 var(--accent); }
  50%  { box-shadow: 0 0 0 4px rgba(31, 111, 235, .2); }
  100% { box-shadow: 0 0 0 0 transparent; }
}

.step-editor.flash {
  animation: step-flash 0.5s ease-out;
}
```

---

## 11. Empty State

When the plan has zero steps:

```css
/* ── Empty steps placeholder ───────────────────────────── */
.steps-empty {
  display: grid;
  place-items: center;
  padding: 40px 20px;
  color: var(--text-muted);
  font-size: 13px;
  border: 2px dashed var(--border-card);
  border-radius: 8px;
  text-align: center;
  gap: 8px;
}

.steps-empty .hint {
  font-size: 11px;
  color: rgba(131, 163, 189, .6);
}
```

```html
<div class="steps-empty">
  <div>No steps yet</div>
  <div class="hint">Click "Add Step" to build your mission plan.</div>
</div>
```

---

## 12. Responsive Behaviour

### 12.1 Existing breakpoint — keep

```css
@media (max-width: 760px) {
  .plan-body { grid-template-columns: 1fr; }
}
```

### 12.2 Additional breakpoints for editor components

```css
/* Stack editor fields vertically on narrow screens */
@media (max-width: 520px) {
  .param-field {
    grid-template-columns: 1fr;
  }

  .step-editor {
    grid-template-columns: 28px minmax(0, 1fr);
  }

  .step-editor .step-actions {
    grid-column: 1 / -1;
    justify-content: flex-end;
    opacity: 1;                              /* always visible on touch */
  }

  .step-toolbar {
    flex-wrap: wrap;
  }

  .planner-head-editor {
    flex-direction: column;
    align-items: stretch;
  }

  .status-bar {
    flex-direction: column;
    gap: 4px;
    padding: 6px 12px;
  }
}
```

---

## 13. Summary of ALL New CSS Classes

For quick reference, every new class introduced by this spec (excluding existing ones
like `.step`, `.card`, `.warn`, etc.):

| Class                    | Purpose                                      |
|--------------------------|----------------------------------------------|
| `.step-editor`           | Interactive step card in editor mode         |
| `.cat-action`            | Green category accent (border + badge)       |
| `.cat-preflight`         | Blue category accent                         |
| `.cat-motion`            | Orange category accent                       |
| `.step-body`             | Step content wrapper (verb + summary)        |
| `.step-verb`             | Step verb name with optional badge           |
| `.step-summary`          | One-line parameter summary, truncated        |
| `.verb-badge`            | Category pill (e.g. "action" in green)       |
| `.step-actions`          | Right-floated button row (fades in on hover) |
| `.step-btn`              | Small icon button (reorder, delete)          |
| `.step-btn.move-up`      | Up-arrow reorder button                      |
| `.step-btn.move-down`    | Down-arrow reorder button                    |
| `.step-btn.danger`       | Red-tinted delete button                     |
| `.step-toolbar`          | Bar holding Add Step, plan name, save btns   |
| `.btn-add-step`          | Dashed-border + Add Step button              |
| `.plan-name-input`       | Underline-style plan name text input         |
| `.btn-save-plan`         | Accent-coloured Save button                  |
| `.btn-discard-plan`      | Muted Discard button                         |
| `.step-menu`             | Dropdown picker listing all 9 verbs          |
| `.step-menu-group`       | Category group inside the dropdown           |
| `.step-menu-heading`     | Category name heading in dropdown            |
| `.step-menu-item`        | Individual verb row in dropdown              |
| `.step-menu-desc`        | Verb description line in dropdown            |
| `.param-editor`          | Inline parameter form container              |
| `.param-editor-divider`  | Horizontal rule inside the editor            |
| `.param-field`           | Label + input row in the editor              |
| `.param-label`           | Uppercase field label                        |
| `.param-input`           | Number/text input for parameters             |
| `.param-select`          | Enum dropdown for parameters                 |
| `.param-unit`            | Unit suffix after numeric inputs             |
| `.param-editor-actions`  | Apply/Cancel button row                      |
| `.btn-apply`             | Accent Apply button                          |
| `.btn-cancel`            | Muted Cancel button                          |
| `.plan-warnings`         | Warning items container                      |
| `.warning-item`          | Single warning (amber, ⚠ icon)               |
| `.plan-errors`           | Error items container                        |
| `.error-item`            | Single error (red, ✖ icon)                   |
| `.plan-name-display`     | Large plan name heading                      |
| `.plan-file-display`     | File path display below name                 |
| `.plan-section-heading`  | "Errors" / "Warnings" section titles         |
| `.danger`                | Red text utility                             |
| `.danger-bg`             | Red background utility                       |
| `.btn-danger`            | Full-width danger button                     |
| `.delete-confirm`        | Inline delete confirmation strip             |
| `.btn-confirm-yes`       | Red confirm button                           |
| `.btn-confirm-no`        | Muted cancel button                          |
| `.status-bar`            | Fixed footer bar                             |
| `.status-bar-left`       | Left segment of status bar                   |
| `.status-bar-center`     | Center segment (dirty indicator)             |
| `.status-bar-right`      | Right segment (validation + timestamp)       |
| `.status-dot`            | Coloured dot (ok/warn/bad)                   |
| `.status-dirty`          | Amber "Unsaved changes" text                 |
| `.status-clean`          | Green "All changes saved" text               |
| `.planner-head-editor`   | Editor-mode header layout                    |
| `.planner-mode-toggle`   | Preview/Editor toggle button group           |
| `.mode-btn`              | Individual toggle button                     |
| `.mode-btn.active`       | Active toggle button (accent fill)           |
| `.planner-header-actions`| Import/Export/New button group               |
| `.btn-header`            | Header action button                         |
| `.steps-empty`           | Empty state placeholder                      |
| `.steps-empty .hint`     | Empty state hint text                        |
| `.step-editor.flash`     | Move animation target                        |

---

## 14. Implementation Order

The CSS rules above should be added to the `<style>` block in `DASHBOARD_HTML`
**after** the existing rules. No existing rule should be deleted or altered.

1. Add custom properties (`:root` block with `--cat-action`, `--cat-preflight`,
   `--cat-motion`, `--danger`, etc.) — §1.
2. Add editor step card rules (`.step-editor`, `.verb-badge`, `.step-actions`,
   `.step-btn`) — §3.2.
3. Add toolbar rules (`.step-toolbar`, `.btn-add-step`, `.plan-name-input`) — §4.
4. Add dropdown menu rules (`.step-menu`, `.step-menu-item`) — §4.1.
5. Add parameter editor rules (`.param-editor`, `.param-field`, `.param-input`,
   `.param-select`, `.param-unit`, `.btn-apply`, `.btn-cancel`) — §5.1.
6. Add warnings/errors panel rules (`.plan-warnings`, `.warning-item`,
   `.plan-errors`, `.error-item`) — §6.1.
7. Add `.danger` utility and `.btn-danger` — §7.
8. Add status bar rules (`.status-bar`, `.status-dot`, `.status-dirty`) — §8.
9. Add planner-layout rules (`.planner-head-editor`, `.planner-mode-toggle`,
   `.mode-btn`, `.planner-header-actions`, `.btn-header`) — §9.2.
10. Add animation and empty state (`.step-flash`, `.steps-empty`) — §10, §11.
11. Add responsive overrides — §12.

The JavaScript implementation (step data model, event handlers, form binding,
API calls) is **out of scope** for this spec and will be specified separately.

---

## 15. Colour Reference Card

```
                     BG            TEXT         BORDER        HOVER/ELEVATED
────────────────────────────────────────────────────────────────────────────
Body                 #06101e       #e7f4ff      —             —
Card                 #0a1528       —            rgba(100,160,220,.18)  —
Panel / planner      #071326       —            rgba(100,160,220,.2)   —
Step (preview)       #09182d       —            rgba(100,160,220,.14)  —
Step (editor)        #09182d       —            rgba(100,160,220,.14)  #0c1e38
Input / select       #09182d       #e7f4ff      rgba(100,160,220,.28)  rgba(100,160,220,.5)
Video frame          #02060c       —            —             —

Action badge         rgba(46,160,67,.15)  #2ea043  rgba(46,160,67,.35)  —
Preflight badge      rgba(31,111,235,.15) #1f6feb  rgba(31,111,235,.35) —
Motion badge         rgba(212,120,42,.15) #d4782a  rgba(212,120,42,.35) —

Danger / delete      rgba(255,95,117,.12) #ff5f75  rgba(255,95,117,.3)  rgba(255,95,117,.18)
Warning              rgba(255,204,102,.08) #ffcc66  rgba(255,204,102,.2) —
Error                rgba(255,95,117,.08) #ff5f75  rgba(255,95,117,.2)  —

Ok / good            —             #50fa7b      —             —
Accent / primary     —             #1f6feb      —             —
Muted / secondary    —             #83a3bd      —             —
Bright               —             #cfe6ff      —             —
```
