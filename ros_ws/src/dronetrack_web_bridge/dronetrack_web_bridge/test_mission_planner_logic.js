/**
 * Smoke tests for mission-planner JS logic (vanilla, no framework).
 *
 * Run:  node test_mission_planner_logic.js
 *
 * Covers: addStep, moveStepUp, moveStepDown, deleteStep, localStorage
 * save/restore, and lint/warnings-panel logic.
 */

// ── tiny assert helper ────────────────────────────────────────────────
var _pass = 0, _fail = 0;

function assert(cond, msg) {
    if (cond) { _pass++; } else { _fail++; console.error('FAIL: ' + msg); }
}

function assertEq(a, b, msg) {
    if (a === b) { _pass++; } else { _fail++; console.error('FAIL: ' + msg + '  expected=' + JSON.stringify(b) + ' got=' + JSON.stringify(a)); }
}

function assertDeep(a, b, msg) {
    if (JSON.stringify(a) === JSON.stringify(b)) { _pass++; }
    else { _fail++; console.error('FAIL: ' + msg + '  expected=' + JSON.stringify(b) + ' got=' + JSON.stringify(a)); }
}

function done() {
    console.log(_pass + ' passed, ' + _fail + ' failed');
    process.exit(_fail > 0 ? 1 : 0);
}

// ── step schema (mirrors mission_plan_model.py STEP_SCHEMA) ────────────
var STEP_SCHEMA = {
    takeoff:          { label: 'Take Off',          category: 'action',    params: { altitude_m:    { type:'float', default:3.0  } } },
    prime_offboard:   { label: 'Prime Offboard',    category: 'preflight', params: { hold_s:        { type:'float', default:1.5  } } },
    scan:             { label: 'Scan / Seek',       category: 'motion',    params: { direction:     { type:'enum', default:'ccw',  options:['ccw','cw'] },
                                                                                      yaw_deg:       { type:'float', default:180.0 },
                                                                                      yaw_rate_deg_s:{ type:'float', default:20.0  },
                                                                                      until:         { type:'enum', default:'locked', options:['locked','none'] },
                                                                                      timeout_s:     { type:'float', default:12.0  } } },
    track_center:     { label: 'Track Center',      category: 'motion',    params: { distance_m:    { type:'float', default:3.0  },
                                                                                      until:         { type:'enum', default:'centered', options:['centered','approach_done','none'] },
                                                                                      timeout_s:     { type:'float', default:15.0  } } },
    approach:         { label: 'Approach Target',   category: 'motion',    params: { distance_m:    { type:'float', default:2.0  },
                                                                                      timeout_s:     { type:'float', default:20.0  } } },
    orbit:            { label: 'Orbit',             category: 'motion',    params: { radius_m:      { type:'float', default:2.0  },
                                                                                      speed_m_s:     { type:'float', default:0.4  },
                                                                                      revolutions:   { type:'float', default:1.0  },
                                                                                      timeout_s:     { type:'float', default:45.0  } } },
    rtl:              { label: 'Return to Launch',  category: 'action',    params: { timeout_s:     { type:'float', default:15.0  } } },
    land:             { label: 'Land',              category: 'action',    params: { timeout_s:     { type:'float', default:10.0  } } },
    hold:             { label: 'Hold',              category: 'preflight', params: { status:        { type:'str',   default:'holding position' },
                                                                                      timeout_s:     { type:'float', default:0.0   } } }
};

// ── functions under test ───────────────────────────────────────────────

/** Build default params object for a verb. */
function defaultParams(verb) {
    var schema = STEP_SCHEMA[verb];
    if (!schema) return {};
    var p = {};
    for (var k in schema.params) {
        p[k] = schema.params[k]['default'];
    }
    return p;
}

/** Create a new step object { type, params }. */
function createStep(verb) {
    if (!STEP_SCHEMA[verb]) return null;
    return { type: verb, params: defaultParams(verb) };
}

/** Add a step at the end of the plan. */
function addStep(plan, verb) {
    var step = createStep(verb);
    if (!step) return false;
    plan.steps.push(step);
    return true;
}

/** Move a step up by one position (swap with previous). */
function moveStepUp(plan, index) {
    if (index <= 0 || index >= plan.steps.length) return false;
    var tmp = plan.steps[index - 1];
    plan.steps[index - 1] = plan.steps[index];
    plan.steps[index] = tmp;
    return true;
}

/** Move a step down by one position (swap with next). */
function moveStepDown(plan, index) {
    if (index < 0 || index >= plan.steps.length - 1) return false;
    var tmp = plan.steps[index];
    plan.steps[index] = plan.steps[index + 1];
    plan.steps[index + 1] = tmp;
    return true;
}

/** Delete a step at the given index. */
function deleteStep(plan, index) {
    if (index < 0 || index >= plan.steps.length) return false;
    plan.steps.splice(index, 1);
    return true;
}

/** Serialize plan to JSON and store in localStorage. */
function savePlan(plan, key, storage) {
    storage = storage || (typeof localStorage !== 'undefined' ? localStorage : null);
    if (!storage) return false;
    try {
        storage.setItem(key, JSON.stringify(plan));
        return true;
    } catch (e) {
        return false;
    }
}

/** Deserialize plan from localStorage. */
function loadPlan(key, storage) {
    storage = storage || (typeof localStorage !== 'undefined' ? localStorage : null);
    if (!storage) return null;
    try {
        var raw = storage.getItem(key);
        if (raw === null || raw === undefined) return null;
        return JSON.parse(raw);
    } catch (e) {
        return null;
    }
}

/** Lint the plan and return an array of warning strings.
 *  Mirrors mission_plan_model.lint_steps(). */
function lintPlan(plan) {
    var warnings = [];
    var steps = plan.steps || [];

    if (steps.length === 0) {
        warnings.push('Plan is empty');
        return warnings;
    }

    var hasPrime = false;
    var hasMotion = false;
    var seenPrime = false;

    for (var i = 0; i < steps.length; i++) {
        var step = steps[i];
        var verb = step.type || '';
        var isMotion = (verb === 'scan' || verb === 'track_center' || verb === 'approach' || verb === 'orbit');

        if (isMotion) {
            hasMotion = true;
            if (!seenPrime) {
                warnings.push('Step ' + (i+1) + ' (' + verb + '): motion step before prime_offboard — add a prime_offboard step first');
            }
        }

        if (verb === 'prime_offboard') {
            seenPrime = true;
            hasPrime = true;
        }

        if (verb === 'scan' || verb === 'approach' || verb === 'orbit') {
            var timeout = (step.params && step.params.timeout_s !== undefined) ? step.params.timeout_s : undefined;
            if (timeout === undefined || timeout === null) {
                warnings.push('Step ' + (i+1) + ' (' + verb + '): no timeout set — step may run indefinitely');
            }
        }

        if (verb === 'scan') {
            var until = (step.params && step.params.until !== undefined) ? step.params.until : undefined;
            var t = (step.params && step.params.timeout_s !== undefined) ? step.params.timeout_s : undefined;
            if ((until === undefined || until === null || until === 'none') && (t === undefined || t === null)) {
                warnings.push('Step ' + (i+1) + ' (scan): has neither \'until\' nor \'timeout_s\' — the step could run indefinitely');
            }
        }
    }

    if (hasMotion && !hasPrime) {
        warnings.push('Plan contains motion steps but no prime_offboard — offboard mode will not be enabled');
    }

    return warnings;
}

// ── localStorage mock ──────────────────────────────────────────────────
function makeMockStorage() {
    var store = Object.create(null);
    return {
        getItem: function (k) { return Object.prototype.hasOwnProperty.call(store, k) ? store[k] : null; },
        setItem: function (k, v) { store[k] = String(v); },
        removeItem: function (k) { delete store[k]; }
    };
}

// ═══════════════════════════════════════════════════════════════════════
//  TESTS
// ═══════════════════════════════════════════════════════════════════════

// ── (1) addStep adds a step with correct verb & default params ─────────
(function () {
    var plan = { steps: [] };
    var ok = addStep(plan, 'takeoff');
    assert(ok, 'addStep takeoff returns true');
    assertEq(plan.steps.length, 1, 'plan has 1 step after addStep takeoff');
    assertEq(plan.steps[0].type, 'takeoff', 'step type is takeoff');
    assertDeep(plan.steps[0].params, { altitude_m: 3.0 }, 'takeoff default params');
})();

(function () {
    var plan = { steps: [] };
    addStep(plan, 'scan');
    assertEq(plan.steps.length, 1, 'plan has 1 step after addStep scan');
    assertDeep(plan.steps[0].params, {
        direction: 'ccw', yaw_deg: 180.0, yaw_rate_deg_s: 20.0,
        until: 'locked', timeout_s: 12.0
    }, 'scan default params match schema');
})();

(function () {
    var plan = { steps: [] };
    addStep(plan, 'orbit');
    assertDeep(plan.steps[0].params, {
        radius_m: 2.0, speed_m_s: 0.4, revolutions: 1.0, timeout_s: 45.0
    }, 'orbit default params match schema');
})();

(function () {
    var plan = { steps: [] };
    addStep(plan, 'hold');
    assertDeep(plan.steps[0].params, {
        status: 'holding position', timeout_s: 0.0
    }, 'hold default params');
})();

(function () {
    var plan = { steps: [] };
    var ok = addStep(plan, 'bogus_verb');
    assert(!ok, 'addStep bogus_verb returns false');
    assertEq(plan.steps.length, 0, 'bogus_verb does not add step');
})();

(function () {
    // Verify all 9 verbs produce a step with correct type and non-empty params
    var verbs = ['takeoff', 'prime_offboard', 'scan', 'track_center',
                 'approach', 'orbit', 'rtl', 'land', 'hold'];
    var plan = { steps: [] };
    verbs.forEach(function (v) {
        addStep(plan, v);
    });
    assertEq(plan.steps.length, 9, 'all 9 verbs added');
    for (var i = 0; i < verbs.length; i++) {
        assertEq(plan.steps[i].type, verbs[i], 'step ' + i + ' type is ' + verbs[i]);
        assert(typeof plan.steps[i].params === 'object' && plan.steps[i].params !== null,
               'step ' + i + ' params is object');
    }
})();

// ── (2) moveStepUp / moveStepDown reordering ──────────────────────────
(function () {
    var plan = { steps: [] };
    addStep(plan, 'takeoff');
    addStep(plan, 'prime_offboard');
    addStep(plan, 'scan');

    // move step 2 (scan) up
    var ok = moveStepUp(plan, 2);
    assert(ok, 'moveStepUp(2) returns true');
    assertEq(plan.steps[0].type, 'takeoff', 'step 0 still takeoff');
    assertEq(plan.steps[1].type, 'scan', 'step 1 is now scan (moved up)');
    assertEq(plan.steps[2].type, 'prime_offboard', 'step 2 is now prime_offboard');
})();

(function () {
    var plan = { steps: [] };
    addStep(plan, 'takeoff');
    addStep(plan, 'scan');
    addStep(plan, 'orbit');

    // move step 1 (scan) down
    var ok = moveStepDown(plan, 1);
    assert(ok, 'moveStepDown(1) returns true');
    assertEq(plan.steps[0].type, 'takeoff', 'step 0 unchanged');
    assertEq(plan.steps[1].type, 'orbit', 'step 1 is now orbit (moved down)');
    assertEq(plan.steps[2].type, 'scan', 'step 2 is now scan');
})();

(function () {
    // boundary: moveStepUp on index 0 should fail
    var plan = { steps: [] };
    addStep(plan, 'takeoff');
    addStep(plan, 'land');
    assert(!moveStepUp(plan, 0), 'moveStepUp(0) returns false (boundary)');
    assertEq(plan.steps[0].type, 'takeoff', 'first step unchanged');
})();

(function () {
    // boundary: moveStepDown on last index should fail
    var plan = { steps: [] };
    addStep(plan, 'takeoff');
    addStep(plan, 'land');
    assert(!moveStepDown(plan, 1), 'moveStepDown(last) returns false');
    assertEq(plan.steps[1].type, 'land', 'last step unchanged');
})();

(function () {
    // boundary: moveStepUp on negative index
    var plan = { steps: [] };
    addStep(plan, 'takeoff');
    assert(!moveStepUp(plan, -1), 'moveStepUp(-1) returns false');
})();

(function () {
    // boundary: moveStepDown out of range
    var plan = { steps: [] };
    addStep(plan, 'takeoff');
    assert(!moveStepDown(plan, 5), 'moveStepDown(5) on single-element returns false');
})();

// ── (3) deleteStep removes the right index ─────────────────────────────
(function () {
    var plan = { steps: [] };
    addStep(plan, 'takeoff');
    addStep(plan, 'prime_offboard');
    addStep(plan, 'scan');
    addStep(plan, 'land');

    assert(deleteStep(plan, 1), 'deleteStep(1) returns true');
    assertEq(plan.steps.length, 3, 'length is 3 after delete');
    assertEq(plan.steps[0].type, 'takeoff', 'step 0 unchanged');
    assertEq(plan.steps[1].type, 'scan', 'step 1 is now scan (was index 2)');
    assertEq(plan.steps[2].type, 'land', 'step 2 is land');
})();

(function () {
    // delete last element
    var plan = { steps: [] };
    addStep(plan, 'takeoff');
    addStep(plan, 'land');
    assert(deleteStep(plan, 1), 'deleteStep(last) returns true');
    assertEq(plan.steps.length, 1, 'length 1 after delete last');
    assertEq(plan.steps[0].type, 'takeoff', 'remaining is takeoff');
})();

(function () {
    // delete first element
    var plan = { steps: [] };
    addStep(plan, 'takeoff');
    addStep(plan, 'land');
    assert(deleteStep(plan, 0), 'deleteStep(0) returns true');
    assertEq(plan.steps.length, 1, 'length 1 after delete first');
    assertEq(plan.steps[0].type, 'land', 'remaining is land');
})();

(function () {
    // delete out of range
    var plan = { steps: [] };
    addStep(plan, 'takeoff');
    assert(!deleteStep(plan, 5), 'deleteStep(5) out of range returns false');
    assertEq(plan.steps.length, 1, 'length unchanged on out-of-range delete');
})();

(function () {
    // delete negative index
    var plan = { steps: [] };
    addStep(plan, 'takeoff');
    assert(!deleteStep(plan, -1), 'deleteStep(-1) returns false');
    assertEq(plan.steps.length, 1, 'length unchanged on negative delete');
})();

// ── (4) localStorage save / restore (mock) ────────────────────────────
(function () {
    var storage = makeMockStorage();
    var plan = { steps: [] };
    addStep(plan, 'takeoff');
    addStep(plan, 'prime_offboard');
    addStep(plan, 'scan');
    addStep(plan, 'orbit');

    var saved = savePlan(plan, 'test_plan', storage);
    assert(saved, 'savePlan returns true with mock storage');

    var restored = loadPlan('test_plan', storage);
    assert(restored !== null, 'loadPlan returns non-null');
    assertEq(restored.steps.length, 4, 'restored plan has 4 steps');
    assertEq(restored.steps[0].type, 'takeoff', 'restored step 0 is takeoff');
    assertDeep(restored.steps[0].params, { altitude_m: 3.0 }, 'restored takeoff params match');
    assertEq(restored.steps[2].type, 'scan', 'restored step 2 is scan');
    assertDeep(restored.steps[2].params, {
        direction: 'ccw', yaw_deg: 180.0, yaw_rate_deg_s: 20.0,
        until: 'locked', timeout_s: 12.0
    }, 'restored scan params match');
})();

(function () {
    // loading a nonexistent key returns null
    var storage = makeMockStorage();
    assertEq(loadPlan('nonexistent', storage), null, 'loadPlan nonexistent returns null');
})();

(function () {
    // loading corrupt JSON returns null
    var storage = makeMockStorage();
    storage.setItem('bad_plan', '{not valid json');
    assertEq(loadPlan('bad_plan', storage), null, 'loadPlan corrupt json returns null');
})();

(function () {
    // empty plan round-trips correctly
    var storage = makeMockStorage();
    var plan = { steps: [] };
    savePlan(plan, 'empty_plan', storage);
    var restored = loadPlan('empty_plan', storage);
    assertDeep(restored, { steps: [] }, 'empty plan round-trips');
})();

(function () {
    // storage isolation — two keys don't interfere
    var st = makeMockStorage();
    var a = { steps: [{ type: 'takeoff', params: { altitude_m: 5.0 } }] };
    var b = { steps: [{ type: 'land', params: { timeout_s: 20.0 } }] };
    savePlan(a, 'plan_a', st);
    savePlan(b, 'plan_b', st);
    assertEq(loadPlan('plan_a', st).steps[0].type, 'takeoff', 'plan_a isolated');
    assertEq(loadPlan('plan_b', st).steps[0].type, 'land', 'plan_b isolated');
})();

// ── (5) lint / warnings panel logic ────────────────────────────────────
(function () {
    // empty plan
    var warnings = lintPlan({ steps: [] });
    assert(warnings.length > 0, 'empty plan produces warnings');
    assert(warnings.indexOf('Plan is empty') >= 0, 'empty plan warns "Plan is empty"');
})();

(function () {
    // no step type named "takeoff" is not a special lint — verifying plan with takeoff only
    var plan = { steps: [] };
    addStep(plan, 'takeoff');
    var warnings = lintPlan(plan);
    assertEq(warnings.length, 0, 'takeoff-only plan has no warnings');
})();

(function () {
    // motion step before prime_offboard
    var plan = { steps: [] };
    addStep(plan, 'scan');
    addStep(plan, 'prime_offboard');
    var warnings = lintPlan(plan);
    assert(warnings.length >= 1, 'scan before prime produces warning');
    assert(warnings.some(function (w) { return w.indexOf('motion step before prime_offboard') >= 0; }),
           'contains motion-before-prime warning');
})();

(function () {
    // plan with motion but no prime_offboard at all
    var plan = { steps: [] };
    addStep(plan, 'scan');
    addStep(plan, 'orbit');
    var warnings = lintPlan(plan);
    assert(warnings.some(function (w) { return w.indexOf('no prime_offboard') >= 0; }),
           'contains no-prime_offboard-at-all warning');
})();

(function () {
    // clean plan: prime_offboard then motion
    var plan = { steps: [] };
    addStep(plan, 'prime_offboard');
    addStep(plan, 'track_center');
    var warnings = lintPlan(plan);
    assertEq(warnings.length, 0, 'prime then motion has no warnings');
})();

(function () {
    // scan with no timeout and until=none
    var plan = { steps: [] };
    addStep(plan, 'prime_offboard');
    var scanStep = createStep('scan');
    scanStep.params.until = 'none';
    scanStep.params.timeout_s = undefined;
    plan.steps.push(scanStep);
    var warnings = lintPlan(plan);
    assert(warnings.some(function (w) { return w.indexOf('neither') >= 0; }),
           'scan with until=none and no timeout warns about indefinite run');
})();

(function () {
    // approach with no timeout
    var plan = { steps: [] };
    addStep(plan, 'prime_offboard');
    var appr = createStep('approach');
    appr.params.timeout_s = undefined;
    plan.steps.push(appr);
    var warnings = lintPlan(plan);
    assert(warnings.some(function (w) { return w.indexOf('no timeout set') >= 0; }),
           'approach with no timeout produces warning');
})();

(function () {
    // orbit with no timeout
    var plan = { steps: [] };
    addStep(plan, 'prime_offboard');
    var orb = createStep('orbit');
    orb.params.timeout_s = undefined;
    plan.steps.push(orb);
    var warnings = lintPlan(plan);
    assert(warnings.some(function (w) { return w.indexOf('no timeout set') >= 0; }),
           'orbit with no timeout produces warning');
})();

(function () {
    // full valid plan: takeoff, prime_offboard, track_center, approach, orbit, rtl, land
    var plan = { steps: [] };
    ['takeoff', 'prime_offboard', 'track_center', 'approach', 'orbit', 'rtl', 'land'].forEach(function (v) {
        addStep(plan, v);
    });
    var warnings = lintPlan(plan);
    assertEq(warnings.length, 0, 'full orbit plan with defaults has no lint warnings');
})();

(function () {
    // rtl and land don't trigger motion-before-prime warnings
    var plan = { steps: [] };
    addStep(plan, 'rtl');
    addStep(plan, 'land');
    var warnings = lintPlan(plan);
    assert(warnings.indexOf('Plan is empty') < 0, 'non-motion verbs do not produce false warnings');
})();

(function () {
    // hold step is category preflight but is not prime_offboard; motion after hold but before prime
    // should still warn
    var plan = { steps: [] };
    addStep(plan, 'hold');
    addStep(plan, 'scan');
    var warnings = lintPlan(plan);
    assert(warnings.some(function (w) { return w.indexOf('motion step before prime_offboard') >= 0; }),
           'hold then scan (no prime) produces motion-before-prime warning');
    assert(warnings.some(function (w) { return w.indexOf('no prime_offboard') >= 0; }),
           'hold then scan also produces no-prime_offboard-at-all warning');
})();

// ── report ─────────────────────────────────────────────────────────────
done();
