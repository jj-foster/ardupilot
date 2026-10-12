# Native X-tail mixer

AI-assisted implementation for the jj-foster diagnostic SITL model, based on
Plane-4.7.1 (`dbe792162d06cab66c3475fd5556bf7a120f119e`).
Scientific calibration and retained acceptance reports live in
quadplane-tailsitter `sim/reports/native-xtail/`.

Set `XT_ENABLE=1` and assign exactly one each of servo functions 190, 191, 192,
193 to upper-left, upper-right, lower-left, lower-right respectively. Disabled
is the default. Reboot after changing enable. MIN/TRIM/MAX and REVERSED perform
servo calibration; +/-4500 means full normalized travel, not 45 physical degrees.
Positive PWM relative to trim corresponds to the diagnostic physics positive
rotation. The imported neutral geometry already includes incidence.

Aircraft FRD demands are positive right-wing-down roll, nose-up pitch and
nose-right yaw. ArduPlane's logical aileron/elevator/rudder signs map directly
to these demands. Tailsitter hover conversion and scaling remain upstream.
The fixed UL/UR/LL/LR matrix rows are `(1,-1,-1)`, `(-1,-1,1)`, `(1,-1,1)`,
`(-1,-1,-1)`. `XT_R_GAIN`, `XT_P_GAIN`, `XT_Y_GAIN` accept finite values 0..2.
Defaults 0.5 are initial travel settings and do not represent equal authority.
Pitch takes priority; roll and yaw share one scale when travel is exhausted.

The output runs after tailsitter selection and forced-flare handling, before
landing surface neutralization and PWM calculation. MANUAL_RCMASK cannot bypass
the four tail channels. Bench surface operation, safety PWM gating, RC/GCS
failsafe mode selection and auto-landing neutralization follow stock policies.
Legacy aileron/elevator/rudder, elevon, V-tail and differential spoiler assignments
conflict with this first implementation and fail arming checks. Invalid or
disabled native assignments are neutralized and fail pre-arm checks.

Forward-flight feedback blocks signed I increments further into an allocator
limit, retaining normal underspeed and unwinding behavior. VTOL feedback maps
aircraft yaw/pitch/roll to VTOL roll/pitch/yaw, augments motor limits at the PID
input and expires after consumption. This conservatively limits a shared PID
even when motors have remaining authority; no combined-authority estimator is
implemented. Zero axis gain generates no allocator saturation flag.

AFS aerodynamic termination latches the allocation of the stock full-positive
logical axis demands so tailsitter output cannot overwrite it. The IO failsafe
uses its corresponding calibrated PWM values. The stock VTOL-land termination
action continues to select QLAND.

XTAL, XTRQ and XTLM logs at 20 Hz record mode, controller selection, body-axis demands,
requested/applied outputs, limiter scale and residuals. Correlate these with
PIDR/PIDP/PIDY, PIQR/PIQP/PIQY, RCOU, TSIT and physics logs. Logging remains under
HAL_LOGGING_ENABLED. AP_PLANE_XTAIL_ENABLED permits compile-time removal.

Allocator tests: `./waf configure --board sitl`,
`./waf --targets tests/test_xtail_allocation`,
`./build/sitl/tests/test_xtail_allocation`.

This model is diagnostic: sparse one-tail aerodynamic data, superposition and
post-stall assumptions are unvalidated. Surface travel saturation does not
detect stall. Hardware flight qualification and dynamic motor/tail allocation
are outside this implementation. Consult retained reports for executed tests
and failures; a compiled binary alone is not flight acceptance.

XTLM records motor and surface limit masks in VTOL roll/pitch/yaw order (bits 0/1/2), without modifying motor flags. Zero-gain axes do not count as active surfaces in motors-only Q-assist.
