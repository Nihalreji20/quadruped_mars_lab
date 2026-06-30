#!/usr/bin/env python3
"""
FREE_BASE_TWO_STEP_TEST.py
==========================
Stage 6B -- Four-Step Full Crawl Gait Validation  (1 -> 2 -> 4 -> 3)

CHANGES FROM THE TWO-STEP VERSION
    The original file only implemented Steps 1 and 2.  Steps 3 and 4 have
    been added using the EXACT same pattern as Steps 1 and 2 — identical
    lift formula, identical sweep formula, identical translate formula.
    No new logic was introduced.

    Specific changes:
      1. generate_two_step_sequence() renamed generate_full_gait_sequence()
         and extended with Steps 3 (Leg 4) and 4 (Leg 3) — each block is a
         copy-paste of the Step 1/2 block with only the leg-id and phase-name
         strings changed.

      2. main() execution loop updated:
         - current_step_context now tracks 1-4 instead of 1-2.
         - active_leg map extended to cover steps 3 and 4.
         - Diagnostic dicts and boundary keys extended for STEP_3 / STEP_4.
         - Final report prints all four step displacements and pass/fail uses
           STEP_4 as the terminal boundary.
         - pass_additive checks disp_total > disp1 (same logic, wider span).

    Everything else — LEG_CONFIG, IK solver, sign tables, RADIAL_REACH
    conversion, LIFT/SWEEP/LOWER formulae, translate formula — is
    character-for-character identical to crawl_gait_test.py.

GAIT ORDER
    1 (FL) -> 2 (BR) -> 4 (BL) -> 3 (FR)
"""


    overall_max_pitch = max(max_pitch.values())

    print(f"  [Final Report]")
    print(f"  1. Total Displacement             : {disp_total:.1f} mm")
    print(f"  2. Overall Max Roll               : {overall_max_roll:.1f}°")
    print(f"  3. Overall Max Pitch              : {overall_max_pitch:.1f}°")
    print(f"  4. Maximum Joint Error            : {max_joint_error:.1f}°")
    print(f"  5. IK Failures                    : {ik_failures}")
    print(f"  6. Height Drift                   : {max_z_drift:.3f} m")

    overall = pass_ik and pass_fall and pass_err and pass_additive

    print("\n" + "=" * 70)
    if overall:
        print("  STAGE 6B COMPLETE — FULL CRAWL GAIT VALIDATED.")
        print("  Robot preserved stance state across all four steps and gained")
        print("  additive displacement using Stage 4 sine-wave lift mechanics.")
    else:
        reasons = []
        if not pass_ik:       reasons.append(f"IK failures: {ik_failures}")
        if not pass_fall:     reasons.append(f"Robot fell below {MIN_Z_HEIGHT_M} m")
        if not pass_err:      reasons.append(f"Joint error {max_joint_error:.1f}° > {MAX_JOINT_ERR_DEG}°")
        if not pass_additive: reasons.append("Step 4 did not produce additive displacement over Step 1")
        print("  STAGE 6B FAILED.")
        for r in reasons:
            print(f"    - {r}")

    print("\n  Holding final posture. Close window to exit.")

    try:
        while p.isConnected():
            p.stepSimulation()
            time.sleep(1.0 / CONTROL_RATE_HZ)
    except (KeyboardInterrupt, p.error):
        pass
    finally:
        if p.isConnected():
            p.disconnect()

if __name__ == "__main__":
    main()
