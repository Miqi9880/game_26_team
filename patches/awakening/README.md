# armor_detector.patched (2026-09-04)
Applied onto third/awakening/src/tasks/auto_aim/armor_detect/armor_detector.cpp
(third/awakening is compiled directly & not tracked in git; rollback via
 dev/backup_awakening_* on the robot, original archived in this repo? add below)
Changes:
- light-level floors: min_light_length_px / min_light_area_px
- armor pair geometric filter is_armor_pair(): length ratio, center/top/bottom y-diff,
  angle diff, endpoint overlap (kills cross-plate A.right+B.left pairings)
- CV candidate dedup by bbox IoU (keep larger; uses cv->duplicated)
- optional min_number_conf gate
Params (auto_aim_params.yaml armor_detector.cv.*) with rollback-friendly defaults.
