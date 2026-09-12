END BRANCH ADAPTER
==================

Purpose
-------
Convert camera_ws fused traffic-light direction into a DR route branch,
without changing camera_ws and without owning any drive/wheel command.

Inputs
------
/drive_mode
/camera/traffic_light_fused/aspect
/camera/traffic_light_fused/confidence
/camera/traffic_light_fused/diagnostics

Output
------
/dr/end_branch  : END_AA / END_AB / NONE

Safety / current state
----------------------
The physical mapping between GREEN_DOWN / GREEN_LEFT and END_AA / END_AB is
not yet confirmed. Therefore BOTH mappings are NONE by default.

Until configured, this node always fails closed and the existing DR follower
keeps its 5 s fallback to END_AA.

After the course mapping is confirmed, edit:
  config/end_branch_adapter.yaml

Example ONLY after confirmation:
  green_down_branch: END_AA
  green_left_branch: END_AB

Do not use that example as an assumed course truth.

Follower connection
-------------------
Launch the DR follower with:
  end_branch_topic:=/dr/end_branch
