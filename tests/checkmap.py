"""기존 시험의 체크리스트 항목 표 — 시험 파일을 고치지 않고 A~D 를 붙인다.

  A 정상 동작 / B 경계·파라미터 / C 실패 시 안전 / D 노드 입출력 (TEST_PLAN.md 3.1)

새로 쓰는 시험은 이 표 대신 시험 함수에 @pytest.mark.check('A') 를 직접 붙인다.
시험 함수의 marker 가 있으면 그쪽이 우선한다. 둘 다 없으면 'untagged' 로 표시된다.
값은 공백으로 구분한 시험 이름 (test_ 접두사 생략, parametrize 는 원래 이름).
"""

_CLEAR_TRAJECTORY = (
    'clear_trajectory_service_registered '
    'clear_trajectory_clears_populated_trail_and_response_content '
    'clear_trajectory_immediate_empty_marker_and_preserves_target_markers '
    'clear_trajectory_preserves_mission_and_control_state_and_no_cmd_vel '
    'clear_trajectory_records_new_trail_on_next_odom ')

CHECKS = {
    # ── behaviors ──
    'test_avoid_fsm': {
        'A': 'curvature_straight_is_zero curvature_avoid_path_positive_and_bounded '
             'follow_straight_full_speed_no_steer follow_curvature_slows_down '
             'first_tick_stops_and_acquires second_tick_follows backstop_regenerates '
             'violation_resolved_by_ladder trigger_a_fires_after_passing_r1 '
             'trigger_a2_fires_on_none_to_finite trigger_a2_fires_on_r1_drop '
             'violation_repaired_without_regen escape_backs_up escape_steers_stern_to_low_dri',
        'B': 'follow_caps_target_to_waypoint_when_close follow_large_heading_error_slow_forward '
             'follow_slows_near_waypoint follow_combines_large_heading_and_waypoint_slowdown_in_old_order '
             'reset_clears_plan trigger_a2_ignores_small_drop violation_regen_when_repair_cannot_fix '
             'violation_wait_slows_when_margin_left escape_no_chatter_before_min_time '
             'escape_recovers_after_min_time_and_streak escape_streak_resets_on_failure',
        'C': 'escape_on_close_violation escape_after_no_plan_retry_timeout '
             'escape_surrounded_still_commands escape_rear_softening_still_repels',
    },
    'test_behavior_buoys': {
        'A': 'straight_ahead left_is_positive_bearing yaw_rotates_frame boat_offset_translates '
             'roundtrip_with_project_to_world visible_sorted_by_range',
        'B': 'visible_filters_by_color visible_drops_far_buoys visible_drops_behind_buoys',
        'C': 'none_when_no_odom',
    },
    'test_bspline_planner': {
        'A': 'simple_scene_generates_clean_path spline_endpoints_and_start_tangent '
             'check_detects_and_repair_fixes arc_pick_prefers_lower_dri_inside_angle_slack '
             'hard_filter_picks_gap_not_detour radii_monotone_including_p1 '
             'stationary_p1_uses_heading_and_min_dist p1_direction_is_heading_not_velocity '
             'p1_length_uses_surge_component_only progress_idx_monotone samples_spacing_is_half_resolution',
        'B': 'arc_pick_prefers_straight_when_risk_gap_is_within_slack '
             'arc_pick_does_not_chase_dri_minimum_outside_angle_slack radii_monotone_in_r7_fallback '
             'p1_reverse_motion_collapses_to_min_dist waypoint_inside_clamps_r_end '
             'waypoint_far_r_end_is_grid_bound ref_obstacle_rear_hard_cut '
             'ref_obstacle_bearing_weighted_distance ladder_independent_of_near_obstacle '
             'ladder_spread_when_obstacle_at_horizon r_end_derived_from_actual_grid_geometry',
        'C': 'r7_fallback_is_straight_to_waypoint generate_returns_none_when_surrounded '
             'repair_fails_without_improvement repair_respects_fov_limit_and_terminates',
    },
    'test_dock_target': {
        'A': 'entrance_xy_becomes_fsm_bearing_and_range three_waypoints_are_collinear_and_in_order',
        'B': 'marker_height_is_ignored_for_horizontal_berth_distance waypoint_at_current_boat_position_is_valid',
        'C': 'invalid_or_behind_entrance_fails_closed dock_plan_rejects_reversed_or_too_shallow_geometry',
    },
    'test_docking_fsm': {
        'A': 'approach_uses_global_staging_goal pre_entry_is_first_local_waypoint '
             'mouth_must_be_reached_before_berth berth_uses_marker_distance_not_mouth_distance',
        'B': 'pre_entry_does_not_drive_with_large_bearing_error align_requires_continuous_axis_hold '
             'berth_requires_continuous_confirmation reverse_requires_time_distance_and_staging',
        'C': 'acquire_requires_plan_and_times_out berth_point_reached_without_one_meter_mark_aborts '
             'plan_loss_inside_slot_aborts_and_stays_stopped axis_error_inside_slot_aborts reverse_timeout_aborts',
    },
    'test_dri': {
        'A': 'peak_is_on_the_obstacle_cell spread_is_isotropic closer_obstacle_is_stronger '
             'frontal_obstacle_is_stronger_than_beam closer_obstacle_gets_bigger_radius '
             'cells_combine_by_max_not_sum passable_gap_is_not_blocked_by_distant_neighbours '
             'risk_at_many_matches_risk_at grid_geometry_is_preserved',
        'B': 'gain_shrinks_far_side_without_distorting_shape gain_keeps_near_frontal_wider_than_far_side '
             'below_threshold_cells_contribute_nothing default_threshold_requires_four_hits '
             'max_keeps_dri_bounded_by_peak_amplitude zero_gains_reduce_to_uniform_gaussian '
             'build_dri_under_10ms empty_grid_is_all_zero',
        'C': 'unknown_cells_contribute_nothing risk_at_outside_window_is_inf',
    },
    'test_gate_corridor': {
        'A': 'three_pairs_selected_in_order_and_beyond_count_is_configurable '
             'gate_path_passes_midpoint_and_dri_rejects_obstacle_on_it crossing_requires_going_between_posts',
        'B': 'lookahead_safety_checks_the_goal_when_follower_targets_it',
        'C': 'live_scan_must_show_both_posts_even_if_old_map_keeps_them '
             'missing_post_or_wall_is_not_a_gate unobserved_cells_block_the_gate_path',
    },
    'test_gate_follower': {
        'A': 'three_pairs_pass_before_completion_and_output_stays_preview_only',
        'C': 'missing_third_pair_and_stale_scan_fail_closed '
             'cached_gate_stops_when_one_post_disappears_from_live_scan',
    },
    'test_los_controller': {
        'A': 'bspline_path_has_analytic_curvature_and_reaches_end circle_lookahead_and_curvature_wrap_seam '
             'circle_counts_one_lap_and_stops',
        'B': 'curvature_inverse_speed_reduces_thrust circle_can_run_continuously_and_reverse_direction '
             'circle_supports_fractional_laps thrust_ratio_scales_los_and_rejects_invalid_values',
        'C': 'invalid_path_rejected',
    },
    'test_motion_controller': {
        'A': 'aligned_world_velocity_commands_forward_thrust leftward_velocity_generates_positive_yaw_command '
             'yaw_rate_is_damped',
        'B': 'large_heading_error_caps_speed reverse_vector_uses_stern_as_heading_reference',
        'C': 'zero_velocity_stops_without_undefined_heading invalid_velocity_calibration_is_rejected',
    },
    'test_search_circler_los': {
        'B': 'search_uses_stable_circle_path_and_keeps_orbiting_outside_entry_radius',
    },
    'test_waypoint_controller': {
        'A': 'waypoint_is_meters_and_degrees approach_turns_toward_target_and_slows_near_it '
             'aligns_final_heading_and_then_stops position_only_arrives_without_heading_alignment',
        'B': 'thrust_ratio_is_bounded_and_scales_approach thrust_ratio_scales_heading_alignment '
             'position_only_keeps_old_low_forward_turn position_only_slow_radius_can_match_station_behavior '
             'heading_wrap_and_drift_reset_arrival',
        'C': 'target_behind_boat_does_not_drive_forward invalid_odom_cannot_finish_goal',
    },
    # ── perception ──
    'test_buoy_tracker': {
        'A': 'project_forward_at_origin project_left_is_positive_y project_yaw_rotates_body '
             'project_translates_by_boat_position confirmed_after_n_hits nearby_detections_merge_one_track '
             'distant_detections_are_separate_tracks color_majority_vote id_stable_across_frames '
             'ema_smooths_position',
        'B': 'gate_radius_boundary_inside_merges gate_radius_boundary_outside_spawns '
             'two_detections_same_spot_claim_distinct_tracks new_tracks_get_incrementing_ids '
             'id_not_reused_after_prune ema_alpha_one_takes_latest confirmed_only_returns_confirmed '
             'reobservation_keeps_track_alive',
        'C': 'single_hit_not_confirmed prune_after_drop_time track_color_defaults_unknown_when_empty '
             'empty_update_returns_empty',
    },
    'test_depth_utils': {
        'A': 'realsense_16uc1_millimetres_to_metres gazebo_32fc1_is_already_metres',
        'C': 'unknown_encoding_is_rejected distance_ignores_zero_and_non_finite_values '
             'distance_outside_image_is_invalid',
    },
    'test_dock_geometry': {
        'A': 'mask_median_depth_and_plane_normal entrance_is_flattened_boat_local_xy',
        'C': 'insufficient_depth_fails_closed',
    },
    # ── hardware: 위치 추정 ──
    'test_indoor_lidar_odom': {
        'A': 'quaternion_yaw_conversion odom_angular_rate_comes_from_panel_not_imu '
             'each_lidar_panel_measurement_sets_odom_heading calibration_estimates_bias_and_uses_lidar_yaw '
             'lidar_yaw_truth_mode_directly_sets_odom_without_imu',
        'B': 'resolve_pi_ambiguous_yaw_follows_imu_reference resolve_pi_ambiguous_yaw_keeps_matching_branch '
             'lidar_180_flip_is_aligned_to_imu_heading calibration_uses_180_degree_branch_at_start '
             'rejected_lidar_yaw_does_not_delay_next_correction',
        'C': 'does_not_publish_before_lidar_pose stale_imu_keeps_lidar_odom_and_previous_branch '
             'stale_lidar_stops_odom_until_reacquisition invalid_lidar_orientation_is_rejected '
             'stale_lidar_reacquisition_does_not_reset_imu_yaw calibration_rejects_rotating_boat '
             'calibration_rejects_heading_far_from_180_degree_reference',
        'D': 'publishes_fused_pose_when_both_inputs_are_fresh',
    },
    'test_lidar_boat_tracker': {
        'A': 'scan_to_pose_detects_base_link_facing_default_heading corner_gives_full_heading',
        'B': 'a3_720_beam_scan_detects_boat_eight_meters_away reacquire_after_loss_follows_last_yaw',
        'C': 'short_reflection_is_not_a_panel',
    },
    'test_lidar_marker_pose': {
        'A': 'right_wall_laser_points_into_pool_coordinates estimate_base_pose_for_axis_aligned_markers '
             'estimate_base_pose_for_rotated_markers selects_port_thin_and_starboard_thick_at_sixty_centimeters',
        'B': 'yaw_gate_handles_pi_wraparound',
        'C': 'rejects_pair_outside_separation_tolerance rejects_implausible_jump_from_previous_pose',
    },
    'test_lidar_panel_pose': {
        'A': 'fits_rotated_panel_and_midpoint',
        'B': 'fits_dotted_panel_with_small_lidar_noise maintains_previous_heading_branch_at_180_degrees '
             'prefers_expected_panel_length_over_other_line',
        'C': 'rejects_short_or_curved_reflections',
    },
    'test_lidar_sail_pose': {
        'A': 'recovers_full_yaw_when_both_panels_visible single_panel_puts_corner_at_line_end_and_covers_points',
        'B': 'tracks_every_heading_at_far_range_with_previous_pose '
             'default_heading_resolves_ambiguity_without_previous_pose '
             'single_line_follows_reference_yaw_instead_of_flipping confirmed_l_overrides_wrong_reference_yaw '
             'one_or_two_points_on_other_panel_pick_the_right_l',
        'C': 'rejects_unrelated_short_object',
    },
    'test_pose_velocity': {
        'A': 'normalize_angle_wraps_across_pi world_to_body_is_inverse_of_planner_rotation '
             'constant_velocity_converges velocity_is_body_frame_not_world stationary_stays_zero',
        'B': 'yaw_rate_fallback_tracks_rotation yaw_rate_fallback_survives_pi_crossing '
             'longer_window_reduces_noise filter_further_reduces_noise',
        'C': 'detection_glitch_is_rejected reset_prevents_differencing_across_gap',
    },
    'test_yaw_bias_ekf': {
        'A': 'predict_integrates_bias_corrected_gyro lidar_updates_converge_constant_gyro_bias',
        'B': 'update_uses_shortest_angle_across_wraparound reset_yaw_preserves_learned_bias',
        'C': 'large_lidar_innovation_is_rejected',
    },
    'test_imu_dead_reckoning': {
        'A': 'rotate_vector_quarter_turn_about_z stationary_calibration_estimates_sensor_bias '
             'one_second_constant_acceleration_integrates_position',
        'B': 'reset_pose_preserves_calibration_and_zeros_motion',
        'C': 'large_timestamp_gap_does_not_jump_position',
    },
    'test_onboard_scan_filter': {
        'A': '120_to_240_in_published_scan_frame quaternion_to_roll_pitch',
        'B': 'negative_increment_and_empty_intensities is_tilted_threshold_3_deg '
             'is_tilted_asymmetric_pitch_minus_4_to_plus_1',
        'C': 'invalidate_all_ranges',
    },
    # ── hardware: 제어 ──
    'test_thruster_backend': {
        'A': 'ratio_to_pwm_neutral ratio_to_pwm_forward_max ratio_to_pwm_reverse_max '
             'thruster_mixer_straight thruster_mixer_turn slew_rate_limiter',
        'B': 'ratio_to_pwm_deadband_jump ratio_to_pwm_clamping thruster_mixer_trim_and_invert '
             'thruster_mixer_final_output_limit serial_backend_protocol_formatting dummy_backend',
        'C': 'pca9685_send_fails_when_not_open',
    },
    'test_thruster_driver_teleop': {
        'C': 'jetson_caps_remote_commands_and_rejects_nan',
        'D': 'laptop_mode_overrides_auto_and_timeout_stays_neutral',
    },
    'test_health': {
        'A': 'rate_and_frame_are_reported',
        'B': 'disabled_topic_is_ok missing_topic_transitions_after_grace_period',
        'C': 'stale_topic_is_error',
    },
    # ── hardware: 수조 시험 도구 ──
    'test_straight_line_test': {
        'A': 'cruise_control_on_track cross_track_correction goal_arrival',
        'B': 'normalize_angle yaw_from_quaternion coordinates_near_wall_do_not_trigger_automatic_stop',
        'C': 'emergency_stop_topic',
        'D': 'start_mission_topic_and_pause start_service_trigger trajectory_recording_lifecycle '
             'path_markers_include_goal_tolerance_and_actual_trajectory ' + _CLEAR_TRAJECTORY,
    },
    'test_bspline_track_test': {
        'A': 'bspline_generation_and_tank_bounds six_point_cubic_matches_simulation_bspline '
             'curve_tracking_initial_step upcoming_curvature_slows_before_the_turn goal_arrival_stop',
        'B': 'clamped_bspline_passes_only_endpoints two_point_bspline_is_finite_straight_line',
        'C': 'bspline_few_points_exception emergency_stop_handling',
        'D': 'start_mission_topic_and_pause start_service_trigger trajectory_recording_lifecycle '
             'rviz_vis_markers_include_goal_tolerance_and_trail ' + _CLEAR_TRAJECTORY,
    },
    'test_circle_drive_test': {
        'A': 'start_mission_and_control lap_completion_stop vector_field_convergence',
        'B': 'normalize_angle curvature_reduces_forward_command_for_tighter_circle',
        'C': 'initial_state_and_wait emergency_stop',
        'D': 'start_service trajectory_recording_lifecycle markers_include_orbit_center_and_trail '
             + _CLEAR_TRAJECTORY,
    },
    'test_station_keeping_test': {
        'A': 'start_and_drive_towards_target reverse_on_slight_overshoot heading_correction_in_pos_deadband',
        'B': 'deadband_idle deadband_idle_within_0_5m',
        'C': 'wait_for_start emergency_stop',
        'D': 'trajectory_recording_lifecycle markers_include_waypoint_deadband_heading_and_trail '
             + _CLEAR_TRAJECTORY,
    },
    'test_los_path_test': {
        'A': 'both_test_modes_build_safe_tank_paths',
        'B': 'continuous_circle_and_invalid_mode',
        'D': 'ros_node_previews_both_paths_without_motor_topic',
    },
    'test_waypoint_sequence_test': {
        'B': 'build_requires_five_safe_targets_with_optional_headings',
        'C': 'stale_odom_does_not_advance_or_command_motion',
        'D': 'ros_node_holds_each_goal_then_reacquires_final_position',
    },
    'test_real_obstacle_avoid_test': {
        'A': 'start_requires_confirmed_front_obstacle',
        'B': 'side_wall_does_not_count_as_front_obstacle auto_arm_when_wait_for_start_false',
        'C': 'preview_uses_obstacle_and_stale_scan_disarms invalid_yaw_blocks_start '
             'drive_mode_caps_commands_and_stops_on_scan_loss',
        'D': 'rviz_path_and_markers_clear_on_scan_loss',
    },
    'test_manual_dri_monitor': {
        'C': 'sensor_loss_clears_manual_dri_without_command_publisher',
        'D': 'visualization_is_scaled_existing_dri_in_odom_frame',
    },
    'test_tank_mission_guard': {
        'C': 'drive_clamps_and_disarms_on_stale_scan',
        'D': 'preview_has_no_cmd_vel_publisher',
    },
    'test_test_coordinates': {
        'A': 'straight_line_coords bspline_track_coords circle_drive_coords station_keeping_coords',
        'B': 'bspline_presets',
        'C': 'yaml_file_integrity',
    },
    'test_trajectory_visualization': {
        'A': 'initialization_defaults always_records_first_eligible_point distance_downsampling '
             'iteration_and_indexing create_trail_marker create_goal_tolerance_marker',
        'B': 'bounded_capacity reset_and_clear create_trail_marker_single_point',
        'C': 'invalid_parameters',
    },
    # ── 시험 기반 ──
    'test_sim_helpers': {
        'A': 'thrust_mapping_matches_twist2thrust_and_saturates straight_thrust_reaches_analytic_terminal_speed '
             'positive_angular_turns_left_and_reaches_steady_rate scan_hits_circle_and_segment_at_expected_range '
             'grid_marks_obstacle_cells_only recorder_metrics',
        'B': 'zero_command_drifts_with_current odom_noise_is_reproducible_with_seed '
             'scan_rotates_with_boat_yaw world_clearance_and_bounds',
        'D': 'ros_adapters_roundtrip_pose_and_clock',
    },
}


def lookup(file_stem: str, test_name: str):
    """(파일 이름, 시험 함수 이름) → 'A'~'D' 또는 None."""
    name = test_name[5:] if test_name.startswith('test_') else test_name
    for check, names in CHECKS.get(file_stem, {}).items():
        if name in names.split():
            return check
    return None
