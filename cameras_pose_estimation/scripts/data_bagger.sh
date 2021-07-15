#!/bin/bash 
rosbag record --split --duration=5m -O subset /tf /tf_static /camera/depth/color/points /pylon_camera_node/image_raw /d415/filtered_points /nir_overlay_intel/cog /nir_overlay_intel/cog_offset /plancloud /short_tissue_traj /filtered_tissue_traj /tissue_traj /camera/color/camera_info /camera/depth/camera_info /d415/passthrough /flea/camera/image_color /freeze_path /iiwa/joint_states /initialpose /markers/cog /nir_overlay_intel/cog /nir_overlay_intel/kuka_polygon /nir_overlay_intel/overlayDBG /nir_overlay_intel/polygon3D_cog /pause_blob_tracker /pylon_camera_node/camera_info /tracking/pause /niroverlay_2D /path_confidence /user_gui /iiwa/control_mode /hapticdbg /confidence_threshold /planefit_dbg /density_dbg /reset_auto_traj /iiwa/auto/command /iiwa/command/JointPosition /phantom/button /phantom/force_feedback /phantom/joint_states /phantom/pose /phantom/state /robot/worldpos /plan


 
