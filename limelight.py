"""
Translated into Python from https://github.com/LimelightVision/limelightlib-wpijava/blob/main/LimelightHelpers.java
"""

from dataclasses import dataclass

from ntcore import NetworkTable, NetworkTableEntry, NetworkTableInstance
from wpimath.geometry import (
	Pose2d,
	Pose3d,
	Rotation2d,
	Rotation3d,
	Translation2d,
	Translation3d,
)

from math import radians

@dataclass
class RawFiducial:
	id: int = 0
	txyc: float = 0
	tync: float = 0
	ta: float = 0
	dist_to_camera: float = 0
	dist_to_robot: float = 0
	ambiguity: float = 0

@dataclass
class RawDetection:
	class_id: int = 0
	txyc: float = 0
	tync: float = 0
	ta: float = 0
	corner0_x: float = 0
	corner0_y: float = 0
	corner1_x: float = 0
	corner1_y: float = 0
	corner2_x: float = 0
	corner2_y: float = 0
	corner3_x: float = 0
	corner3_y: float = 0

@dataclass
class PoseEstimate:
	pose: Pose2d
	timestamp_seconds: float
	latency: float
	tag_count: int
	tag_span: float
	avg_tag_dist: float
	avg_tag_area: float
	raw_fiducials: list[RawFiducial]
	

class LimelightHelpers:

	@staticmethod
	def _sanitize_name(name: str | None) -> str:
		if name == "" or name is None:
			return "limelight"
		else:
			return name
	
	@staticmethod
	def _to_Pose3D(in_data: list[float]) -> Pose3d:
		if len(in_data) < 6:
			return Pose3d()
		else:
			return Pose3d(
				Translation3d(in_data[0], in_data[1], in_data[2]),
				Rotation3d(radians(in_data[3]), radians(in_data[4]), radians(in_data[5]))
			)
		
	@staticmethod
	def _to_Pose2D(in_data: list[float]) -> Pose2d:
		if len(in_data) < 6:
			return Pose2d()
		else:
			return Pose2d(
				Translation2d(in_data[0], in_data[1]),
				Rotation2d(radians(in_data[5]))
			)
		
	@staticmethod
	def _extract_array_entry(in_data: list[float], position: int) -> float:
		if len(in_data) < position + 1:
			return 0
		else:
			return in_data[position]
		
	@staticmethod
	def _get_botpose_estimate(limelight_name: str, entry_name: str) -> PoseEstimate:
		pose_entry = LimelightHelpers.get_limelight_NTTableEntry(limelight_name, entry_name)
		pose_array = pose_entry.getDoubleArray([])
		pose = LimelightHelpers._to_Pose2D(pose_array)
		latency = LimelightHelpers._extract_array_entry(pose_array, 6)
		tag_count = int(LimelightHelpers._extract_array_entry(pose_array, 7))
		tag_span = LimelightHelpers._extract_array_entry(pose_array, 8)
		tag_dist = LimelightHelpers._extract_array_entry(pose_array, 9)
		tag_area = LimelightHelpers._extract_array_entry(pose_array, 10)
		# getlastchange() in microseconds, ll latency in milliseconds
		timestamp = (pose_entry.getLastChange() / 1000000) - (latency / 1000)

		raw_fiducials = []
		vals_per_fiducial = 7
		expected_total_vals = 11 + vals_per_fiducial * tag_count

		if len(pose_array) != expected_total_vals:
			# Don't populate fiducials
			return PoseEstimate(pose, timestamp, latency, tag_count, tag_span, tag_dist, tag_area, raw_fiducials)
		else:
			for i in range(tag_count):
				base_index = 11 + (i * vals_per_fiducial)
				id = int(pose_array[base_index])
				txnc = pose_array[base_index + 1]
				tync = pose_array[base_index + 2]
				ta = pose_array[base_index + 3]
				dist_to_camera = pose_array[base_index + 4]
				dist_to_robot = pose_array[base_index + 5]
				ambiguity = pose_array[base_index + 6]
				raw_fiducials.append(
					RawFiducial(id, txnc, tync, ta, dist_to_camera, dist_to_robot, ambiguity)
				)

			return PoseEstimate(pose, timestamp, latency, tag_count, tag_span, tag_dist, tag_area, raw_fiducials)

	@staticmethod
	def _get_raw_fiducials(limelight_name: str) -> list[RawFiducial]:
		entry = LimelightHelpers.get_limelight_NTTableEntry(limelight_name, "rawfiducials")
		raw_fiducial_array = entry.getDoubleArray([])
		vals_per_entry = 7
		if len(raw_fiducial_array) % vals_per_entry != 0:
			return [RawFiducial()]
		
		num_fiducials = len(raw_fiducial_array) // vals_per_entry
		raw_fiducials = []

		for i in range(num_fiducials):
			base_index = i * vals_per_entry
			id = int(LimelightHelpers._extract_array_entry(raw_fiducial_array, base_index))
			txnc = LimelightHelpers._extract_array_entry(raw_fiducial_array, base_index + 1)
			tync = LimelightHelpers._extract_array_entry(raw_fiducial_array, base_index + 2)
			ta = LimelightHelpers._extract_array_entry(raw_fiducial_array, base_index + 3)
			dist_to_camera = LimelightHelpers._extract_array_entry(raw_fiducial_array, base_index + 4)
			dist_to_robot = LimelightHelpers._extract_array_entry(raw_fiducial_array, base_index + 5)
			ambiguity = LimelightHelpers._extract_array_entry(raw_fiducial_array, base_index + 6)

			raw_fiducials.append(
				RawFiducial(id, txnc, tync, ta, dist_to_camera, dist_to_robot, ambiguity)
			)

	@staticmethod
	def get_raw_detections(limelight_name: str) -> list[RawDetection]:
		entry = LimelightHelpers.get_limelight_NTTableEntry(limelight_name, "rawdetections")
		raw_detection_array = entry.getDoubleArray([])
		vals_per_entry = 11
		if len(raw_detection_array) % vals_per_entry != 0:
			return [RawDetection()]
		
		num_detections = len(raw_detection_array) // vals_per_entry
		raw_detections = []

		for i in range(num_detections):
			base_index = i * vals_per_entry
			class_id = int(LimelightHelpers._extract_array_entry(raw_detection_array, base_index))
			txnc = LimelightHelpers._extract_array_entry(raw_detection_array, base_index + 1)
			tync = LimelightHelpers._extract_array_entry(raw_detection_array, base_index + 2)
			ta = LimelightHelpers._extract_array_entry(raw_detection_array, base_index + 3)
			corner0_x = LimelightHelpers._extract_array_entry(raw_detection_array, base_index + 4)
			corner0_y = LimelightHelpers._extract_array_entry(raw_detection_array, base_index + 5)
			corner1_x = LimelightHelpers._extract_array_entry(raw_detection_array, base_index + 6)
			corner1_y = LimelightHelpers._extract_array_entry(raw_detection_array, base_index + 7)
			corner2_x = LimelightHelpers._extract_array_entry(raw_detection_array, base_index + 8)
			corner2_y = LimelightHelpers._extract_array_entry(raw_detection_array, base_index + 9)
			corner3_x = LimelightHelpers._extract_array_entry(raw_detection_array, base_index + 10)
			corner3_y = LimelightHelpers._extract_array_entry(raw_detection_array, base_index + 11)

			raw_detections.append(
				RawDetection(class_id, txnc, tync, ta, corner0_x, corner0_y, corner1_x, corner1_y, corner2_x, corner2_y, corner3_x, corner3_y)
			)

		return raw_detections

	@staticmethod
	def _print_PoseEstimate(pose: PoseEstimate) -> None:
		if pose is None:
			print("No PoseEstimate available.")
			return
		
		print("Pose Estimate Information:")
		print(f"Timestamp (Seconds): {pose.timestamp_seconds}")
		print(f"Latency: {pose.latency}")
		print(f"Tag Count: {pose.tag_count}")
		print(f"Tag Span: {pose.tag_span} meters")
		print(f"Average Tag Distance: {pose.avg_tag_dist} meters")
		print(f"Average Tag Area: {pose.avg_tag_area} of image")
		print()

		if pose.raw_fiducials is None or len(pose.raw_fiducials) == 0:
			print("No RawFiducials data available.")
			return
		
		print("Raw Fiducials Details:")
		for i in range(len(pose.raw_fiducials)):
			fiducial = pose.raw_fiducials[i]
			print(f" Fiducial #{i+1}:")
			print(f" ID: {fiducial.id}")
			print(f" TXNC: {fiducial.txyc}")
			print(f" TYNC: {fiducial.tync}")
			print(f" TA: {fiducial.ta}")
			print(f" Distance to Camera: {fiducial.dist_to_camera} meters")
			print(f" Distance to Robot: {fiducial.dist_to_robot} meters")
			print(f" Ambiguity: {fiducial.ambiguity}")
			print()

	@staticmethod
	def get_limelight_NTTable(table_name: str) -> NetworkTable:
		return NetworkTableInstance.getDefault().getTable(LimelightHelpers._sanitize_name(table_name))
	
	@staticmethod
	def get_limelight_NTTableEntry(table_name: str, entry_name: str) -> NetworkTableEntry:
		return LimelightHelpers.get_limelight_NTTable(table_name).getEntry(entry_name)
	
	@staticmethod
	def get_limelight_NTDouble(table_name: str, entry_name: str) -> float:
		return LimelightHelpers.get_limelight_NTTableEntry(table_name, entry_name).getDouble(0.0)
	
	@staticmethod
	def set_limelight_NTDouble(table_name: str, entry_name: str, val: float) -> None:
		LimelightHelpers.get_limelight_NTTableEntry(table_name, entry_name).setDouble(val)

	@staticmethod
	def get_limelight_NTDoubleArray(table_name: str, entry_name: str) -> list[float]:
		return LimelightHelpers.get_limelight_NTTableEntry(table_name, entry_name).getDoubleArray([])

	@staticmethod
	def set_limelight_NTDoubleArray(table_name: str, entry_name: str, val: list[float]) -> None:
		LimelightHelpers.get_limelight_NTTableEntry(table_name, entry_name).setDoubleArray(val)

	@staticmethod
	def get_limelight_NTString(table_name: str, entry_name: str) -> str:
		return LimelightHelpers.get_limelight_NTTableEntry(table_name, entry_name).getString("")
	
	@staticmethod
	def get_limelight_NTStringArray(table_name: str, entry_name: str) -> list[str]:
		return LimelightHelpers.get_limelight_NTTableEntry(table_name, entry_name).getStringArray([])
	
	@staticmethod
	def get_limelight_url_string(table_name: str, request: str):
		return f"http://{LimelightHelpers._sanitize_name(table_name)}.local:5807/{request}"
	# TODO: Add MalformedURLException from java version

	@staticmethod
	def get_tx(limelight_name: str) -> float:
		return LimelightHelpers.get_limelight_NTDouble(limelight_name, "tx")
	
	@staticmethod
	def get_ty(limelight_name: str) -> float:
		return LimelightHelpers.get_limelight_NTDouble(limelight_name, "ty")
	
	@staticmethod
	def get_ta(limelight_name: str) -> float:
		return LimelightHelpers.get_limelight_NTDouble(limelight_name, "ta")
	
	@staticmethod
	def get_t2d_array(limelight_name: str) -> list[float]:
		return LimelightHelpers.get_limelight_NTDoubleArray(limelight_name, "t2d")
	
	@staticmethod
	def get_target_count(limelight_name: str) -> float:
		t2d = LimelightHelpers.get_t2d_array(limelight_name)
		if len(t2d) == 17:
			return t2d[1]
		else:
			return 0
		
	@staticmethod
	def get_classifier_class_index(limelight_name: str) -> float:
		t2d = LimelightHelpers.get_t2d_array(limelight_name)
		if len(t2d) == 17:
			return t2d[10]
		else:
			return 0
		
	@staticmethod
	def get_detector_class_index(limelight_name: str) -> float:
		t2d = LimelightHelpers.get_t2d_array(limelight_name)
		if len(t2d) == 17:
			return t2d[11]
		else:
			return 0
		
	@staticmethod
	def get_classifier_class(limelight_name: str) -> str:
		return LimelightHelpers.get_limelight_NTString(limelight_name, "tcclass")
	
	@staticmethod
	def get_detector_class(limelight_name: str) -> str:
		return LimelightHelpers.get_limelight_NTString(limelight_name, "tdclass")
	
	@staticmethod
	def get_latency_pipeline(limelight_name: str) -> float:
		return LimelightHelpers.get_limelight_NTDouble(limelight_name, "tl")
	
	@staticmethod
	def get_latency_capture(limelight_name: str) -> float:
		return LimelightHelpers.get_limelight_NTDouble(limelight_name, "cl")
	
	@staticmethod
	def get_current_pipeline_index(limelight_name: str) -> float:
		return LimelightHelpers.get_limelight_NTDouble(limelight_name, "getpipe")
	
	@staticmethod
	def get_current_pipeline_type(limelight_name: str) -> str:
		return LimelightHelpers.get_limelight_NTString(limelight_name, "getpipetype")
	
	@staticmethod
	def get_JSON_dump(limelight_name: str) -> str:
		return LimelightHelpers.get_limelight_NTString(limelight_name, "json")
	
	@staticmethod
	def get_botpose(limelight_name: str) -> list[float]:
		return LimelightHelpers.get_limelight_NTDoubleArray(limelight_name, "botpose")
	
	@staticmethod
	def get_botpose_wpired(limelight_name: str) -> list[float]:
		return LimelightHelpers.get_limelight_NTDoubleArray(limelight_name, "botpose_wpired")
	
	@staticmethod
	def get_botpose_wpiblue(limelight_name: str) -> list[float]:
		return LimelightHelpers.get_limelight_NTDoubleArray(limelight_name, "botpose_wpiblue")
	
	@staticmethod
	def get_botpose_targetspace(limelight_name: str) -> list[float]:
		return LimelightHelpers.get_limelight_NTDoubleArray(limelight_name, "botpose_targetspace")
	
	@staticmethod
	def get_camerapose_targetspace(limelight_name: str) -> list[float]:
		return LimelightHelpers.get_limelight_NTDoubleArray(limelight_name, "camerapose_targetspace")
	
	@staticmethod
	def get_camerapose_robotspace(limelight_name: str) -> list[float]:
		return LimelightHelpers.get_limelight_NTDoubleArray(limelight_name, "camerapose_robotspace")
	
	@staticmethod
	def get_targetpose_cameraspace(limelight_name: str) -> list[float]:
		return LimelightHelpers.get_limelight_NTDoubleArray(limelight_name, "targetpose_cameraspace")
	
	@staticmethod
	def get_targetpose_robotspace(limelight_name: str) -> list[float]:
		return LimelightHelpers.get_limelight_NTDoubleArray(limelight_name, "targetpose_robotspace")
	
	@staticmethod
	def get_target_color(limelight_name: str) -> list[float]:
		return LimelightHelpers.get_limelight_NTDoubleArray(limelight_name, "tc")
	
	@staticmethod
	def get_fiducial_id(limelight_name: str) -> float:
		return LimelightHelpers.get_limelight_NTDouble(limelight_name, "tid")
	
	@staticmethod
	def get_neural_class_id(limelight_name: str) -> str:
		return LimelightHelpers.get_limelight_NTString(limelight_name, "tclass")
	
	@staticmethod
	def get_raw_barcode_data(limelight_name: str) -> list[str]:
		return LimelightHelpers.get_limelight_NTStringArray(limelight_name, "rawbarcodes")
	

	@staticmethod
	def get_botpose_3d(limelight_name: str) -> Pose3d:
		pose_array = LimelightHelpers.get_botpose(limelight_name)
		return LimelightHelpers._to_Pose3D(pose_array)
	
	@staticmethod
	def get_botpose_3d_wpired(limelight_name: str) -> Pose3d:
		pose_array = LimelightHelpers.get_botpose_wpired(limelight_name)
		return LimelightHelpers._to_Pose3D(pose_array)
	
	@staticmethod
	def get_botpose_3d_wpiblue(limelight_name: str) -> Pose3d:
		pose_array = LimelightHelpers.get_botpose_wpiblue(limelight_name)
		return LimelightHelpers._to_Pose3D(pose_array)
	
	@staticmethod
	def get_botpose_3d_targetspace(limelight_name: str) -> Pose3d:
		pose_array = LimelightHelpers.get_botpose_targetspace(limelight_name)
		return LimelightHelpers._to_Pose3D(pose_array)
	
	@staticmethod
	def get_camerapose_3d_targetspace(limelight_name: str) -> Pose3d:
		pose_array = LimelightHelpers.get_camerapose_targetspace(limelight_name)
		return LimelightHelpers._to_Pose3D(pose_array)
	
	@staticmethod
	def get_targetpose_3d_cameraspace(limelight_name: str) -> Pose3d:
		pose_array = LimelightHelpers.get_targetpose_cameraspace(limelight_name)
		return LimelightHelpers._to_Pose3D(pose_array)
	
	@staticmethod
	def get_targetpose_3d_robotspace(limelight_name: str) -> Pose3d:
		pose_array = LimelightHelpers.get_targetpose_robotspace(limelight_name)
		return LimelightHelpers._to_Pose3D(pose_array)
	
	@staticmethod
	def get_camerapose_3d_robotspace(limelight_name: str) -> Pose3d:
		pose_array = LimelightHelpers.get_camerapose_robotspace(limelight_name)
		return LimelightHelpers._to_Pose3D(pose_array)
	

	@staticmethod
	def get_botpose_2d_wpiblue(limelight_name: str) -> Pose2d:
		"""Gets the Pose2d for easy use with Odometry vision pose estimator
		(addVisionMeasurement)"""
		result = LimelightHelpers.get_botpose_wpiblue(limelight_name)
		return LimelightHelpers._to_Pose2D(result)
	
	@staticmethod
	def get_botpose_estimate_wpiblue(limelight_name: str) -> PoseEstimate:
		"""Gets the Pose2d and timestamp for use with WPILib pose estimator (addVisionMeasurement) when you are on the BLUE
		alliance"""
		return LimelightHelpers._get_botpose_estimate(limelight_name, "botpose_wpiblue")
	
	@staticmethod
	def get_botpose_estimate_wpiblue_megatag2(limelight_name: str) -> PoseEstimate:
		"""Gets the Pose2d and timestamp for use with WPILib pose estimator (addVisionMeasurement) when you are on the BLUE
		alliance"""
		return LimelightHelpers._get_botpose_estimate(limelight_name, "botpose_orb_wpiblue")
	
	@staticmethod
	def get_botpose_2d_wpired(limelight_name: str) -> Pose2d:
		"""Gets the Pose2d for easy use with Odometry vision pose estimator
		(addVisionMeasurement)"""
		result = LimelightHelpers.get_botpose_wpired(limelight_name)
		return LimelightHelpers._to_Pose2D(result)
	
	@staticmethod
	def get_botpose_estimate_wpired(limelight_name: str) -> PoseEstimate:
		"""Gets the Pose2d and timestamp for use with WPILib pose estimator (addVisionMeasurement) when you are on the RED
		alliance"""
		return LimelightHelpers._get_botpose_estimate(limelight_name, "botpose_wpired")
	
	@staticmethod
	def get_botpose_estimate_wpired_megatag2(limelight_name: str) -> PoseEstimate:
		"""Gets the Pose2d and timestamp for use with WPILib pose estimator (addVisionMeasurement) when you are on the RED
		alliance"""
		return LimelightHelpers._get_botpose_estimate(limelight_name, "botpose_orb_wpired")
	
	@staticmethod
	def get_botpose_2d(limelight_name: str) -> Pose2d:
		result = LimelightHelpers.get_botpose(limelight_name)
		return LimelightHelpers._to_Pose2D(result)
	
	@staticmethod
	def get_tv(limelight_name: str) -> bool:
		return 1.0 == LimelightHelpers.get_limelight_NTDouble(limelight_name, "tv")
	

	@staticmethod
	def set_pipeline_index(limelight_name: str, pipeline_index: int) -> None:
		LimelightHelpers.set_limelight_NTDouble(limelight_name, "pipeline", pipeline_index)

	@staticmethod
	def set_priority_tag_id(limelight_name: str, id: int) -> None:
		LimelightHelpers.set_limelight_NTDouble(limelight_name, "priorityid", id)

	@staticmethod
	def set_LED_to_pipeline_control(limelight_name: str) -> None:
		"""The LEDs will be controlled by Limelight pipeline settings, and not by robot code.
		"""
		LimelightHelpers.set_limelight_NTDouble(limelight_name, "ledMode", 0)

	@staticmethod
	def set_LED_to_force_off(limelight_name: str) -> None:
		LimelightHelpers.set_limelight_NTDouble(limelight_name, "ledMode", 1)

	@staticmethod
	def set_LED_to_force_blink(limelight_name: str) -> None:
		LimelightHelpers.set_limelight_NTDouble(limelight_name, "ledMode", 2)

	@staticmethod
	def set_LED_to_force_on(limelight_name: str) -> None:
		LimelightHelpers.set_limelight_NTDouble(limelight_name, "ledMode", 3)

	@staticmethod
	def set_stream_mode_to_standard(limelight_name: str) -> None:
		LimelightHelpers.set_limelight_NTDouble(limelight_name, "stream", 0)

	@staticmethod
	def set_stream_mode_to_PiPMain(limelight_name: str) -> None:
		LimelightHelpers.set_limelight_NTDouble(limelight_name, "stream", 1)

	@staticmethod
	def set_stream_mode_to_PiPSecondary(limelight_name: str) -> None:
		LimelightHelpers.set_limelight_NTDouble(limelight_name, "stream", 2)

	
	@staticmethod
	def set_crop_window(limelight_name: str, crop_x_min: float, crop_x_max: float, crop_y_min: float, crop_y_max: float) -> None:
		"""Sets the crop window. The crop windows in the UI must be completely open for 
		dynamic cropping to work."""
		entries = [crop_x_min, crop_x_max, crop_y_min, crop_y_max]
		LimelightHelpers.set_limelight_NTDoubleArray(limelight_name, "crop", entries)

	@staticmethod
	def set_fiducial_3d_offset(limelight_name: str, offset_x: float, offset_y: float, offset_z: float) -> None:
		"""Sets 3D offset point for easy 3D targeting."""
		entries = [offset_x, offset_y, offset_z]
		LimelightHelpers.set_limelight_NTDoubleArray(limelight_name, "fiducial_offset_set", entries)

	@staticmethod
	def set_robot_orientation(limelight_name: str, yaw: float, yaw_rate: float, pitch: float, pitch_rate: float, roll: float, roll_rate: float) -> None:
		entries = [yaw, yaw_rate, pitch, pitch_rate, roll, roll_rate]
		LimelightHelpers.set_limelight_NTDoubleArray(limelight_name, "robot_orientation_set", entries)

	@staticmethod
	def set_fiducial_id_filters_override(limelight_name: str, valid_ids: list[int]) -> None:
		# Convert to floats
		valid_ids_float = []
		for i in range(len(valid_ids)):
			valid_ids_float.append(
				float(valid_ids[i])
			)
		LimelightHelpers.set_limelight_NTDoubleArray(limelight_name, "fiducial_id_filters_set", valid_ids_float)

	@staticmethod
	def set_fiducial_downscaling_override(limelight_name: str, downscale: float) -> None:
		d = 0
		if downscale == 1.0:
			d = 1
		elif downscale == 1.5:
			d = 2
		elif downscale == 2:
			d = 3
		elif downscale == 3:
			d = 4
		elif downscale == 4:
			d = 5
		LimelightHelpers.set_limelight_NTDouble(limelight_name, "fiducial_downscale_set", d)

	@staticmethod
	def set_camerapose_robotspace(limelight_name: str, forward: float, side: float, up: float, roll: float, pitch: float, yaw: float) -> None:
		entries = [forward, side, up, roll, pitch, yaw]
		LimelightHelpers.set_limelight_NTDoubleArray(limelight_name, "camerapose_robotspace_set", entries)

	
	@staticmethod
	def set_python_script_data(limelight_name: str, outgoing_python_data: list[float]) -> None:
		LimelightHelpers.set_limelight_NTDoubleArray(limelight_name, "llrobot", outgoing_python_data)

	@staticmethod
	def get_python_script_data(limelight_name: str) -> list[float]:
		return LimelightHelpers.get_limelight_NTDoubleArray(limelight_name, "llpython")
	
	"""
	The following functions from LimelightHelpers.java are not present here:
	- public static CompletableFuture<Boolean> takeSnapshot(String tableName, String snapshotName)
	- private static boolean SYNCH_TAKESNAPSHOT(String tableName, String snapshotName)
	- public static LimelightResults getLatestResults(String limelightName)
	"""
	

	@staticmethod
	def take_snapshot(table_name: str, snapshot_name: str) -> None:
		pass # TODO: implement

	@staticmethod
	def _synch_take_snapshot(table_name: str, snapshot_name: str) -> bool:
		return False # TODO: implement
	
	@staticmethod
	def get_latest_results(limelight_name: str) -> None:
		pass # TODO: implement
	