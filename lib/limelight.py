"""
Translated into Python from https://github.com/LimelightVision/limelightlib-wpijava/blob/main/LimelightHelpers.java.
Version 1.11
"""

import threading
from dataclasses import dataclass
from math import radians
from urllib.parse import urlparse, ParseResult

from ntcore import NetworkTable, NetworkTableEntry, NetworkTableInstance, DoubleArrayEntry
from wpimath import units
from wpimath.geometry import (
	Pose2d,
	Pose3d,
	Rotation2d,
	Rotation3d,
	Translation2d,
	Translation3d
)


class ConcurrentDict(dict):
	"""
	Thread-safe dictionary that ensures atomic operations for concurrent access.

	Similar to Java's ConcurrentHashMap.
	"""

	def __init__(self):
		super().__init__()
		self._lock = threading.Lock()

	def __getitem__(self, key):
		with self._lock:
			return super().__getitem__(key)

	def __setitem__(self, key, value):
		with self._lock:
			super().__setitem__(key, value)

	def compute_if_absent(self, key, mapping_function):
		"""Computes and stores the value if the key is absent, ensuring thread safety."""
		if key in self:
			return super().__getitem__(key)

		with self._lock:
			super().__setitem__(key, mapping_function())
			return super().__getitem__(key)

@dataclass
class RawFiducial:
	"""Represents a Limelight Raw Fiducial result from Limelight's NetworkTables output."""
	id: int = 0
	txyc: float = 0
	tync: float = 0
	ta: float = 0
	dist_to_camera: float = 0
	dist_to_robot: float = 0
	ambiguity: float = 0

@dataclass
class RawDetection:
	"""Represents a Limelight Raw Neural Detector result from Limelight's NetworkTables output."""
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
	"""Represents a 3D Pose Estimate."""
	pose: Pose2d
	timestamp_seconds: float
	latency: float
	tag_count: int
	tag_span: float
	avg_tag_dist: float
	avg_tag_area: float
	raw_fiducials: list[RawFiducial]
	is_megatag_2: bool

	def __init__(self, pose: Pose2d=Pose2d(), timestamp_seconds: float=0, latency: float=0, tag_count: int=0, tag_span: float=0, avg_tag_dist: float=0, avg_tag_area: float=0, raw_fiducials=None, is_megatag_2: bool=False):
		if raw_fiducials is None:
			raw_fiducials = []
		self.pose = pose
		self.timestamp_seconds = timestamp_seconds
		self.latency = latency
		self.tag_count = tag_count
		self.tag_span = tag_span
		self.avg_tag_dist = avg_tag_dist
		self.avg_tag_area = avg_tag_area
		self.raw_fiducials = raw_fiducials
		self.is_megatag_2 = is_megatag_2

@dataclass
class IMUData:
	"""Encapsulates the state of an internal Limelight IMU."""
	robot_yaw = 0.0
	roll = 0.0
	pitch = 0.0
	yaw = 0.0
	gyro_x = 0.0
	gyro_y = 0.0
	gyro_z = 0.0
	accel_x = 0.0
	accel_y = 0.0
	accel_z = 0.0

	def __init__(self, imu_data: list[float] | None=None):
		if imu_data is not None and len(imu_data) >= 10:
			self.robot_yaw = imu_data[0]
			self.roll = imu_data[1]
			self.pitch = imu_data[2]
			self.yaw = imu_data[3]
			self.gyro_x = imu_data[4]
			self.gyro_y = imu_data[5]
			self.gyro_z = imu_data[6]
			self.accel_x = imu_data[7]
			self.accel_y = imu_data[8]
			self.accel_z = imu_data[9]


class LimelightHelpers:
	"""
	LimelightHelpers provides static methods and classes for interfacing with Limelight vision cameras in FRC.
	This library supports all Limelight features including AprilTag tracking, Neural Networks, and standard color/retroreflective tracking.
	"""
	_double_array_entries = ConcurrentDict()

	@staticmethod
	def _sanitize_name(name: str | None) -> str:
		if name == "" or name is None:
			return "limelight"
		else:
			return name

	@staticmethod
	def to_Pose3D(in_data: list[float]) -> Pose3d:
		"""
		Takes a 6-length array of pose data and converts it to a Pose3d object.
		Array format: [x, y, z, roll, pitch, yaw] where angles are in degrees.
		@param in_data Array containing pose data [x, y, z, roll, pitch, yaw]
		@return Pose3d object representing the pose, or empty Pose3d if invalid data
		"""
		if len(in_data) < 6:
			return Pose3d()
		else:
			return Pose3d(
				Translation3d(in_data[0], in_data[1], in_data[2]),
				Rotation3d(radians(in_data[3]), radians(in_data[4]), radians(in_data[5]))
			)

	@staticmethod
	def to_Pose2D(in_data: list[float]) -> Pose2d:
		"""
		Takes a 6-length array of pose data and converts it to a Pose2d object.
		Uses only x, y, and yaw components, ignoring z, roll, and pitch.
		Array format: [x, y, z, roll, pitch, yaw] where angles are in degrees.
		@param in_data Array containing pose data [x, y, z, roll, pitch, yaw]
		@return Pose2d object representing the pose, or empty Pose2d if invalid data
		"""
		if len(in_data) < 6:
			return Pose2d()
		else:
			return Pose2d(
				Translation2d(in_data[0], in_data[1]),
				Rotation2d(radians(in_data[5]))
			)

	@staticmethod
	def pose_3d_to_array(pose: Pose3d) -> list[float]:
		"""
		Converts a Pose3d object to an array of doubles in the format [x, y, z, roll, pitch, yaw].
		Translation components are in meters, rotation components are in degrees.

		@param pose The Pose3d object to convert
		@return A 6-element array containing [x, y, z, roll, pitch, yaw]
		"""
		return [
			pose.translation().X(),
			pose.translation().Y(),
			pose.translation().Z(),
			units.radiansToDegrees(pose.rotation().X()),
			units.radiansToDegrees(pose.rotation().Y()),
			units.radiansToDegrees(pose.rotation().Z())
		]

	@staticmethod
	def pose_2d_to_array(pose: Pose2d) -> list[float]:
		"""
		Converts a Pose2d object to an array of doubles in the format [x, y, z, roll, pitch, yaw].
		Translation components are in meters, rotation components are in degrees.
		Note: z, roll, and pitch will be 0 since Pose2d only contains x, y, and yaw.

		@param pose The Pose2d object to convert
		@return A 6-element array containing [x, y, 0, 0, 0, yaw]
		"""
		return [
			pose.translation().X(),
			pose.translation().Y(),
			0,
			units.radiansToDegrees(0),
			units.radiansToDegrees(0),
			units.radiansToDegrees(pose.rotation().radians())
		]

	@staticmethod
	def _extract_array_entry(in_data: list[float], position: int) -> float:
		if len(in_data) < position + 1:
			return 0
		return in_data[position]

	@staticmethod
	def _get_botpose_estimate(limelight_name: str, entry_name: str, is_megatag_2: bool) -> PoseEstimate:
		pose_entry = LimelightHelpers.get_limelight_double_array_entry(limelight_name, entry_name)

		ts_value = pose_entry.getAtomic()
		pose_array = ts_value.value
		timestamp = ts_value.time

		if len(pose_array) == 0:
			# Handle the case where no data is available
			return PoseEstimate()

		pose = LimelightHelpers.to_Pose2D(pose_array)
		latency = LimelightHelpers._extract_array_entry(pose_array, 6)
		tag_count = int(LimelightHelpers._extract_array_entry(pose_array, 7))
		tag_span = LimelightHelpers._extract_array_entry(pose_array, 8)
		tag_dist = LimelightHelpers._extract_array_entry(pose_array, 9)
		tag_area = LimelightHelpers._extract_array_entry(pose_array, 10)

		# Convert server timestamp from microseconds to seconds and adjust for latency
		adjusted_timestamp = (timestamp / 1000000.0) - (latency / 1000.0)

		raw_fiducials = []
		vals_per_fiducial = 7
		expected_total_vals = 11 + vals_per_fiducial * tag_count

		if len(pose_array) == expected_total_vals:
			for i in range(tag_count):
				base_index = 11 + (i * vals_per_fiducial)
				tag_id = int(pose_array[base_index])
				txnc = pose_array[base_index + 1]
				tync = pose_array[base_index + 2]
				ta = pose_array[base_index + 3]
				dist_to_camera = pose_array[base_index + 4]
				dist_to_robot = pose_array[base_index + 5]
				ambiguity = pose_array[base_index + 6]
				raw_fiducials.append(RawFiducial(tag_id, txnc, tync, ta, dist_to_camera, dist_to_robot, ambiguity))

		return PoseEstimate(pose, adjusted_timestamp, latency, tag_count, tag_span, tag_dist, tag_area, raw_fiducials, is_megatag_2)

	@staticmethod
	def get_raw_fiducials(limelight_name: str) -> list[RawFiducial]:
		"""
		Gets the latest raw fiducial/AprilTag detection results from NetworkTables.

		@param limelight_name Name/identifier of the Limelight
		@return Array of RawFiducial objects containing detection details
		"""
		entry = LimelightHelpers.get_limelight_NTTableEntry(limelight_name, "rawfiducials")
		raw_fiducial_array = entry.getDoubleArray([])
		vals_per_entry = 7
		if len(raw_fiducial_array) % vals_per_entry != 0:
			return []

		num_fiducials = len(raw_fiducial_array) // vals_per_entry
		raw_fiducials = []

		for i in range(num_fiducials):
			base_index = i * vals_per_entry
			tag_id = int(LimelightHelpers._extract_array_entry(raw_fiducial_array, base_index))
			txnc = LimelightHelpers._extract_array_entry(raw_fiducial_array, base_index + 1)
			tync = LimelightHelpers._extract_array_entry(raw_fiducial_array, base_index + 2)
			ta = LimelightHelpers._extract_array_entry(raw_fiducial_array, base_index + 3)
			dist_to_camera = LimelightHelpers._extract_array_entry(raw_fiducial_array, base_index + 4)
			dist_to_robot = LimelightHelpers._extract_array_entry(raw_fiducial_array, base_index + 5)
			ambiguity = LimelightHelpers._extract_array_entry(raw_fiducial_array, base_index + 6)

			raw_fiducials.append(
				RawFiducial(tag_id, txnc, tync, ta, dist_to_camera, dist_to_robot, ambiguity)
			)

		return raw_fiducials

	@staticmethod
	def get_raw_detections(limelight_name: str) -> list[RawDetection]:
		"""
		Gets the latest raw neural detector results from NetworkTables

		@param limelight_name Name/identifier of the Limelight
		@return Array of RawDetection objects containing detection details
		"""
		entry = LimelightHelpers.get_limelight_NTTableEntry(limelight_name, "rawdetections")
		raw_detection_array = entry.getDoubleArray([])
		vals_per_entry = 12
		if len(raw_detection_array) % vals_per_entry != 0:
			return []

		num_detections = len(raw_detection_array) // vals_per_entry
		raw_detections = []

		for i in range(num_detections):
			base_index = i * vals_per_entry # Starting index for this detection's data
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
	def print_pose_estimate(pose: PoseEstimate) -> None:
		"""
		Prints detailed information about a PoseEstimate to standard output.
		Includes timestamp, latency, tag count, tag span, average tag distance,
		average tag area, and detailed information about each detected fiducial.

		@param pose The PoseEstimate object to print. If null, prints "No PoseEstimate available."
		"""
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
		print(f"Is MegaTag2: {pose.is_megatag_2}")
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
	def valid_pose_estimate(pose: PoseEstimate) -> bool:
		return pose is not None and pose.raw_fiducials is not None and len(pose.raw_fiducials) != 0

	@staticmethod
	def get_limelight_NTTable(table_name: str) -> NetworkTable:
		return NetworkTableInstance.getDefault().getTable(LimelightHelpers._sanitize_name(table_name))

	@staticmethod
	def flush() -> None:
		NetworkTableInstance.getDefault().flush()

	@staticmethod
	def get_limelight_NTTableEntry(table_name: str, entry_name: str) -> NetworkTableEntry:
		return LimelightHelpers.get_limelight_NTTable(table_name).getEntry(entry_name)

	@staticmethod
	def get_limelight_double_array_entry(table_name: str, entry_name: str) -> DoubleArrayEntry:
		return LimelightHelpers._double_array_entries.compute_if_absent(
			f"{table_name}/{entry_name}",
			lambda: LimelightHelpers.get_limelight_NTTable(table_name).getDoubleArrayTopic(entry_name).getEntry([])
		)

	@staticmethod
	def get_limelight_NTDouble(table_name: str, entry_name: str) -> float:
		return LimelightHelpers.get_limelight_NTTableEntry(table_name, entry_name).getDouble(0.0)

	@staticmethod
	def set_limelight_NTDouble(table_name: str, entry_name: str, val: float) -> None:
		LimelightHelpers.get_limelight_NTTableEntry(table_name, entry_name).setDouble(val)

	@staticmethod
	def set_limelight_NTDoubleArray(table_name: str, entry_name: str, val: list[float]) -> None:
		LimelightHelpers.get_limelight_NTTableEntry(table_name, entry_name).setDoubleArray(val)

	@staticmethod
	def get_limelight_NTDoubleArray(table_name: str, entry_name: str) -> list[float]:
		return LimelightHelpers.get_limelight_NTTableEntry(table_name, entry_name).getDoubleArray([])

	@staticmethod
	def get_limelight_NTString(table_name: str, entry_name: str) -> str:
		return LimelightHelpers.get_limelight_NTTableEntry(table_name, entry_name).getString("")

	@staticmethod
	def get_limelight_NTStringArray(table_name: str, entry_name: str) -> list[str]:
		return LimelightHelpers.get_limelight_NTTableEntry(table_name, entry_name).getStringArray([])

	@staticmethod
	def get_limelight_url_string(table_name: str, request: str) -> ParseResult | None:
		try:
			return urlparse(f"http://{LimelightHelpers._sanitize_name(table_name)}.local:5807/{request}")
		except ValueError:
			print("bad LL URL")
		return None

	@staticmethod
	def get_tv(limelight_name: str) -> bool:
		"""
		Does the Limelight have a valid target?
		@param limelight_name Name of the Limelight camera ("" for default)
		@return True if a valid target is present, false otherwise
		"""
		return 1.0 == LimelightHelpers.get_limelight_NTDouble(limelight_name, "tv")

	@staticmethod
	def get_tx(limelight_name: str) -> float:
		"""
		Gets the horizontal offset from the crosshair to the target in degrees.
		@param limelight_name Name of the Limelight camera ("" for default)
		@return Horizontal offset angle in degrees
		"""
		return LimelightHelpers.get_limelight_NTDouble(limelight_name, "tx")

	@staticmethod
	def get_ty(limelight_name: str) -> float:
		"""
		Gets the vertical offset from the crosshair to the target in degrees.
		@param limelight_name Name of the Limelight camera ("" for default)
		@return Vertical offset angle in degrees
		"""
		return LimelightHelpers.get_limelight_NTDouble(limelight_name, "ty")

	@staticmethod
	def get_txnc(limelight_name: str) -> float:
		"""
		Gets the horizontal offset from the principal pixel/point to the target in degrees.  This is the most accurate 2d metric if you are using a calibrated camera, and you don't need adjustable crosshair functionality.
		@param limelight_name Name of the Limelight camera ("" for default)
		@return Horizontal offset angle in degrees
		"""
		return LimelightHelpers.get_limelight_NTDouble(limelight_name, "txnc")

	@staticmethod
	def get_tync(limelight_name: str) -> float:
		"""
		Gets the vertical offset from the principal pixel/point to the target in degrees. This is the most accurate 2d metric if you are using a calibrated camera, and you don't need adjustable crosshair functionality.
		@param limelight_name Name of the Limelight camera ("" for default)
		@return Vertical offset angle in degrees
		"""
		return LimelightHelpers.get_limelight_NTDouble(limelight_name, "tync")

	@staticmethod
	def get_ta(limelight_name: str) -> float:
		"""
		Gets the target area as a percentage of the image (0-100%).
		@param limelight_name Name of the Limelight camera ("" for default)
		@return Target area percentage (0-100)
		"""
		return LimelightHelpers.get_limelight_NTDouble(limelight_name, "ta")

	@staticmethod
	def get_t2d_array(limelight_name: str) -> list[float]:
		"""
		T2D is an array that contains several targeting metrics
		@param limelight_name Name of the Limelight camera
		@return Array containing  [targetValid, targetCount, targetLatency, captureLatency, tx, ty, txnc, tync, ta, tid, targetClassIndexDetector,
		targetClassIndexClassifier, targetLongSidePixels, targetShortSidePixels, targetHorizontalExtentPixels, targetVerticalExtentPixels, targetSkewDegrees]
		"""
		return LimelightHelpers.get_limelight_NTDoubleArray(limelight_name, "t2d")

	@staticmethod
	def get_target_count(limelight_name: str) -> float:
		"""
		Gets the number of targets currently detected.
		@param limelight_name Name of the Limelight camera
		@return Number of detected targets
		"""
		t2d = LimelightHelpers.get_t2d_array(limelight_name)
		if len(t2d) == 17:
			return int(t2d[1])
		else:
			return 0

	@staticmethod
	def get_classifier_class_index(limelight_name: str) -> float:
		"""
		Gets the classifier class index from the currently running neural classifier pipeline
		@param limelight_name Name of the Limelight camera
		@return Class index from classifier pipeline
		"""
		t2d = LimelightHelpers.get_t2d_array(limelight_name)
		if len(t2d) == 17:
			return int(t2d[10])
		else:
			return 0

	@staticmethod
	def get_detector_class_index(limelight_name: str) -> float:
		"""
		Gets the detector class index from the primary result of the currently running neural detector pipeline.
		@param limelight_name Name of the Limelight camera
		@return Class index from detector pipeline
		"""
		t2d = LimelightHelpers.get_t2d_array(limelight_name)
		if len(t2d) == 17:
			return int(t2d[11])
		else:
			return 0

	@staticmethod
	def get_classifier_class(limelight_name: str) -> str:
		"""
		Gets the current neural classifier result class name.
		@param limelight_name Name of the Limelight camera
    	@return Class name string from classifier pipeline
		"""
		return LimelightHelpers.get_limelight_NTString(limelight_name, "tcclass")

	@staticmethod
	def get_detector_class(limelight_name: str) -> str:
		"""
		Gets the primary neural detector result class name.
		@param limelight_name Name of the Limelight camera
		@return Class name string from detector pipeline
		"""
		return LimelightHelpers.get_limelight_NTString(limelight_name, "tdclass")

	@staticmethod
	def get_latency_pipeline(limelight_name: str) -> float:
		"""
		Gets the pipeline's processing latency contribution.
		@param limelight_name Name of the Limelight camera
		@return Pipeline latency in milliseconds
		"""
		return LimelightHelpers.get_limelight_NTDouble(limelight_name, "tl")

	@staticmethod
	def get_latency_capture(limelight_name: str) -> float:
		"""
		Gets the capture latency.
		@param limelight_name Name of the Limelight camera
		@return Capture latency in milliseconds
		"""
		return LimelightHelpers.get_limelight_NTDouble(limelight_name, "cl")

	@staticmethod
	def get_current_pipeline_index(limelight_name: str) -> float:
		"""
		Gets the active pipeline index.
		@param limelight_name Name of the Limelight camera
		@return Current pipeline index (0-9)
		"""
		return LimelightHelpers.get_limelight_NTDouble(limelight_name, "getpipe")

	@staticmethod
	def get_current_pipeline_type(limelight_name: str) -> str:
		"""
		Gets the current pipeline type.
		@param limelight_name Name of the Limelight camera
		@return Pipeline type string (e.g. "retro", "apriltag", etc.)
		"""
		return LimelightHelpers.get_limelight_NTString(limelight_name, "getpipetype")

	@staticmethod
	def get_JSON_dump(limelight_name: str) -> str:
		"""
		Gets the full JSON results dump.
		@param limelight_name Name of the Limelight camera
    	@return JSON string containing all current results
		"""
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
		pose_array = LimelightHelpers.get_limelight_NTDoubleArray(limelight_name, "botpose")
		return LimelightHelpers.to_Pose3D(pose_array)

	@staticmethod
	def get_botpose_3d_wpired(limelight_name: str) -> Pose3d:
		"""
		(Not Recommended) Gets the robot's 3D pose in the WPILib Red Alliance Coordinate System.
		@param limelight_name Name/identifier of the Limelight
     	@return Pose3d object representing the robot's position and orientation in Red Alliance field space
		"""
		pose_array = LimelightHelpers.get_limelight_NTDoubleArray(limelight_name, "botpose_wpired")
		return LimelightHelpers.to_Pose3D(pose_array)

	@staticmethod
	def get_botpose_3d_wpiblue(limelight_name: str) -> Pose3d:
		"""
		(Recommended) Gets the robot's 3D pose in the WPILib Blue Alliance Coordinate System.
		@param limelight_name Name/identifier of the Limelight
		@return Pose3d object representing the robot's position and orientation in Blue Alliance field space
		"""
		pose_array = LimelightHelpers.get_limelight_NTDoubleArray(limelight_name, "botpose_wpiblue")
		return LimelightHelpers.to_Pose3D(pose_array)

	@staticmethod
	def get_botpose_3d_targetspace(limelight_name: str) -> Pose3d:
		"""
		Gets the robot's 3D pose with respect to the currently tracked target's coordinate system.
		@param limelight_name Name/identifier of the Limelight
		@return Pose3d object representing the robot's position and orientation relative to the target
		"""
		pose_array = LimelightHelpers.get_limelight_NTDoubleArray(limelight_name, "botpose_targetspace")
		return LimelightHelpers.to_Pose3D(pose_array)

	@staticmethod
	def get_camerapose_3d_targetspace(limelight_name: str) -> Pose3d:
		"""
		Gets the camera's 3D pose with respect to the currently tracked target's coordinate system.
		@param limelight_name Name/identifier of the Limelight
		@return Pose3d object representing the camera's position and orientation relative to the target
		"""
		pose_array = LimelightHelpers.get_limelight_NTDoubleArray(limelight_name, "camerapose_targetspace")
		return LimelightHelpers.to_Pose3D(pose_array)

	@staticmethod
	def get_targetpose_3d_cameraspace(limelight_name: str) -> Pose3d:
		"""
		Gets the target's 3D pose with respect to the camera's coordinate system.
		@param limelight_name Name/identifier of the Limelight
		@return Pose3d object representing the target's position and orientation relative to the camera
		"""
		pose_array = LimelightHelpers.get_limelight_NTDoubleArray(limelight_name, "targetpose_cameraspace")
		return LimelightHelpers.to_Pose3D(pose_array)

	@staticmethod
	def get_targetpose_3d_robotspace(limelight_name: str) -> Pose3d:
		"""
		Gets the target's 3D pose with respect to the robot's coordinate system.
		@param limelight_name Name/identifier of the Limelight
		@return Pose3d object representing the target's position and orientation relative to the robot
		:param limelight_name:
		:return:
		"""
		pose_array = LimelightHelpers.get_limelight_NTDoubleArray(limelight_name, "targetpose_robotspace")
		return LimelightHelpers.to_Pose3D(pose_array)

	@staticmethod
	def get_camerapose_3d_robotspace(limelight_name: str) -> Pose3d:
		"""
		Gets the camera's 3D pose with respect to the robot's coordinate system.
		@param limelight_name Name/identifier of the Limelight
		@return Pose3d object representing the camera's position and orientation relative to the robot
		"""
		pose_array = LimelightHelpers.get_limelight_NTDoubleArray(limelight_name, "camerapose_robotspace")
		return LimelightHelpers.to_Pose3D(pose_array)

	@staticmethod
	def get_botpose_2d_wpiblue(limelight_name: str) -> Pose2d:
		"""
		Gets the Pose2d for easy use with Odometry vision pose estimator
		(addVisionMeasurement)
		@param limelight_name Name/identifier of the Limelight
		@return Pose2d object representing the robot's position and orientation in Blue Alliance field space
		"""
		result = LimelightHelpers.get_botpose_wpiblue(limelight_name)
		return LimelightHelpers.to_Pose2D(result)

	@staticmethod
	def get_botpose_estimate_wpiblue(limelight_name: str) -> PoseEstimate:
		"""
		Gets the MegaTag1 Pose2d and timestamp for use with WPILib pose estimator (addVisionMeasurement) in the WPILib Blue alliance coordinate system.
		@param limelight_name Name/identifier of the Limelight
		@return PoseEstimate object
		"""
		return LimelightHelpers._get_botpose_estimate(limelight_name, "botpose_wpiblue", False)

	@staticmethod
	def get_botpose_estimate_wpiblue_megatag2(limelight_name: str) -> PoseEstimate:
		"""
		Gets the MegaTag2 Pose2d and timestamp for use with WPILib pose estimator (addVisionMeasurement) in the WPILib Blue alliance coordinate system.
		Make sure you are calling setRobotOrientation() before calling this method.
		@param limelight_name Name/identifier of the Limelight
		@return PoseEstimate object
		"""
		return LimelightHelpers._get_botpose_estimate(limelight_name, "botpose_orb_wpiblue", True)

	@staticmethod
	def get_botpose_2d_wpired(limelight_name: str) -> Pose2d:
		"""
		Gets the Pose2d for easy use with Odometry vision pose estimator
		(addVisionMeasurement)
		@param limelight_name Name/identifier of the Limelight
		@return Pose2d object representing the robot's position and orientation in Red Alliance field space
		"""
		result = LimelightHelpers.get_botpose_wpired(limelight_name)
		return LimelightHelpers.to_Pose2D(result)

	@staticmethod
	def get_botpose_estimate_wpired(limelight_name: str) -> PoseEstimate:
		"""
		Gets the MegaTag1 Pose2d and timestamp for use with WPILib pose estimator (addVisionMeasurement) in the WPILib Red alliance coordinate system.
		@param limelight_name Name/identifier of the Limelight
		@return PoseEstimate object
		"""
		return LimelightHelpers._get_botpose_estimate(limelight_name, "botpose_wpired", False)

	@staticmethod
	def get_botpose_estimate_wpired_megatag2(limelight_name: str) -> PoseEstimate:
		"""
		Gets the MegaTag2 Pose2d and timestamp for use with WPILib pose estimator (addVisionMeasurement) in the WPILib Red alliance coordinate system.
		Make sure you are calling setRobotOrientation() before calling this method.
		@param limelight_name Name/identifier of the Limelight
		@return PoseEstimate object
		"""
		return LimelightHelpers._get_botpose_estimate(limelight_name, "botpose_orb_wpired", True)

	@staticmethod
	def get_botpose_2d(limelight_name: str) -> Pose2d:
		"""
		Gets the Pose2d for easy use with Odometry vision pose estimator
		(addVisionMeasurement)
		@param limelight_name
		@return Pose2d object
		"""
		result = LimelightHelpers.get_botpose(limelight_name)
		return LimelightHelpers.to_Pose2D(result)

	@staticmethod
	def get_IMU_data(limelight_name: str) -> IMUData:
		"""
		Gets the current IMU data from NetworkTables.
		IMU data is formatted as [robotYaw, Roll, Pitch, Yaw, gyroX, gyroY, gyroZ, accelX, accelY, accelZ].
		Returns all zeros if data is invalid or unavailable.
		@param limelight_name Name/identifier of the Limelight
		@return IMUData object containing all current IMU data
		"""
		imu_data = LimelightHelpers.get_limelight_NTDoubleArray(limelight_name, "imu")
		if imu_data is None or len(imu_data) < 10:
			return IMUData() # Returns object with all zeros
		return IMUData(imu_data)


	@staticmethod
	def set_pipeline_index(limelight_name: str, pipeline_index: int) -> None:
		LimelightHelpers.set_limelight_NTDouble(limelight_name, "pipeline", pipeline_index)

	@staticmethod
	def set_priority_tag_id(limelight_name: str, tag_id: int) -> None:
		LimelightHelpers.set_limelight_NTDouble(limelight_name, "priorityid", tag_id)

	@staticmethod
	def set_LED_to_pipeline_control(limelight_name: str) -> None:
		"""
		Sets LED mode to be controlled by the current pipeline.
		@param limelight_name Name/identifier of the Limelight
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
		"""
		Enables standard side-by-side stream mode.
		@param limelight_name Name/identifier of the Limelight
		"""
		LimelightHelpers.set_limelight_NTDouble(limelight_name, "stream", 0)

	@staticmethod
	def set_stream_mode_to_PiPMain(limelight_name: str) -> None:
		"""
		Enables Picture-in-Picture mode with secondary stream in the corner.
		@param limelight_name Name/identifier of the Limelight
		"""
		LimelightHelpers.set_limelight_NTDouble(limelight_name, "stream", 1)

	@staticmethod
	def set_stream_mode_to_PiPSecondary(limelight_name: str) -> None:
		"""
		Enables Picture-in-Picture mode with primary stream in the corner.
		@param limelight_name Name/identifier of the Limelight
		"""
		LimelightHelpers.set_limelight_NTDouble(limelight_name, "stream", 2)


	@staticmethod
	def set_crop_window(limelight_name: str, crop_x_min: float, crop_x_max: float, crop_y_min: float, crop_y_max: float) -> None:
		"""
		Sets the crop window for the camera. The crop window in the UI must be completely open.
		@param limelight_name Name/identifier of the Limelight
		@param crop_x_min Minimum X value (-1 to 1)
		@param crop_x_max Maximum X value (-1 to 1)
		@param crop_y_min Minimum Y value (-1 to 1)
		@param crop_y_max Maximum Y value (-1 to 1)
		"""
		entries = [crop_x_min, crop_x_max, crop_y_min, crop_y_max]
		LimelightHelpers.set_limelight_NTDoubleArray(limelight_name, "crop", entries)

	@staticmethod
	def set_fiducial_3d_offset(limelight_name: str, x: float, y: float, z: float) -> None:
		"""
		Sets the 3D point-of-interest offset for the current fiducial pipeline.
		https://docs.limelightvision.io/docs/docs-limelight/pipeline-apriltag/apriltag-3d#point-of-interest-tracking
		@param limelight_name Name/identifier of the Limelight
		@param x offset in meters
		@param y offset in meters
		@param z offset in meters
		"""
		entries = [x, y, z]
		LimelightHelpers.set_limelight_NTDoubleArray(limelight_name, "fiducial_offset_set", entries)

	@staticmethod
	def set_robot_orientation(limelight_name: str, yaw: float, yaw_rate: float, pitch: float, pitch_rate: float, roll: float, roll_rate: float) -> None:
		"""
		Sets robot orientation values used by MegaTag2 localization algorithm.
		@param limelight_name Name/identifier of the Limelight
		@param yaw Robot yaw in degrees. 0 = robot facing red alliance wall in FRC
		@param yaw_rate (Unnecessary) Angular velocity of robot yaw in degrees per second
		@param pitch (Unnecessary) Robot pitch in degrees
		@param pitch_rate (Unnecessary) Angular velocity of robot pitch in degrees per second
		@param roll (Unnecessary) Robot roll in degrees
		@param roll_rate (Unnecessary) Angular velocity of robot roll in degrees per second
		"""
		LimelightHelpers._set_robot_orientation(limelight_name, yaw, yaw_rate, pitch, pitch_rate, roll, roll_rate, True)

	@staticmethod
	def set_robot_orientation_no_flush(limelight_name: str, yaw: float, yaw_rate: float, pitch: float, pitch_rate: float, roll: float, roll_rate: float) -> None:
		LimelightHelpers._set_robot_orientation(limelight_name, yaw, yaw_rate, pitch, pitch_rate, roll, roll_rate, False)

	@staticmethod
	def _set_robot_orientation(limelight_name: str, yaw: float, yaw_rate: float, pitch: float, pitch_rate: float, roll: float, roll_rate: float, flush: bool) -> None:
		entries = [yaw, yaw_rate, pitch, pitch_rate, roll, roll_rate]
		LimelightHelpers.set_limelight_NTDoubleArray(limelight_name, "robot_orientation_set", entries)
		if flush:
			LimelightHelpers.flush()

	@staticmethod
	def set_imu_mode(limelight_name: str, mode: int) -> None:
		"""
		Configures the IMU mode for MegaTag2 Localization
		@param limelight_name Name/identifier of the Limelight
		@param mode IMU mode.
		"""
		LimelightHelpers.set_limelight_NTDouble(limelight_name, "imumode_set", mode)

	@staticmethod
	def set_fiducial_id_filters_override(limelight_name: str, valid_ids: list[int]) -> None:
		"""
		Overrides the valid AprilTag IDs that will be used for localization.
		Tags not in this list will be ignored for robot pose estimation.
		@param limelight_name Name/identifier of the Limelight
		@param valid_ids Array of valid AprilTag IDs to track
		"""
		valid_ids_float = []
		for i in range(len(valid_ids)):
			valid_ids_float.append(
				float(valid_ids[i])
			)
		LimelightHelpers.set_limelight_NTDoubleArray(limelight_name, "fiducial_id_filters_set", valid_ids_float)

	@staticmethod
	def set_fiducial_downscaling_override(limelight_name: str, downscale: float) -> None:
		"""
		Sets the downscaling factor for AprilTag detection.
		Increasing downscale can improve performance at the cost of potentially reduced detection range.
		@param limelight_name Name/identifier of the Limelight
		@param downscale Valid values are 1.0 (no downscale), 1.5, 2.0, 3.0, 4.0. Set to 0 for pipeline control.
		"""
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
		"""
		Sets the camera pose relative to the robot.
		@param limelight_name Name/identifier of the Limelight
		@param forward offset in meters
		@param side offset in meters
		@param up offset in meters
		@param roll angle in degrees
		@param pitch angle in degrees
		@param yaw angle in degrees
		"""
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
