from lib.limelight import LimelightHelpers, PoseEstimate


class Limelight:
    """Helper class for controlling Limelights."""

    def __init__(self, name: str, default_std_devs: tuple[float, float, float]) -> None:
        self._name = name
        self._default_std_devs = default_std_devs

    def name(self) -> str:
        return self._name

    def default_std_devs(self) -> tuple[float, float, float]:
        return self._default_std_devs

    def get_global_pose_estimate(self, is_megatag_2: bool = True) -> PoseEstimate:
        if is_megatag_2:
            return LimelightHelpers.get_botpose_estimate_wpiblue_megatag2(self._name)
        return LimelightHelpers.get_botpose_estimate_wpiblue(self._name)