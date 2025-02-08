import os.path
from importlib import metadata

from commands2 import CommandScheduler, TimedCommandRobot
from packaging.version import Version
from phoenix6 import utils, SignalLogger
from wpilib import DataLogManager, DriverStation, RobotBase, Timer, SmartDashboard, RobotController

import elasticlib
from robot_container import RobotContainer


class OilSpill(TimedCommandRobot):

    def __init__(self, period = 0.02) -> None:
        super().__init__(period)

        DriverStation.silenceJoystickConnectionWarning(not DriverStation.isFMSAttached())
        self.container = RobotContainer()

        if RobotBase.isReal():
            DataLogManager.start("/home/lvuser/logs")
        else:
            DataLogManager.start()
        DriverStation.startDataLog(DataLogManager.getLog())

        if utils.is_simulation():
            elasticlib.start_elastic_server("127.0.0.1")
        else:
            elasticlib.start_elastic_server("10.63.43.2")
            elasticlib.start_elastic_server("10.0.1.200")

        DataLogManager.log("Robot initialized")

    @staticmethod
    def get_deploy_directory():
        if os.path.exists("/home/lvuser"):
            return "/home/lvuser/py/deploy"
        else:
            return os.path.join(os.getcwd(), "deploy")

    def robotPeriodic(self) -> None:
        # Log important info
        SmartDashboard.putNumber("Match Time", Timer.getMatchTime())
        SmartDashboard.putNumber("Battery Voltage", RobotController.getBatteryVoltage())

    def _simulationPeriodic(self) -> None:
        pass

    def autonomousInit(self) -> None:
        DataLogManager.log("Autonomous period started")

        if has_outdated_pathplanner():
            elasticlib.send_notification(
                elasticlib.Notification(
                    level="WARNING",
                    title="Incorrect PathPlannerLib Version",
                    description="Must be newer than 2025.2.1!"
                )
            )

        selected_auto = self.container.get_autonomous_command()
        if selected_auto is not None:
            selected_auto.schedule()
            
    def autonomousPeriodic(self) -> None:
        pass
    
    def autonomousExit(self) -> None:
        DataLogManager.log("Autonomous period ended")
            
    def teleopInit(self) -> None:
        DataLogManager.log("Teleoperated period started")

    def teleopExit(self) -> None:
        DataLogManager.log("Teleoperated period ended")

    def testInit(self):
        DataLogManager.log("Test period started")
        CommandScheduler.getInstance().cancelAll()
        CommandScheduler.getInstance().disable()

    def disabledInit(self):
        SignalLogger.stop()

    def testExit(self):
        DataLogManager.log("Test period ended")
        CommandScheduler.getInstance().enable()
    
    def disabledPeriodic(self) -> None:
        pass

    def teleopPeriodic(self) -> None:
        pass


def has_outdated_pathplanner() -> bool:
    return Version(metadata.version("robotpy-pathplannerlib")) <= Version("2025.2.1")
