import os.path

from commands2 import CommandScheduler, TimedCommandRobot
from cscore import CameraServer
from ntcore import NetworkTableInstance
from phoenix6 import SignalLogger
from wpilib import DataLogManager, DriverStation, Timer
from wpinet import WebServer, PortForwarder

from constants import Constants
from lib import elasticlib
from lib.elasticlib import Notification, NotificationLevel
from robot_container import RobotContainer


class Leviathan(TimedCommandRobot):

    def __init__(self, period = 0.02) -> None:
        super().__init__(period)

        DriverStation.silenceJoystickConnectionWarning(not DriverStation.isFMSAttached())
        self.container = RobotContainer()

        SignalLogger.enable_auto_logging(False)
        SignalLogger.stop()
        DataLogManager.start(period=0.3)
        #DriverStation.startDataLog(DataLogManager.getLog())

        CameraServer.startAutomaticCapture()
        CameraServer.startAutomaticCapture()

        #WebServer.getInstance().start(5800, self.get_deploy_directory())
        # port_forwarder = PortForwarder.getInstance()
        # for i in range(10): # Forward limelight ports for use when tethered at events.
        #     port_forwarder.add(5800 + i, f"{Constants.VisionConstants.FRONT_CENTER}.local", 5800 + i)
        #     port_forwarder.add(5800 + i + 10, f"{Constants.VisionConstants.BACK_CENTER}.local", 5800 + i)

        DataLogManager.log("Robot initialized")

        dashboard_nt = NetworkTableInstance.getDefault().getTable("Elastic")
        self._match_time_pub = dashboard_nt.getFloatTopic("Match Time").publish()

    @staticmethod
    def get_deploy_directory():
        if os.path.exists("/home/lvuser"):
            return "/home/lvuser/py/deploy"
        else:
            return os.path.join(os.getcwd(), "deploy")

    def robotPeriodic(self) -> None:
        self._match_time_pub.set(Timer.getMatchTime())

    def _simulationPeriodic(self) -> None:
        pass

    def autonomousInit(self) -> None:
        DataLogManager.log("Autonomous period started")

        selected_auto = self.container.get_autonomous_command()
        if selected_auto is not None:
            DataLogManager.log(f"Selected Auto: {selected_auto.getName()}")
            selected_auto.schedule()

        elasticlib.select_tab("Autonomous")
            
    def autonomousPeriodic(self) -> None:
        pass
    
    def autonomousExit(self) -> None:
        DataLogManager.log("Autonomous period ended")
        elasticlib.select_tab("Teleop")
            
    def teleopInit(self) -> None:
        DataLogManager.log("Teleoperated period started")

    def teleopExit(self) -> None:
        DataLogManager.log("Teleoperated period ended")
        if DriverStation.isFMSAttached():
            elasticlib.send_notification(
                Notification(
                    level=NotificationLevel.INFO.value,
                    title="Good match!",
                    description="(again)" if DriverStation.getReplayNumber() > 1 else ""
                )
            )

    def testInit(self):
        DataLogManager.log("Test period started")
        CommandScheduler.getInstance().cancelAll()
        elasticlib.select_tab("Debug")
        SignalLogger.start()

    def disabledInit(self):
        self.container.vision.set_throttle(150)

    def disabledExit(self):
        self.container.vision.set_throttle(0)

    def testExit(self):
        DataLogManager.log("Test period ended")
    
    def disabledPeriodic(self) -> None:
        pass

    def teleopPeriodic(self) -> None:
        pass

    def testPeriodic(self) -> None:
        pass
