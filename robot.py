import os.path

from commands2 import CommandScheduler, TimedCommandRobot
from phoenix6 import utils, SignalLogger
from wpilib import DataLogManager, DriverStation, SmartDashboard, Timer, RobotController
from wpinet import WebServer, PortForwarder

from constants import Constants
from robot_container import RobotContainer
from subsystems.vision import VisionSubsystem


class Leviathan(TimedCommandRobot):

    def __init__(self, period = 0.02) -> None:
        super().__init__(period)

        DriverStation.silenceJoystickConnectionWarning(not DriverStation.isFMSAttached())
        self.container = RobotContainer()

        SignalLogger.enable_auto_logging(True)
        DataLogManager.start()
        DriverStation.startDataLog(DataLogManager.getLog())

        WebServer.getInstance().start(5800, self.get_deploy_directory())
        port_forwarder = PortForwarder.getInstance()
        for i in range(10): # Forward limelight ports for use when tethered at events.
            port_forwarder.add(5800 + i, f"{Constants.VisionConstants.FRONT_CENTER}.local", 5800 + i)
            port_forwarder.add(5800 + i + 10, f"{Constants.VisionConstants.BACK_CENTER}.local", 5800 + i)

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

        if utils.is_simulation():
            self.container.robot_state.update_mechanisms()

    def _simulationPeriodic(self) -> None:
        pass

    def autonomousInit(self) -> None:
        DataLogManager.log("Autonomous period started")
        self.container.drivetrain.pigeon2.set_yaw(self.container.drivetrain.get_state().pose.rotation().degrees())
        self.container.vision.set_desired_state(VisionSubsystem.SubsystemState.MEGA_TAG_2)

        selected_auto = self.container.get_autonomous_command()
        if selected_auto is not None:
            DataLogManager.log(f"Selected Auto: {selected_auto.getName()}")
            selected_auto.schedule()
            
    def autonomousPeriodic(self) -> None:
        pass
    
    def autonomousExit(self) -> None:
        DataLogManager.log("Autonomous period ended")
            
    def teleopInit(self) -> None:
        DataLogManager.log("Teleoperated period started")
        self.container.drivetrain.pigeon2.set_yaw(self.container.drivetrain.get_state().pose.rotation().degrees())
        self.container.vision.set_desired_state(VisionSubsystem.SubsystemState.MEGA_TAG_2)

    def teleopExit(self) -> None:
        DataLogManager.log("Teleoperated period ended")

    def testInit(self):
        DataLogManager.log("Test period started")
        CommandScheduler.getInstance().cancelAll()

    def disabledInit(self):
        self.container.vision.set_desired_state(VisionSubsystem.SubsystemState.MEGA_TAG_1)
        SignalLogger.stop()

    def testExit(self):
        DataLogManager.log("Test period ended")
    
    def disabledPeriodic(self) -> None:
        pass

    def teleopPeriodic(self) -> None:
        pass