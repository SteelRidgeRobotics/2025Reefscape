# Steel Ridge's 2025 Robot

This is the repository for our 2025 robot. We haven't named it yet.

This year, we have a swerve drive, and a climber arm to climb the deep cage, as well as an elevator, pivot, and intake to pick up and score coral.

## Superstructure

We are using a simultaneously similar but very different framework from most teams this year. Instead of multiple subsystems being linked together in a robot container file, we have a central superstructure that handles all subsystems' states. Instead of controlling individual subsystems, the controller determines the state of the superstructure as a whole. We also have a robot state class that keeps track of odometry measurements and robot pose estimations for use in the rest of our robot.

## Swerve Drive

Just as most swerve drives, the swerve drive consists of 8 motors: 4 for driving, 4 for steering. We use CTRE's swerve API to maximize performance on the subsystem. For more information, see [documentation](https://v6.docs.ctr-electronics.com/en/stable/docs/api-reference/mechanisms/swerve/swerve-overview.html).

## Climber

Our climber is based on the RustHOUNDS' Ri3D robot's deep climb mechanism, utilizing an arm that rotates the robot onto the deep cage with a motor.

## Elevator

Our elevator moves the pivot and intake up and down. Incredible. It is composed of two motors that moves the elevator up and down. Incredible.

## Pivot

The pivot rotates the intake so it can switch between intaking coral and placing coral. A motor controls the position of the pivot.

## Intake

The intake intakes coral. A motor on the intake rotates wheels that pick up the coral.

## Autonomous Path Finding

This year, we recognized the importance of autonomous routines to help simplify driver tasks due to lack of visibility on the field. We use PathPlanner's AutoBuilder to create paths on-the-fly that drive the robot from wherever it is on the field to the beginning of a path which we already pre-planned in the PathPlanner application. Robot localization for this is done via two measurements from our robot state class: The odometry we have on our swerve drivetrain, and the MegaTag2 localizer on our Limelight.
