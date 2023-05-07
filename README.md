# FRC Team 8033's 2023 code for the Charged Up game.
This project follows the standard WPIlib command-based structure of code. See [RobotContainer.java](src/main/java/frc/robot/RobotContainer.java) for general integration and button bindings. Autos are created in the [AutoChooser](src/main/java/frc/robot/commands/AutoChooser.java) file. Logging is handled by the [Logging Wrapper](src/main/java/frc/lib/logging/LoggingWrapper.java).

### **Subsystems**
 - [Swerve Drivetrain,](src/main/java/frc/robot/subsystems/SwerveSubsystem.java) based on 364lib. Contains code to integrate pose estimators, pathplanner auto, and generic driving commands.
 - [Intake Subsystem,](src/main/java/frc/robot/subsystems/IntakeSubsystem.java) our ground cube intake. A simple piston and motor subsystem with command factory methods.
 - [Roller Claw Grabber Subsystem,](src/main/java/frc/robot/subsystems/RollerClawGrabberSubsystem.java) our old grabber mechanism that we ran at Central Valley Regional. This mechanism had problems with dropping cones so we swapped it for the [Greybots Grabber](src/main/java/frc/robot/subsystems/GreybotsGrabberSubsystem.java) for Monterey Bay Regional and Worlds.
 - [Arm Subsystem,](src/main/java/frc/robot/subsystems/ArmSubsystem.java) a wrist which the Roller Claw Grabber was mounted on. Removed along with the old grabber.
 - [Greybots Grabber,](src/main/java/frc/robot/subsystems/GreybotsGrabberSubsystem.java) our v2 grabber that we ran at MBR and Worlds. The mechanism had a pivoting second roller to allow intaking from the single substation. There are some jank falcon native unit setpoints in this subsystem, due to the very fast turnaround for the mechanism.
 - [Elevator Subsystem,](src/main/java/frc/robot/subsystems/ElevatorSubsystem.java) a GreyT cascading elevator which extended the grabber for scoring and substation intaking.
 - [Routing Subsystem,](src/main/java/frc/robot/subsystems/RoutingSubsystem.java) a pair of routing wheels which help vector cubes from the ground intake to the grabber. Was built as its own subsystem because original plans for a cone ground intake involved using routing for orientation and/or orientation detection.
 - [LED Subsystem,](src/main/java/frc/robot/subsystems/LEDSubsystem.java) our leds along the superstructure of the robot. Indicates to the driver information about what piece we have collected and also look fabulous. Most of the color logic is in [Robot Container.](src/main/java/frc/robot/RobotContainer.java)
 - [Superstructure Subsystem,](src/main/java/frc/robot/subsystems/SuperstructureSubsystem.java) a coordination subsystem. In the future we shouldn't use this pattern and instead handle this logic in robot container. Main use was coordinating the intake extending with the elevator extending for auto scoring and collision prevention.
 - [Apriltag Vision Subsystem](src/main/java/frc/robot/subsystems/ApriltagVisionSubsystem.java) and [Tape Vision Subsystem](src/main/java/frc/robot/subsystems/TapeVisionSubsystem.java) manage collecting vision data and processing it into pose estimates. We never got the estimates from apriltags accurate enough to use in competition and never got the tape estimation working. This is an area of focus in our offseason, and we will likely switch from using Limelight 2+ hardware to custom hardware.

### **Credits**:
 - [364lib swerve library](https://github.com/Team364/BaseFalconSwerve)
 - [Photonvision vision library](https://photonvision.org/)
 - [5026 apriltag pose estimator code](https://github.com/Iron-Panthers/FRC-2023/blob/037d52ac93f4e46a2518491acd2e195d429d6dbd/src/main/java/frc/robot/subsystems/VisionSubsystem.java)
 - [Pathplanner auto library](https://github.com/mjansen4857/pathplanner)
 - [AdvantageScope logging visualization with WPILog](https://github.com/Mechanical-Advantage/AdvantageScope)