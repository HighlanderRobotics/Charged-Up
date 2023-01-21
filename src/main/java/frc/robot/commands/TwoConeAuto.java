package frc.robot.commands;

public class TwoConeAuto {
    DrivetrainSubsystem drivetrainSystem,
    LimeLightSubsystem LimeLightSubsystem,
    IntakeSubsystem intakeSubsystem,
    ElevatorSubsystem elevatorSubsystem,

    addCommands(
        new RunCommand(() -> {intakeSubsystem.activate();}, intakeSubsystem),
        
    )

}
