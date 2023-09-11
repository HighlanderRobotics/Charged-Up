package frc.robot.commands;

import java.util.function.Supplier;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.lib.choreolib.ChoreoTrajectory;
import frc.lib.choreolib.TrajectoryManager;
import frc.robot.subsystems.SuperstructureSubsystem;
import frc.robot.subsystems.Elevator.ElevatorSubsystem;
import frc.robot.subsystems.Elevator.ElevatorSubsystem.ScoringLevels;
import frc.robot.subsystems.Grabber.GrabberSubsystem;
import frc.robot.subsystems.Grabber.GrabberSubsystem.GamePiece;
import frc.robot.subsystems.Intake.IntakeSubsystem;
import frc.robot.subsystems.Routing.RoutingSubsystem;
import frc.robot.subsystems.Swerve.SwerveSubsystem;

public class ChoreoAutoChooser {
    LoggedDashboardChooser<Supplier<Command>> chooser = new LoggedDashboardChooser<Supplier<Command>>(
            "Choreo Auto Chooser");
    SwerveSubsystem swerveSubsystem;
    IntakeSubsystem intakeSubsystem;
    ElevatorSubsystem elevatorSubsystem;
    GrabberSubsystem grabberSubsystem;
    RoutingSubsystem routingSubsystem;
    SuperstructureSubsystem superstructureSubsystem;

    public ChoreoAutoChooser(
            SwerveSubsystem swerveSubsystem,
            IntakeSubsystem intakeSubsystem,
            ElevatorSubsystem elevatorSubsystem,
            RoutingSubsystem routingSubsystem,
            GrabberSubsystem grabberSubsystem,
            SuperstructureSubsystem superstructureSubsystem) {

        this.intakeSubsystem = intakeSubsystem;
        this.swerveSubsystem = swerveSubsystem;
        this.elevatorSubsystem = elevatorSubsystem;
        this.grabberSubsystem = grabberSubsystem;
        this.routingSubsystem = routingSubsystem;
        this.superstructureSubsystem = superstructureSubsystem;

        chooser.addOption("Demo Path 1", () -> DemoPath1());
    }

    public void configureChoreo() {
        TrajectoryManager.getInstance().LoadTrajectories();
    }

    public ChoreoTrajectory getPath(String fileName) {
        return TrajectoryManager.getInstance().getTrajectory(fileName);
    }

    private Command intake() {
        return Commands.parallel(
            intakeSubsystem.runCommand().withTimeout(1.0),
                routingSubsystem.runCommand(),
                grabberSubsystem.intakeCubeCommand())
            .withTimeout(1.3)
            .asProxy();
        
        }

        private Command scoreLevelThree() {
            return new SequentialCommandGroup(
                new InstantCommand(() -> swerveSubsystem.setLevel(ScoringLevels.L3, true))
                    .alongWith(new InstantCommand(() -> grabberSubsystem.gamePiece = GamePiece.Cone))
                    .andThen(superstructureSubsystem.scoreNoAim().asProxy().andThen(new WaitCommand(0.5)))
            );
        }

    private Command DemoPath1() {
        return swerveSubsystem.choreoTrajFollow(getPath("DemoPath1.json"))
            .alongWith(
                new SequentialCommandGroup(
                    new WaitCommand(2.35),
                    new InstantCommand(() -> intakeSubsystem.runCommand())
                )
            );
    }

    private Command twoPiece() {
        return scoreLevelThree()
            .andThen(
                swerveSubsystem.choreoTrajFollow(getPath("2Piece.json"))
                .alongWith(
                    new SequentialCommandGroup(
                        new WaitCommand(2.35),
                        intake(),
                        new WaitCommand(1)
                    )
                )
            );
    }

}