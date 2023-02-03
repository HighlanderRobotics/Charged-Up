package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ElevatorSubsystem extends SubsystemBase {
    WPI_TalonFX elevatorMotor;
    boolean enabled = true;
    
    
    public ElevatorFeedforward feedforward = new ElevatorFeedforward(0.0, 0.0, 0.0);
    ProfiledPIDController pidController = new ProfiledPIDController(0.0, 0.0, 0.0, 
    new TrapezoidProfile.Constraints(0.0,0.0));
    
    public ElevatorSubsystem() {
        
        elevatorMotor = new WPI_TalonFX(0);

        elevatorMotor.config_kP(0, 0.0);
        elevatorMotor.config_kI(0, 0.0);
        elevatorMotor.config_kD(0, 0.0);
        elevatorMotor.config_kF(0, 0.0); // fill in these values later
    }
    private void useOutput(double output, TrapezoidProfile.State state) {
        elevatorMotor.set(ControlMode.PercentOutput, output + feedforward.calculate(state.velocity));
    }
    public void setGoal(double position) {
        pidController.setGoal(position);
    }
    private double getMeasurement() {
        return elevatorMotor.getSelectedSensorPosition();
    }
    public void extendElevator() {
        
    }
    public void retractElevator() {

    }
    public void releaseElevator() {

    }
    
    public boolean isAtSetpoint(){
        if (Math.abs(pidController.getGoal().position - elevatorMotor.getSelectedSensorPosition()) < Constants.elevatorMargin) {
            return true;
        }
        else {
            return false;
        }
        
    }
    /*goal is referring to the column of 3 scoring spots, which all have the same corresponding point on the map and for path related things, while 
    level refers to the top/mid/bottom levels of positions */

    public void pickGoal(){

    }
    public void pickTopLevel(){
        
    }
    public void pickMidLevel(){

    }
    public void pickBottomLevel(){

    }

    @Override
    public void periodic() {
        if (enabled) {
            useOutput(pidController.calculate(getMeasurement()), pidController.getSetpoint());
        }
    }
}
