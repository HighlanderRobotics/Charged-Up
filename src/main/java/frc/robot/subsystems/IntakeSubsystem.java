package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

public class IntakeSubsystem {

    public final WPI_TalonFX intakeMotor;
    DoubleSolenoid intakeSolenoid; // = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, Constants.INTAKE_SOLENOID_FORWARD, Constants.INTAKE_SOLENOID_BACKWARD);

    public IntakeSubsystem(){
        intakeMotor = new WPI_TalonFX(Constants.INTAKE_MOTOR);
        intakeMotor.config_kP(0, 0.15, 20);
        intakeMotor.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 20.0, 40.0, 0.5));
        intakeSolenoid.set(kReverse);

    }

    public void setIntakeRPM(double rpm){
        intakeMotor.set(TalonFXControlMode.Velocity, Falcon.rpmToTicks(rpm));
      }

     /**Toggles the intake extension: If its out it goes in, if its in it goes out */
    public void toggleIntake(){
        intakeSolenoid.toggle();
    }

    /**Sets the intake to extend */
    public void extend(){
        intakeSolenoid.set(kForward);
    }

    /**Sets the intake to retract */
    public void retract(){
        intakeSolenoid.set(kReverse);
    }
}
