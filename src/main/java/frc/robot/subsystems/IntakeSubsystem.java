package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase{

    //public final WPI_TalonFX intakeMotor;
    // = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, Constants.INTAKE_SOLENOID_FORWARD, Constants.INTAKE_SOLENOID_BACKWARD);

    public IntakeSubsystem(){
        //intakeMotor = new WPI_TalonFX(Constants.INTAKE_MOTOR);
        //intakeMotor.config_kP(0, 0.15, 20);
        //intakeMotor.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 20.0, 40.0, 0.5));

    }

    public void activate(){

    }

    
}
