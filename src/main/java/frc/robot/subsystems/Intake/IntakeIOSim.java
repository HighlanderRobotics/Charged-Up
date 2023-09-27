// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Intake;

/** Add your docs here. */
public class IntakeIOSim implements IntakeIO {
    boolean isExtended = false;
    double percentOut = 0.0;

    @Override
    public IntakeIOInputsAutoLogged updateInputs() {
        var inputs = new IntakeIOInputsAutoLogged();
        inputs.isExtended = isExtended;
        inputs.percentOut = percentOut;
        return inputs;
    }

    @Override
    public void extend() {
        isExtended = true;
    }

    @Override
    public void retract() {
        isExtended = false;
    }

    @Override
    public void setPercentOut(double percentOut) {
        this.percentOut = percentOut;
    }
}
