// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib.math;

import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N4;

/** Add your docs here. */
public class XMatrix extends Matrix<N4, N4> {
    public XMatrix() {
        super(Matrix.eye(Nat.N4()));
    }

    public static Matrix<N4, N4> translationMatrix (Translation3d trans) {
        var matBuilder = new MatBuilder<>(Nat.N4(), Nat.N4());
        var mat = matBuilder.fill(
            1., 0., 0., trans.getX(),
            0., 1., 0., trans.getY(),
            0., 0., 1., trans.getZ(),
            0., 0., 0., 1.
        );
        return mat;
    }

    public static Matrix<N4, N4> zRotationMatrix (double radians) {
        var matBuilder = new MatBuilder<>(Nat.N4(), Nat.N4());
        var mat = matBuilder.fill(
            Math.cos(radians), -Math.sin(radians), 0., 0.0,
            Math.sin(radians), Math.cos(radians), 0., 0.0,
            0., 0., 1., 0.0,
            0., 0., 0., 1.
        );
        return mat;
    }

    public static Vector<N4> mulVector(Vector<N4> vec, Matrix<N4, N4> mat) {
        return new Vector<N4>(mat.times(vec));
    }
}
