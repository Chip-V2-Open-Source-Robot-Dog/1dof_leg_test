package edu.mit.chip.utils;

import com.revrobotics.CANPIDController;

public class PIDConstants {
    public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, maxRPM;

    public PIDConstants(double kP, double kI, double kD, double kIz, double kFF, double kMaxOutput, double kMinOutput, double maxRPM) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;

        this.kIz = kIz;
        this.kFF = kFF;

        this.kMaxOutput = kMaxOutput;
        this.kMinOutput = kMinOutput;
        this.maxRPM = maxRPM;
    }

    public void load(CANPIDController revPIDController) {
        revPIDController.setP(kP);
        revPIDController.setI(kI);
        revPIDController.setD(kD);

        revPIDController.setIZone(kIz);
        revPIDController.setFF(kFF);

        revPIDController.setOutputRange(kMinOutput, kMaxOutput);
    }
}