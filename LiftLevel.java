package org.firstinspires.ftc.teamcode;

public enum  LiftLevel {
    
    BOTTOM (0), ABOVE_GROUND (.15), ABOVE_ONE (1.2), ABOVE_TWO (2.1);
    
    private double rotations;
    
    LiftLevel(double rotations) {
        this.rotations = rotations;
    }
    double getRotations() { return rotations; }
}
