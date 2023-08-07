package org.firstinspires.ftc.teamcode.commands.helpers;

public class Location {

    public double scissorsPosition;
    public int turretPosition, sliderPosition;

    public Location(double scissorsPosition, int turretPosition, int sliderPosition) {
        this.scissorsPosition = scissorsPosition;
        this.turretPosition = turretPosition;
        this.sliderPosition = sliderPosition;
    }
}
