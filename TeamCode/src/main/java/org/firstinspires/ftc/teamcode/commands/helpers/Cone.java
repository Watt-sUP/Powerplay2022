package org.firstinspires.ftc.teamcode.commands.helpers;

/**
 * <p>Helper class for storing cone data.</p>
 * <p>Made for autonomous purposes.</p>
 */
public class Cone {
    public int stickPos = 0;
    public int conePos = 0;
    public int glisPos;
    public double coneScissors = 0;
    public double stickScissors = 0;
    public enum Junctions {
        Middle,
        High
    }
    public Junctions targetJunction;

    /**
     * Creates a new cone object
     * @param stickPos The turret position for turning to the stick
     * @param conePos The turret position for turning to the cone
     * @param coneScissors The scissors position for grabbing the cone
     * @param stickScissors The scissors position for reaching the stick
     */
    public Cone(int glisPos, int conePos, int stickPos, double coneScissors, double stickScissors) {
        this.glisPos = glisPos;
        this.stickPos = stickPos;
        this.conePos = conePos;
        this.coneScissors = coneScissors;
        this.stickScissors = stickScissors;
    }

    public Cone(Junctions junction, int glisPos) {
        targetJunction = junction;
        this.glisPos = glisPos;
    }
}