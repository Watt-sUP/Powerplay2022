package org.firstinspires.ftc.teamcode.hardware;

/**
 * <p>Helper class for storing cone data.</p>
 * <p>Made for autonomous purposes.</p>
 */
public class Cone {
    public int stickPos, glisPos;
    public int conePos;
    public double coneScissors, stickScissors;

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
}