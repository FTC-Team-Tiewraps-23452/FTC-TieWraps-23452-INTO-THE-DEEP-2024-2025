package org.firstinspires.ftc.teamcode.lib.geometry;

/**
 * A change in distance along a 2D arc since the last pose update. We can use ideas from
 * differential calculus to create new Pose2d objects from a Twist2d and vice versa.
 *
 * <p>A Twist can be used to represent a difference between two poses.
 */
public class Twist2d {
    /** Linear "dx" component. */
    public double dx;

    /** Linear "dy" component. */
    public double dy;

    /** Angular "dtheta" component (radians). */
    public double dtheta;

    public Twist2d() {}

    /**
     * Constructs a Twist2d with the given values.
     *
     * @param dx Change in x direction relative to robot.
     * @param dy Change in y direction relative to robot.
     * @param dtheta Change in angle relative to robot.
     */
    public Twist2d(double dx, double dy, double dtheta) {
        this.dx = dx;
        this.dy = dy;
        this.dtheta = dtheta;
    }

    @Override
    public String toString() {
        return String.format("Twist2d(dX: %.2f, dY: %.2f, dTheta: %.2f)", dx, dy, dtheta);
    }
}
