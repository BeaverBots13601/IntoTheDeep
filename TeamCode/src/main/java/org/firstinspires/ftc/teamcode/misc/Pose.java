package org.firstinspires.ftc.teamcode.misc;


/**
 * Object to conveniently manage position data. By standard, units should be in inches
 */
public class Pose {
    private final double x;
    private final double y;
    private final double angle;

    public Pose() {
        this.x = 0;
        this.y = 0;
        this.angle = 0;
    }

    public Pose(double x, double y, double angle) {
        this.x = x;
        this.y = y;
        this.angle = angle;
    }

    public double getX() {
        return x;
    }
    public double getY() {
        return y;
    }
    public double getAngle() {
        return angle;
    }

    /**
     * @param x x value of the point
     * @param y y value of the point
     * @param angle angle change of the point
     * @return rotation of point (x, y) around angle
     */
    public static Pose rotatePosition(double x, double y, double angle) {
        return new Pose(x * Math.cos(angle) - y * Math.sin(angle),
                x * Math.sin(angle) + y * Math.cos(angle), angle);
    }

    /**
     * @return double angle within the range [-π, π]
     */
    public static double normalizeAngle(double angle) {
        while (angle > Math.PI) {
            angle -= 2 * Math.PI;
        }
        while (angle < -Math.PI) {
            angle += 2 * Math.PI;
        }
        return angle;
    }


    public static double xTolerance = 0.1;
    public static double yTolerance = 0.1;
    public static double angleTolerance = Math.PI / 32;

    public boolean isAtPoseWithinTolerance(Pose currentPose){
        if(Math.abs(x - currentPose.getX()) < xTolerance) return false;
        if(Math.abs(y - currentPose.getY()) < yTolerance) return false;
        if(Math.abs(angle - currentPose.getAngle()) < angleTolerance) return false;

        return true;
    }


}