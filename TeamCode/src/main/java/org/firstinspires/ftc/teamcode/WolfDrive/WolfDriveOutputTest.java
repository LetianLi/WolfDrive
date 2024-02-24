package org.firstinspires.ftc.teamcode.WolfDrive;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Rotation2d;
import com.acmerobotics.roadrunner.Vector2d;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

/**
 * Local test file to make sure outputs are right. Run main() in Android Studio with Coverage
 */
public class WolfDriveOutputTest {

    private static final String greenColor = "\033[0;32m";
    private static final String yellowColor = "\033[0;33m";
    private static final String redColor = "\033[0;31m";
    private static final String resetColor = "\033[0m"; // must use reset after using red so effect clears
    private static final String errorString = redColor + "ERROR" + resetColor;
    private static final String okString = greenColor + "OKAY" + resetColor;
    private static final String naString = yellowColor + "N/A (Correction or Turning Applied)" + resetColor;

    // Run isolated. Might need to run with coverage.
    public static void main(String[] args)
    {
        System.out.println();
        updateWheelForceVectors();
//        updateWheelForceVectorsRandom(-1, 1, 30); // send a really broken robot in and see if it still works. This is not supported.

        System.out.println("**************** Curvature Tests (Full Power) ****************");
        test(1, 0, 0, 0, 0, 0, 0, 10, 2, 20, 0);
        test(1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);
        test(1, 0, 0, 0, 0, 0, 0, 10, -2, 20, 0);

        System.out.println("**************** All Angle Tests (Half Power) ****************");
        test(0.5, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);
        test(0.5, 30, 0, 0, 0, 0, 0, 0, 0, 0, 0);
        test(0.5, 60, 0, 0, 0, 0, 0, 0, 0, 0, 0);
        test(0.5, 90, 0, 0, 0, 0, 0, 0, 0, 0, 0);
        test(0.5, 120, 0, 0, 0, 0, 0, 0, 0, 0, 0);
        test(0.5, 150, 0, 0, 0, 0, 0, 0, 0, 0, 0);
        test(0.5, 180, 0, 0, 0, 0, 0, 0, 0, 0, 0);
        test(0.5, 210, 0, 0, 0, 0, 0, 0, 0, 0, 0);
        test(0.5, 240, 0, 0, 0, 0, 0, 0, 0, 0, 0);
        test(0.5, 270, 0, 0, 0, 0, 0, 0, 0, 0, 0);
        test(0.5, 300, 0, 0, 0, 0, 0, 0, 0, 0, 0);
        test(0.5, 330, 0, 0, 0, 0, 0, 0, 0, 0, 0);
        test(0.5, 360, 0, 0, 0, 0, 0, 0, 0, 0, 0);

        System.out.println("**************** Strafe tests ****************");
        test(1, 89, 0, 0, 0, 0, 0, 0, 0, 0, 0);
        test(1, 90, 0, 0, 0, 0, 0, 0, 0, 0, 0);
        test(1, 91, 0, 0, 0, 0, 0, 0, 0, 0, 0);
        test(1, 269, 0, 0, 0, 0, 0, 0, 0, 0, 0);
        test(1, 270, 0, 0, 0, 0, 0, 0, 0, 0, 0);
        test(1, 271, 0, 0, 0, 0, 0, 0, 0, 0, 0);

        System.out.println("**************** Invalid Curvature Tests and No Power Tests ****************");
        test(0.5, 90, 0, 0, 0, 0, 0, 0, 12, 0, 30);
        test(0, 90, 0, 0, 0, 0, 0, 0, 12, 0, 30);
        test(0, 180, 1, 0, 0, 0, 0, 0, 12, 0, 30);
        test(0.5, 180, 0, 30, 30, 0, 0, 16, 12, 30, 30);

    }

    public static double maxVelocityX = 77; // max positive straight velocity. Record using MaxVelStraightTest.
    public static double maxVelocityY = 51; // max positive sideways velocity. Record using MaxVelStrafeTest.
    public static double centripetalWeighting = 0; // adjust by trial and error for how much smoothing you need. Wolfpack calculates it but I can't be bothered.
    public static double dashboardVectorScale = 1;

    private static Vector2d leftFrontWheelForceVector;  // these vectors should not change during the match,
    private static Vector2d leftBackWheelForceVector;   // but are left non-final for modifying maxVelocities
    private static Vector2d rightBackWheelForceVector;
    private static Vector2d rightFrontWheelForceVector;
    private static double smallestWheelForceMagnitude;

    private static Vector2d centripetalCircleCenterDrawn = null;
    private static Vector2d centripetalCircleRadiusDrawn = null;
    private static Vector2d centripetalVectorDrawn = null;
    private static Vector2d robotDriveDirectionDrawn = null;

    private static final RingBuffer positionBuffer = new RingBuffer(3);

    public static void test(double stickPower, double stickAngle, double turnPower, double velPower, double velAngle, double p1x, double p1y, double p2x, double p2y, double p3x, double p3y) {

        positionBuffer.put(new Pose2d(p3x, p3y, 0));
        positionBuffer.put(new Pose2d(p2x, p2y, 0));
        positionBuffer.put(new Pose2d(p1x, p1y, 0));

        Vector2d stick = Rotation2d.Companion.exp(Math.toRadians(stickAngle)).vec().times(stickPower);
        PoseVelocity2d velocity = new PoseVelocity2d(Rotation2d.Companion.exp(Math.toRadians(velAngle)).vec().times(velPower), 0);

        double[] wheelPowers = driveWithCorrection(new PoseVelocity2d(stick, turnPower), velocity);

        System.out.println("Ring buffer: " + positionBuffer);
        if (centripetalCircleCenterDrawn == null) System.out.println("Circle stats: No circle");
        else System.out.println(String.format("Circle stats: %s", getCircleString()));
        System.out.println(String.format("Stick:           %s", formatVector(stick)));
        System.out.println(String.format("Current vel:     %s", formatVector(velocity.linearVel)));
        System.out.println(String.format("Correction:      %s", getCorrectionString()));
        System.out.println(String.format("Drive direction: %s", getDriveDirectionString()));
        System.out.println(getWheelPowerString(wheelPowers));

        double actualDriveDirection = Math.toDegrees(leftFrontWheelForceVector.times(wheelPowers[0]).plus(leftBackWheelForceVector.times(wheelPowers[1])).plus(rightBackWheelForceVector.times(wheelPowers[2])).plus(rightFrontWheelForceVector.times(wheelPowers[3])).angleCast().log());
        boolean driveDirectionCorrect = Math.abs(AngleUnit.normalizeDegrees(actualDriveDirection-Math.toDegrees(stick.angleCast().log()))) <= 1e-6;
        boolean driveIsTurning = Math.signum(wheelPowers[0]) == Math.signum(wheelPowers[1]) && Math.signum(wheelPowers[2]) == Math.signum(wheelPowers[3]) && Math.signum(wheelPowers[0]) != Math.signum(wheelPowers[3]);
        if (driveIsTurning) actualDriveDirection = Double.NaN;
        System.out.println(String.format("Actual drive direction: %f°, %s", actualDriveDirection, driveDirectionCorrect&&!driveIsTurning?okString:(centripetalCircleCenterDrawn==null&&turnPower==0?errorString:naString)));
        System.out.println();
    }

    /**
     * Flashes new maxVelocities into the force vectors.
     */
    public static void updateWheelForceVectors() {
        leftFrontWheelForceVector = new Vector2d(maxVelocityX, -maxVelocityY).times(1);
        leftBackWheelForceVector = new Vector2d(maxVelocityX, maxVelocityY).times(0.5);
        rightBackWheelForceVector = new Vector2d(maxVelocityX, -maxVelocityY).times(1);
        rightFrontWheelForceVector = new Vector2d(maxVelocityX, maxVelocityY).times(1);
        smallestWheelForceMagnitude = getAbsMin(leftBackWheelForceVector.norm(), leftFrontWheelForceVector.norm(), rightBackWheelForceVector.norm(), rightFrontWheelForceVector.norm());
    }

    /**
     * Puts some random directional offsets into the force vectors
     */
    public static void updateWheelForceVectorsRandom(double scaleVectorMin, double scaleVectorMax, double offsetVectorMaxRadius) {
        leftFrontWheelForceVector = new Vector2d(maxVelocityX + (offsetVectorMaxRadius-2*offsetVectorMaxRadius*Math.random()), -maxVelocityY + (offsetVectorMaxRadius-2*offsetVectorMaxRadius*Math.random())).times(scaleVectorMin+(scaleVectorMax-scaleVectorMin)*Math.random());
        leftBackWheelForceVector = new Vector2d(maxVelocityX + (offsetVectorMaxRadius-2*offsetVectorMaxRadius*Math.random()), maxVelocityY + (offsetVectorMaxRadius-2*offsetVectorMaxRadius*Math.random())).times(scaleVectorMin+(scaleVectorMax-scaleVectorMin)*Math.random());
        rightBackWheelForceVector = new Vector2d(maxVelocityX + (offsetVectorMaxRadius-2*offsetVectorMaxRadius*Math.random()), -maxVelocityY + (offsetVectorMaxRadius-2*offsetVectorMaxRadius*Math.random())).times(scaleVectorMin+(scaleVectorMax-scaleVectorMin)*Math.random());
        rightFrontWheelForceVector = new Vector2d(maxVelocityX + (offsetVectorMaxRadius-2*offsetVectorMaxRadius*Math.random()), maxVelocityY + (offsetVectorMaxRadius-2*offsetVectorMaxRadius*Math.random())).times(scaleVectorMin+(scaleVectorMax-scaleVectorMin)*Math.random());
        smallestWheelForceMagnitude = getAbsMin(leftBackWheelForceVector.norm(), leftFrontWheelForceVector.norm(), rightBackWheelForceVector.norm(), rightFrontWheelForceVector.norm());
    }

    /**
     * Sets wheel powers depending on given robot drive direction
     */
    public static double[] setDrivePowers(PoseVelocity2d powers) {
        // Calculate starting power for forwards, using linear velocity and scaling down to weakest wheel
        double drivePower = powers.linearVel.norm();
        double leftFrontPower = drivePower * smallestWheelForceMagnitude / leftFrontWheelForceVector.norm();
        double leftBackPower = drivePower * smallestWheelForceMagnitude / leftBackWheelForceVector.norm();
        double rightBackPower = drivePower * smallestWheelForceMagnitude / rightBackWheelForceVector.norm();
        double rightFrontPower = drivePower * smallestWheelForceMagnitude / rightFrontWheelForceVector.norm();
        // Rotate wheel force vectors and scale with drive power
        double driveDirection = powers.linearVel.angleCast().log();
        Vector2d newLeftFrontWheelForceVector = leftFrontWheelForceVector.angleCast().plus(-driveDirection).vec().times(leftFrontPower);
        Vector2d newLeftBackWheelForceVector = leftBackWheelForceVector.angleCast().plus(-driveDirection).vec().times(leftBackPower);
        Vector2d newRightBackWheelForceVector = rightBackWheelForceVector.angleCast().plus(-driveDirection).vec().times(rightBackPower);
        Vector2d newRightFrontWheelForceVector = rightFrontWheelForceVector.angleCast().plus(-driveDirection).vec().times(rightFrontPower);

        System.out.println(String.format("(%+7.2f, %7.2f)  (%+7.2f, %7.2f)", newLeftFrontWheelForceVector.x, newLeftFrontWheelForceVector.y, newRightFrontWheelForceVector.x, newRightFrontWheelForceVector.y));
        System.out.println(String.format("(%+7.2f, %7.2f)  (%+7.2f, %7.2f)", newLeftBackWheelForceVector.x, newLeftBackWheelForceVector.y, newRightBackWheelForceVector.x, newRightBackWheelForceVector.y));

        // Find vectors with the most X value
        if (Math.abs(newLeftFrontWheelForceVector.x) >= Math.abs(newRightFrontWheelForceVector.x)) { // leftFront and rightBack contribute most to drive direction
            // Reverse primary vectors if necessary
            if (newLeftFrontWheelForceVector.x < 0) {
                newLeftFrontWheelForceVector = newLeftFrontWheelForceVector.times(-1);
                leftFrontPower *= -1;
                newRightBackWheelForceVector = newRightBackWheelForceVector.times(-1);
                rightBackPower *= -1;
            }

            double extraneousY = newLeftFrontWheelForceVector.y;
            if (newRightFrontWheelForceVector.y != 0 && newLeftBackWheelForceVector.y != extraneousY) { // Make secondary vector y's cancel out primary's
                double rightFrontFactor = -extraneousY / newRightFrontWheelForceVector.y;
                newRightFrontWheelForceVector = newRightFrontWheelForceVector.times(rightFrontFactor);
                rightFrontPower *= rightFrontFactor;
                newLeftBackWheelForceVector = newLeftBackWheelForceVector.times(rightFrontFactor);
                leftBackPower *= rightFrontFactor;
            } else if (newLeftBackWheelForceVector.x < 0) { // Reverse secondary vectors if necessary
                newLeftBackWheelForceVector = newLeftBackWheelForceVector.times(-1);
                leftBackPower *= -1;
                newRightFrontWheelForceVector = newRightFrontWheelForceVector.times(-1);
                rightFrontPower *= -1;
            }
        } else { // rightFront and leftBack contribute most to drive direction
            // Reverse primary vectors if necessary
            if (newRightFrontWheelForceVector.x < 0) {
                newRightFrontWheelForceVector = newRightFrontWheelForceVector.times(-1);
                rightFrontPower *= -1;
                newLeftBackWheelForceVector = newLeftBackWheelForceVector.times(-1);
                leftBackPower *= -1;
            }

            double extraneousY = newRightFrontWheelForceVector.y;
            if (newLeftFrontWheelForceVector.y != 0 && newRightBackWheelForceVector.y != extraneousY) { // Make secondary vector y's cancel out primary's
                double leftFrontFactor = -extraneousY / newLeftFrontWheelForceVector.y;
                newLeftFrontWheelForceVector = newLeftFrontWheelForceVector.times(leftFrontFactor);
                leftFrontPower *= leftFrontFactor;
                newRightBackWheelForceVector = newRightBackWheelForceVector.times(leftFrontFactor);
                rightBackPower *= leftFrontFactor;
            } else if (newRightBackWheelForceVector.x < 0) { // Reverse secondary vectors if necessary
                newRightBackWheelForceVector = newRightBackWheelForceVector.times(-1);
                rightBackPower *= -1;
                newLeftFrontWheelForceVector = newLeftFrontWheelForceVector.times(-1);
                leftFrontPower *= -1;
            }
        }

        System.out.println(String.format("(%+7.2f, %7.2f)  (%+7.2f, %7.2f)", newLeftFrontWheelForceVector.x, newLeftFrontWheelForceVector.y, newRightFrontWheelForceVector.x, newRightFrontWheelForceVector.y));
        System.out.println(String.format("(%+7.2f, %7.2f)  (%+7.2f, %7.2f)", newLeftBackWheelForceVector.x, newLeftBackWheelForceVector.y, newRightBackWheelForceVector.x, newRightBackWheelForceVector.y));

        Vector2d wheelForceSum = newLeftFrontWheelForceVector.plus(newLeftBackWheelForceVector).plus(newRightBackWheelForceVector).plus(newRightFrontWheelForceVector);
        boolean wheelForceSumDirectionCorrect = Math.abs(Math.toDegrees(wheelForceSum.angleCast().log()))<=1e-6;
        System.out.println(String.format("Adding wheel forces: mag=%f, angle=%f°, (%5.2f, %5.2f), %s", wheelForceSum.norm(), Math.toDegrees(wheelForceSum.angleCast().log()), wheelForceSum.x, wheelForceSum.y, wheelForceSumDirectionCorrect?okString:errorString));

        // consider turn power
        leftFrontPower = leftFrontPower - powers.angVel;
        leftBackPower = leftBackPower - powers.angVel;
        rightBackPower = rightBackPower + powers.angVel;
        rightFrontPower = rightFrontPower + powers.angVel;
        double maxPower = getAbsMax(leftFrontPower, leftBackPower, rightBackPower, rightFrontPower);
        if (maxPower > 1) {
            leftFrontPower = leftFrontPower / maxPower;
            leftBackPower = leftBackPower / maxPower;
            rightBackPower = rightBackPower / maxPower;
            rightFrontPower = rightFrontPower / maxPower;
        }

        return new double[]{ leftFrontPower, leftBackPower, rightBackPower, rightFrontPower };
    }

    /**
     * Drives with additional centripetal correction
     */
    public static double[] driveWithCorrection(PoseVelocity2d powers, PoseVelocity2d currentVelocity) {
        // Find centripetal power
        Vector2d centripetalPower = calculateCentripetalPower(currentVelocity);

        // Combine with joystick power
        Vector2d combinedPower = combinePower(powers.linearVel, centripetalPower);

        robotDriveDirectionDrawn = combinedPower.times(dashboardVectorScale);

        return setDrivePowers(new PoseVelocity2d(combinedPower, powers.angVel));
    }

    /**
     * Adds two power vectors together, ensuring that the result is not of bigger magnitude than the mainPower
     */
    private static Vector2d combinePower(Vector2d mainPower, Vector2d correctivePower) {
        // get max power
        double maxPower = mainPower.norm();

        // Sum
        Vector2d combinedPower = mainPower.plus(correctivePower);

        // Scale down
        Vector2d scaledCombinedPower = combinedPower;
        if (combinedPower.norm() > maxPower) { // scale down if over max. Never scale up
            scaledCombinedPower = combinedPower.div(combinedPower.norm()).times(maxPower);
        }

        return scaledCombinedPower;
    }

    /**
     * Calculates the centripetal vector by fitting 3 past positions and considering current linear velocity
     */
    private static Vector2d calculateCentripetalPower(PoseVelocity2d currentVelocity) {
        // Read the three points from ring buffer
        if (!positionBuffer.isFull()) return new Vector2d(0, 0);
        Vector2d p3 = positionBuffer.read(0).position; // y(t=3)
        Vector2d p2 = positionBuffer.read(1).position; // y(t=2)
        Vector2d p1 = positionBuffer.read(2).position; // y(t=1)

        // Find circumcenter
        double ax = p1.x;
        double ay = p1.y;
        double bx = p2.x;
        double by = p2.y;
        double cx = p3.x;
        double cy = p3.y;
        double d = 2 * (ax * (by - cy) + bx * (cy - ay) + cx * (ay - by));
        double ux = ((ax * ax + ay * ay) * (by - cy) + (bx * bx + by * by) * (cy - ay) + (cx * cx + cy * cy) * (ay - by)) / d;
        double uy = ((ax * ax + ay * ay) * (cx - bx) + (bx * bx + by * by) * (ax - cx) + (cx * cx + cy * cy) * (bx - ax)) / d;
        Vector2d circumcenter = new Vector2d(ux, uy);

        if (Double.isFinite(circumcenter.x) && Double.isFinite(circumcenter.y)) {
            centripetalCircleCenterDrawn = circumcenter.times(1);

            Vector2d circleVector = circumcenter.minus(p3); // represents vector from p3 to the center of the circle
            centripetalCircleRadiusDrawn = circleVector.times(1);

            Vector2d centripetalVector = circleVector.div(circleVector.sqrNorm()); // scales circleVector with mag r to centripetalVector with mag 1/r

            // Calculate power
            double power = centripetalWeighting * centripetalVector.norm() * currentVelocity.linearVel.sqrNorm(); // F=mv^2/r, power = weight*F

            Vector2d centripetalPower = centripetalVector.div(centripetalVector.norm()).times(power); // change magnitude to power
            centripetalVectorDrawn = centripetalPower.times(dashboardVectorScale);

            return centripetalPower;
        } else {
            centripetalCircleCenterDrawn = null;
            centripetalCircleRadiusDrawn = null;
            centripetalVectorDrawn = null;
            return new Vector2d(0, 0);
        }
    }

    /**
     * Returns the largest magnitude of the given values
     */
    private static double getAbsMax(double... values) {
        double max = 0;
        for (double value : values) {
            max = Math.max(max, Math.abs(value));
        }
        return max;
    }

    /**
     * Returns the smallest magnitude of the given values
     */
    private static double getAbsMin(double... values) {
        double min = Double.MAX_VALUE;
        for (double value : values) {
            min = Math.min(min, Math.abs(value));
        }
        return min;
    }

    public static String formatVector(Vector2d vector) {
        if (vector == null) return "";
        return String.format("mag=%5.1f, angle=%5.1f°, (%5.2f, %5.2f)", vector.norm(), Math.toDegrees(vector.angleCast().log()), vector.x, vector.y);
    }

    public static String getCircleString() {
        if (centripetalCircleCenterDrawn == null) {
            return "No circle";
        } else {
            return String.format("r=%.2f, center=(%.2f, %.2f)", centripetalCircleRadiusDrawn.norm(), centripetalCircleCenterDrawn.x, centripetalCircleCenterDrawn.y);
        }
    }

    public static String getCorrectionString() {
        if (centripetalVectorDrawn == null) {
            return "None";
        } else {
            return formatVector(centripetalVectorDrawn);
        }
    }

    public static String getDriveDirectionString() {
        if (robotDriveDirectionDrawn == null) {
            return "Stopped";
        } else {
            return formatVector(robotDriveDirectionDrawn);
        }
    }

    public static String getWheelPowerString(double[] wheelPowers) {
        return String.format("%+1.3f  %+1.3f\n%+1.3f  %+1.3f", wheelPowers[0], wheelPowers[3], wheelPowers[1], wheelPowers[2]);
    }
}