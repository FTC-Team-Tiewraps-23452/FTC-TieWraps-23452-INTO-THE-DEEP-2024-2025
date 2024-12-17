package org.firstinspires.ftc.teamcode.robot.util;

import org.firstinspires.ftc.teamcode.lib.geometry.Rotation2d;
import org.firstinspires.ftc.teamcode.lib.geometry.Translation2d;
import org.firstinspires.ftc.teamcode.lib.util.Units;

/**
 * With this software template, a library is included which contains
 * a lot of useful functions.
 *
 * In this file, examples of how to use several of these functions are shown
 *
 * To use the functions, they should first be imported.
 * This is done above after the "import" statement
 * This should be done in each file where you want to use the functions
 */
public class MathExample {

    public MathExample() {}

    /**
     * This is an example of how to use the rotation and translation objects
     */
    public void translationAndRotation(){
        /*
         * Instead of storing angles as doubles of integers, they can be stored
         * as a Rotation2d object, which automatically takes care of the units
         * so you don't have to
         */

        //Make a new Rotation2d object and give it a value in radians
        Rotation2d angle = Rotation2d.fromRadians(2 * Math.PI);

        //This could also be done with degrees
        Rotation2d secondAngle = Rotation2d.fromDegrees(180);

        //With these objects we can do a lot of things
        //For example calculating cosines and sines
        double angleSine = angle.getSin();
        double secondAngleCosine = secondAngle.getCos();

        //Both objects can also be added to eachother
        //Because they are Rotation2d objects, units are automatically taken care of
        Rotation2d thirdAngle = angle.plus(secondAngle);

        /*
         * The same solution is present in the library for coordinates
         * Instead of storing coordinates as doubles, integers or arrays, they can be stored
         * as a Translation2d object
         * For the Translation2d, you do have to make sure that you are consistent with units
         */

        //Make two Translation2d objects
        Translation2d translation = new Translation2d(3,4);
        Translation2d secondTranslation = new Translation2d(-1.3, 5.2);

        //Translation2d also has some useful functions
        //For example calculating the distance from (0,0) to the point
        double distance = translation.getNorm();

        //Or the distance from one point to another
        double distanceOneToTwo = translation.getDistance(secondTranslation);

        //The x and y can also be requested back from the translation
        double x = translation.getX();
        double y = translation.getY();
    }

    /**
     * Example of how to convert units with library functions
     */
    public void unitConversion(){

        //The library also contains several functions to convert between certain units
        //For example for distances
        double distanceInches = 2.3;
        double distanceMeters = Units.inchesToMeters(distanceInches);

        //And angles
        double angleDegrees = 45;
        double angleRadians = Units.degreesToRadians(angleDegrees);

        //And velocities
        double speedRPM = 50; //RPM = Rotations Per Minute
        double speedRadPerS = Units.rotationsPerMinuteToRadiansPerSecond(speedRPM); //Rad/s = radians per second

    }
}
