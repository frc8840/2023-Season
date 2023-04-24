package frc.robot.utils;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;

public class VisionSettings {
    public static final String CAMERA_NAME = "main_camera";

    //Should be camera position, but never got time to set it.
    public static final Transform3d cameraPosition = new Transform3d(
        new Translation3d(
            (-Measurements.Robot.WIDTH / 2) + Units.inchesToMeters(5.5), //Horizontal distance from the center of the robot to the camera.
            (Measurements.Robot.LENGTH / 2) - Units.inchesToMeters(1), //Vertical distance from the center of the robot to the camera.
            Units.inchesToMeters(7.75) //Distance from the ground to the camera.
        ),
        new Rotation3d( //The rotation of the camera relative to the robot.
            0, //Roll
            0, //Pitch
            0 //Yaw
        ) 
    );
}
