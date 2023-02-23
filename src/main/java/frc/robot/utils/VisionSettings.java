package frc.robot.utils;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;

public class VisionSettings {
    public static final String CAMERA_NAME = "main_camera";

    public static final Transform3d cameraPosition = new Transform3d(
        new Translation3d(
            0, //Horizontal distance from the center of the robot to the camera.
            0, //Vertical distance from the center of the robot to the camera.
            0 //Distance from the ground to the camera.
        ),
        new Rotation3d( //The rotation of the camera relative to the robot.
            0, //Roll
            0, //Pitch
            0 //Yaw
        ) 
    );
}
