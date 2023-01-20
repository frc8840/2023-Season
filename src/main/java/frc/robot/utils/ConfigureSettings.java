package frc.robot.utils;

import edu.wpi.first.math.util.Units;
import frc.team_8840_lib.utils.controllers.swerve.SwerveSettings;
import frc.team_8840_lib.utils.controllers.swerve.SwerveType;
import frc.team_8840_lib.utils.controllers.swerve.structs.PIDStruct;

public class ConfigureSettings {
    
    public static SwerveSettings getSwerveDriveSettings() {
        SwerveSettings settings = new SwerveSettings(SwerveType.SPARK_MAX);

        //Adjust a few PID settings to the type, but we'll have to edit down below a few things.
        settings.defaultAdjustToType();

        //We don't know the track width or wheel base yet, so we'll leave it blank for now.
        settings.trackWidth = 0.0;
        settings.wheelBase = 0.0;

        //We know the wheel diameter, so we'll set it here.
        //TODO: Ask fabrication or design teams for the precise wheel diameter (again).
        settings.wheelDiameter = Units.inchesToMeters(3.94);

        //TODO: Ask fabrication or design teams for the precise drive gear ratio (should be found on the Swerve Drive Specialties website).
        settings.driveGearRatio = 6.86;
        settings.angleGearRatio = 12.8;

        settings.angleOffsets[0] = 0; //First module
        settings.angleOffsets[1] = 0; //Second module
        settings.angleOffsets[2] = 0; //Third module
        settings.angleOffsets[3] = 0; //Fourth module

        //TODO: Check if the gyro is inverted or not.
        settings.invertGyro = false;

        settings.drivePID = new PIDStruct(0.01, 0, 0, 0);
        
        settings.driveKA = 0.0;
        settings.driveKV = 0.0;
        settings.driveKS = 0.0;
        
        //Once done, we'll update the kinematics to reflect the changes.
        settings.updateKinematics();

        //We'll also set the threshold for controls/angle changes/whatever to 1%.
        settings.threshold = 0.01;
        settings.useThresholdAsPercentage = true;

        return settings;
    }
}
