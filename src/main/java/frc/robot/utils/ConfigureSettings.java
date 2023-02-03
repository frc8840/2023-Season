package frc.robot.utils;

import edu.wpi.first.math.util.Units;
import frc.team_8840_lib.controllers.SwerveGroup;
import frc.team_8840_lib.utils.controllers.swerve.SwerveSettings;
import frc.team_8840_lib.utils.controllers.swerve.SwerveType;
import frc.team_8840_lib.utils.controllers.swerve.structs.PIDStruct;

public class ConfigureSettings {
    
    public static SwerveSettings getSwerveDriveSettings() {
        SwerveSettings settings = new SwerveSettings(SwerveType.SPARK_MAX);

        //Adjust a few PID settings to the type, but we'll have to edit down below a few things.
        settings.defaultAdjustToType();

        //Track width is 18.75in, and wheel base is 27.75in.
        settings.trackWidth = Units.inchesToMeters(18.75);
        settings.wheelBase = Units.inchesToMeters(22.75);

        //We know the wheel diameter, so we'll set it here.
        //TODO: Ask fabrication or design teams for the precise wheel diameter (again).
        settings.wheelDiameter = Units.inchesToMeters(3.94);

        //Drive Gear Ratio is 6.75 to 1
        settings.driveGearRatio = 6.75;
        //Angle Gear Ratio is 150/7 to 1
        settings.angleGearRatio = 150.0/7.0;

        // settings.angleOffsets[0] = 77.43; //First module
        // settings.angleOffsets[1] = 35.51; //Second module
        // settings.angleOffsets[2] = 138.33; //Third module
        // settings.angleOffsets[3] = 324.14; //Fourth module

        settings.angleOffsets[SwerveGroup.ModuleIndex.kTOP_LEFT.getIndex()] = 0;
        settings.angleOffsets[SwerveGroup.ModuleIndex.kTOP_RIGHT.getIndex()] = 0;
        
        settings.angleOffsets[SwerveGroup.ModuleIndex.kBOTTOM_LEFT.getIndex()] = 0;
        settings.angleOffsets[SwerveGroup.ModuleIndex.kBOTTOM_RIGHT.getIndex()] = 0;

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
