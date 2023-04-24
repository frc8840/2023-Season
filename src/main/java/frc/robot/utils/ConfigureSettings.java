package frc.robot.utils;

import edu.wpi.first.math.util.Units;
import frc.team_8840_lib.controllers.SwerveGroup;
import frc.team_8840_lib.listeners.Robot;
import frc.team_8840_lib.utils.controllers.swerve.SwerveSettings;
import frc.team_8840_lib.utils.controllers.swerve.SwerveType;
import frc.team_8840_lib.utils.controllers.swerve.structs.PIDStruct;

public class ConfigureSettings {
    
    /**
     * Gets the settings for the swerve drive.
     * @return The settings for the swerve drive.
     */
    public static SwerveSettings getSwerveDriveSettings() {
        //Declare a new SwerveSettings object.
        SwerveSettings settings = new SwerveSettings(SwerveType.SPARK_MAX);

        //Adjust a few PID settings to the type, but we'll have to edit down below a few things.
        settings.defaultAdjustToType();

        //Track width is 18.75in, and wheel base is 22.75in.
        //Actually this isn't accurate, I should've updated this, this might be why a few things might be a lil off.
        settings.trackWidth = Units.inchesToMeters(18.75);
        settings.wheelBase = Units.inchesToMeters(22.75);

        //We know the wheel diameter, so we'll set it here.
        settings.wheelDiameter = Units.inchesToMeters(3.94);

        //Drive Gear Ratio is 6.75 to 1
        settings.driveGearRatio = 6.75;
        //Angle Gear Ratio is 150/7 to 1
        settings.angleGearRatio = 150.0/7.0;

        if (Robot.isReal()) {
            //Set the angle offsets for each module.
            settings.angleOffsets[SwerveGroup.ModuleIndex.kTOP_LEFT.getIndex()] = ModuleConstants.TopLeft.OFFSET;
            settings.angleOffsets[SwerveGroup.ModuleIndex.kTOP_RIGHT.getIndex()] = ModuleConstants.TopRight.OFFSET;
            
            settings.angleOffsets[SwerveGroup.ModuleIndex.kBOTTOM_LEFT.getIndex()] = ModuleConstants.BottomLeft.OFFSET;
            settings.angleOffsets[SwerveGroup.ModuleIndex.kBOTTOM_RIGHT.getIndex()] = ModuleConstants.BottomRight.OFFSET;
        } else {
            settings.angleOffsets = new double[] {0,0,0,0};
        }

        //Gyro is inverted.
        settings.invertGyro = true;

        //CanCoder is inverted.
        settings.canCoderInverted = true;
        //Drive and turn are not inverted.
        settings.turnInverted = false;
        settings.driveInverted = false;

        if (Robot.isReal()) {
            //If real, set the PID values.
            settings.drivePID = new PIDStruct(0.025, 0, 0, 0);
            settings.turnPID = new PIDStruct(0.012, 0, 0, 0);

            settings.driveKA = 0.0;
            settings.driveKV = 0.0;
            settings.driveKS = 0.0;
        }
        
        //We'll set the max speed to the max speed of the mk4i l2 NEO since that's what we're using.
        settings.maxSpeed = SwerveSettings.SDS.MK4i.L2.maxSpeed_NEO;

        //We don't want to do manual conversion if the robot is real.
        settings.doManualConversion = Robot.isSimulation();
        //We want to use manual offset.
        settings.manualOffset = true;
        
        //Once done, we'll update the kinematics to reflect the changes.
        settings.updateKinematics();

        //We'll also set the threshold for controls/angle changes/whatever to 1%.
        settings.threshold = 0.01;
        settings.useThresholdAsPercentage = true;

        return settings;
    }
}
