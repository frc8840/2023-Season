package frc.robot;

import java.util.TimerTask;

import com.revrobotics.REVPhysicsSim;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.auto.AutoBalance;
import frc.robot.commands.auto.movement.PIDRotate;
import frc.robot.utils.Measurements;
import frc.robot.utils.ModuleConstants;
import frc.team_8840_lib.info.console.Logger;
import frc.team_8840_lib.input.communication.CommunicationManager;
import frc.team_8840_lib.listeners.EventListener;
import frc.team_8840_lib.listeners.Robot;
import frc.team_8840_lib.pathing.PathPlanner;
import frc.team_8840_lib.pathing.PathConjugate.ConjugateType;
import frc.team_8840_lib.utils.GamePhase;
import frc.team_8840_lib.utils.controllers.Pigeon;

public class ChargedUpRobot extends EventListener {

    //General methods (not related to driving)
    private static ChargedUpRobot instance;

    public static ChargedUpRobot getInstance() {
        return instance;
    }

    public ChargedUpRobot() {
        instance = this;
    }


    //VARIABLES
    private RobotContainer m_robotContainer;

    //ROBOT INIT
    @Override
    public void robotInit() {
        //Create a new robot container, which handles the subsystems and commands.
        m_robotContainer = new RobotContainer();

        //We'll initialize our autonomous here.
        try {
            //Call the initialization method in AutonomousContainer.
            //This will load all of the autonomous paths.
            AutonomousContainer.init();
        } catch (Exception e) {
            //We need to catch any errors that occur during autonomous initialization.
            //This is if files are uploaded incorrectly/missing.
            Logger.Log("[Auto] Failed to initialize autonomous!");
            //Print error to the console to help debug.
            e.printStackTrace();
        }

        //Create a new field object in Network Tables.
        CommunicationManager.getInstance().createField();

        //Set autonomous to be at a fixed rate of every 1/32 seconds.
        Robot.getInstance().subscribeFixedPhase(new TimerTask() {
            @Override
            public void run() {
                onFixedAutonomous();
            }
        }, GamePhase.Autonomous);

        //Tell 8840-utils to wait till the swerve drive is ready to be used
        Robot.getRealInstance()
            //Set the fullfillment conditions to be either 5000ms has passed, or the swerve drive has declared itself ready.
            .waitForFullfillConditions(5000, () -> m_robotContainer.getDriveSubsystem().ready())
            .onFinishFullfillment(() -> {
                //Once the swerve drive is ready, reset the odometry.
                m_robotContainer.getDriveSubsystem().resetOdometry();
            });
    }

    //ROBOT PERIODIC

    @Override
    public void robotPeriodic() {
        //Run the REVPhysicsSim if we're in simulation mode.
        if (Robot.isSimulation()) {
            REVPhysicsSim.getInstance().run();
        }

        //Update the Network Tables with the swerve drive information.
        CommunicationManager.getInstance().updateSwerveInfo(RobotContainer.getInstance().getDriveSubsystem().getSwerveDrive());
        //If the field exists
        if (CommunicationManager.getInstance().fieldExists()) {
            //Update the swerve odometry
            RobotContainer.getInstance().getDriveSubsystem().getSwerveDrive().updateOdometry();
            //I forget why I put this twice, this periodic method is from when I was still making swerve code, before the season (can be found in the autonomous example in 8840-utils)
            //Not really changing it because it works, and I don't want to break anything.

            //Set the robot pose in Network Tables
            CommunicationManager.getInstance().updateFieldObjectPose("SwerveRobot", RobotContainer.getInstance().getDriveSubsystem().getSwerveDrive().getPose());
            //Update the other odometry in Network Tables
            RobotContainer.getInstance().getDriveSubsystem().getSwerveDrive().updateFieldRobot();
        }

        //Old method that should've been removed since it takes up unnecessary space on Network Tables.
        //I'll leave it here for now since there's no damage being done keeping it.
        //It checks if the pigeon exists, and if it does, it updates the Network Tables with the tilt.
        if (Pigeon.hasPigeon(ModuleConstants.PIGEON_ID) != null) {
            CommunicationManager.getInstance().updateInfo("autobalance", "tilt", AutoBalance.getTilt());
        }

        //Run any commands that are scheduled (WPILib default).
        CommandScheduler.getInstance().run();
    }
    
    //AUTONOMOUS METHODS

    @Override
    public void onAutonomousEnable() {
        //Tell robot container that the phase changed.
        m_robotContainer.onPhaseChange();

        //If the auto has not been selected, just go with whatever the field layout is.
        if (!PathPlanner.selectedAuto()) {
            //I think I just forgot to write the method to select the auto based on the side.
            //It's fine though, but I'll leave this here for now.
            Logger.Log("[Auto] No auto selected! Going with the suggested auto based on the field layout.");
        } else {
            //If the auto has been selected, start it.
            PathPlanner.getSelectedAuto().start();
        }
    }

    public void onFixedAutonomous() {
        //While the auto is not finished, run the auto.
        if (!PathPlanner.getSelectedAuto().finished()) {
            //If it's a path, run the drive code.
            if (PathPlanner.getSelectedAuto().getCurrentType() == ConjugateType.Path) {
                //TODO: create a method in the drive subsystem that takes in the pose and the last pose to drive.

                //Move to the next pose, and then get the pose in the path
                Pose2d pose = PathPlanner.getSelectedAuto().current().getPath().moveToNext();
                
                //Get the very last pose in the path
                Pose2d finalPose = PathPlanner.getSelectedAuto().current().getPath().getFinalPose();

                //Convert over to a field accurate pose due to conversion issues between 8840-app and here, etc.
                finalPose = new Pose2d(
                    Units.inchesToMeters(Measurements.Field.WIDTH) - finalPose.getX(),
                    finalPose.getY(), Rotation2d.fromDegrees(0)
                );

                boolean hasRotationGoal = PathPlanner.getSelectedAuto().current().getPath().hasRotationGoal();
                Rotation2d rotationGoal = PathPlanner.getSelectedAuto().current().getPath().getRotationGoal();
                Rotation2d currentRotation = Rotation2d.fromDegrees(Pigeon.getPigeon(ModuleConstants.PIGEON_ID).getYawPitchRoll()[0]);

                double rotation = 0;

                //If the path has a rotation goal, calculate the rotation.
                //This was never used since I wasn't able to figure it out rotating in place beforehand.
                if (hasRotationGoal) {
                    rotation = PIDRotate.pid.calculate(currentRotation.getDegrees(), rotationGoal.getDegrees()) * 2;
                    
                    rotation /= 180;

                    if (Math.abs(rotation) > 2) {
                        rotation = Math.signum(rotation) * 2;
                    }

                    if (Math.abs(rotationGoal .getDegrees() - currentRotation.getDegrees()) < 5) {
                        rotation = 0;
                    }
                }

                //Convert the pose to a field accurate pose.
                pose = new Pose2d( 
                    Units.inchesToMeters(Measurements.Field.WIDTH) - pose.getX(), 
                    pose.getY(), Rotation2d.fromDegrees(0)
                );

                //Get the last pose in the path
                Pose2d lastPose = PathPlanner.getSelectedAuto().current().getPath().getLastPose();

                //Convert the last pose to a field accurate pose.
                lastPose = new Pose2d(
                    Units.inchesToMeters(Measurements.Field.WIDTH) - lastPose.getX(),
                    lastPose.getY(), Rotation2d.fromDegrees(0)
                );

                //Find Difference between the two poses
                double xDiff = -Units.metersToFeet(pose.getTranslation().getX() - lastPose.getTranslation().getX());
                double yDiff = Units.metersToFeet(pose.getTranslation().getY() - lastPose.getTranslation().getY());

                //Create a new translation with the difference
                Translation2d translation = new Translation2d(xDiff / Robot.DELTA_TIME, yDiff / Robot.DELTA_TIME);

                //Scale the translation by 1.1
                //This scalar is to offset friction and other factors.
                //I've found that by testing a few paths and see where it goes without a scalar.
                //After, find the differences between the expected and actual path, and then scale the translation by a general value that will get it close enough.
                //I found that ~1.1 is a good value, and it worked out perfectly.
                //I don't know really how accurate it is, but it worked perfectly for us.
                translation.times(1.1);

                //Move the robot based on the translation and rotation.
                RobotContainer.getInstance().getDriveSubsystem().getSwerveDrive().drive(translation, rotation, true, Robot.isReal());

                //Update NT robot pose
                CommunicationManager.getInstance().updateRobotPose(pose);

                //Find the difference between the final pose and the current pose.
                double xDifferenceBetweenFinalPose = Math.abs(finalPose.getTranslation().getX() - pose.getTranslation().getX());
                double yDifferenceBetweenFinalPose = Math.abs(finalPose.getTranslation().getY() - pose.getTranslation().getY());

                //This is the leniency for the final pose.
                //Should've been declared somewhere else, but it's here since I was low on time.
                final double poseLeniency = Units.inchesToMeters(1);

                //If the expected position is within the leniency, move to the next path.
                if (xDifferenceBetweenFinalPose < poseLeniency && yDifferenceBetweenFinalPose < poseLeniency) {
                    PathPlanner.getSelectedAuto().next();
                }
            }

            //Run the fixed execute method.
            PathPlanner.getSelectedAuto().fixedExecute();
        }
    }

    @Override
    public void onAutonomousPeriodic() {

    }

    //TELEOPERATED METHODS

    @Override
    public void onTeleopEnable() {
        //Tell robot container that the phase changed.
        m_robotContainer.onPhaseChange();
    }

    @Override
    public void onTeleopPeriodic() {
    }

    // DISABLED METHODS

    @Override
    public void onDisabled() {        
    }

    @Override
    public void onDisabledPeriodic() {        
    }

    // TESTING METHODS

    @Override
    public void onTestEnable() {
        //m_robotContainer.getDriveSubsystem().setBrakeMode(BrakeMode.COAST);
    }

    @Override
    public void onTestPeriodic() {        
    }
}
