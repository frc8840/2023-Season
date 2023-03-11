package frc.robot;

import java.util.TimerTask;

import com.revrobotics.REVPhysicsSim;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.utils.Measurements;
import frc.team_8840_lib.info.console.Logger;
import frc.team_8840_lib.input.communication.CommunicationManager;
import frc.team_8840_lib.listeners.EventListener;
import frc.team_8840_lib.listeners.Robot;
import frc.team_8840_lib.pathing.PathPlanner;
import frc.team_8840_lib.pathing.PathConjugate.ConjugateType;
import frc.team_8840_lib.utils.GamePhase;

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
        m_robotContainer = new RobotContainer();

        //We'll initialize our autonomous here
        try {
            AutonomousContainer.init();
        } catch (Exception e) {
            Logger.Log("[Auto] Failed to initialize autonomous!");
            e.printStackTrace();
        }

        CommunicationManager.getInstance().createField();

        Robot.getInstance().subscribeFixedPhase(new TimerTask() {
            @Override
            public void run() {
                onFixedAutonomous();
            }
        }, GamePhase.Autonomous);

        //Wait till the swerve drive is ready to be used
        Robot.getRealInstance()
            .waitForFullfillConditions(5000, () -> m_robotContainer.getDriveSubsystem().ready())
            .onFinishFullfillment(() -> {
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

        CommunicationManager.getInstance().updateSwerveInfo(RobotContainer.getInstance().getDriveSubsystem().getSwerveDrive());
        if (CommunicationManager.getInstance().fieldExists()) {
            RobotContainer.getInstance().getDriveSubsystem().getSwerveDrive().updateOdometry();

            CommunicationManager.getInstance().updateFieldObjectPose("SwerveRobot", RobotContainer.getInstance().getDriveSubsystem().getSwerveDrive().getPose());

            RobotContainer.getInstance().getDriveSubsystem().getSwerveDrive().updateFieldRobot();
        }

        CommandScheduler.getInstance().run();
    }
    
    //AUTONOMOUS METHODS

    @Override
    public void onAutonomousEnable() {
        m_robotContainer.onPhaseChange();

        if (!PathPlanner.selectedAuto()) {
            Logger.Log("[Auto] No logger selected! Going with the suggested auto based on the field layout.");
        } else {
            PathPlanner.getSelectedAuto().start();
        }
    }

    public void onFixedAutonomous() {
        if (!PathPlanner.getSelectedAuto().finished()) {
            if (PathPlanner.getSelectedAuto().getCurrentType() == ConjugateType.Path) {
                //TODO: create a method in the drive subsystem that takes in the pose and the last pose to drive.
                Pose2d pose = PathPlanner.getSelectedAuto().current().getPath().moveToNext();

                pose = new Pose2d( 
                    Units.inchesToMeters(Measurements.Field.WIDTH) - pose.getX(), 
                    pose.getY(), pose.getRotation()
                );
                // Pose2d lastPose = PathPlanner.getSelectedAuto().current().getPath().getLastPose();

                // //Find Difference between the two poses
                // double xDiff = (pose.getTranslation().getX() - lastPose.getTranslation().getX());
                // double yDiff = (pose.getTranslation().getY() - lastPose.getTranslation().getY());

                // //Create a new translation with the difference
                // Translation2d translation = new Translation2d(xDiff / Robot.DELTA_TIME, yDiff / Robot.DELTA_TIME);

                // //Use pose to calculate the swerve module states
                // swerveDrive.drive(translation, pose.getRotation().getRadians(), true, false);

                CommunicationManager.getInstance().updateRobotPose(pose);
            }
            PathPlanner.getSelectedAuto().fixedExecute();
        }
    }

    @Override
    public void onAutonomousPeriodic() {

    }

    //TELEOPERATED METHODS

    @Override
    public void onTeleopEnable() {
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
