package frc.robot;

import com.revrobotics.REVPhysicsSim;

import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.DriveSubsystem.BrakeMode;
import frc.team_8840_lib.input.communication.CommunicationManager;
import frc.team_8840_lib.input.controls.SimulatedController;
import frc.team_8840_lib.listeners.EventListener;
import frc.team_8840_lib.listeners.Robot;
import frc.team_8840_lib.pathing.PathPlanner;

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

    private PathPlanner pathPlanner;

    private RobotContainer m_robotContainer;

    //ROBOT INIT
    @Override
    public void robotInit() {
        m_robotContainer = new RobotContainer();

        CommunicationManager.getInstance().createField();

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

        CommandScheduler.getInstance().run();
    }
    
    //AUTONOMOUS METHODS

    @Override
    public void onAutonomousEnable() {
        m_robotContainer.onPhaseChange();
    }

    public void onFixedAutonomous() {
        // if (pathPlanner == null) {
        //     return;
        // }
        
        // if (!pathPlanner.finished()) {
        //     //Get the current pose
        //     Pose2d pose = pathPlanner.moveToNext();
        //     //Get the last pose
        //     Pose2d lastPose = pathPlanner.getLastPose();

        //     //Find the difference between the two poses
        //     double xDiff = (pose.getTranslation().getX() - lastPose.getTranslation().getX());
        //     double yDiff = (pose.getTranslation().getY() - lastPose.getTranslation().getY());

        //     //Create a new translation with the difference
        //     Translation2d translation = new Translation2d(xDiff / Robot.DELTA_TIME, yDiff / Robot.DELTA_TIME);

        //     //Use pose to calculate the swerve module states
        //     swerveDrive.drive(translation, pose.getRotation().getRadians(), true, false);
        // }
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
        // TODO Auto-generated method stub
        
    }

    @Override
    public void onDisabledPeriodic() {
        // TODO Auto-generated method stub
        
    }

    // TESTING METHODS

    @Override
    public void onTestEnable() {
        m_robotContainer.getDriveSubsystem().setBrakeMode(BrakeMode.COAST);
    }

    @Override
    public void onTestPeriodic() {
        // TODO Auto-generated method stub
        
    }
}
