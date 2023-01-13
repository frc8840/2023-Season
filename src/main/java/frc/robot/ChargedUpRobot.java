package frc.robot;

import java.util.TimerTask;

import com.revrobotics.REVPhysicsSim;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.subsystems.BalanceSubsystem;
import frc.robot.utils.ConfigureSettings;
import frc.robot.utils.Ports;
import frc.team_8840_lib.controllers.SwerveGroup;
import frc.team_8840_lib.examples.SwerveDrive;
import frc.team_8840_lib.input.communication.CommunicationManager;
import frc.team_8840_lib.input.controls.GameController;
import frc.team_8840_lib.input.controls.GameController.Type;
import frc.team_8840_lib.listeners.EventListener;
import frc.team_8840_lib.listeners.Robot;
import frc.team_8840_lib.pathing.PathPlanner;
import frc.team_8840_lib.utils.GamePhase;
import frc.team_8840_lib.utils.controllers.Pigeon;
import frc.team_8840_lib.utils.controllers.swerve.SwerveSettings;

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

    private SwerveGroup swerveDrive;

    private RobotContainer robotContainer;

    //ROBOT INIT

    @Override
    public void robotInit() {
        //Generate the SwerveSettings object.
        SwerveSettings settings = ConfigureSettings.getSwerveDriveSettings();

        //Set the SwerveGroup object to the swerve drive.
        swerveDrive = new SwerveGroup("NEO Swerve Drive", settings,
                new int[] { //Drive IDs
                    Ports.TopLeft.DRIVE,
                    Ports.TopRight.DRIVE,
                    Ports.BottomLeft.DRIVE,
                    Ports.BottomRight.DRIVE
                }, 

                new int[] { //Turn/Steering IDs
                    Ports.TopLeft.TURN,
                    Ports.TopRight.TURN,
                    Ports.BottomLeft.TURN,
                    Ports.BottomRight.TURN
                }, 

                new int[] { //Encoder IDs
                    Ports.TopLeft.ENCODER, 
                    Ports.TopRight.ENCODER, 
                    Ports.BottomLeft.ENCODER, 
                    Ports.BottomRight.ENCODER
                },

                new Pigeon(Pigeon.Type.TWO, Ports.PIGEON_ID, false)
        );

        //Set a listener to wait for the autonomous path to be set through the dashboard.
        //When it's set, we'll pass it over to the path planner.
        CommunicationManager.getInstance().waitForAutonomousPath(points -> {
            if (points.length == 0) {
                //Ignore it. This can happen to request the autonomous to be set to nothing, but we can ignore it.
                return;
            }
            pathPlanner = new PathPlanner();
            pathPlanner.updateTimePoints(points);
        });

        //Subscribe the fixed autonomous method to the Autonomous phase, running at a fixed rate of 0.03125 seconds.
        Robot.getInstance().subscribeFixedPhase(new TimerTask() {
            @Override
            public void run() {
                onFixedAutonomous();
            }
        }, GamePhase.Autonomous);

        //Add the GameController to the GameControllerManager.
        if (Robot.isSimulation()) {
            GameController.expectController(-1, Type.Simulated);
        } else {
            GameController.expectController(0, Type.Joystick);
        }
    }

    //ROBOT PERIODIC

    @Override
    public void robotPeriodic() {
        //Run the REVPhysicsSim if we're in simulation mode.
        if (Robot.isSimulation()) {
            REVPhysicsSim.getInstance().run();
        }

        if (swerveDrive != null) {
            //Update about swerve info
            CommunicationManager.getInstance().updateSwerveInfo(swerveDrive);
            
            //If field exists
            if (CommunicationManager.getInstance().fieldExists()) {
                swerveDrive.updateOdometry(); //Update swerve odometry

                //Update the position of the field robot
                CommunicationManager.getInstance().updateFieldObjectPose("SwerveRobot", swerveDrive.getPose());

                //Update even more stuff
                swerveDrive.updateFieldRobot();
            }
        }
    }
    
    //AUTONOMOUS METHODS

    @Override
    public void onAutonomousEnable() {

    }

    public void onFixedAutonomous() {
        if (pathPlanner == null) {
            return;
        }
        
        if (!pathPlanner.finished()) {
            //Get the current pose
            Pose2d pose = pathPlanner.moveToNext();
            //Get the last pose
            Pose2d lastPose = pathPlanner.getLastPose();

            //Find the difference between the two poses
            double xDiff = (pose.getTranslation().getX() - lastPose.getTranslation().getX());
            double yDiff = (pose.getTranslation().getY() - lastPose.getTranslation().getY());

            //Create a new translation with the difference
            Translation2d translation = new Translation2d(xDiff / Robot.DELTA_TIME, yDiff / Robot.DELTA_TIME);

            //Use pose to calculate the swerve module states
            swerveDrive.drive(translation, pose.getRotation().getRadians(), true, false);
        }
    }

    @Override
    public void onAutonomousPeriodic() {

    }

    //TELEOPERATED METHODS

    @Override
    public void onTeleopEnable() {

    }

    @Override
    public void onTeleopPeriodic() {
        // TODO Auto-generated method stub
        
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
        // TODO Auto-generated method stub
        
    }

    @Override
    public void onTestPeriodic() {
        // TODO Auto-generated method stub
        
    }


    // Get methods

    public SwerveGroup getSwerveDrive() {
        return swerveDrive;
    }
}
