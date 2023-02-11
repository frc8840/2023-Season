package frc.robot;

import java.util.TimerTask;

import com.revrobotics.REVPhysicsSim;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.utils.ConfigureSettings;
import frc.robot.utils.ModuleConstants;
import frc.team_8840_lib.controllers.SwerveGroup;
import frc.team_8840_lib.input.communication.CommunicationManager;
import frc.team_8840_lib.input.controls.SimulatedController;
import frc.team_8840_lib.listeners.EventListener;
import frc.team_8840_lib.listeners.Robot;
import frc.team_8840_lib.pathing.PathPlanner;
import frc.team_8840_lib.utils.controllers.Pigeon;
import frc.team_8840_lib.utils.controllers.swerve.SwerveSettings;
import frc.team_8840_lib.utils.controls.Axis;
import frc.team_8840_lib.utils.math.MathUtils;

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

    private SimulatedController simulatedController;
    public Joystick joystick;

    //ROBOT INIT

    @Override
    public void robotInit() {
        //Generate the SwerveSettings object.
        SwerveSettings settings = ConfigureSettings.getSwerveDriveSettings();

        int[] drivePorts = new int[4];
        int[] turnPorts = new int[4];
        int[] encoderPorts = new int[4];

        drivePorts[SwerveGroup.ModuleIndex.kTOP_LEFT.getIndex()] = ModuleConstants.TopLeft.DRIVE;
        turnPorts[SwerveGroup.ModuleIndex.kTOP_LEFT.getIndex()] = ModuleConstants.TopLeft.TURN;
        encoderPorts[SwerveGroup.ModuleIndex.kTOP_LEFT.getIndex()] = ModuleConstants.TopLeft.ENCODER;

        drivePorts[SwerveGroup.ModuleIndex.kTOP_RIGHT.getIndex()] = ModuleConstants.TopRight.DRIVE;
        turnPorts[SwerveGroup.ModuleIndex.kTOP_RIGHT.getIndex()] = ModuleConstants.TopRight.TURN;
        encoderPorts[SwerveGroup.ModuleIndex.kTOP_RIGHT.getIndex()] = ModuleConstants.TopRight.ENCODER;

        drivePorts[SwerveGroup.ModuleIndex.kBOTTOM_LEFT.getIndex()] = ModuleConstants.BottomLeft.DRIVE;
        turnPorts[SwerveGroup.ModuleIndex.kBOTTOM_LEFT.getIndex()] = ModuleConstants.BottomLeft.TURN;
        encoderPorts[SwerveGroup.ModuleIndex.kBOTTOM_LEFT.getIndex()] = ModuleConstants.BottomLeft.ENCODER;

        drivePorts[SwerveGroup.ModuleIndex.kBOTTOM_RIGHT.getIndex()] = ModuleConstants.BottomRight.DRIVE;
        turnPorts[SwerveGroup.ModuleIndex.kBOTTOM_RIGHT.getIndex()] = ModuleConstants.BottomRight.TURN;
        encoderPorts[SwerveGroup.ModuleIndex.kBOTTOM_RIGHT.getIndex()] = ModuleConstants.BottomRight.ENCODER;

        //Set the SwerveGroup object to the swerve drive.
        swerveDrive = new SwerveGroup("NEO Swerve Drive", settings,
                drivePorts,
                turnPorts,
                encoderPorts,
                new Pigeon(Pigeon.Type.TWO, ModuleConstants.PIGEON_ID, false)
        );

        //Set a listener to wait for the autonomous path to be set through the dashboard.
        //When it's set, we'll pass it over to the path planner.
        // CommunicationManager.getInstance().waitForAutonomousPath(points -> {
        //     if (points.length == 0) {
        //         //Ignore it. This can happen to request the autonomous to be set to nothing, but we can ignore it.
        //         return;
        //     }
        //     pathPlanner = new PathPlanner();
        //     pathPlanner.updateTimePoints(points);
        // });

        //Subscribe the fixed autonomous method to the Autonomous phase, running at a fixed rate of 0.03125 seconds.
        // Robot.getInstance().subscribeFixedPhase(new TimerTask() {
        //     @Override
        //     public void run() {
        //         onFixedAutonomous();
        //     }
        // }, GamePhase.Autonomous);

        swerveDrive.providePower(true);
        swerveDrive.doStateOptimization(true);
        swerveDrive.doSetAngle(true);
        swerveDrive.doSetSpeed(true);

        simulatedController = new SimulatedController();
        joystick = new Joystick(0);

        CommunicationManager.getInstance().createField();

        //Wait till the swerve drive is ready to be used
        Robot.getRealInstance()
            .waitForFullfillConditions(5000, () -> swerveDrive.ready())
            .onFinishFullfillment(() -> {
                swerveDrive.resetOdometry(new Pose2d(7, 4, new Rotation2d(0)));
            });
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
        swerveDrive.setIndividualBrakeModes(true, false);
        swerveDrive.resetOdometry(new Pose2d(7, 4, new Rotation2d(0)));
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
        swerveDrive.setIndividualBrakeModes(true, false);
    }

    boolean inXFormation = false;
    boolean inLockMode = false;
    
    boolean inNoLock = false;

    boolean testSpeed = false;

    double facing = 0;
    double angularVelocity = 1;

    @Override
    public void onTeleopPeriodic() {
        // TODO Auto-generated method stub
        double x = Robot.isReal() ? joystick.getRawAxis(0) : simulatedController.getAxis(Axis.Horizontal);
        double y = Robot.isReal() ? -joystick.getRawAxis(1) : simulatedController.getAxis(Axis.Vertical);

        SmartDashboard.putNumber("X", x);
        SmartDashboard.putNumber("Y", y);

        double turn = joystick.getRawAxis(2);

        if (joystick.getRawButtonPressed(12)) {
            testSpeed = !testSpeed;
        }

        if (testSpeed) {
            swerveDrive.getModules()[2].setSpeed(3 / swerveDrive.getSettings().maxSpeed);
            return;
        }

        if (joystick.getRawButtonPressed(11)) {
            inNoLock = !inNoLock;

            if (inNoLock) {
                SmartDashboard.putString("No Lock", "On");
                swerveDrive.setIndividualBrakeModes(true, false);
            } else {
                SmartDashboard.putString("No Lock", "Off");
                swerveDrive.setIndividualBrakeModes(true, false);
            }
        }

        if (inNoLock) {
            return;
        }

        if (joystick.getRawButtonPressed(8)) {
            inLockMode = !inLockMode;

            if (inLockMode) {
                SmartDashboard.putString("to 0 degrees", "On");
            } else {
                SmartDashboard.putString("to 0 degrees", "Off");
            }
        }

        if (inLockMode) {
            //Set all modules to 0 degrees
            swerveDrive.setAllModuleAngles(0);
            swerveDrive.stop();
            return;
        }

        if (joystick.getRawButtonPressed(1)) {
            inXFormation = !inXFormation;
            
            if (inXFormation) {
                SmartDashboard.putString("X Formation", "On");
                swerveDrive.setIndividualBrakeModes(true, true);
            } else {
                swerveDrive.setIndividualBrakeModes(true, false);
                SmartDashboard.putString("X Formation", "Off");
            }
        }

        if (inXFormation) {
            swerveDrive.applyXBrake();
            swerveDrive.stop();
            return;
        }

        if (turn >= 0.01) {
            facing += angularVelocity * turn;
            SmartDashboard.putNumber("Facing", facing);
            SmartDashboard.updateValues();
        }

        if ((Math.abs(x) < 0.01 && Math.abs(y) < 0.01) && turn < 0.01) {
            swerveDrive.stop();
            return;
        }

        double facingRad = Math.toRadians(facing); 
        Translation2d movement = new Translation2d(0.1, 0.3).times(3);

        // CommunicationManager.getInstance().logSwerveStates(
        //     "Swerve Drive",
        //     "Swerve States",
        //     swerveDrive.driveStates(movement, 0, true, true)
        // );

        swerveDrive.drive(movement, 0, true, true);
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
        swerveDrive.setBrakeModes(false);
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
