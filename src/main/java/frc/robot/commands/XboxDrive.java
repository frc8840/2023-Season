package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.DriveSubsystem.BrakeMode;
import frc.robot.utils.ControllerConstants;
import frc.team_8840_lib.info.console.Logger;
import frc.team_8840_lib.input.controls.SimulatedController;
import frc.team_8840_lib.listeners.Robot;
import frc.team_8840_lib.utils.GamePhase;
import frc.team_8840_lib.utils.controls.Axis;

public class XboxDrive extends CommandBase {
    public static enum DriveMode {
        NORMAL,
        X_BRAKE,
        ZEROED,
        SPINNY_BOI,
        TESTING;
        public boolean normalOr(DriveMode driveMode) {
            return this == NORMAL || this == driveMode;
        }
    }

    private final static double maxSpeed = 10;
    private final static double slowModeSpeed = 3;

    /**
     * Buttons:
     * 
     * A: Spin
     * B: Zero out
     * X: Reset to Absolute
     * Y: Testing
     * 
     * Left Bumper: Slow Mode (while hold)
     * Right Bumper: X Mode (while hold)
     * 
     * Left Joystick Button:
     * Right Joystick Button: 
     * 
     */
    private XboxController controller;
    private SimulatedController simulatedController;

    private DriveSubsystem driveSubsystem;

    private DriveMode driveMode = DriveMode.NORMAL;
    private boolean inSlowMode = false;

    private Trigger xModeTrigger; //Binded to right bumper
    private Trigger zeroModeTrigger; //Binded to B button
    private Trigger spinnyBoiTrigger; //Binded to A button
    private Trigger testTrigger; //Binded to Y button
    private Trigger slowModeTrigger; //Binded to left bumper

    public XboxDrive(DriveSubsystem driveSubsystem) {
        //Setup Controllers
        if (Robot.isReal() || Robot.os() == Robot.OS.WINDOWS) this.controller = new XboxController(ControllerConstants.DRIVE_XBOX_CONTROLLER);
        this.simulatedController = new SimulatedController();

        //Setup Drive Subsystem
        this.driveSubsystem = driveSubsystem;
        addRequirements(driveSubsystem);

        if (Robot.isReal()) {
            driveSubsystem.getSwerveDrive().setGyroStartOffset(Rotation2d.fromDegrees(180));

            //Add Triggers to Controllers
            xModeTrigger = new Trigger(controller::getRightBumper).onTrue(
                Commands.runOnce(() -> {
                    if (!driveMode.normalOr(DriveMode.X_BRAKE)) return;
                    driveMode = driveMode == DriveMode.NORMAL ? DriveMode.X_BRAKE : DriveMode.NORMAL;
                    adjustBrakeModeBasedOnMode();
                })
            ).onFalse(
                Commands.runOnce(() -> {
                    if (!driveMode.normalOr(DriveMode.X_BRAKE)) return;
                    driveMode = driveMode == DriveMode.NORMAL ? DriveMode.X_BRAKE : DriveMode.NORMAL;
                    adjustBrakeModeBasedOnMode();

                    if (driveMode == DriveMode.NORMAL) {
                        RobotContainer.getInstance().getDriveSubsystem().getSwerveDrive().triggerNoOptimization();
                    }
                })
            );

            slowModeTrigger = new Trigger(controller::getLeftBumper).onTrue(
                Commands.runOnce(() -> {
                    inSlowMode = true;
                })
            ).onFalse(
                Commands.runOnce(() -> {
                    inSlowMode = false;
                })
            );

            zeroModeTrigger = new Trigger(controller::getBButton).onTrue(
                Commands.runOnce(() -> {
                    // if (!driveMode.normalOr(DriveMode.ZEROED)) return;
                    // driveMode = driveMode == DriveMode.NORMAL ? DriveMode.ZEROED : DriveMode.NORMAL;
                })
            );

            spinnyBoiTrigger = new Trigger(controller::getAButton).onTrue(
                Commands.runOnce(() -> {
                    // if (!driveMode.normalOr(DriveMode.SPINNY_BOI)) return;
                    // driveMode = driveMode == DriveMode.NORMAL ? DriveMode.SPINNY_BOI : DriveMode.NORMAL;
                })
            );

            testTrigger = new Trigger(controller::getYButton).onTrue(
                Commands.runOnce(() -> {
                    // if (!driveMode.normalOr(DriveMode.TESTING)) return;
                    // driveMode = driveMode == DriveMode.NORMAL ? DriveMode.TESTING : DriveMode.NORMAL;
                })
            );
        }
    }

    public double getForward() {
        return Robot.isSimulation() ? simulatedController.getAxis(Axis.Vertical) : -controller.getLeftY();
    }

    public double getStrafe() {
        return Robot.isSimulation() ? simulatedController.getAxis(Axis.Horizontal) : controller.getLeftX();
    }

    public double getRightX() {
        return Robot.isSimulation() ? 0 : controller.getRightX();
    }

    public double getRightY() {
        return Robot.isSimulation() ? 0 : controller.getRightY();
    }

    @Override
    public void initialize() {
        Logger.Log("[" + getName() + "] Initialized.");
    }

    private int count = 0;
    private double testrot = 0;

    @Override
    public void execute() {
        //If not in teleop, return and do nothing
        if (GamePhase.getCurrentPhase() != GamePhase.Teleop) return;
        
        //Send the drive mode to network tables
        SmartDashboard.putString("Drive Mode", driveMode.name());

        if (controller.getXButtonPressed()) {
            // driveSubsystem.getSwerveDrive().resetToAbsolute();
            // Logger.Log("[Swerve] RESET TO ABSOLUTE!");
            // return;
        }

        //Based on the drive mode we are in, do different things.
        if (driveMode == DriveMode.SPINNY_BOI) {
            driveSubsystem.getSwerveDrive().spin(controller.getRightX() * 18);
            return;
        } else if (driveMode == DriveMode.X_BRAKE) {
            driveSubsystem.getSwerveDrive().applyXBrake();
            driveSubsystem.getSwerveDrive().stop();
            return;
        } else if (driveMode == DriveMode.ZEROED) {
            driveSubsystem.getSwerveDrive().setAllModuleAngles(0);
            driveSubsystem.getSwerveDrive().stop();
            return;
        } else if (driveMode == DriveMode.TESTING) {
            if (controller.getLeftBumperPressed()) {
                count += 1;
                testrot = 0;
            }

            int index = count % 4;

            if (controller.getRightBumperPressed()) {
                testrot += 1;
            }

            driveSubsystem.getSwerveDrive().getModules()[index].setDesiredState(
                new SwerveModuleState(0.5, Rotation2d.fromDegrees(testrot))
            , true, true);
            
            controller.setRumble(RumbleType.kBothRumble, 0.75);
            return;
        }

        //If we are in auto drive, return and do nothing
        if (driveSubsystem.isInAutoDrive()) return;
        
        //If the threshold is not met, stop the robot
        if (Math.abs(getForward()) < 0.1 && Math.abs(getStrafe()) < 0.1) {
            if (Math.abs(getRightX()) < 0.1) {
                driveSubsystem.getSwerveDrive().stop();
            } else {
                //If the rotate threshold is met, rotate the robot
                driveSubsystem.getSwerveDrive().spin(getRightX() * 13);
            }
            return;
        }

        //Construct a Translation2d with the forward and strafe values
        Translation2d driveTranslation = new Translation2d(
            getForward(),
            getStrafe()
        );

        //Multiply the translation by the max speed or the slow mode speed based on whether or not we are in slow mode
        driveTranslation = driveTranslation.times(inSlowMode ? slowModeSpeed : maxSpeed);

        //Drive the robot
        driveSubsystem.getSwerveDrive().drive(driveTranslation, getRightX() * 13, true, Robot.isReal());
    }

    public void adjustBrakeModeBasedOnMode() {
        if (driveMode == DriveMode.X_BRAKE) {
            driveSubsystem.setBrakeMode(BrakeMode.BRAKE);
        } else {
            driveSubsystem.setBrakeMode(BrakeMode.NORMAL);
        }
    }

    @Override
    public void end(boolean interrupted) {
        Logger.Log("[" + getName() + "] Ended.");
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
