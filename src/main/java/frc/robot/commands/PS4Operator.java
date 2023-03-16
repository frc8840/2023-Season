package frc.robot.commands;

import java.util.Arrays;
import java.util.Timer;
import java.util.TimerTask;

import org.photonvision.EstimatedRobotPose;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.arm.ArmSubsystem;
import frc.robot.subsystems.arm.ArmSubsystem.ArmState;
import frc.robot.subsystems.intake.GrabberSubsystem;
import frc.robot.subsystems.vision.VisionSubsystem;
import frc.robot.utils.ControllerConstants;
import frc.robot.utils.Measurements;
import frc.team_8840_lib.info.console.Logger;
import frc.team_8840_lib.input.communication.CommunicationManager;
import frc.team_8840_lib.listeners.Robot;
import frc.team_8840_lib.utils.math.MathUtils;
import frc.team_8840_lib.utils.math.units.RectangleBounds;

public class PS4Operator extends CommandBase {
    public static final boolean runAutoAlign = false;
    
    public enum OperateState {
        AUTO_ALIGN,
        NONE;
    }

    public enum ArmOperationMode {
        OPEN_LOOP_MANUAL,
        CLOSED_LOOP_MANUAL,
        PRESET_POSITIONS;
    }

    public enum ArmLocation {
        IN_POSITION,
        RESTING;
    }

    /**
     * -- Controls: --
     * 
     * Square: Cube Intake
     * Circle: (Arm Operate, Manual Closed Loop): Move arm to position 
     * Triangle: Auto Align
     * Cross: Cone Intake
     * 
     * Arrows: 
     * 
     * L1: Select Position
     * L2: Change Side (display + operations)
     * R1: Move arm to pre-set position.
     * R2: STOP ARM
     * 
     * L3:
     * R3:
     */
    private PS4Controller controller;

    private Trigger coneTrigger;
    private Trigger cubeTrigger;

    private Trigger switchSideTrigger;
    private Trigger selectTrigger;

    private int rowHover = 0;
    private int columnHover = 0;

    private int selectedRow = 0;
    private int selectedColumn = 0;

    private boolean justMoved = false;

    private Translation2d startAutoAlignPosition = null;

    private GrabberSubsystem grabberSubsystem;
    private ArmSubsystem armSubsystem;

    private OperateState state = OperateState.NONE;

    private ArmOperationMode armOperationMode = ArmOperationMode.OPEN_LOOP_MANUAL;
    private ArmLocation armLocation = ArmLocation.RESTING;

    private String side = "blue";

    private double baseAngle = 0;

    public PS4Operator(GrabberSubsystem grabberSubsystem, ArmSubsystem armSubsystem) {
        controller = new PS4Controller(ControllerConstants.OPERATOR_PS4_CONTROLLER);

        this.grabberSubsystem = grabberSubsystem;
        this.armSubsystem = armSubsystem;

        coneTrigger = new Trigger(controller::getCrossButton).onTrue(
            Commands.runOnce(() -> {
                if (grabberSubsystem.getLoadedPiece() == GrabberSubsystem.LoadedPiece.NONE) {
                    grabberSubsystem.intake(GrabberSubsystem.LoadedPiece.CONE);
                } else {
                    grabberSubsystem.outtake();
                }

                grabberSubsystem.updateDashboard();
            })
        ).onFalse(
            Commands.runOnce(() -> {
                if (grabberSubsystem.getState() == GrabberSubsystem.GrabberState.ACTIVE) {
                    grabberSubsystem.stopPickOrPlace();
                }

                grabberSubsystem.updateDashboard();
            })
        );

        cubeTrigger = new Trigger(controller::getSquareButton).onTrue(
            Commands.runOnce(() -> {
                if (grabberSubsystem.getLoadedPiece() == GrabberSubsystem.LoadedPiece.NONE) {
                    grabberSubsystem.intake(GrabberSubsystem.LoadedPiece.CUBE);
                } else {
                    grabberSubsystem.outtake();
                }

                grabberSubsystem.updateDashboard();
            })
        ).onFalse(
            Commands.runOnce(() -> {
                if (grabberSubsystem.getState() == GrabberSubsystem.GrabberState.ACTIVE) {
                    grabberSubsystem.stopPickOrPlace();
                }

                grabberSubsystem.updateDashboard();
            })
        );

        switchSideTrigger = new Trigger(controller::getL2Button).onTrue(
            Commands.runOnce(() -> {
                changeSide();
            })
        );

        selectTrigger = new Trigger(controller::getL1Button).onTrue(
            Commands.runOnce(() -> {
                selectedRow = rowHover;
                selectedColumn = columnHover;
                updateSelected();
            })
        );

        new Trigger(controller::getTriangleButton).onTrue(
            Commands.runOnce(() -> {
                if (!runAutoAlign) return;

                if (this.state == OperateState.AUTO_ALIGN) {
                    interruptedAutoAlign = true;
                    DriveSubsystem drive = RobotContainer.getInstance().getDriveSubsystem();
                    drive.getSwerveDrive().stop();
                    drive.setInAutoDrive(false);
                    state = OperateState.NONE;
                    return;
                }
                
                boolean success = calculateAndRunAutoAlign();
                if (success) {
                    Logger.Log("[AutoAlign]: Running auto align!");
                    state = OperateState.AUTO_ALIGN;
                } else {
                    Logger.Log("[AutoAlign]: Failed to run auto align!");
                }
            })
        );

        addRequirements(grabberSubsystem, armSubsystem);
    }

    @Override
    public void initialize() {
        updateSelected();

        CommunicationManager.getInstance().updateInfo(
            "custom_component", "Placing Display/side", "blue" //TODO: change depending on alliance
        );
    }

    public void updateSelected() {
        CommunicationManager.getInstance().updateInfo(
            "custom_component", "Placing Display/hover", (columnHover * 3) + rowHover
        );

        CommunicationManager.getInstance().updateInfo(
            "custom_component", "Placing Display/selected", (selectedColumn * 3) + selectedRow
        );
    }

    private boolean interruptedAutoAlign = false;
    
    /*
     * Unit standard: inches.
     */
    public boolean calculateAndRunAutoAlign() {
        RectangleBounds[] positions = Measurements.Grid.Rows.getBlueDimensionsList();

        int index = (selectedColumn * 3) + selectedRow;

        boolean isLeftSide = side == "blue";

        double x = isLeftSide ? positions[index].getCenterX() : Measurements.Field.WIDTH - positions[index].getCenterX();
        double y = (Measurements.Field.HEIGHT - Measurements.Grid.DEPTH) + positions[index].getCenterY();

        if (Robot.isReal()) {
            if (VisionSubsystem.getInstance().hasTarget()) {
                EstimatedRobotPose estPose = VisionSubsystem.getInstance().getEstimatedRobotPose();
                
                startAutoAlignPosition = new Translation2d(
                    Units.metersToInches(estPose.estimatedPose.getX()),
                    Units.metersToInches(estPose.estimatedPose.getY())
                );
            } else {
                Logger.Log("[AutoAlign] No target found! Cannot calculate distance from target!");
                return false;
            }
        } else {
            //Get closest 
            Pose2d rawRobotPose = RobotContainer.getInstance().getDriveSubsystem().getSwerveDrive().getPose();

            //Adjust to canvas coordinates due to y being flipped
            Pose2d robotPose = new Pose2d(
                new Translation2d(
                    rawRobotPose.getX(), 
                    Units.inchesToMeters(Measurements.Field.HEIGHT) - rawRobotPose.getY()
                ),
                rawRobotPose.getRotation()
            );

            //Find the closest measurement to the robot
            int closestIndex = -1;
            double closestDistance = Double.MAX_VALUE;

            int i = 0;
            for (RectangleBounds rb : positions) {
                double distance = MathUtils.distance(
                    Units.metersToInches(robotPose.getX()),
                    Units.metersToInches(robotPose.getY()),
                    Measurements.Field.WIDTH - rb.getCenterX(),
                    rb.getCenterY()
                );

                if (distance < closestDistance) {
                    closestDistance = distance;
                    closestIndex = i;
                }

                i++;
            }

            if (closestIndex == -1) return false;

            startAutoAlignPosition = new Translation2d(
                Units.metersToInches(robotPose.getX()),
                Units.metersToInches(robotPose.getY())
            );

            SmartDashboard.putNumber("close", closestIndex);
        }

        double yDistance = Math.abs(startAutoAlignPosition.getY() - y);
        double xDistance = Math.abs(
            startAutoAlignPosition.getX() - 
            (isLeftSide ? 
                Measurements.Grid.DEPTH + Units.metersToInches(0.5) : 
                Measurements.Field.WIDTH - Measurements.Grid.DEPTH - Units.metersToInches(0.5)
            )
        );

        if (xDistance > 12 && !Robot.isSimulation()) { //more than a foot away
            Logger.Log("[AutoAlign] Too far from the grid, cannot auto align! (XDistance: " + xDistance + "ft)");
            return false;
        }

        boolean goingDown = startAutoAlignPosition.getY() < y;

        final double maxAutoAlignSpeed = 3; //m/s

        double timeToReach = Units.inchesToMeters(yDistance) / maxAutoAlignSpeed;

        DriveSubsystem drive = RobotContainer.getInstance().getDriveSubsystem();

        drive.setInAutoDrive(true);

        drive.getSwerveDrive().drive(
            new Translation2d(
                0,
                (isLeftSide ? 1 : -1) * (goingDown ? -maxAutoAlignSpeed : maxAutoAlignSpeed)
            ),
            0,
            true,
            Robot.isReal()
        );

        Logger.Log("[AutoAlign] Starting auto align: moving " + (goingDown ? "down" : "up") + " at 3m/s for " + timeToReach + " seconds.");

        TimerTask task = new TimerTask() {
            @Override
            public void run() {
                if (interruptedAutoAlign) {
                    interruptedAutoAlign = false;
                    return;
                }
                RobotContainer.getInstance().getDriveSubsystem().setInAutoDrive(false);
                RobotContainer.getInstance().getDriveSubsystem().getSwerveDrive().stop();
                
                state = OperateState.NONE;

                Logger.Log("[AutoAlign] Finished auto align!");
            }
        };

        Timer timer = new Timer();
        timer.schedule(task, (long) (timeToReach * 1000));
    
        return true;
    }

    public void changeSide() {
        side = side == "blue" ? "red" : "blue";
        CommunicationManager.getInstance().updateInfo(
            "custom_component", "Placing Display/side", side //TODO: change depending on alliance
        );
    }

    @Override
    public void execute() {
        //Auto Align Chooser
        if (!justMoved && controller.getPOV() >= 0) {
            justMoved = true;

            if (controller.getPOV() == 270) {
                rowHover += (side == "blue" ? -1 : 1);
            } else if (controller.getPOV() == 90) {
                rowHover += (side == "blue" ? 1 : -1);
            } else if (controller.getPOV() == 0) {
                columnHover -= 1;
            } else if (controller.getPOV() == 180) {
                columnHover += 1;
            }

            if (columnHover < 0) {
                columnHover = 0;
            } else if (columnHover > 8) {
                columnHover = 8;
            }

            if (rowHover < 0) {
                rowHover = 0;
            } else if (rowHover > 2) {
                rowHover = 2;
            }

            updateSelected();
        } else if (controller.getPOV() < 0) {
            justMoved = false;
        }

        if (controller.getR2Button()) {
            armSubsystem.baseOpenLoop(0);
            armSubsystem.elbowOpenLoop(0);
            armSubsystem.stop();
            return;
        }
        
        if (this.state == OperateState.AUTO_ALIGN) {
            if (controller.getCircleButtonPressed()) {
                
            }
        } else {
            if (this.armOperationMode == ArmOperationMode.CLOSED_LOOP_MANUAL) {
                baseAngle += Math.abs(controller.getLeftY()) > 0.1 ? controller.getLeftY() * 0.0001 : 0;
                
                if (baseAngle > 90) {
                    baseAngle = 90;
                } else if (baseAngle < 0) {
                    baseAngle = 0;
                }
                
                SmartDashboard.putNumber("choose_angle", baseAngle);

                if (controller.getCircleButtonPressed() && baseAngle > 0 && baseAngle < 90) {
                    SmartDashboard.putNumber("arm_angle", baseAngle);
                    armSubsystem.setBasePosition(Rotation2d.fromDegrees(baseAngle));
                }
            } else if (this.armOperationMode == ArmOperationMode.OPEN_LOOP_MANUAL) {
                if (Math.abs(controller.getLeftY()) > 0.1) {
                    armSubsystem.baseOpenLoop(-controller.getLeftY() * 0.6);
                } else {
                    armSubsystem.baseOpenLoop(0);
                }

                if (Math.abs(controller.getRightY()) > 0.1) {
                    armSubsystem.elbowOpenLoop(controller.getRightY() * 0.6);
                } else {
                    armSubsystem.elbowOpenLoop(0);
                }
            } else if (this.armOperationMode == ArmOperationMode.PRESET_POSITIONS) {
                final int[] poleGrids = new int[] {
                    0, 2, 3, 5, 6, 8
                };

                if (controller.getR1ButtonPressed()) {
                    String operationName = "none";

                    if (armLocation == ArmLocation.RESTING) {
                        if (Arrays.binarySearch(poleGrids, selectedRow) >= 0) {
                            switch (selectedColumn) {
                                case 0:
                                    armSubsystem.setPosition(ArmState.PLACING_UPPER_CONE);
                                    operationName = "upper_cone";
                                    break;
                                case 1:
                                    armSubsystem.setPosition(ArmState.PLACING_MIDDLE_CONE);
                                    operationName = "middle_cone";
                                    break;
                                default:
                                    armSubsystem.setPosition(ArmState.PLACING_HYBRID);
                                    operationName = "hybrid";
                                    break;
                            }
                        } else {
                            switch (selectedColumn) {
                                case 0:
                                    armSubsystem.setPosition(ArmState.PLACING_UPPER_CUBE);
                                    operationName = "upper_cube";
                                    break;
                                case 1:
                                    armSubsystem.setPosition(ArmState.PLACING_MIDDLE_CUBE);
                                    operationName = "middle_cube";
                                    break;
                                default:
                                    armSubsystem.setPosition(ArmState.PLACING_HYBRID);
                                    operationName = "hybrid";
                                    break;
                            }
                        }

                        armLocation = ArmLocation.IN_POSITION;
                    } else if (armLocation == ArmLocation.IN_POSITION) {
                        armSubsystem.setPosition(ArmState.RESTING);
                        operationName = "resting";

                        armLocation = ArmLocation.RESTING;
                    }

                    Logger.Log("[Arm] Moved arm into position of " + operationName);
                    CommunicationManager.getInstance().updateInfo("arm", "position_name", operationName);
                }
            }
        }
    }
}
