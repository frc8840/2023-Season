// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.displays.DisplayContainer;
import frc.robot.testing.EmptyLogger;
import frc.robot.testing.SingularModuleTesting;
import frc.team_8840_lib.info.console.FileWriter;
import frc.team_8840_lib.info.console.Logger;
import frc.team_8840_lib.listeners.Robot;

/**
 * Do NOT add any static variables to this class, or any initialization at all. Unless you know what
 * you are doing, do not modify this file except to change the parameter class to the startRobot
 * call.
 */
public final class Main {
  private Main() {}

  //set this to false in competition, and remove entirely for production
  private static final boolean testing = false;

  /**
   * Main initialization function. Do not perform any initialization here.
   *
   * <p>If you change your main robot class, change the parameter type.
   */
    public static void main(String... args) {
        DisplayContainer.init();

        if (!testing) {
            //Assign the listener to ChargedUpRobot
            Robot.assignListener(new ChargedUpRobot());

            String os = System.getProperty("os.name");
            System.out.println("[MAIN] Using OS " + os + ".");
            if (os.startsWith("Mac OS X")) {
                //Logger will write to the default folder path on the roboRIO (~/8840applogs)
                Logger.setWriter(new FileWriter());
                System.out.println("[MAIN] Initialized with FileWriter.");
            } else {
                //Don't log if not on the test machine.
                Logger.setWriter(new EmptyLogger());
                System.out.println("[MAIN] Initialized with EmptyLogger.");
            }

            //Start the robot
            RobotBase.startRobot(Robot::new);
        } else {
            //RobotBase.startRobot(ChargedUpRobotTesting::new);

            Robot.assignListener(new SingularModuleTesting());

            //Logger will write to the default folder path on the roboRIO (~/8840applogs)
            Logger.setWriter(new EmptyLogger());

            //Start the robot
            RobotBase.startRobot(Robot::new);
        }
    }
}
