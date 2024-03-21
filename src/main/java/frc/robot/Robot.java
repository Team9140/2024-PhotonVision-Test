// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Limelight;

import java.util.List;

public class Robot extends TimedRobot {
    private Arm arm;
    private Drivetrain drive;
    private Intake intake;

    private Limelight vision;

    CommandXboxController xb = new CommandXboxController(Constants.Ports.CONTROLLER);
     Joystick joyleft = new Joystick(Constants.Ports.JOYLEFT);
     Joystick joyright = new Joystick(Constants.Ports.JOYRIGHT);

    SendableChooser<Integer> autoChooser = new SendableChooser<>();

    public Robot() {
        super(Constants.Units.SECONDS_PER_LOOP);
    }

    @Override
    public void robotInit() {
        Constants.UpdateSettings();

        this.arm = Arm.getInstance();
        this.drive = Drivetrain.getInstance();
        this.intake = Intake.getInstance();
        this.vision = Limelight.getInstance();

        xb.rightBumper().onTrue(this.intake.intakeCone()).onFalse(this.intake.holdCone());
        xb.leftBumper().onTrue(this.intake.intakeCube()).onFalse(this.intake.holdCube());
        xb.rightTrigger().onTrue(this.intake.throwItem()).onFalse(this.intake.off());

        xb.y().onTrue(this.arm.setHighNode());
        xb.a().onTrue(this.arm.setFloor());
        xb.x().onTrue(this.arm.setStow());
        xb.b().onTrue(this.arm.setMidNode());

        // xb.start().onTrue(this.arm.setRadiansAndFinish(0.0).andThen(this.arm.setRadiansAndFinish(1.0)));

        drive.setDefaultCommand(Commands.run(() -> {
            drive.curvatureDrive(-xb.getHID().getLeftY(), -xb.getHID().getRightX(), xb.getHID().getLeftTriggerAxis() > 0.5);
        }, drive));

        this.addAutoChoice("Sequence", "High Cube & Balance", 4, true);
        this.addAutoChoice("Sequence", "High Cube & Drive & Balance", 6);
        this.addAutoChoice("Score", "High Cube", 1);
        this.addAutoChoice("Score", "High Cube no move", 7);
        this.addAutoChoice("Score", "Mid Cube", 0);
        this.addAutoChoice("Balance", "Forward", 5);
        this.addAutoChoice("Balance", "Backward", 2);
        this.addAutoChoice("Test", "Test", 3);

        SmartDashboard.putData(this.autoChooser);
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();

        SmartDashboard.putNumber("Time" /* with a capital T */, Timer.getFPGATimestamp());

        this.drive.updateSlew();
    }

    @Override
    public void autonomousInit() {
        Constants.UpdateSettings();
        CommandScheduler.getInstance().cancelAll();
        switch (this.autoChooser.getSelected()) {
            case 0:
                // midCubeDriveAuto
                new SequentialCommandGroup(
                        this.intake.holdCube().raceWith(new WaitCommand(0.1)),
                        this.arm.setRadians(Constants.Arm.Positions.MID_NODE),
                        new WaitCommand(2.0),
                        this.intake.throwItem().raceWith(new WaitCommand(1.0)),
                        this.arm.setRadians(Constants.Arm.Positions.STOW),
                        new WaitCommand(2.0),
                        this.intake.off().raceWith(new WaitCommand(0.1)),
                        this.drive.crawlDistance(Constants.Auto.DRIVE_BACK_METERS, -Constants.Auto.DRIVE_MPS)).schedule();
                break;

            case 1:
                // highCubeDriveAuto
                new SequentialCommandGroup(
                        this.arm.setRadians(Constants.Arm.Positions.STOW),
                        this.intake.off(),
                        this.drive.resetEncodersCommand(),

                        this.drive.crawlDistance(0.4, Constants.Auto.DRIVE_MPS / 4.0).alongWith(
                                        this.intake.holdCube().raceWith(this.arm.setRadiansAndFinish(Constants.Arm.Positions.HIGH_NODE)))
                                .withTimeout(100),

                        this.intake.throwItem().withTimeout(0.25),
                        this.drive.resetEncodersCommand(),
                        this.arm.setRadians(Constants.Arm.Positions.STOW),
                        new WaitCommand(0.9),
                        // this.arm.setRadiansAndFinish(Constants.Arm.Positions.STOW).withTimeout(2.0),
                        this.intake.off(),
                        // new WaitCommand(0.5),

                        this.drive.crawlDistance(Constants.Auto.DRIVE_BACK_METERS + 0.0, -Constants.Auto.DRIVE_MPS)).schedule();
                break;

            case 2:
                // backwardBalanceAuto
                new SequentialCommandGroup(
                        this.drive.crawlUntilTilt(Constants.Auto.B_BALANCE_DRIVE_MPS),
                        this.drive.crawlUntilLevel(Constants.Auto.B_LEVEL_MPS)).schedule();
                ;
                break;

            case 3:
                // trajectoryAuto
                new SequentialCommandGroup(
                        this.drive.crawlUntilOverBackwards(-Constants.Auto.DRIVE_MPS)).schedule();
                break;

            case 4:
                // highCubeBalanceAuto
                new SequentialCommandGroup(
                        this.arm.setRadians(Constants.Arm.Positions.STOW),
                        this.intake.off(),
                        this.drive.resetEncodersCommand(),
                        this.drive.crawlDistance(0.4, Constants.Auto.DRIVE_MPS / 4.0).withTimeout(2.0),
                        this.intake.holdCube().raceWith(this.arm.setRadiansAndFinish(Constants.Arm.Positions.HIGH_NODE)),
                        this.intake.throwItem().withTimeout(1.0),
                        this.arm.setRadiansAndFinish(Constants.Arm.Positions.STOW).withTimeout(2.0),
                        this.intake.off(),
                        new WaitCommand(0.5),
                        this.drive.resetEncodersCommand(),
                        this.drive.crawlUntilTilt(Constants.Auto.B_BALANCE_DRIVE_MPS),
                        this.drive.crawlUntilLevel(Constants.Auto.B_LEVEL_MPS)).schedule();
                break;

            case 5:
                // forwardBalanceAuto
                new SequentialCommandGroup(
                        this.drive.crawlUntilTilt(Constants.Auto.F_BALANCE_DRIVE_MPS),
                        this.drive.crawlUntilLevel(Constants.Auto.F_LEVEL_MPS)).schedule();
                break;

            case 6:
                new SequentialCommandGroup(
                        this.arm.setRadians(Constants.Arm.Positions.STOW),
                        this.intake.off(),
                        this.drive.resetEncodersCommand(),

                        this.drive.crawlDistance(0.4, Constants.Auto.DRIVE_MPS / 4.0).alongWith(
                                        this.intake.holdCube().raceWith(this.arm.setRadiansAndFinish(Constants.Arm.Positions.HIGH_NODE)))
                                .withTimeout(100),

                        this.intake.throwItem().withTimeout(0.25),
                        this.drive.resetEncodersCommand(),
                        this.arm.setRadians(Constants.Arm.Positions.STOW),
                        new WaitCommand(0.9),
                        // this.arm.setRadiansAndFinish(Constants.Arm.Positions.STOW).withTimeout(2.0),
                        this.intake.off(),
                        // new WaitCommand(0.5),

                        this.drive.crawlDistance(Constants.Auto.DRIVE_BACK_METERS + 0.45, -Constants.Auto.DRIVE_MPS),
                        // new WaitCommand(0.25),
                        this.drive.crawlUntilTilt(Constants.Auto.F_BALANCE_DRIVE_MPS),
                        this.drive.crawlUntilLevel(Constants.Auto.F_LEVEL_MPS)).schedule();
                break;
            case 7:
                // highCubeStayAuto
                new SequentialCommandGroup(
                        this.arm.setRadians(Constants.Arm.Positions.STOW),
                        this.intake.off(),
                        this.drive.resetEncodersCommand(),

                        this.drive.crawlDistance(0.4, Constants.Auto.DRIVE_MPS / 4.0).alongWith(
                                        this.intake.holdCube().raceWith(this.arm.setRadiansAndFinish(Constants.Arm.Positions.HIGH_NODE)))
                                .withTimeout(100),

                        this.intake.throwItem().withTimeout(0.25),
                        this.drive.resetEncodersCommand(),
                        this.arm.setRadians(Constants.Arm.Positions.STOW),
                        new WaitCommand(0.9),
                        // this.arm.setRadiansAndFinish(Constants.Arm.Positions.STOW).withTimeout(2.0),
                        this.intake.off()
                        // new WaitCommand(0.5),
                ).schedule();
                break;
            case 99:
                break;
        }
    }

    @Override
    public void teleopInit() {
        Constants.UpdateSettings();
    }

    private void addAutoChoice(String l, String t, int i, boolean d) {
        if (d) {
            this.autoChooser.setDefaultOption("[Auto - " + l + "] " + t + " (Default)", Integer.valueOf(i));
        } else {
            this.autoChooser.addOption("[Auto - " + l + "] " + t, Integer.valueOf(i));
        }
    }

    private void addAutoChoice(String l, String t, int i) {
        this.addAutoChoice(l, t, i, false);
    }
}