package frc.robot.commands;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.RobotContainer;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.SwerveDriveSubsystem;

public class Drive extends Command {
        private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
        private Distance DriveCircumference = Meters
                        .of(new Translation2d(TunerConstants.kFrontLeftXPos, TunerConstants.kFrontLeftYPos).getNorm())
                        .times(2 * Math.PI);
        private double MaxAngularRate = RotationsPerSecond
                        .of(TunerConstants.kSpeedAt12Volts.times(Seconds.of(1)).div(DriveCircumference).magnitude())
                        .in(RadiansPerSecond); // 3/4 of a rotation per second

        private SwerveRequest idle = new SwerveRequest.Idle();

        /* Setting up bindings for necessary control of the swerve drive platform */
        private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
                        .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors

        final static double DEADBAND = 0.20;

        private final SwerveDriveSubsystem swerveDrive;
        private final CommandXboxController controller1;

        public Drive(SwerveDriveSubsystem swerveDrive) {
                this.swerveDrive = swerveDrive;
                controller1 = RobotContainer.getController1();
                addRequirements(swerveDrive);
        }

        static double deadband(double in) {
                return Math.abs(in) < DEADBAND ? 0 : (Math.abs(in) - DEADBAND) * Math.signum(in);
        }

        static double sigSqr(double in) {
                return Math.pow(in, 2) * Math.signum(in);
        }

        static double shapeInput(double in) {
                return Math.min(sigSqr(deadband(in)), 1);
        }

        @Override
        public void execute() {
                if (Math.pow(controller1.getLeftY(), 2) + Math.pow(controller1.getLeftX(), 2)
                                + Math.pow(controller1.getRightX(), 2) < 0.05) {
                        swerveDrive.setControl(idle);
                } else {
                        swerveDrive.setControl(drive
                                        .withVelocityX(shapeInput(-controller1.getLeftY()) * MaxSpeed) // Drive forward with negative Y (forward)
                                        .withVelocityY(shapeInput(-controller1.getLeftX()) * MaxSpeed) // Drive left with negative X (left)
                                        .withRotationalRate(shapeInput(-controller1.getRightX())
                                                        * MaxAngularRate)); // Drive counterclockwise with negative X (left)
                }
        }

        @Override
        public void end(boolean interrupted) {
                swerveDrive.setControl(idle);
        }
}
