package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.InchesPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import java.util.function.Supplier;

import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.SlotConfigs;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ElevatorSubsystem extends SubsystemBase {

    private final int LEADER_ID = 14;
    private final int FOLLOWER_ID = 15;

    private final TalonFX elevator = new TalonFX(LEADER_ID);
    private final TalonFX follower = new TalonFX(FOLLOWER_ID);

    private final Angle MAX_HEIGHT_ANGLE = Rotations.of(5); // TODO: fix max
    private final Distance MAX_HEIGHT_LINEAR = Inches.of(53.6); // Checked with Manny. Total travel of elevator

    private final State state = new State();

    @SuppressWarnings("unused")
    private final Telemetry telemetry = new Telemetry(this);

    private final boolean DISABLE_MOTION_MAGIC = true;

    public ElevatorSubsystem() {
        super();

        follower.setControl(new Follower(LEADER_ID, true));

        elevator.setPosition(0);

        var slotConfigs = new SlotConfigs();

        slotConfigs.kP = 0.25;
        slotConfigs.kI = 0;
        slotConfigs.kD = 0;

        slotConfigs.kG = 0;
        slotConfigs.kS = 0;
        slotConfigs.kV = 0;
        slotConfigs.kA = 0;

        elevator.getConfigurator().apply(slotConfigs);

        var mmConfigs = new MotionMagicConfigs();

        mmConfigs.MotionMagicCruiseVelocity = 100;
        mmConfigs.MotionMagicAcceleration = 300;
        mmConfigs.MotionMagicJerk = 2500;

        elevator.getConfigurator().apply(mmConfigs);

        var feedbackConfigs = new FeedbackConfigs();

        feedbackConfigs.RotorToSensorRatio = 1.0;
        feedbackConfigs.SensorToMechanismRatio = 1.0;

        elevator.getConfigurator().apply(feedbackConfigs);

        setDefaultCommand(powerDownwardsToZero());
    }

    Command raiseToHeight(Distance targetHeight) {
        return runEnd(() -> setHeight(targetHeight), this::setVoltageOutToZero);
    }

    Command cutPower() {
        return run(() -> setVoltageOutToZero());
    }

    Command powerDownwardsToZero() {
        return runEnd(() -> {
            if (isAtBottomOfTravel() || state.isClimbing())
                setVoltageOutToZero();
            else
                setHeight(Inches.of(0));
        }, this::setVoltageOutToZero);
    }

    public Command l1() {
        return raiseToHeight(Inches.of(10.0));
    }

    public Command l2() {
        return raiseToHeight(Inches.of(20.0));
    }

    public Command l3() {
        return raiseToHeight(Inches.of(30.0));
    }

    public Command l4() {
        return raiseToHeight(Inches.of(40.0));
    }

    public Command pickUp() {
        return raiseToHeight(Inches.of(15.0));
    }

    Command voltageOut(Supplier<Voltage> volts) {
        return run(() -> elevator.setControl(new VoltageOut(volts.get())));
    }

    Distance toHeight(Angle angle) {
        return angle.div(MAX_HEIGHT_ANGLE).times(MAX_HEIGHT_LINEAR);
    }

    Angle fromHeight(Distance height) {
        return height.div(MAX_HEIGHT_LINEAR).times(MAX_HEIGHT_ANGLE);
    }

    public Distance getHeight() {
        return toHeight(elevator.getPosition().getValue());
    }

    public LinearVelocity getVelocity() {
        var rotationsPerSecond = elevator.getVelocity().getValue().in(RotationsPerSecond);

        var inchesPerRotation = MAX_HEIGHT_LINEAR.in(Inches) / MAX_HEIGHT_ANGLE.in(Rotations);

        return InchesPerSecond.of(rotationsPerSecond * inchesPerRotation);
    }

    public boolean isAtBottomOfTravel() {
        return getHeight().isNear(Inches.of(0), Inches.of(1));
    }

    public void setHeight(Distance height) {
        var targetPosition = fromHeight(height);
        if (targetPosition.gt(MAX_HEIGHT_ANGLE))
            targetPosition = MAX_HEIGHT_ANGLE;
        if (targetPosition.lt(Rotations.of(0)))
            targetPosition = Rotations.of(0);
        if (!DISABLE_MOTION_MAGIC)
            elevator.setControl(new MotionMagicVoltage(targetPosition));
        else
            setVoltageOutToZero();
    }

    public void setVoltageOutToZero() {
        elevator.setControl(new VoltageOut(0));
    }

    public State getState() {
        return state;
    }

    public class State {
        private boolean climbing;

        public boolean isClimbing() {
            return climbing;
        }
    }

    public class Telemetry {
        final ShuffleboardLayout elevatorCommands = Shuffleboard.getTab("Commands").getLayout("Elevator",
                BuiltInLayouts.kList);

        final ShuffleboardLayout elevatorStats = Shuffleboard.getTab("Commands").getLayout("Elevator Stats",
                BuiltInLayouts.kList);

        final GenericEntry voltageOutEntry = elevatorCommands.add("Raw Voltage Out", 0.0)
                .withWidget(BuiltInWidgets.kNumberSlider).getEntry();

        Telemetry(ElevatorSubsystem elevator) {
            elevatorCommands.add("L1", l1());
            elevatorCommands.add("L2", l2());
            elevatorCommands.add("L3", l3());
            elevatorCommands.add("L4", l4());

            elevatorCommands.add("Apply Voltage Out", voltageOut(() -> Volts.of(voltageOutEntry.getDouble(0.0))));

            elevatorStats.addDouble("Angle (Deg)", () -> elevator.elevator.getPosition().getValue().in(Degrees));
            elevatorStats.addDouble("Height (In.)", () -> elevator.getHeight().in(Inches));
            elevatorStats.addDouble("Voltage Output", () -> elevator.elevator.getMotorVoltage().getValue().in(Volts));
        }
    }
}
