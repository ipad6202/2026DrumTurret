package frc.robot.subsystems.shooter.lifter;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.util.io.motors.elevator.Elevator;
import frc.robot.util.io.motors.elevator.ElevatorIO;
import frc.robot.util.io.motors.elevator.ElevatorIOSim;
import frc.robot.util.subsystems.ExtendedSubsystem;

public class Lifter extends ExtendedSubsystem {
    private final Elevator<LifterConstants.ElevatorState> leftLifter;
    private final Elevator<LifterConstants.ElevatorState> rightLifter;
    private final Elevator<LifterConstants.ElevatorState> centerLifter;

    public Lifter(){
        boolean sim = true; //placeholder for now argh
        ElevatorIO leftLifterIO;
        ElevatorIO rightLifterIO;
        ElevatorIO centerLifterIO;
        if (sim){
            leftLifterIO = new ElevatorIOSim(DCMotor.getKrakenX44(1), LifterConstants.ELEVATOR_GEAR_RATIO, LifterConstants.CARRIAGE_MASS, LifterConstants.DRUM_RADIUS, LifterConstants.MIN_HEIGHT, LifterConstants.MAX_HEIGHT, LifterConstants.ELEVATOR_KP, LifterConstants.ELEVATOR_KD, 0);
            rightLifterIO = new ElevatorIOSim(DCMotor.getKrakenX44(1), LifterConstants.ELEVATOR_GEAR_RATIO, LifterConstants.CARRIAGE_MASS, LifterConstants.DRUM_RADIUS, LifterConstants.MIN_HEIGHT, LifterConstants.MAX_HEIGHT, LifterConstants.ELEVATOR_KP, LifterConstants.ELEVATOR_KD, 0);
            centerLifterIO = new ElevatorIOSim(DCMotor.getKrakenX44(1), LifterConstants.ELEVATOR_GEAR_RATIO, LifterConstants.CARRIAGE_MASS, LifterConstants.DRUM_RADIUS, LifterConstants.MIN_HEIGHT, LifterConstants.MAX_HEIGHT, LifterConstants.ELEVATOR_KP, LifterConstants.ELEVATOR_KD, 0);
        } else { //placeholder
            leftLifterIO = new ElevatorIO() {};
            rightLifterIO = new ElevatorIO() {};
            centerLifterIO = new ElevatorIO() {};
        }

        leftLifter = new Elevator<LifterConstants.ElevatorState>("leftLifter", leftLifterIO, LifterConstants.ElevatorState.class, LifterConstants.SETPOINTS, LifterConstants.HOMING_VOLTAGE, LifterConstants.HOMING_VELOCITY_THRESHOLD, LifterConstants.MAX_MANUAL_VOLTAGE);
        rightLifter = new Elevator<LifterConstants.ElevatorState>("rightLifter", rightLifterIO, LifterConstants.ElevatorState.class, LifterConstants.SETPOINTS, LifterConstants.HOMING_VOLTAGE, LifterConstants.HOMING_VELOCITY_THRESHOLD, LifterConstants.MAX_MANUAL_VOLTAGE);
        centerLifter = new Elevator<LifterConstants.ElevatorState>("centerLifter", centerLifterIO, LifterConstants.ElevatorState.class, LifterConstants.SETPOINTS, LifterConstants.HOMING_VOLTAGE, LifterConstants.HOMING_VELOCITY_THRESHOLD, LifterConstants.MAX_MANUAL_VOLTAGE);
    }

    @Override
    public void periodic(){
        leftLifter.periodic();
        rightLifter.periodic();
        centerLifter.periodic();
    }

    public void raise(){
        leftLifter.setState(LifterConstants.ElevatorState.RAISED);
        rightLifter.setState(LifterConstants.ElevatorState.RAISED);
        centerLifter.setState(LifterConstants.ElevatorState.RAISED);
    }

    public void lower(){
        leftLifter.setState(LifterConstants.ElevatorState.LOWERING);
        rightLifter.setState(LifterConstants.ElevatorState.LOWERING);
        centerLifter.setState(LifterConstants.ElevatorState.LOWERING);
    }

    public double getLeftPosMeters(){
        return LifterConstants.getPositionMeters(leftLifter.getPositionDeg());
    }

    public double getRightPosMeters(){
        return LifterConstants.getPositionMeters(rightLifter.getPositionDeg());
    }
    
    public double getCenterPosMeters(){
        return LifterConstants.getPositionMeters(centerLifter.getPositionDeg());
    }

    public Command homeLifters(){
        return Commands.parallel(leftLifter.homingSequence(), rightLifter.homingSequence(), centerLifter.homingSequence());
    }

    public Command extendLifters(){
        return Commands.runOnce(this::raise);
    }

    public Command compressLifters(){
        return Commands.runOnce(this::lower);
    }

    public Command manualControl(DoubleSupplier magnitude){
        return Commands.parallel(leftLifter.manualControl(magnitude), rightLifter.manualControl(magnitude), centerLifter.manualControl(magnitude));
    }


}
