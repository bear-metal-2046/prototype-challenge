package org.tahomarobotics.robot.indexer;

import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.tahomarobotics.robot.RobotMap;
import org.tahomarobotics.robot.collector.Collector;
import org.tahomarobotics.robot.util.SubsystemIF;

public class Indexer extends SubsystemIF {
    private static final Indexer INSTANCE = new Indexer();

    private State state = State.DISABLED;

    private Indexer() {
        motor.getConfigurator().apply(IndexerConstants.indexMotorConfiguration);
    }

    public static Indexer getInstance() {
        return INSTANCE;
    }


    // MOTORS
    private final TalonFX motor = new TalonFX(RobotMap.INDEXER_MOTOR);

    // BEAM BRAKES
    private final DigitalInput collectorBeamBrake = new DigitalInput(RobotMap.BEAM_BREAK_ONE);
    private final DigitalInput shooterBeamBrake = new DigitalInput(RobotMap.BEAM_BREAK_TWO);

    // CONTROL REQUESTS
    private final MotionMagicVelocityVoltage  indexerControl = new MotionMagicVelocityVoltage(0);

    // STATUS SIGNALS
    private double motorVelocity;

    private void statusSignals() {
        motorVelocity = motor.getVelocity().getValueAsDouble();
        SmartDashboard.putNumber("Indexer Velocity: ", motorVelocity);
        SmartDashboard.putBoolean("Shooter Beam Broke?", shooterBeamBroke());
        SmartDashboard.putBoolean("Collector Beam Broke?", collectorBeamBroke());
    }


    // STATE TRANSITIONS

    private void setToIntaking() {
        state = State.INTAKING;
    }

    private void setToEjecting() {
        state = State.EJECTING;
    }

    private void setToDisable() {
        state = State.DISABLED;
    }

    private void setToIndexing() {
        state = State.INDEXING;
    }

    private void setToCollected() {
        state = State.COLLECTED;
    }

    // GETTERS

    private boolean shooterBeamBroke() {
        return shooterBeamBrake.get();
    }

    private boolean collectorBeamBroke() {
        return collectorBeamBrake.get();
    }

    // SETTERS

    private void disable() {
        motor.stopMotor();
    }

    private void intake() {
        motor.setControl(indexerControl.withVelocity(IndexerConstants.INTAKE_RPS));
    }

    private void index() {
        motor.setControl(indexerControl.withVelocity(IndexerConstants.INDEX_RPS));
        if (shooterBeamBroke()) {
            disable();
            setToCollected();
        }
    }

    private void eject() {
        motor.setControl(indexerControl.withVelocity(IndexerConstants.EJECT_RPS));
    }

    // STATE MACHINE
    Collector collector = Collector.getInstance();


    // state machine in a method called in periodic
    private void stateMachine() {
        switch (state) {
            case DISABLED -> {
                disable();
                if (collector.isCollecting()) setToIntaking();
                if (collector.isEjecting()) setToEjecting();
            }
            case INTAKING -> {
                intake();
                if (collector.isEjecting()) setToEjecting();
                if (collector.isDisabled()) setToDisable();
                if (collectorBeamBroke()) setToIndexing();
            }
            case INDEXING -> {
                index();
                if (collector.isEjecting()) setToEjecting();
            }
            case COLLECTED -> {
                disable();
                if (collector.isEjecting()) setToEjecting();
            }
            case EJECTING -> {
                eject();
                if (collector.isDisabled()) setToDisable();
            }
        }
    }

    @Override
    public void periodic() {
        statusSignals();
        stateMachine();
    }


    // STATES
    public enum State {
        DISABLED,
        INTAKING,
        INDEXING,
        COLLECTED,
        EJECTING,
    }
}