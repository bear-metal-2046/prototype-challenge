package org.tahomarobotics.robot.indexer;

import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.compound.Diff_MotionMagicVoltage_Velocity;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.DigitalInput;
import org.tahomarobotics.robot.RobotMap;
import org.tahomarobotics.robot.collector.Collector;
import org.tahomarobotics.robot.collector.CollectorConstants;
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

    private final MotionMagicVelocityVoltage  indexerControl = new MotionMagicVelocityVoltage(0);

    // MOTORS
    private final TalonFX motor = new TalonFX(RobotMap.INDEXER_MOTOR);

    // BEAM BRAKES
    private final DigitalInput collectorBeamBrake = new DigitalInput(RobotMap.BEAM_BREAK_ONE);
    private final DigitalInput shooterBeamBrake = new DigitalInput(RobotMap.BEAM_BREAK_ONE);

    // TODO: CONTROL REQUESTS



    // TODO: STATUS SIGNALS



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

    @Override
    public void periodic() {
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

    // STATES

    public enum State {
        DISABLED,
        INTAKING,
        INDEXING,
        COLLECTED,
        EJECTING,
    }
}
