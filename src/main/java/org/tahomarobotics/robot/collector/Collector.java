package org.tahomarobotics.robot.collector;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import org.tahomarobotics.robot.RobotMap;
import org.tahomarobotics.robot.collector.commands.CollectorZeroCommand;
import org.tahomarobotics.robot.util.SubsystemIF;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Collector extends SubsystemIF {
    private static final Collector INSTANCE = new Collector();

    // MOTORS
    private final TalonFX deployLeft = new TalonFX(RobotMap.DEPLOY_MOTOR_LEFT);
    private final TalonFX deployRight = new TalonFX(RobotMap.DEPLOY_MOTOR_RIGHT);
    private final TalonFX collectMotor = new TalonFX(RobotMap.COLLECTOR_MOTOR);


    // CONTROL REQUESTS
    private final MotionMagicVoltage deployControl = new MotionMagicVoltage(CollectorConstants.STOW_POSITION);
    private final MotionMagicVelocityVoltage collectorControl = new MotionMagicVelocityVoltage(0);


    // STATUS SIGNALS
    private double leftMotorPosition, rightMotorPosition, collectorVelocity;

    private void runStatusSignals() {
        leftMotorPosition = deployLeft.getPosition().getValueAsDouble();
        rightMotorPosition = deployRight.getPosition().getValueAsDouble();
        collectorVelocity = collectMotor.getVelocity().getValueAsDouble();
        SmartDashboard.putNumber("Left Deploy Position: ", leftMotorPosition);
        SmartDashboard.putNumber("Right Deploy Position: ", rightMotorPosition);
        SmartDashboard.putNumber("Collector Velocity: ", collectorVelocity);
        SmartDashboard.putString("Deployment State: ", deploymentState.toString());
        SmartDashboard.putString("Collection State: ", collectionState.toString());
    }
    // gets motor positions and velocity, collector states, and prints on SmartDashboard; called in periodic

    // STATE
    private CollectionState collectionState = CollectionState.DISABLED;
    private DeploymentState deploymentState = DeploymentState.STOWED, preEjectState;

    private Collector() {
        deployLeft.getConfigurator().apply(CollectorConstants.deployMotorConfiguration);
        deployRight.getConfigurator().apply(CollectorConstants.deployMotorConfiguration);
        collectMotor.getConfigurator().apply(CollectorConstants.collectMotorConfiguration);
    }

    public static Collector getInstance() {
        return INSTANCE;
    }

    //  GETTERS

    public double getLeftDeployVelocity() {
        return deployLeft.getVelocity().getValueAsDouble();
    }

    public double getRightDeployVelocity() {
        return deployRight.getVelocity().getValueAsDouble();
    }

    public boolean isCollecting() {
        return collectionState.equals(CollectionState.COLLECTING);
    }

    public boolean isEjecting() {
        return collectionState.equals(CollectionState.EJECTING);
    }

    public boolean isDisabled() {
        return collectionState.equals(CollectionState.DISABLED);
    }
    //collector state booleans for indexer

    private boolean isDeployed() {
        return deploymentState.equals(DeploymentState.DEPLOYED);
    }


    // SETTERS
    private void setDeployPosition(double position) {
        deployRight.setControl(deployControl.withPosition(position));
        deployLeft.setControl(new Follower(RobotMap.DEPLOY_MOTOR_RIGHT, false));
    }

    private void setCollectorVelocity(double velocity) {
        collectMotor.setControl(collectorControl.withVelocity(velocity));
    }

    public void setDeployVoltage(double voltage) {
        deployLeft.setVoltage(voltage);
        deployRight.setVoltage(voltage);
    }

    public void zeroDeploy() {
        deployLeft.setPosition(CollectorConstants.ZERO_POSITION);
        deployRight.setPosition(CollectorConstants.ZERO_POSITION);
    }

    public void setShouldEject(boolean check) {
        shouldEject = check;
    }

    public void setShouldCollect(boolean check) {
        shouldCollect = check;
    }

    public void toggleDeploy() {
        shouldDeploy = !shouldDeploy;
    }

    // STATE MACHINE
    boolean shouldDeploy = false;
    boolean shouldEject = false;
    boolean shouldCollect = false;

    // state machine in a method called in periodic
    private void stateMachine() {
        switch (deploymentState) {
            case DEPLOYED -> {
                if (!shouldDeploy) setDeployStow();
                if (shouldEject) setDeployEject();
                if (shouldDeploy) setDeployDeployed();
            }
            case EJECT -> {
                if (!shouldEject) deployUneject();
            }
            case STOWED -> {
                if (shouldDeploy) setDeployDeployed();
                if (shouldEject) setDeployEject();
            }
        }
        switch (collectionState) {
            case COLLECTING -> {
                if (!shouldCollect) disableCollector();
                if (shouldEject) collectorEject();
                if (shouldCollect) collectorCollect();
            }
            case EJECTING -> {
                if (!shouldEject) disableCollector();
            }
            case DISABLED -> {
                if (shouldCollect) collectorCollect();
                if (shouldEject) collectorEject();
            }
        }
    }

    @Override
    public void onTeleopInit() {
        new CollectorZeroCommand().schedule();
    }

    @Override
    public void periodic() {
        runStatusSignals();
        stateMachine();
    }


    // STATE TRANSITIONS
    private void deployUneject() {
        switch(preEjectState) {
            case DEPLOYED -> setDeployDeployed();
            case STOWED -> setDeployStow();
            default -> {}
        }
    }

    public void setDeployDeployed() {
        setDeployPosition(CollectorConstants.COLLECT_POSITION);
        preEjectState = deploymentState;
        deploymentState = DeploymentState.DEPLOYED;
    }

    public void setDeployStow() {
        setDeployPosition(CollectorConstants.STOW_POSITION);
        preEjectState = deploymentState;
        deploymentState = DeploymentState.DEPLOYED;
    }

    public void setDeployEject() {
        setDeployPosition(CollectorConstants.EJECT_POSITION);
        deploymentState = DeploymentState.EJECT;
    }

    public void disableCollector() {
        collectMotor.stopMotor();
        collectionState = CollectionState.DISABLED;
    }

    public void collectorEject() {
        setCollectorVelocity(CollectorConstants.EJECTION_RPS);
        collectionState = CollectionState.EJECTING;
    }

    public void collectorCollect() {
        setCollectorVelocity(CollectorConstants.COLLECT_MAX_RPS);
        collectionState = CollectionState.COLLECTING;
    }

    // STATES
    public enum CollectionState {
        COLLECTING,
        DISABLED,
        EJECTING
    }

    public enum DeploymentState {
        DEPLOYED,
        STOWED,
        EJECT,
    }
}