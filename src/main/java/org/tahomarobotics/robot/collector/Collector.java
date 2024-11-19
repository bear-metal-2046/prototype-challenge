package org.tahomarobotics.robot.collector;

import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import org.tahomarobotics.robot.RobotMap;
import org.tahomarobotics.robot.util.SubsystemIF;

public class Collector extends SubsystemIF {
    private static final Collector INSTANCE = new Collector();

    private final MotionMagicVoltage deployControl = new MotionMagicVoltage(CollectorConstants.STOW_POSITION);
    private final MotionMagicVelocityVoltage collectorControl = new MotionMagicVelocityVoltage(0);

    // MOTORS
    private final TalonFX deployLeft = new TalonFX(RobotMap.DEPLOY_MOTOR_LEFT);
    private final TalonFX deployRight = new TalonFX(RobotMap.DEPLOY_MOTOR_RIGHT);
    private final TalonFX collectMotor = new TalonFX(RobotMap.COLLECTOR_MOTOR);

    // CONTROL REQUESTS

    // STATUS SIGNALS

    // STATE
    private CollectionState collectionState = CollectionState.DISABLED;
    private DeploymentState deploymentState = DeploymentState.STOWED, preEjectState;

    private Collector() {
        deployLeft.getConfigurator().apply(CollectorConstants.deployMotorConfiguration);
        deployRight.getConfigurator().apply(CollectorConstants.deployMotorConfiguration);
        deployRight.setInverted(true);
        collectMotor.getConfigurator().apply(CollectorConstants.collectMotorConfiguration);
    }

    public static Collector getInstance() {
        return INSTANCE;
    }

    // GETTERS

    // SETTERS
    private void setDeployPosition(double position) {
        deployRight.setControl(deployControl.withPosition(position));
        deployLeft.setControl(deployControl.withPosition(position));
    }

    private void setRollerVelocity(double velocity) {
        collectMotor.setControl(collectorControl.withVelocity(velocity));
    }

    public void toggleDeploy() {
        shouldDeploy = !shouldDeploy;
    }

    // STATE MACHINE
    boolean shouldDeploy = false;
    boolean shouldEject = false;
    boolean shouldCollect = false;

    @Override
    public void periodic() {
        switch (deploymentState) {
            case DEPLOYED -> {
                if (!shouldDeploy) setDeployStow();
                if (shouldEject) setDeployEject();
            }
            case EJECT -> {
                if (!shouldEject)
            }
            case STOWED -> {
            }
        }
        switch (collectionState) {
            case COLLECTING -> {

            }
            case EJECTING -> {


            }
            case DISABLED -> {
            }
        }
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
        setDeployPosition(CollectorConstants.);
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


    // STATES
    public enum CollectionState {
        COLLECTING,
        DISABLED,
        EJECTING
    }

    public enum DeploymentState {
        DEPLOYED,
        STOWED,
        EJECT
    }
}
