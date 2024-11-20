package org.tahomarobotics.robot.collector.commands;

import edu.wpi.first.wpilibj2.command.Command;
import org.tahomarobotics.robot.collector.Collector;
import org.tahomarobotics.robot.collector.CollectorConstants;


public class CollectorZeroCommand extends Command {

    Collector collector = Collector.getInstance();

    public CollectorZeroCommand() {
        addRequirements(collector);
    }

    @Override
    public void execute() {
        collector.setDeployVoltage(CollectorConstants.COLLECTOR_ZERO_VOLTAGE);
    }

    @Override
    public boolean isFinished() {
        return Math.abs(collector.getLeftDeployVelocity()) < CollectorConstants.COLLECTOR_ZERO_VELOCITY_TOLERANCE
                && Math.abs(collector.getRightDeployVelocity()) < CollectorConstants.COLLECTOR_ZERO_VELOCITY_TOLERANCE;
    }

    @Override
    public void end(boolean interrupted) {
        collector.setDeployStow();
        collector.zeroDeploy();
    }
}
