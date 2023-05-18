package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.PivotSubsystem;

import java.util.function.DoubleSupplier;

public class PivotCommand extends CommandBase {
    private final PivotSubsystem pivotSubsystem;
    private double power;

    public PivotCommand(final PivotSubsystem pivot, final DoubleSupplier power) {
        pivotSubsystem = pivot;
        this.power = power.getAsDouble();

        addRequirements(pivotSubsystem);
    }

    @Override
    public void execute() {
        pivotSubsystem.setPower(power);
    }

}
