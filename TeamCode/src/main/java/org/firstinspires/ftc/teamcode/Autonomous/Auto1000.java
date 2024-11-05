package org.firstinspires.ftc.teamcode.Autonomous;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Subsystems.MecanumSubsystem;

@Autonomous
public class Auto1000 extends LinearOpMode {

    MecanumSubsystem mecanumSubsystem;

    @Override
    public void runOpMode() throws InterruptedException {
        mecanumSubsystem = new MecanumSubsystem(hardwareMap);

        waitForStart();
        //mecanumSubsystem.encoderDrive("forward", 5000);
        mecanumSubsystem.TeleOperatedDrive(-1,0,0);
        sleep(1500);
        mecanumSubsystem.TeleOperatedDrive(0,0,0);
    }
}
