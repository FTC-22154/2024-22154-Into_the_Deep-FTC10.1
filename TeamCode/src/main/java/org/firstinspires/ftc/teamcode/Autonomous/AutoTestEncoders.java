package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;


import org.firstinspires.ftc.teamcode.Subsystems.MecanumSubsystem;

@Autonomous
public class AutoTestEncoders extends LinearOpMode {

    MecanumSubsystem mecanumSubsystem;

    ElapsedTime runtime = new ElapsedTime();
    @Override
    public void runOpMode() throws InterruptedException {
        mecanumSubsystem = new MecanumSubsystem(hardwareMap);

        waitForStart();
        encoderDriveToPosition("forward", 5000,5000);
//        mecanumSubsystem.encoderDrive("forward", 5000);
//        sleep(5000);
        //mecanumSubsystem.TeleOperatedDrive(1,0,0);
    }

    public void encoderDriveToPosition(String direction, int counts, double timeout){
        mecanumSubsystem.encoderDrive(direction, counts);
        runtime.reset();
        while (opModeIsActive() && runtime.seconds() < timeout && mecanumSubsystem.areMotorsBusy()){}
        mecanumSubsystem.driveMotorsOff();
    }
}
