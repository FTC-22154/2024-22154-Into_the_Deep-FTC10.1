package org.firstinspires.ftc.teamcode.Autonomous;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.ElevatorSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.MecanumSubsystem;
@Config
@Autonomous
public class AutonAscent1 extends LinearOpMode {

    // params to configure from the robot's FTC Dashboard
    public static class Params {
        public int step1Sleep = 2000;
        public double step1Pwr = 0.5;
        public int step2Sleep = 1000;
        public double step2Pwr = 0.5;
        public int step3Sleep = 2000;
        public int Step3Pos = 900;
        public int step4Sleep = 2500;
        public double step4Pwr = 0.2;
    }

    public static AutonAscent1.Params myParams = new AutonAscent1.Params();
    MecanumSubsystem myMecanumSubsystem;
    ElevatorSubsystem elevatorSubsystem;
    ArmSubsystem armSubsystem;
    @Override
    public void runOpMode() throws InterruptedException {
        myMecanumSubsystem = new MecanumSubsystem(hardwareMap);
        elevatorSubsystem = new ElevatorSubsystem(hardwareMap);
        armSubsystem = new ArmSubsystem(hardwareMap);

        waitForStart();
        // Step 1
        myMecanumSubsystem.TeleOperatedDrive(myParams.step1Pwr,0,0);
        sleep(myParams.step1Sleep);
        /* Stop */ myMecanumSubsystem.TeleOperatedDrive(0,0,0);

        // Step 2
        myMecanumSubsystem.TeleOperatedDrive(0,0,myParams.step2Pwr);
        sleep(myParams.step2Sleep);
        /* Stop */ myMecanumSubsystem.TeleOperatedDrive(0,0,0);

        // Step 3
        armSubsystem.pivotToPos(myParams.Step3Pos);
        sleep(myParams.step3Sleep);

        // Step 4
        myMecanumSubsystem.TeleOperatedDrive(myParams.step4Pwr,0,0);
        sleep(myParams.step4Sleep);

        //TODO REMEMBER TO ADD SLEEP FUNCTIONS AS NEEDED

    }


}
