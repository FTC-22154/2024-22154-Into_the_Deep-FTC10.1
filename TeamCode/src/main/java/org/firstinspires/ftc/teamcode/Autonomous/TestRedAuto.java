package org.firstinspires.ftc.teamcode.Autonomous;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.Subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.ElevatorSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.MecanumSubsystem;

@Autonomous
public class TestRedAuto extends LinearOpMode {

    MecanumDrive mecanumDrive;
    ElevatorSubsystem elevatorSubsystem;
    ArmSubsystem armSubsystem;

    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d initialPose = new Pose2d(-37.5,-61, Math.toRadians(180));
        mecanumDrive = new MecanumDrive(hardwareMap,initialPose);
        elevatorSubsystem = new ElevatorSubsystem(hardwareMap);
        armSubsystem = new ArmSubsystem(hardwareMap);

        Action traj1 = mecanumDrive.actionBuilder(initialPose)
                        .strafeTo(new Vector2d(-37.5, -52))
                        .lineToX(-52.5)
                        .turn(Math.toRadians(45))
                        .build();

        waitForStart();

        Actions.runBlocking(
                new SequentialAction(
                        traj1,
                        elevatorSubsystem.elevatorAuto(3000)

                )
        );

    }
}
