package org.firstinspires.ftc.teamcode.Opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.ElevatorSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.MecanumSubsystem;

@TeleOp(name = "Teleop", group = "TeleOp")
public class TeleOpTest extends OpMode {

    MecanumSubsystem mecanumSubsystem;
    ElevatorSubsystem elevatorSubsystem;
    ArmSubsystem armSubsystem;

    @Override
    public void init(){
        mecanumSubsystem = new MecanumSubsystem(hardwareMap);
        elevatorSubsystem = new ElevatorSubsystem(hardwareMap);
        armSubsystem = new ArmSubsystem(hardwareMap);
    }

    @Override
    public void loop(){

        telemetry.addData("limitSwitchLimitSwitching?", armSubsystem.blockInGrabber());
        telemetry.addData("leftElevatorEncoderCounts", elevatorSubsystem.leftEncoderCounts());
        telemetry.addData("rightElevatorEncoderCounts", elevatorSubsystem.rightEncoderCounts());
        telemetry.addData("extensionMotorEncoderCounts", armSubsystem.extensionEncoderCounts());
        telemetry.addData("isA?", gamepad2.a);
        telemetry.addData("Encoding", mecanumSubsystem.encoderCounts());

        double forward = -gamepad1.left_stick_y;
        double turn = gamepad1.right_stick_x;
        double strafe = gamepad1.left_stick_x;
        double elevate = -gamepad2.left_stick_y;
        double extend = gamepad2.right_stick_y;

        if(gamepad2.a && !armSubsystem.blockInGrabber()){
            armSubsystem.intake(1);
        } else if (gamepad2.b){
            armSubsystem.intake(-1);
        }else{
            armSubsystem.intake(0);
        }

        mecanumSubsystem.TeleOperatedDrive(forward, strafe, turn);
        elevatorSubsystem.elevatorLift(elevate);
        armSubsystem.extendIntake(extend);

        telemetry.update();
    }
}