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

//        telemetry.addData("limitSwitchLimitSwitching?", armSubsystem.blockInGrabber());
//        telemetry.addData("leftElevatorEncoderCounts", elevatorSubsystem.leftEncoderCounts());
//        telemetry.addData("rightElevatorEncoderCounts", elevatorSubsystem.rightEncoderCounts());
        telemetry.addData("extensionMotorEncoderCounts", armSubsystem.extensionEncoderCounts());
        telemetry.addData("rotate", armSubsystem.rotatePos());
        telemetry.addData("armDistance", armSubsystem.armDist());
//        telemetry.addData("isA?", gamepad2.a);
//        telemetry.addData("Encoding", mecanumSubsystem.encoderCounts());

        double forward = -gamepad1.left_stick_y;
        double turn = gamepad1.right_stick_x;
        double strafe = -gamepad1.left_stick_x;
        double extend = -gamepad2.right_stick_y;
        double rotate = gamepad2.left_stick_x;

//        if(gamepad2.a && !armSubsystem.blockInGrabber()){
//            armSubsystem.intake(1);
//        } else if (gamepad2.b){
//            armSubsystem.intake(-1);
//        }else{
//            armSubsystem.intake(0);
//        }

        if(gamepad2.a){
            armSubsystem.upDown(0.72);
        }else if(gamepad2.b){
            armSubsystem.upDown(0);
        }else if(gamepad2.x){
            armSubsystem.upDown(0.1);
        }else if(gamepad2.y){
            armSubsystem.upDown(0.45);
        }

        if(gamepad2.right_trigger > 0.1){
            armSubsystem.intake(-1);
        }else if(gamepad2.left_trigger > 0.1){
            armSubsystem.intake(1);
        }else {
            armSubsystem.intake(0);
        }

        if(gamepad1.left_trigger > 0.1){
            elevatorSubsystem.elevatorLift(gamepad1.left_trigger);
        } else if(gamepad1.right_trigger > 0.1){
            elevatorSubsystem.elevatorLift(-gamepad1.right_trigger);
        }

        if(gamepad1.a){
            elevatorSubsystem.eleHook(2);
        }else if(gamepad1.b){
            elevatorSubsystem.eleHook(-2);
        } else if (gamepad1.y){
            elevatorSubsystem.eleHook(0);
        }

        mecanumSubsystem.TeleOperatedDrive(forward, strafe, turn);
        armSubsystem.extendIntake(extend);
        armSubsystem.rotatePower(rotate);

        telemetry.update();
    }
}