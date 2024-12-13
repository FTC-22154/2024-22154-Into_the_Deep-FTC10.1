package org.firstinspires.ftc.teamcode.Opmodes;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.ElevatorSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.MecanumSubsystem;

@TeleOp(name = "Teleop", group = "TeleOp")
@Config
public class TeleOpTest extends OpMode {

    public static class Params {
        // Enter your constants below!! :) <--
        public double lowerExtensionBound = 4.75;
        public double upperExtensionBound = 40;
        public double lowerBucketExtension = 18;
        public double upperBucketExtension = 36;
        public double routinesTimeout = 4;

    }

    public static TeleOpTest.Params PARAMS = new TeleOpTest.Params();
    ElapsedTime routinesRuntime = new ElapsedTime();

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
        telemetry.addData("rotate", armSubsystem.getRotatePos());
        telemetry.addData("armDistance", armSubsystem.armDist());
        telemetry.addData("Pivot Tgt",armSubsystem.pivotGetTargetPos());
//        telemetry.addData("isA?", gamepad2.a);
//        telemetry.addData("Encoding", mecanumSubsystem.encoderCounts());

        double forward = -gamepad1.left_stick_y;
        double turn = gamepad1.right_stick_x;
        double strafe = -gamepad1.left_stick_x;
        double extend = -gamepad2.right_stick_y;
        double rotate = -gamepad2.left_stick_y;

//        if(gamepad2.a && !armSubsystem.blockInGrabber()){
//            armSubsystem.intake(1);
//        } else if (gamepad2.b){
//            armSubsystem.intake(-1);
//        }else{
//            armSubsystem.intake(0);
//        }

        if(gamepad2.y){
            armSubsystem.wristUpDown(armSubsystem.PARAMS.wristDeliverPos);
        }else if(gamepad2.x){
            armSubsystem.wristUpDown(0);
        }else if(gamepad2.a){
            armSubsystem.wristUpDown(0.1);
        }else if(gamepad2.b){
            armSubsystem.wristUpDown(armSubsystem.PARAMS.wristSpecIn);
        }

        if(gamepad2.right_trigger > 0.1){
            armSubsystem.intake(-1);
        }else if(gamepad2.left_trigger > 0.1){
            armSubsystem.intake(1);
        }else {
            armSubsystem.intake(0);
        }

        if(gamepad2.dpad_down || gamepad2.dpad_up || gamepad2.dpad_left || gamepad2.dpad_right) {
            if (gamepad2.dpad_down) {
                armSubsystem.smplPickup();
            } else if (gamepad2.dpad_up) {
                armSubsystem.smplPlaceHigh();
            } else if (gamepad2.dpad_left) {
                armSubsystem.specPlaceHigh();
            } else if (gamepad2.dpad_right) {
                armSubsystem.specPickup();
            }
            routinesRuntime.reset();
            while(routinesRuntime.seconds() < PARAMS.routinesTimeout && armSubsystem.pivotMotorBusy()){}
            // Need function to turn motors off?  <-- I don't think so... -AJV
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

        if(armSubsystem.armDist() < PARAMS.lowerExtensionBound){
            armSubsystem.extendIntake(1, 99999);
        } else if (armSubsystem.armDist() > PARAMS.upperExtensionBound){
            armSubsystem.extendIntake(-ArmSubsystem.PARAMS.extendIntakePwr, 99999);
        } else {
            if(gamepad2.right_bumper){
                armSubsystem.extendDistance(PARAMS.upperBucketExtension);
            } else if(gamepad2.left_bumper){
                armSubsystem.extendDistance(PARAMS.lowerBucketExtension);
            } else {
                armSubsystem.extendIntake(extend, 99999);
            }
        }

        mecanumSubsystem.TeleOperatedDrive(forward, strafe, turn);
        //armSubsystem.extendPower(extend);
        armSubsystem.pivotByPos(-rotate);
        telemetry.addData("My test param",armSubsystem.getTestParam());
        telemetry.addData("Extend Motor RunMode",armSubsystem.extendMotorGetMode());
        telemetry.update();
    }
}