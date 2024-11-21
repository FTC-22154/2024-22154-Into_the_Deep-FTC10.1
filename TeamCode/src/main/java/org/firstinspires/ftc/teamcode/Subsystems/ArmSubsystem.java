package org.firstinspires.ftc.teamcode.Subsystems;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.TouchSensor;

public class ArmSubsystem {

    DcMotor exm, im;

    //TouchSensor ls;

    public ArmSubsystem(HardwareMap hardwareMap){
        exm = hardwareMap.get(DcMotor.class,"exm");
        im = hardwareMap.get(DcMotor.class,"im");
        //ls = hardwareMap.get(TouchSensor.class, "ls");

        exm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        im.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        exm.setDirection(DcMotor.Direction.FORWARD);
        im.setDirection(DcMotor.Direction.REVERSE);

        exm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        im.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        exm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        im.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void extendIntake(double power){
        exm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        exm.setPower(power);
    }

    public void encoderExtend(double counts){
        exm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        int eTarget = Math.toIntExact(Math.round(exm.getCurrentPosition() + counts));
        exm.setTargetPosition(eTarget);
        if(counts < 0){
            exm.setPower(-0.75);
        } else {
            exm.setPower(0.75);
        }
    }

    public class ExtensionAuto implements Action {
        private boolean initialized = false;
        public int extensionTarget;
        ExtensionAuto(int extensionTarget){
            this.extensionTarget = extensionTarget;
        }
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            double extendPos = exm.getCurrentPosition();
            if (!initialized && extendPos > extensionTarget) {
                exm.setPower(-0.8);
                initialized = true;
            } else if (!initialized && extendPos < extensionTarget){
                exm.setPower(0.8);
                initialized = true;
            }

            packet.put("extendPos", extendPos);
            if ((exm.getPower() == -0.8 && extendPos < extensionTarget) || (exm.getPower() == 0.75 && extendPos > extensionTarget)) {
                return true;
            } else {
                exm.setPower(0);
                return false;
            }
        }
    }
    public Action extensionAuto(int extensionTarget){
        return new ExtensionAuto(extensionTarget);
    }

    public class IntakeAuto implements Action {
        private boolean initialized = false;
        // 0 = spit, 1 = intake, 2 = none
        public int intake;
        IntakeAuto(int intake) {
            this.intake = intake;
        }
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized && intake == 1) {
                im.setPower(-0.8);
                initialized = true;
            } else if (!initialized && intake == 0) {
                im.setPower(0.8);
                initialized = true;
            }
                return false;

        }
    }
    public Action intakeAuto(int intake){
        return new IntakeAuto(intake);
    }

    public void intake(double power){
        im.setPower(power);
    }

//    //public boolean blockInGrabber(){
//        return ls.isPressed();
//    }

    public int extensionEncoderCounts(){return exm.getCurrentPosition();}

}
