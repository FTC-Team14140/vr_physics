package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * Example OpMode. Controls robot using left joystick, with arcade drive.
 */

//@Disabled
@TeleOp(name = "two wheel arcade drive", group = "TwoWheel")
public class TwoWheelArcadeDrive extends LinearOpMode {

    DcMotor left;
    DcMotor right;
    DcMotor armExtensionMotor;
    DcMotor armRotationMotor;
    Servo fingerServo;
    ColorSensor colorSensor;
    GyroSensor gyro;
    DistanceSensor frontDistance, leftDistance, backDistance, rightDistance;

    public void runOpMode(){
        left = hardwareMap.dcMotor.get("left_motor");
        right = hardwareMap.dcMotor.get("right_motor");
        left.setDirection(DcMotor.Direction.REVERSE);
        armExtensionMotor = hardwareMap.dcMotor.get("arm_extension_motor");
        armRotationMotor = hardwareMap.dcMotor.get("arm_rotation_motor");
        fingerServo = hardwareMap.servo.get("finger_servo");
        colorSensor = hardwareMap.colorSensor.get("color_sensor");
        gyro = hardwareMap.gyroSensor.get("gyro_sensor");
        gyro.init();
        frontDistance = hardwareMap.get(DistanceSensor.class, "front_distance");
        leftDistance = hardwareMap.get(DistanceSensor.class, "left_distance");
        backDistance = hardwareMap.get(DistanceSensor.class, "back_distance");
        rightDistance = hardwareMap.get(DistanceSensor.class, "right_distance");


        telemetry.addData("Press Start When Ready","");
        telemetry.update();
        waitForStart();
        while (opModeIsActive()){
            float fwd = -gamepad1.left_stick_y;
            float steer = gamepad1.left_stick_x;
            float leftPower = fwd - steer;
            float rightPower = fwd + steer;
            float scale = (float)Math.max(1.0, Math.max(Math.abs(leftPower), Math.abs(rightPower)));
            leftPower /= scale;
            rightPower /= scale;
            left.setPower(leftPower);
            right.setPower(rightPower);

            armRotationMotor.setPower(gamepad1.right_stick_y);

            if (gamepad1.y) armExtensionMotor.setPower(1.0);
            else if (gamepad1.a) armExtensionMotor.setPower(-1.0);
            else armExtensionMotor.setPower(0);

            if (gamepad1.x) fingerServo.setPosition(1);
            else if (gamepad1.b) fingerServo.setPosition(0);

            telemetry.addData("Gamepad 1 left stick controls drive","");
            telemetry.addData("GP1 Rt. Stick-Y controls arm rotation","");
            telemetry.addData("Gamepad 1 X and B control fingers.","");
            telemetry.addData("Color","R %d  G %d  B %d", colorSensor.red(), colorSensor.green(), colorSensor.blue());
            telemetry.addData("Heading"," %.1f", gyro.getHeading());
            telemetry.addData("Encoders","Left %d  Right %d", left.getCurrentPosition(), right.getCurrentPosition());
            telemetry.addData("Distance", " Fr %.1f  Lt %.1f  Rt %.1f  Bk %.1f  ",
                    frontDistance.getDistance(DistanceUnit.CM), leftDistance.getDistance(DistanceUnit.CM),
                    rightDistance.getDistance(DistanceUnit.CM), backDistance.getDistance(DistanceUnit.CM)
            );
            telemetry.update();
            }
        left.setPower(0);
        right.setPower(0);
    }
}
