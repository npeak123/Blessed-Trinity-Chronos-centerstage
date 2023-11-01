package org.firstinspires.ftc.teamcode.teleops;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp
public class teleOpWithoutDirection extends LinearOpMode{


    @Override
    public void runOpMode() throws InterruptedException {
        // Declare our motors
        // Make sure your ID's match your configuration
        DcMotor frontLeftMotor = hardwareMap.dcMotor.get("frontLeftMotor");
        DcMotor backLeftMotor = hardwareMap.dcMotor.get("backLeftMotor");
        DcMotor frontRightMotor = hardwareMap.dcMotor.get("frontRightMotor");
        DcMotor backRightMotor = hardwareMap.dcMotor.get("backRightMotor");
        DcMotor leftSlide = hardwareMap.dcMotor.get("leftSlide");
        DcMotor rightSlide = hardwareMap.dcMotor.get("rightSlide");
        DcMotor armMotor = hardwareMap.dcMotor.get("armMotor");
        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        Servo lClawServo = hardwareMap.servo.get("LClawServo");
        Servo rClawServo = hardwareMap.servo.get("RClawServo");
        CRServo clawServo = hardwareMap.crservo.get("ClawServo");
        lClawServo.setDirection(Servo.Direction.REVERSE);

        lClawServo.setPosition(0.5);
        rClawServo.setPosition(0.1);
        clawServo.setPower(1);


        // Reverse the right side motors. This may be wrong for your setup.
        // If your robot moves backwards when commanded to go forwards,
        // reverse the left side instead.
        // See the note about this earlier on this page.
        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        rightSlide.setDirection(DcMotorSimple.Direction.REVERSE);

        // Retrieve the IMU from the hardware map
        IMU imu = hardwareMap.get(IMU.class, "imu");
        // Adjust the orientation parameters to match your robot
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));
        // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
        imu.initialize(parameters);
        double pos = 0.5;

        waitForStart();

        if (isStopRequested()) return;


        while (opModeIsActive()) {
            double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
            double x = gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;
            double ry = gamepad2.left_stick_y;
            double ly = gamepad2.right_stick_y;
            double trigger = gamepad1.right_trigger;
            boolean rClose = true;
            double up = gamepad2.right_trigger + 1;
            double down = gamepad2.left_trigger;

            // This button choice was made so that it is hard to hit on accident,
            // it can be freely changed based on preference.
            // The equivalent button is start on Xbox-style controllers.
            if (gamepad1.options) {
                imu.resetYaw();
            }
            if (gamepad2.b) {
                rClawServo.setPosition(0.1);

            }
            if (gamepad2.a) {
                rClawServo.setPosition(0.25);

            }
            if (gamepad2.y) {
                lClawServo.setPosition(90);
            }
            if (gamepad2.x) {
                lClawServo.setPosition(0.5);
            }
            clawServo.setPower(down);
            double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

            // Rotate the movement direction counter to the bot's rotation
            double rotX = x - y;
            double rotY = x + y;

            rotX = rotX * 1.1;  // Counteract imperfect strafing

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio,
            // but only if at least one is out of the range [-1, 1]

            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
            double frontLeftPower = (y - x - rx) / denominator;
            double backLeftPower = (y + x - rx) / denominator;
            double frontRightPower = (y + x + rx) / denominator;
            double backRightPower = (y - x + rx) / denominator;
            double NOS = gamepad1.right_trigger + 1;


            frontLeftMotor.setPower(frontLeftPower * NOS);
            backLeftMotor.setPower(backLeftPower * NOS);
            frontRightMotor.setPower(frontRightPower * NOS);
            backRightMotor.setPower(backRightPower * NOS);
            leftSlide.setPower(-ly);
            rightSlide.setPower(-ly);

            armMotor.setPower(-ry / 2);

            //telemetry.addData("lClawServo:", lClawServo.getPosition());
            //telemetry.addData("rClawServo:", rClawServo.getPosition());
        }

    }
}

