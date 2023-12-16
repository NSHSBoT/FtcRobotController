/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

import java.util.List;


/*
 * This OpMode illustrates the concept of driving a path based on time.
 * The code is structured as a LinearOpMode
 *
 * The code assumes that you do NOT have encoders on the wheels,
 *   otherwise you would use: RobotAutoDriveByEncoder;
 *
 *   The desired path in this example is:
 *   - Drive forward for 3 seconds
 *   - Spin right for 1.3 seconds
 *   - Drive Backward for 1 Second
 *
 *  The code is written in a simple form with no optimizations.
 *  However, there are several ways that this type of sequence could be streamlined,
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */

@Autonomous(name="Robot: Auto Drive By Evans", group="Robot")
//@Disabled
public class RobotAutoDriveByEvans extends LinearOpMode {
    /* Declare OpMode members. */
    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;
    //arm!
    private DcMotorEx armMotor = null;
    public Servo claw_servo;

    private  VoltageSensor myControlHubVoltageSensor;
    private ElapsedTime runtime = new ElapsedTime();
    private static final boolean USE_WEBCAM = true;  // true for webcam, false for phone camera

    // TFOD_MODEL_ASSET points to a model file stored in the project Asset location,
    // this is only used for Android Studio when using models in Assets.
    private static final String TFOD_MODEL_ASSET = "first.tflite";
    // TFOD_MODEL_FILE points to a model file stored onboard the Robot Controller's storage,
    // this is used when uploading models directly to the RC using the model upload interface.
    private static final String TFOD_MODEL_FILE = "/sdcard/FIRST/tflitemodels/first.tflite";
    // Define the labels recognized in the model for TFOD (must be in training order!)
    private static final String[] LABELS = new String[]{
            "BAnvil", "RAnvil"
    };

    /**
     * The variable to store our instance of the TensorFlow Object Detection processor.
     */
    private TfodProcessor tfod;

    private void initTfod() {

        // Create the TensorFlow processor by using a builder.
        tfod = new TfodProcessor.Builder()

                // With the following lines commented out, the default TfodProcessor Builder
                // will load the default model for the season. To define a custom model to load,
                // choose one of the following:
                //   Use setModelAssetName() if the custom TF Model is built in as an asset (AS only).
                //   Use setModelFileName() if you have downloaded a custom team model to the Robot Controller.
                //.setModelAssetName(TFOD_MODEL_ASSET)
                .setModelFileName(TFOD_MODEL_FILE)

                // The following default settings are available to un-comment and edit as needed to
                // set parameters for custom models.
                .setModelLabels(LABELS)
                .setIsModelTensorFlow2(true)
                .setIsModelQuantized(true)
                .setModelInputSize(300)
                .setModelAspectRatio(16.0 / 9.0)

                .build();

        // Create the vision portal by using a builder.
        VisionPortal.Builder builder = new VisionPortal.Builder();

        // Set the camera (webcam vs. built-in RC phone camera).
        if (USE_WEBCAM) {
            builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
        } else {
            builder.setCamera(BuiltinCameraDirection.BACK);
        }

        // Set and enable the processor.
        builder.addProcessor(tfod);

        // Build the Vision Portal, using the above settings.
        visionPortal = builder.build();

        // Set confidence threshold for TFOD recognitions, at any time.
        tfod.setMinResultConfidence(0.65f);

        // Disable or re-enable the TFOD processor at any time.
        //visionPortal.setProcessorEnabled(tfod, true);

    }   // end method initTfod()

    private VisionPortal visionPortal;

    public void Driveforward(double power, double waitsec) {
        runtime.reset();

        while (runtime.seconds() < waitsec && opModeIsActive()) {
            leftFrontDrive.setPower(power);
            leftBackDrive.setPower(power);
            rightFrontDrive.setPower(power);
            rightBackDrive.setPower(power);
        }
        sleep(1000);
    }

    /*  public void Driveforwardtime(double power, long time) {
          Driveforward(power);
          sleep(1000);
      }
  */

    private void stopdriving() {
        Driveforward(0, 0.2);
        //stopdriving();
    }

    private void turnLeft(double power) {
        // turns robot left 90 degrees

        runtime.reset();
        while (runtime.seconds() < .7 && opModeIsActive()) {
            leftFrontDrive.setPower(-1 * power);
            leftBackDrive.setPower(-1 * power);
            rightFrontDrive.setPower(power);
            rightBackDrive.setPower(power);
        }
    }

    private void strafeRight(double power, double waitsec) {
        //strafe (move horizontally) to the right - right wheels turn inward, left outward
        runtime.reset();
        while (runtime.seconds() < waitsec && opModeIsActive()) {
            leftFrontDrive.setPower(-1 * power);
            leftBackDrive.setPower(power);
            rightFrontDrive.setPower(-1 * power);
            rightBackDrive.setPower(power);
        }
        leftFrontDrive.setPower(0);
        leftBackDrive.setPower(0);
        rightFrontDrive.setPower(0);
        rightBackDrive.setPower(0);
        sleep(1000);
    }

    private void turnRight(double power) {
// turns robot right 90 degrees
        runtime.reset();
        while (runtime.seconds() < .7 && opModeIsActive()) {
            leftFrontDrive.setPower(power);
            leftBackDrive.setPower(power);
            rightFrontDrive.setPower(-1 * power);
            rightBackDrive.setPower(-1 * power);
        }
    }
    private void rotateArm(double power) {
        runtime.reset();
        while (runtime.seconds() < .8 && opModeIsActive()) {
            armMotor.setPower(power);
        }

        armMotor.setPower(0);
        sleep(1000);
        runtime.reset();
        while (runtime.seconds() < .9 && opModeIsActive()) {
            armMotor.setPower(-.15 * power);
        }

    }

    private void strafeLeft(double power, double waitsec) {
        //strafe (move horizontally) to the right - left wheels turn inward, right outward
        runtime.reset();
        while (runtime.seconds() < waitsec && opModeIsActive()) {
            leftFrontDrive.setPower(power);
            leftBackDrive.setPower(-1 * power);
            rightFrontDrive.setPower(power);
            rightBackDrive.setPower(-1 * power);
        }
        leftFrontDrive.setPower(0);
        leftBackDrive.setPower(0);
        rightFrontDrive.setPower(0);
        rightBackDrive.setPower(0);
        sleep(1000);
    }

    public void open() {
        claw_servo.setPosition(1);

    }

    public void close() {
        claw_servo.setPosition(0);
    }
    public void PlacePixel() {
        armMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        int curpos = armMotor.getCurrentPosition();
        int targpos = 2300;
        armMotor.setTargetPosition(targpos);
        armMotor.setPower(0.5);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while (armMotor.isBusy()){
            telemetry.addData("arm", armMotor.getCurrentPosition());
            telemetry.addData("target and curpos", targpos);
            telemetry.update();
        }
        armMotor.setPower(0.0);
        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
/*
    private void place(double position) {
        claw_servo.setPosition(.65);
        if (claw_servo.getPosition()<0) {
            claw_servo.setPosition(0);
        }
    }
    /*
    public void zone1() {

        // for this if the robot sees the game object right away it rotates the arm, goes forward and places pixel
        telemetry.addData("zone",1);
        telemetry.update();
        if (claw_servo.getPosition()<0) {
            claw_servo.setPosition(0);
        }
        strafeRight(.25, 0.2);
       //rotateArm(1);
        armMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        //DcMotorEx.RunMode mode = DcMotorEx.RunM+
        //
        //
        //
        //
        //
        //
        //
        // ode.RUN_USING_ENCODER;
//                armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//                armMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        int curpos = armMotor.getCurrentPosition();
        int targpos = 1850;
        armMotor.setTargetPosition(targpos);
        armMotor.setPower(0.85);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        armMotor.setTargetPosition((int)qutarturn);
//        armMotor.setPower(.01);
//        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while (armMotor.isBusy()){
            telemetry.addData("arm", armMotor.getCurrentPosition());
            telemetry.addData("target and curpos", targpos);
            telemetry.update();
        }
        armMotor.setPower(0.0);
        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        sleep(1000);
        Driveforward(.3,.4);
       // sleep(1000);
       // Driveforward(.9);
        open();

       /* runtime.reset();
        while (runtime.seconds() < 4 && opModeIsActive()) {
            leftFrontDrive.setPower(.8);
            leftBackDrive.setPower(-1*.8);
            rightFrontDrive.setPower(.8);
            rightBackDrive.setPower(-1*.8);
        }
        runtime.reset();
        while (runtime.seconds() < .8 && opModeIsActive()) {
            armMotor.setPower(.5);
        }

        armMotor.setPower(0);
        claw_servo.setPosition(1);
       */

/*    public void zone2() {
        telemetry.addData("zone",2);
        telemetry.update();
        strafeRight(.25, 0.3);
        Driveforward(.5,.3);
        place(0);
    }

    public void zone3() {
        strafeRight(.25, .3);
        Driveforward(.5,.6);
        place(1);
    }
*/

    public void runOpMode() {

        // Initialize the drive system variables.
        leftFrontDrive = hardwareMap.get(DcMotor.class, "left_front_drive");
        leftBackDrive = hardwareMap.get(DcMotor.class, "left_back_drive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front_drive");
        rightBackDrive = hardwareMap.get(DcMotor.class, "right_back_drive");
        armMotor = hardwareMap.get(DcMotorEx.class, "arm_motor");
        claw_servo = hardwareMap.get(Servo.class, "claw_servo");
        myControlHubVoltageSensor =  hardwareMap.get(VoltageSensor.class, "Control Hub");


        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // When run, this OpMode should start both motors driving forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);
        armMotor.setDirection(DcMotor.Direction.FORWARD);

        // armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Ready to run");    //
        telemetry.update();
        double voltageshow;
        double powerRatio = 1;
        voltageshow = myControlHubVoltageSensor.getVoltage();
        if (voltageshow >= 13) {powerRatio=.97;}
        if (voltageshow < 13 && voltageshow >=12.5) {powerRatio=1;}
        if (voltageshow < 12.5 && voltageshow >= 12) {powerRatio=1.06;}
        String location = "far";
        String alliance = "blue";

        if (gamepad1.right_stick_button == true ) {location = "near";}
        if (gamepad1.left_stick_button == true ) {alliance = "red";}

        initTfod();

        // Wait for the DS start button to be touched.
        telemetry.addData("DS preview on/off", "3 dots, Camera Stream");
        telemetry.addData(">", "Touch Play to start OpMode");
        telemetry.addData("voltage start", voltageshow);
        telemetry.addData("location", location);
        telemetry.addData("alliance", alliance);

        telemetry.update();


        waitForStart();





       /* if (claw_servo.getPosition()<0) {
            claw_servo.setPosition(0);
        }

        */
        //claw_servo.setPosition(0);

        int count = 1;
        int zone = 0;
        int loop = 0;

        if (opModeIsActive()) {

            telemetry.addData("zone drive", zone);
            telemetry.addData("count", count);
            telemetry.addData("loop", loop);
            telemetry.addData("location", location);
            telemetry.addData("alliance", alliance);
            telemetry.update();

            Driveforward(.15,0.1);
            sleep(300);
            stopdriving();


            //claw_servo.setPosition(0);




            while (count < 5 && opModeIsActive()) {
                if (tfod != null) {
                    // getUpdatedRecognitions() will return null if no new information is available since
                    // the last time that call was made.
                    List<Recognition> updatedRecognitions = tfod.getFreshRecognitions();
                    if (updatedRecognitions != null) {
                        telemetry.addData("# Object Detected", updatedRecognitions.size());

                        telemetry.update();

                        // step through the list of recognitions and display boundary info.
                        int i = 0;
                        for (Recognition recognition : updatedRecognitions) {
                            /* telemetry.addData(String.format("label (%d)", i), recognition.getLabel());
                            telemetry.addData(String.format("  left,top (%d)", i), "%.03f , %.03f",
                                    recognition.getLeft(), recognition.getTop());
                            telemetry.addData(String.format("  right,bottom (%d)", i), "%.03f , %.03f",
                                    recognition.getRight(), recognition.getBottom());
                            telemetry.update();

                           // needs a
                           */
                            i++;
                            loop++;
                            // check label to see if the camera now sees a anvil

                            if (count == 1 && (recognition.getLabel().equals("BAnvil") || recognition.getLabel().equals("RAnvil"))) {
                                zone = 1;
                                count = 50;


                            }
                            if (count == 2 && (recognition.getLabel().equals("BAnvil") || recognition.getLabel().equals("RAnvil"))) {
                                zone = 2;
                                count = 60;


                            }
                            if (count == 3 && (recognition.getLabel().equals("BAnvil") || recognition.getLabel().equals("RAnvil"))) {
                                zone = 3;
                                count = 70;


                            }
                            if (count == 4 && zone == 0 ) {

                                count=0;
                            }

                        }
                        telemetry.addData("arm position", armMotor.getCurrentPosition());
                        telemetry.addData("Zone", zone );
                        telemetry.addData("count", count);
                        telemetry.addData("loop", loop);

                        telemetry.update();

                    }
                    count++;
                    stopdriving();
                    double pow;
                    if (count == 2 && zone ==0) {
//                        strafeRight(.35, .7);
                        pow = 0.2* powerRatio;
                        turnRight(pow);
                        stopdriving();
                    }
                    if (count == 3 && zone ==0) {
//                        strafeLeft(.3, .71);
                        pow = 0.2* powerRatio;
                        turnLeft(pow);
                        stopdriving();
                    }
                    if (count == 3 && zone ==0) {
//                        strafeLeft(.3, .71);
                        pow = 0.2* powerRatio;
                        turnLeft(pow);
                        stopdriving();
                    }
                    if (count ==4 && zone == 0) {
//                        strafeRight(.3, .7);
                        pow = 0.2* powerRatio;
                        turnRight(pow);
                        count = 1;
                        stopdriving();
                    }

                    telemetry.addData("arm position", armMotor.getCurrentPosition());
                    telemetry.addData("Zone", zone);
                    telemetry.addData("count", count);
                    telemetry.addData("loop", loop);
                    telemetry.update();

                }


// Save more CPU resources when camera is no longer needed.
                //1visionPortal.close();


                // Banished code :(


                /**
                 * Add telemetry about TensorFlow Object Detection (TFOD) recognitions.
                 */
   /* private void telemetryTfod() {

        List<Recognition> currentRecognitions = tfod.getRecognitions();
        telemetry.addData("# Objects Detected", currentRecognitions.size());

        // Step through the list of recognitions and display info for each one.
        for (Recognition recognition : currentRecognitions) {
            double x = (recognition.getLeft() + recognition.getRight()) / 2 ;
            double y = (recognition.getTop()  + recognition.getBottom()) / 2 ;

            telemetry.addData(""," ");
            telemetry.addData("Image", "%s (%.0f %% Conf.)", recognition.getLabel(), recognition.getConfidence() * 100);
            telemetry.addData("- Position", "%.0f / %.0f", x, y);
            telemetry.addData("- Size", "%.0f x %.0f", recognition.getWidth(), recognition.getHeight());
        }   // end for() loop-


        }   // end method telemetryTfod()










/*    Example



        // Step through each leg of the path, ensuring that the Auto mode has not been stopped along the way

        // Step 1:  Drive forward for 3 seconds
        leftDrive.setPower(FORWARD_SPEED);
        rightDrive.setPower(FORWARD_SPEED);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 3.0)) {
            telemetry.addData("Path", "Leg 1: %4.1f S Elapsed", runtime.seconds());
            telemetry.update();
        }

        // Step 2:  Spin right for 1.3 seconds
        leftDrive.setPower(TURN_SPEED);
        rightDrive.setPower(-TURN_SPEED);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 1.3)) {
            telemetry.addData("Path", "Leg 2: %4.1f S Elapsed", runtime.seconds());
            telemetry.update();
        }

        // Step 3:  Drive Backward for 1 Second
        leftDrive.setPower(-FORWARD_SPEED);
        rightDrive.setPower(-FORWARD_SPEED);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 1.0)) {
            telemetry.addData("Path", "Leg 3: %4.1f S Elapsed", runtime.seconds());
            telemetry.update();
        }

        // Step 4:  Stop
        leftDrive.setPower(0);
        rightDrive.setPower(0);

        telemetry.addData("Path", "Complete");
        telemetry.update();
        sleep(1000);

needs a */
            }

            double pow;
            if (zone == 1) {
               // rotateArm(1);
                //Driveforward(.15);
                //place(0);
                //zone1();
                telemetry.addData("zone drive", zone);
                telemetry.addData("count", count);
                telemetry.addData("loop", loop);
//
                telemetry.update();

//                strafeLeft(.25,.7);
                stopdriving();
                PlacePixel();
                sleep(300);
                stopdriving();
                pow = 0.21* powerRatio;
                Driveforward(pow,.4);
                stopdriving();
                open();
                sleep(2000);


             }

            if (zone ==2 ) {
                telemetry.addData("zone drive", zone);
                telemetry.addData("count", count);
                telemetry.addData("loop", loop);
//
                telemetry.update();

                pow = 0.2*powerRatio;
                turnLeft(pow);
                stopdriving();
                PlacePixel();
                sleep(300);
                stopdriving();
                pow=.22*powerRatio;
                Driveforward(pow,0.25);
                stopdriving();
                pow=0.33*powerRatio;
                turnRight(pow);
                stopdriving();
                open();
                sleep(2000);

//                turnLeft(0.2);
//                Driveforward(.15,1.3);
//                PlacePixel();
//                turnRight(0.4);
//                Driveforward(-0.12, 0.2);
//                stopdriving();
//                open();
//                sleep(2000);


            }
            if (zone == 3) {
                telemetry.addData("zone drive", zone);
                telemetry.addData("count", count);
                telemetry.addData("loop", loop);
                telemetry.update();

                pow=0.2*powerRatio;
                turnRight(pow);
                stopdriving();
                PlacePixel();
                sleep(300);
                stopdriving();
                pow=0.23*powerRatio;
                Driveforward(pow,0.25);
                stopdriving();
                pow=0.32*powerRatio;
                turnLeft(pow);
                stopdriving();
                open();
                sleep(2000);
//                turnRight(0.115);
//                PlacePixel();
//                sleep(100);
//                Driveforward(.18,1);
//                turnLeft(0.32);
//                stopdriving();
//                open();
//                sleep(2000);
            }
            telemetry.addData("zone drive", zone);
            telemetry.addData("count", count);
            telemetry.addData("loop", loop);
           // telemetry.addData("Voltage:", voltageshow);

            telemetry.update();
            if (alliance=="red" && location=="near") {
                if (zone ==1) {
                    pow=powerRatio*.7;
                    strafeRight(pow,.4);
                }
                if (zone ==2) {
                    pow=-0.33*powerRatio;
                    turnRight(pow);
                    pow=.22*powerRatio;
                    Driveforward(pow,0.25);
                    pow=powerRatio*.7;
                    strafeRight(pow,.4);
                }
                if (zone==3) {
                    pow=-0.32*powerRatio;
                    turnLeft(pow);
                    pow=-0.23*powerRatio;
                    Driveforward(pow,0.25);
                    stopdriving();
                    pow=powerRatio*.7;
                    strafeRight(pow,.4);
                }
            }
            if (alliance=="blue" && location=="near") {
                if (zone ==1) {
                    pow=powerRatio*.7;
                    strafeLeft(pow,.4);
                }
                if (zone ==2) {
                    pow=-0.33*powerRatio;
                    turnRight(pow);
                    pow=-.22*powerRatio;
                    Driveforward(pow,0.25);
                    pow=powerRatio*.7;
                    strafeLeft(pow,.4);
                }
                if (zone==3) {
                    pow=-0.32*powerRatio;
                    turnLeft(pow);
                    pow=-0.23*powerRatio;
                    Driveforward(pow,0.25);
                    stopdriving();
                    pow=powerRatio*.7;
                    strafeLeft(pow,.4);
                }
            }


        }
    }


}

// Zone 1 code (if I mess something up)
//strafeLeft(.18,.2);
//                armMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
//                        //DcMotorEx.RunMode mode = DcMotorEx.RunMode.RUN_USING_ENCODER;
////                armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
////                armMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//                        int curpos = armMotor.getCurrentPosition();
//                        int targpos = 1950;
//                        armMotor.setTargetPosition(targpos);
//                        armMotor.setPower(0.85);
//                        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
////        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
////        armMotor.setTargetPosition((int)qutarturn);
////        armMotor.setPower(.01);
////        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//
//                        while (armMotor.isBusy()){
//                        telemetry.addData("arm", armMotor.getCurrentPosition());
//                        telemetry.addData("target and curpos", targpos);
//                        telemetry.update();
//                        }
//                        armMotor.setPower(0.0);
//                        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//
//                        Driveforward(.2,.4);
//                        stopdriving();
//                        open();
//        sleep(1000);
//
//
//
//        telemetry.addData("zone drive", zone);
//        telemetry.addData("count", count);
//        telemetry.addData("loop", loop);
//
//        telemetry.update();