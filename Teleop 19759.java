

 import com.qualcomm.robotcore.eventloop.opmode.Disabled;
 import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
 import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
 import com.qualcomm.robotcore.hardware.Servo;
 import com.qualcomm.robotcore.hardware.IMU;
 import com.qualcomm.robotcore.hardware.DcMotorSimple;
 import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
 import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
 import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
 import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
 import com.qualcomm.robotcore.hardware.DcMotor;
 import com.qualcomm.robotcore.util.ElapsedTime;
 import com.qualcomm.robotcore.util.Range;
 
 @TeleOp(name="TeleOp-Reset", group="Linear OpMode")
 
 public class TeleOpReset extends LinearOpMode {
 
     private DcMotor frontLeft;
     private DcMotor frontRight;
     private DcMotor backLeft;
     private DcMotor backRight;
     private IMU imu;
 
     private DcMotor leftSlide;
     private DcMotor rightSlide;
     
     private Servo hand;
     private Servo wrist;
     private Servo claw;
     
     private DcMotor arm;
     
 
     private final double MANUAL_SLIDE_POWER = (0,1]; // power while manual control
     private boolean isResetting = false; // reset mode 
     private final int RESET_OFFSET = (to be changed); // additional descent distance during reset
     private double kSpeed=(0,1];
     @Override
     public void runOpMode() throws InterruptedException {
         // motor initialization
         frontLeft = hardwareMap.get(DcMotor.class, "leftFront");//to be changed
         frontRight = hardwareMap.get(DcMotor.class, "rightFront");//to be changed
         backLeft = hardwareMap.get(DcMotor.class, "leftBack");//to be changed
         backRight = hardwareMap.get(DcMotor.class, "rightBack");//to be changed
         arm = hardwareMap.get(DcMotor.class,"bigArm");//to be changed
 
         // set the motors'direction
         frontLeft.setDirection(DcMotor.Direction.REVERSE);
         backLeft.setDirection(DcMotor.Direction.REVERSE);
         frontRight.setDirection(DcMotor.Direction.FORWARD);
         backRight.setDirection(DcMotor.Direction.FORWARD);
         arm.setDirection(DcMotor.Direction.REVERSE);
         // set the brakig mode of the chassis motor
         frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
         frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
         backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
         backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
         arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
 
         // IMU initialization
         imu = hardwareMap.get(IMU.class, "imu");
         IMU.Parameters parameters = new IMU.Parameters(
             new RevHubOrientationOnRobot(
                 RevHubOrientationOnRobot.LogoFacingDirection.UP,
                 RevHubOrientationOnRobot.UsbFacingDirection.LEFT
             )
         );
         imu.initialize(parameters);
 
 
       // motor initialization
         leftSlide = hardwareMap.get(DcMotor.class, "lift2");
         rightSlide = hardwareMap.get(DcMotor.class, "lift1");
         
 
         // set the motor direction of the slide
         leftSlide.setDirection(DcMotorSimple.Direction.REVERSE);
         rightSlide.setDirection(DcMotorSimple.Direction.FORWARD);
 
         // set the motor mode of the slide
         leftSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
         rightSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
         leftSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
         rightSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
         
         // set the motor mode of the big-arm
         arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
         arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
 
         // set zero point power
         leftSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
         rightSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
         
         
         hand = hardwareMap.get(Servo.class, "hand");
         wrist = hardwareMap.get(Servo.class, "wrist");
         claw = hardwareMap.get(Servo.class, "claw");
        
         //finish initialization
         
         waitForStart();
 
         while (opModeIsActive()) {
             Chassis();
 
              // slide position
             if (gamepad2.dpad_up) {
                 setSlidePosition(position,speed);
                 setArmPosition(420,0.65);//example
                 hand.setPosition(0.9);//example
                 wrist.setPosition(0.5);//example
             } 
             else if (gamepad2.dpad_left) {
                 setSlidePosition(position,speed);
                 setArmPosition(position,speed);
                 hand.setPosition(position,speed);
                 wrist.setPosition(position,speed);
             }
             else if (gamepad2.dpad_right) {
                 setArmPosition(position,speed);
                 setSlidePosition(position,speed);
                 hand.setPosition(position,speed);
                 wrist.setPosition(position,speed);
             } 
            
             // manual slide controling settings (only in non-reset state)
             if (!isResetting) {
                 double slideControl = -gamepad2.right_stick_y;
                 if (Math.abs(slideControl) > 0.1) {
                     // manual speed controling mode
                     leftSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                     rightSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                     leftSlide.setPower(slideControl * MANUAL_SLIDE_POWER);
                     rightSlide.setPower(slideControl * MANUAL_SLIDE_POWER);
                 } else if (leftSlide.getMode() != DcMotor.RunMode.RUN_TO_POSITION) {
                     // stay in the current spot unless !in non-reset state!
                     int currentPosition = leftSlide.getCurrentPosition();
                     setSlidePosition(currentPosition,0.5);
                 }
             }
             
             // big arm manual controlings
             double armControl = gamepad2.left_stick_y;
             if (Math.abs(armControl) > 0.1) {
                 arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                 arm.setPower(armControl * 0.5); // 50% of power during manual arm control
             } else if (arm.getMode() != DcMotor.RunMode.RUN_TO_POSITION) {
                 int currentArmPosition = arm.getCurrentPosition();
                 arm.setTargetPosition(currentArmPosition);
                 arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                 arm.setPower(0.5);
             }
             
             //steering engine
             if(gamepad2.right_bumper){
                 claw.setPosition(0.3);
             }else if(gamepad2.left_bumper){
                 claw.setPosition(0.7);
                 
             }
             
             if(gamepad2.left_trigger>0.5){
                 wrist.setPosition(0.3);
             }else if(gamepad2.right_trigger>0.5){
                 wrist.setPosition(0.8);
                 
             }
             //gamepad2 movement 
             if(gamepad2.a){
                 setSlidePosition(position,speed);
                 setArmPosition(position,speed);
                 hand.setPosition(position,speed);
                 wrist.setPosition(position,speed);
                 
             }
             
             if(gamepad2.y){
                 setSlidePosition(5,1);//example
                 setArmPosition(650,0.5);
                 hand.setPosition(0.7);
                 wrist.setPosition(0.5);
                 
             }
                 
             // add data of slide totelemetry
             telemetry.addData("左滑轨位置", leftSlide.getCurrentPosition());
             telemetry.addData("右滑轨位置", rightSlide.getCurrentPosition());
             telemetry.addData("大臂位置", arm.getCurrentPosition());
             telemetry.addData("滑轨控制", "上键=高位, 右键=中位, 下键=低位, Y键=复位");
             telemetry.addData("手动控制", "使用右摇杆");
 
             telemetry.update();
         }
     }
 
    
     private void Chassis(){                                                                     
         // IMU reset
             if (gamepad1.a) {
                 imu.resetYaw();
                 telemetry.addData("IMU", "已重置方向");
                 telemetry.update();
             }
             
             if(gamepad1.right_bumper||rightSlide.getCurrentPosition()>300){
                 kSpeed=0.4;
             } else{
                 kSpeed=1;
             }
             double y = -gamepad1.left_stick_y;  // front/back
             double x = gamepad1.left_stick_x;   // left/right
             double rx = gamepad1.right_stick_x*0.8; // spinning
 
             // robot orientation
             double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
             // location calculating
             double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
             double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);
 
             //robot controler
             if (gamepad1.dpad_up) {
                 rotX = 0;
                 rotY = 0.9;
             } 
             else if (gamepad1.dpad_down) {
                 rotX = 0;
                 rotY = -0.9;
             } 
             else if (gamepad1.dpad_left) {
                 rotX = -0.9;
                 rotY = 0;
             } 
             else if (gamepad1.dpad_right) {
                 rotX = 0.9;
                 rotY = 0;
             }
             
 
            
             double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
             double frontLeftPower = kSpeed*(rotY + rotX + rx) / denominator;
             double backLeftPower = kSpeed*(rotY - rotX + rx) / denominator;
             double frontRightPower = kSpeed*(rotY - rotX - rx) / denominator;
             double backRightPower = kSpeed*(rotY + rotX - rx) / denominator;
 
            
             frontLeft.setPower(frontLeftPower);
             backLeft.setPower(backLeftPower);
             frontRight.setPower(frontRightPower);
             backRight.setPower(backRightPower);
 
             telemetry.addData("Front Left Power", frontLeftPower);
             telemetry.addData("Back Left Power", backLeftPower);
             telemetry.addData("Front Right Power", frontRightPower);
             telemetry.addData("Back Right Power", backRightPower);
             telemetry.addData("Robot Heading", Math.toDegrees(botHeading));
             telemetry.addData("重置方向", "按A键重置车头方向");
 
             }
         
     
    
     private void setSlidePosition(int position,double POWER) {
         
         leftSlide.setTargetPosition(position);
         rightSlide.setTargetPosition(position);
         
         
         leftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
         rightSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
         
        
         leftSlide.setPower(POWER);
         rightSlide.setPower(POWER);
     }
 
    
     private boolean isSlideBusy() {
         return leftSlide.isBusy() || rightSlide.isBusy();
     }
     
     private void setArmPosition(int position,double POWER) {
         
         arm.setTargetPosition(position);
         
         arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
 
         
         arm.setPower(POWER);
     }
 
     //is slide working
     private boolean isArmBusy() {
         return arm.isBusy();
     }
     
 }
 