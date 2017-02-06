package org.usfirst.frc.team496.robot;

import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.RobotDrive.MotorType;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.SampleRobot;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.cscore.AxisCamera;
import edu.wpi.cscore.CvSink;
import edu.wpi.cscore.CvSource;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.opencv.core.*;
import org.opencv.imgproc.*;
import com.kauailabs.navx.frc.*;

public class Robot extends SampleRobot implements PIDOutput {
	RobotDrive myRobot = new RobotDrive(0, 1, 2, 3);

	Talon climbingMotor = new Talon(4);
	// Joystick stick = new Joystick(0);
	XboxController xbox = new XboxController(1);
	final String defaultAuto = "Default";
	final String customAuto = "My Auto";
	SendableChooser<String> chooser = new SendableChooser<>();
	AHRS ahrs;
	Encoder enc1, enc2, enc3, enc4;
	PowerDistributionPanel pdp;

	/*
	 * Thread visionThread = new Thread(() -> { // Get the Axis camera from
	 * CameraServer AxisCamera camera =
	 * CameraServer.getInstance().addAxisCamera("axis-camera.local"); // Set the
	 * resolution camera.setResolution(640, 480);
	 * 
	 * // Get a CvSink. This will capture Mats from the camera CvSink cvSink =
	 * CameraServer.getInstance().getVideo(); // Setup a CvSource. This will
	 * send images back to the Dashboard CvSource outputStream =
	 * CameraServer.getInstance().putVideo("Rectangle", 640, 480);
	 * 
	 * // Mats are very memory expensive. Lets reuse this Mat. Mat mat = new
	 * Mat();
	 * 
	 * // This cannot be 'true'. The program will never exit if it is. This //
	 * lets the robot stop this thread when restarting robot code or //
	 * deploying. while (!Thread.interrupted()) { // Tell the CvSink to grab a
	 * frame from the camera and put it // in the source mat. If there is an
	 * error notify the output. if (cvSink.grabFrame(mat) == 0) { // Send the
	 * output the error. outputStream.notifyError(cvSink.getError()); // skip
	 * the rest of the current iteration continue; } // Put a rectangle on the
	 * image Imgproc.rectangle(mat, new Point(270, 290), new Point(370, 190),
	 * new Scalar(0, 255, 75), 1); // Give the output stream a new image to
	 * display outputStream.putFrame(mat); } });
	 * 
	 */

	PIDController turnController;
	double rotateToAngleRate;

	static final double kP = 0.03;
	static final double kI = 0.00;
	static final double kD = 0.00;
	static final double kF = 0.00;

	/* This tuning parameter indicates how close to "on target" the */
	/* PID Controller will attempt to get. */

	static final double kToleranceDegrees = 1.0f;

	public Robot() {

		try {
			ahrs = new AHRS(SPI.Port.kMXP);
		} catch (RuntimeException ex) {
			DriverStation.reportError("Error instantiating navX-MXP:  " + ex.getMessage(), true);
		}
		pdp = new PowerDistributionPanel();
		enc1 = new Encoder(0, 1, false, Encoder.EncodingType.k4X);
		enc2 = new Encoder(2, 3, false, Encoder.EncodingType.k4X);
		enc3 = new Encoder(4, 5, false, Encoder.EncodingType.k4X);
		enc4 = new Encoder(6, 7, false, Encoder.EncodingType.k4X);
		double wheel = 6.0;
		double wheel1Ticks = 1082; // our measure was 328
		double circumfrence = Math.PI * wheel;
		double ticks = circumfrence / wheel1Ticks;
		enc1.setDistancePerPulse(ticks);
		myRobot.setExpiration(0.1);
		myRobot.setInvertedMotor(MotorType.kFrontLeft, true);
		myRobot.setInvertedMotor(MotorType.kRearLeft, true);

		turnController = new PIDController(kP, kI, kD, kF, ahrs, this);
		turnController.setInputRange(-180.0f, 180.0f);
		turnController.setOutputRange(-1.0, 1.0);
		turnController.setAbsoluteTolerance(kToleranceDegrees);
		turnController.setContinuous(true);

		LiveWindow.addActuator("DriveSystem", "RotateController", turnController);
		LiveWindow.addSensor("PowerSystem", "Current", pdp);

		LiveWindow.run();

	}

	@Override
	public void robotInit() {
		chooser.addDefault("Default Auto", defaultAuto);
		chooser.addObject("My Auto", customAuto);
		SmartDashboard.putData("Auto modes", chooser);
	}

	@Override
	public void autonomous() {
		enc1.reset();
		enc2.reset();
		enc3.reset();
		enc4.reset();

		myRobot.setSafetyEnabled(false);
		boolean step1 = false, step2 = false;
		while (isAutonomous() && isEnabled()) {
			// System.out.println(enc.getRaw());
			ahrs.reset();
			System.out.println(enc1.getDistance());
			turnController.setSetpoint(0.0f);
			double distance = enc1.getDistance();

			if (distance < 36 && !step1) {
				double currentRotationRate = rotateToAngleRate;
				myRobot.mecanumDrive_Cartesian(0, 0.6, currentRotationRate, ahrs.getAngle());
			} else {
				myRobot.mecanumDrive_Cartesian(0, 0, 0, ahrs.getAngle());
				step1 = true;
			}
			
		

		}

	}

	@Override
	public void operatorControl() {
		myRobot.setSafetyEnabled(true);
		while (isOperatorControl() && isEnabled()) {

			boolean rotateToAngle = false;
			if (xbox.getStartButton() == true) {
				ahrs.reset();
			}

			if (xbox.getTrigger(Hand.kRight)) {
				climbingMotor.set(1.0);
			} else if (xbox.getTrigger(Hand.kLeft)) {
				climbingMotor.set(-1.0);
			}

			else if (xbox.getPOV() <= 315 && xbox.getPOV() >= 225) {
				turnController.setSetpoint(0.0f);
				rotateToAngle = true;
			} else if (xbox.getPOV() <= 135 && xbox.getPOV() >= 45) {
				turnController.setSetpoint(90.0f);
				rotateToAngle = true;
			} else if (xbox.getPOV() == 315 || xbox.getPOV() == 0 || xbox.getPOV() == 45) {
				turnController.setSetpoint(179.9f);
				rotateToAngle = true;
			} else if (xbox.getPOV() <= 225 && xbox.getPOV() >= 135) {
				turnController.setSetpoint(-90.0f);
				rotateToAngle = true;
			}

			double currentRotationRate;
			if (rotateToAngle) {
				turnController.enable();
				currentRotationRate = rotateToAngleRate;
			} else {
				turnController.disable();
				currentRotationRate = xbox.getX(Hand.kLeft);
			}

			myRobot.mecanumDrive_Cartesian(-xbox.getX(Hand.kRight), xbox.getY(Hand.kRight), currentRotationRate,
					ahrs.getAngle());

		}

	}

	@Override
	public void test() {
		LiveWindow.run();
		System.out.println("Current of 14: " + pdp.getCurrent(14));
	}

	@Override
	public void pidWrite(double output) {
		rotateToAngleRate = output;
	}
}
