package org.usfirst.frc.team496.robot;

import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgcodecs.Imgcodecs;
import org.opencv.imgproc.Imgproc;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.cscore.CvSink;
import edu.wpi.cscore.CvSource;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.Relay;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.RobotDrive.MotorType;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.SampleRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.Victor;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.vision.VisionThread;

public class Robot extends SampleRobot implements PIDOutput {
	RobotDrive myRobot = new RobotDrive(0, 1, 2, 3);

	Victor climbingMotor = new Victor(4);

	XboxController xbox = new XboxController(1);
	XboxController opXbox = new XboxController(0);

	final String defaultAuto = "Default";
	final String leftStation = "leftStation";
	final String centerStation = "centerStation";
	final String rightStation = "rightStation";
	final String centerStationWithStrafe = "centerStationWithStrafe";
	
	SendableChooser<String> chooser = new SendableChooser<>();
	AHRS ahrs;
	Encoder enc1, enc2, enc3, enc4;
	PowerDistributionPanel pdp;

	Relay lightSwitch;

	PIDController turnController;

	double rotateToAngleRate;
	boolean changed;
	static final double kP = 0.03;
	static final double kI = 0.005;
	static final double kD = 0.1;
	static final double kF = 0.00;

	static boolean justRotated = false;
	static double timeSinceLastRotation;

	static double timeStart = Timer.getFPGATimestamp();
	static double lastModeSwitchTime = timeStart;
	static int xMultiplier = -1;
	static int yMultiplier = 1;

	private VisionThread pegVisionThread;
	private double centerX = 0.0;
	private final Object imgLock = new Object();
	private boolean hasTarget;
	private double prevTurn;
	private double targetDistance;
	private double distance;

	/* This tuning parameter indicates how close to "on target" the */
	/* PID Controller will attempt to get. */

	static final double kToleranceDegrees = 2.0f;
	UsbCamera camera;

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
		lightSwitch = new Relay(1);

		// lift camera
		UsbCamera camera2 = CameraServer.getInstance().startAutomaticCapture(1);
		camera2.setResolution(160, 120);

		// peg camera
		camera = CameraServer.getInstance().startAutomaticCapture(0);
		camera.setResolution(160, 120);
		// camera.setExposureAuto();

		camera.setExposureManual(5);
		CvSource outputStream = CameraServer.getInstance().putVideo("Peg Vision", 480, 320);
		CvSink cvSink = CameraServer.getInstance().getVideo(camera);
		Mat source = new Mat();

		// Imgcodecs.imwrite("/home/lvuser/peg.jpg", source);

		pegVisionThread = new VisionThread(camera, new PegPipeline(), pipeline -> {
			cvSink.grabFrame(source);
			if (pipeline.filterContoursOutput().size() > 0) {
				synchronized (imgLock) {
					if (pipeline.filterContoursOutput().size() < 2) {
						Rect r = Imgproc.boundingRect(pipeline.filterContoursOutput().get(0));
						Imgproc.rectangle(source, new Point(r.x, r.y), new Point(r.x + r.width, r.y + r.height),
								new Scalar(0, 0, 255), 2);
						centerX = (r.x + r.width) / 2;
						distance = 0.167 * 480 / (2 * centerX * 1.804);/// Math.tan(61degrees)

					} else {
						Rect r = Imgproc.boundingRect(pipeline.filterContoursOutput().get(0));
						Imgproc.rectangle(source, new Point(r.x, r.y), new Point(r.x + r.width, r.y + r.height),
								new Scalar(0, 0, 255), 2);
						Rect r1 = Imgproc.boundingRect(pipeline.filterContoursOutput().get(1));

						Imgproc.rectangle(source, new Point(r1.x, r1.y), new Point(r1.x + r1.width, r1.y + r1.height),
								new Scalar(0, 0, 255), 2);
						// CHANGE THIS
						centerX = (r.x + (r1.x + r1.width)) / 2;
						// ABOVE THIS
						targetDistance = Math.abs(r1.x - r.x);
						distance = 0.68 * 480 / (2 * targetDistance * 1.804);/// Math.tan(61degrees)
					}

				}

				outputStream.putFrame(source);

				hasTarget = true;
				SmartDashboard.putNumber("CenterX", centerX);
				SmartDashboard.putNumber("Distance", distance);

			} else {
				synchronized (imgLock) {
					hasTarget = false;

				}
				outputStream.putFrame(source);
			}
			Timer.delay(1.0 / 20.0);

		});
		pegVisionThread.setDaemon(true);
		pegVisionThread.start();

		LiveWindow.addActuator("DriveSystem", "RotateController", turnController);
		LiveWindow.addSensor("PowerSystem", "Current", pdp);

		LiveWindow.run();

	}

	@Override
	public void robotInit() {
		chooser.addDefault("Default Auto", defaultAuto);
		chooser.addObject("Left Station", leftStation);
		chooser.addObject("Center Station", centerStation);
		chooser.addObject("Right Station", rightStation);
		//chooser.addObject("Center Station w strafe", centerStationWithStrafe);
		
		SmartDashboard.putData("Auto modes", chooser);
	}

	@Override
	public void autonomous() {
		camera.setExposureManual(5);
		enc1.reset();
		enc2.reset();
		enc3.reset();
		enc4.reset();
		SmartDashboard.putBoolean("past db", false);

		myRobot.setSafetyEnabled(false);

		final int DRIVE_FORWARD = 1;
		final int DRIVE_BACKWARD = 2;
		final int ROTATE_TO_TARGET = 3;
		final int DRIVE_TO_TARGET = 4;
		final int END = 5;

		final int DISTANCE_TO_LINE = 187;
		final int DISTANCE_TO_LR_PEG = 93;
		final int DISTANCE_TO_CENTER_PEG = 110;
		final int ROBOT_LENGTH = 34;
		final int AXEL_TO_BACK = 25;
		double timerSinceStrafe = 0;
		// final int ROBOT_WIDTH = 38;

		/*
		 * synchronized (imgLock) {
				centerX = this.centerX;
				distance = this.targetDistance;
				hasTarget = this.hasTarget;

				// System.out.println(centerX);
			}
			// double distance = Fft*FOVpixel/(2Tpixel*Math.tan(37.4)) ;
			// double distance = 0.68 * IMG_WIDTH / (2 * targetDistance *
			// Math.tan(37.4));
			// double distance = 142.304 * 1/targetDistance;
			distance = distance - 2;

			if (hasTarget) {

				double turn = centerX - (IMG_WIDTH / 2);
				prevTurn = turn;
				if (centerX == 0.0) {
					turnRate = 0.0;
				} else {
					turnRate = -turn * 0.008;
				}

				myRobot.arcadeDrive(-0.8, turnRate);
			} else if (distance >= 0) {
				myRobot.arcadeDrive(0, -prevTurn * .008);
			} else {
				myRobot.arcadeDrive(0, 0);
			}
		
		
		*/
		ahrs.reset();
		turnController.reset();
		turnController.setSetpoint(ahrs.getAngle());
		turnController.enable();

		int selection = DRIVE_FORWARD;
		while (isAutonomous() && isEnabled()) {
			String autoMode = chooser.getSelected();
			switch (autoMode) {

			case rightStation:
				switch (selection) {
				case DRIVE_FORWARD:
					double distance = Math.abs(enc1.getDistance());
					if (distance < DISTANCE_TO_LINE - ROBOT_LENGTH + AXEL_TO_BACK) {
						double currentRotationRate = rotateToAngleRate;
						myRobot.mecanumDrive_Cartesian(0, -1*((DISTANCE_TO_LINE - ROBOT_LENGTH + AXEL_TO_BACK) - distance)*.002 - .6, currentRotationRate, 0);
					} else {
						ahrs.reset();
						turnController.enable();
						turnController.setSetpoint(0.0f);
						enc1.reset();
						selection = DRIVE_BACKWARD;
						myRobot.mecanumDrive_Cartesian(0, 0, 0, 0);
						// Timer.delay(5);
					}
					break;
				case DRIVE_BACKWARD:
					distance = Math.abs(enc1.getDistance());
					System.out.println(distance);
					if (distance < Math.abs(DISTANCE_TO_LINE - DISTANCE_TO_LR_PEG)) {

						double currentRotationRate = rotateToAngleRate;
						myRobot.mecanumDrive_Cartesian(0, .004*(DISTANCE_TO_LINE - DISTANCE_TO_LR_PEG - distance) + .6, currentRotationRate, 0);
					} else {
						ahrs.reset();
						turnController.setSetpoint(-30.0f);
						turnController.enable();
						selection = ROTATE_TO_TARGET;
						Timer.delay(.2);

					}
					break;
				case ROTATE_TO_TARGET:
					lightSwitch.set(Relay.Value.kForward);
					System.out.println(ahrs.getAngle());

					double currentRotationRate = rotateToAngleRate;

					System.out.println(currentRotationRate);
					if (ahrs.getAngle() < -33 || ahrs.getAngle() > -27) {
						myRobot.mecanumDrive_Cartesian(0.0, 0.0, currentRotationRate / 2, 0.0);
					} else {
						turnController.setSetpoint(0.0f);

						ahrs.reset();
						enc1.reset();
						selection = DRIVE_TO_TARGET;
					}
					break;
				/*** TODO NEED TO ADD VISION ***/
				case DRIVE_TO_TARGET:
					//distance = Math.abs(enc1.getDistance());
					turnController.enable();
					synchronized (imgLock) {
						centerX = this.centerX;
						distance = this.targetDistance;
						hasTarget = this.hasTarget;

						// System.out.println(centerX);
					}
//					if(centerX )
					if (distance >=0/*distance < 12*/) {
						currentRotationRate = rotateToAngleRate;
						myRobot.mecanumDrive_Cartesian(0, -0.4, currentRotationRate, 0);
					} else {
						// ahrs.reset();

						selection = END;
					}
					break;
				case END:
					myRobot.mecanumDrive_Cartesian(0, 0.0, 0, 0);
					ahrs.reset();
					//lightSwitch.set(Relay.Value.kOff);

					break;
				}
				break;
			case centerStation:
				switch (selection) {
				case DRIVE_FORWARD:
					ahrs.reset();
					enc1.reset();
					selection = DRIVE_TO_TARGET;
					break;
				case DRIVE_TO_TARGET:
					lightSwitch.set(Relay.Value.kForward);
					turnController.enable();
					/*synchronized (imgLock) {
						centerX = this.centerX;
						distance = this.targetDistance;
						hasTarget = this.hasTarget;

						// System.out.println(centerX);
					}
//					if(centerX )
					if (distance >=1.5/*distance < 12) {
						double currentRotationRate = rotateToAngleRate;
						myRobot.mecanumDrive_Cartesian(0, -0.4, currentRotationRate, 0);
					} else {
						// ahrs.reset();

						selection = END;
					}
					break;*/
					double distance = Math.abs(enc1.getDistance());
					turnController.enable();
					if (distance < DISTANCE_TO_CENTER_PEG - AXEL_TO_BACK) {
						double currentRotationRate = rotateToAngleRate;
						myRobot.mecanumDrive_Cartesian(0, -1*(DISTANCE_TO_CENTER_PEG - AXEL_TO_BACK - distance)*.01 -.2, currentRotationRate, 0);
					} else { // ahrs.reset();

						selection = END;
					}
					break;
				case END:
					myRobot.mecanumDrive_Cartesian(0, 0.0, 0, 0);
					ahrs.reset();
					//lightSwitch.set(Relay.Value.kOff);

					break;
				}
				break;
				
/*			case centerStationWithStrafe:
				switch (selection) {
				case DRIVE_FORWARD:
					ahrs.reset();
					enc1.reset();
					selection = DRIVE_TO_TARGET;
					break;
				case DRIVE_TO_TARGET:
					lightSwitch.set(Relay.Value.kForward);
					double distance = Math.abs(enc1.getDistance());
					turnController.enable();
					if (distance < DISTANCE_TO_CENTER_PEG - AXEL_TO_BACK - 3) {
						double currentRotationRate = rotateToAngleRate;
						myRobot.mecanumDrive_Cartesian(0, -1*(DISTANCE_TO_CENTER_PEG - AXEL_TO_BACK - distance)*.01 -.2, currentRotationRate, 0);
					} else { // ahrs.reset();
						ahrs.reset();
						enc1.reset();
						turnController.setSetpoint(0.0f);
						turnController.enable();
						selection = DRIVE_BACKWARD;
						myRobot.mecanumDrive_Cartesian(0, 0, 0, 0);
						Timer.delay(3);
					}
					break;
				case DRIVE_BACKWARD:
					distance = Math.abs(enc1.getDistance());
					System.out.println(distance);
					if (distance < Math.abs(12)) {

						double currentRotationRate = rotateToAngleRate;
						myRobot.mecanumDrive_Cartesian(0, .4, currentRotationRate, 0);
					} else {
						ahrs.reset();
						turnController.reset();
						turnController.setSetpoint(0.0f);
						turnController.enable();
						selection = ROTATE_TO_TARGET;
						timerSinceStrafe = Timer.getFPGATimestamp();

					}
					break;
				case ROTATE_TO_TARGET:
					if (Math.abs(Timer.getFPGATimestamp() - timerSinceStrafe) <= .5) {

						double currentRotationRate = rotateToAngleRate;
						myRobot.mecanumDrive_Cartesian(0.3, 0, currentRotationRate, 0);
					} else {
						ahrs.reset();
						turnController.setSetpoint(0.0f);
						turnController.enable();
						enc1.reset();
						selection = END;

					}
					break;
					
				case END:
					distance = Math.abs(enc1.getDistance());
					System.out.println(distance);
					if (distance < Math.abs(16)) {

						double currentRotationRate = rotateToAngleRate;
						myRobot.mecanumDrive_Cartesian(0, -.5, currentRotationRate, 0);
					} else {
						ahrs.reset();
						turnController.setSetpoint(0.0f);
						turnController.enable();
						myRobot.mecanumDrive_Cartesian(0, 0, 0, 0);

					}
					break;
				}
				break;

*/
			case leftStation:
				// System.out.println(selection);

				switch (selection) {
				case DRIVE_FORWARD:
					double distance = Math.abs(enc1.getDistance());
					if (distance < DISTANCE_TO_LINE - ROBOT_LENGTH + AXEL_TO_BACK) {
						double currentRotationRate = rotateToAngleRate;
						myRobot.mecanumDrive_Cartesian(0, -1*((DISTANCE_TO_LINE - ROBOT_LENGTH + AXEL_TO_BACK) - distance)*.004 - .4, currentRotationRate, 0);
					} else {
						ahrs.reset();
						turnController.enable();
						turnController.setSetpoint(0.0f);
						enc1.reset();
						selection = DRIVE_BACKWARD;
						myRobot.mecanumDrive_Cartesian(0, 0, 0, 0);
						// Timer.delay(5);
					}
					break;
				case DRIVE_BACKWARD:
					distance = Math.abs(enc1.getDistance());
					System.out.println(distance);
					if (distance < Math.abs(DISTANCE_TO_LINE - DISTANCE_TO_LR_PEG)) {

						double currentRotationRate = rotateToAngleRate;
						myRobot.mecanumDrive_Cartesian(0, .006*(DISTANCE_TO_LINE - DISTANCE_TO_LR_PEG - distance) + .4, currentRotationRate, 0);
					} else {
						ahrs.reset();
						turnController.setSetpoint(30.0f);
						turnController.enable();
						selection = ROTATE_TO_TARGET;

					}
					break;
				case ROTATE_TO_TARGET:
					lightSwitch.set(Relay.Value.kForward);
					System.out.println(ahrs.getAngle());

					double currentRotationRate = rotateToAngleRate;

					System.out.println(currentRotationRate);
					if (ahrs.getAngle() > 33 || ahrs.getAngle() < 27) {
						myRobot.mecanumDrive_Cartesian(0.0, 0.0, currentRotationRate / 2, 0.0);
					} else {
						turnController.setSetpoint(0.0f);

						ahrs.reset();
						enc1.reset();
						selection = DRIVE_TO_TARGET;
					}
					break;
				/*** TODO NEED TO ADD VISION ***/
				case DRIVE_TO_TARGET:
					distance = Math.abs(enc1.getDistance());
					turnController.enable();
					if (distance >=1000/*distance < 12*/) {
						currentRotationRate = rotateToAngleRate;
						myRobot.mecanumDrive_Cartesian(0, -0.3, currentRotationRate, 0);
					} else {
						// ahrs.reset();

						selection = END;
					}
					break;
				case END:
					myRobot.mecanumDrive_Cartesian(0, 0.0, 0, 0);
					ahrs.reset();
					lightSwitch.set(Relay.Value.kOff);

					break;
				}
				break;
			default:
				myRobot.mecanumDrive_Cartesian(0, 0.0, 0, 0);
				break;
			}
		}

		Timer.delay(.05);
	}

	@Override
	public void operatorControl() {
		boolean lightOn = false;
		myRobot.setSafetyEnabled(true);
		ahrs.reset();
		turnController.setSetpoint(0.0f);
		camera.setExposureAuto();
		turnController.disable();

		while (isOperatorControl() && isEnabled()) {

			if (opXbox.getAButton()) {
				lightOn = true;
			} else if (opXbox.getBButton()) {
				lightOn = false;
			}
			if (lightOn) {
				lightSwitch.set(Relay.Value.kForward);
			} else {
				lightSwitch.set(Relay.Value.kOff);
			}

			if (opXbox.getRawButton(3)) {
				climbingMotor.set(1.0);
			} else if (opXbox.getRawButton(4)) {
				climbingMotor.set(-1.0);
			} else {
				climbingMotor.set(0.0);
			}

			if (xbox.getAButton() == true) {
				ahrs.reset();
				turnController.setSetpoint(0.0f);
			}

			boolean change = true;
			double currentRotationRate;
			if (Math.abs(xbox.getX(Hand.kLeft)) < .1) {
				turnController.enable();
				currentRotationRate = rotateToAngleRate;
			} else {
				turnController.disable();
				currentRotationRate = (xbox.getX(Hand.kLeft));
				ahrs.reset();
				turnController.setSetpoint(0.0f);
				change = false;
				timeSinceLastRotation = Timer.getFPGATimestamp();

			}
			if (Timer.getFPGATimestamp() - timeSinceLastRotation <= 1 && Math.abs(xbox.getX(Hand.kLeft)) < .1) {
				turnController.setSetpoint(ahrs.getAngle());
				currentRotationRate = rotateToAngleRate;

			}

			if (xbox.getRawButton(6) && (Timer.getFPGATimestamp() - lastModeSwitchTime) > .5) {
				xMultiplier *= -1;
				yMultiplier *= -1;
				lastModeSwitchTime = Timer.getFPGATimestamp();
			} else if (xbox.getRawButton(6)) {
				lastModeSwitchTime = Timer.getFPGATimestamp();
			}

			if (xbox.getPOV() == 0) {
				ahrs.reset();
				turnController.setSetpoint(ahrs.getAngle());
				currentRotationRate = rotateToAngleRate;
			} else if (xbox.getPOV() == 90) {
				ahrs.reset();
				turnController.setSetpoint(90);
				currentRotationRate = rotateToAngleRate;
			} else if (xbox.getPOV() == 180) {
				ahrs.reset();
				turnController.setSetpoint(179.9);
				currentRotationRate = rotateToAngleRate;
			} else if (xbox.getPOV() == 270) {
				ahrs.reset();
				turnController.setSetpoint(-90);
				currentRotationRate = rotateToAngleRate;
			}

			if (Math.abs(xbox.getTriggerAxis(Hand.kLeft)) != 0) {
				myRobot.mecanumDrive_Cartesian(1, 0, currentRotationRate, 0);
			} else if (Math.abs(xbox.getTriggerAxis(Hand.kRight)) != 0) {
				myRobot.mecanumDrive_Cartesian(-1, 0, currentRotationRate, 0);

			}
			// CODE SEG BELOW IS FOR KICK AND MOVING TO COMPENSATE
			else if (xbox.getX(Hand.kRight) != 0 || xbox.getY(Hand.kRight) != 0 || xbox.getX(Hand.kLeft) != 0) {
				myRobot.mecanumDrive_Cartesian(xMultiplier * xbox.getX(Hand.kRight),
						yMultiplier * xbox.getY(Hand.kRight), currentRotationRate, 0);
			}
			/*
			 * else if (Math.abs(xbox.getX(Hand.kRight)) > .1 ||
			 * Math.abs(xbox.getY(Hand.kRight)) > .1 ||
			 * Math.abs(xbox.getX(Hand.kLeft)) > .1) {
			 * myRobot.mecanumDrive_Cartesian(xMultiplier *
			 * xbox.getX(Hand.kRight), yMultiplier * xbox.getY(Hand.kRight),
			 * currentRotationRate, 0); } /*else if(!change){
			 * turnController.setSetpoint(ahrs.getAngle()); }
			 */

			Timer.delay(.005);

		}

	}

	@Override
	public void test() {
		while (isTest() && isEnabled()) {
			LiveWindow.run();
		}

	}

	@Override
	public void pidWrite(double output) {
		rotateToAngleRate = output;
	}
}
