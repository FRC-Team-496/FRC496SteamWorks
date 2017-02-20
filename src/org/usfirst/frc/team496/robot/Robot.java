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
	SendableChooser<String> chooser = new SendableChooser<>();
	AHRS ahrs;
	Encoder enc1, enc2, enc3, enc4;
	PowerDistributionPanel pdp;

	Relay lightSwitch;

	PIDController turnController;

	double rotateToAngleRate;
	boolean changed;
	static final double kP = 0.04;
	static final double kI = 0.00;
	static final double kD = 0.00;
	static final double kF = 0.00;

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
		lightSwitch = new Relay(1);


		
		//lift camera
		UsbCamera camera2 = CameraServer.getInstance().startAutomaticCapture(1);
		camera2.setResolution(160,120);
	
		//peg camera
		UsbCamera camera = CameraServer.getInstance().startAutomaticCapture(0);
		camera.setResolution(160, 120);
		camera.setExposureAuto();
		//camera.setExposureManual(5);
		CvSource outputStream = CameraServer.getInstance().putVideo("Peg Vision", 480, 320);
		CvSink cvSink = CameraServer.getInstance().getVideo(camera);
		Mat source = new Mat();
		
		
		
		//Imgcodecs.imwrite("/home/lvuser/peg.jpg", source);
	
		
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
						targetDistance = r1.x - r.x;
						distance = 0.68 * 480 / (2 * targetDistance * 1.804);/// Math.tan(61degrees)
					}

				}

				outputStream.putFrame(source);

				/*
				 * 
				 * if(r1.area()>180){ targetDistance = r1.x - r.x; distance =
				 * 0.68 * IMG_WIDTH / (2 * targetDistance *
				 * .765);//Math.tan(37.4)); } else{ targetDistance = r1.x - r.x;
				 * distance = 90.0; }
				 */
				// targetDistance = r1.x - r.x;
				// distance = 0.68 * IMG_WIDTH / (2 * targetDistance *
				// .765);//Math.tan(37.4));

				// distance = 3;
				hasTarget = true;
				// SmartDashboard.putNumber("Area", r1.area());
				SmartDashboard.putNumber("CenterX", centerX);
				SmartDashboard.putNumber("Distance", distance);
				// System.out.println(centerX);

			} else {
				synchronized (imgLock) {
					hasTarget = false;

				}
				outputStream.putFrame(source);
			}
			Timer.delay(1.0/20.0);
			

		});
		pegVisionThread.setDaemon(true);
		pegVisionThread.start();

		LiveWindow.addActuator("DriveSystem", "RotateController", turnController);
		LiveWindow.addSensor("PowerSystem", "Current", pdp);

		LiveWindow.run();

	}

	@Override
	public void robotInit() {
		lightSwitch.set(Relay.Value.kForward);
		chooser.addDefault("Default Auto", defaultAuto);
		chooser.addObject("Left Station", leftStation);
		chooser.addObject("Center Station", centerStation);
		chooser.addObject("Right Staion", rightStation);
		SmartDashboard.putData("Auto modes", chooser);
	}

	@Override
	public void autonomous() {
		enc1.reset();
		enc2.reset();
		enc3.reset();
		enc4.reset();

		myRobot.setSafetyEnabled(false);

		final int DRIVE_FORWARD = 1;
		final int ROTATE_TO_TARGET = 2;
		final int DRIVE_TO_TARGET = 3;
		final int BACKUP = 4;
		final int TURN_TO_BASELINE = 5;
		final int DRIVE_TO_BASELINE = 6;
		final int END = 7;
		ahrs.reset();
		turnController.setSetpoint(0.0f);
		int selection = DRIVE_FORWARD;

		while (isAutonomous() && isEnabled()) {
			//System.out.println(selection);
			String autoMode = chooser.getSelected();
			switch (autoMode) {
			case rightStation:
				switch (selection) {
				case DRIVE_FORWARD:
					double distance = Math.abs(enc1.getDistance());
					turnController.enable();
					if (distance < 93) {
						double currentRotationRate = rotateToAngleRate;
						myRobot.mecanumDrive_Cartesian(0, -0.6, currentRotationRate, ahrs.getAngle());
					} else {
						ahrs.reset();

						selection = ROTATE_TO_TARGET;
					}
					break;

				case ROTATE_TO_TARGET:
					//System.out.println(ahrs.getAngle());

					if (ahrs.getAngle() < 45) {
						myRobot.mecanumDrive_Cartesian(0, 0.0, 0.4, ahrs.getAngle());
					} else {
						turnController.setSetpoint(0.0f);
						ahrs.reset();
						enc1.reset();
						selection = DRIVE_TO_TARGET;
					}
					break;
				case DRIVE_TO_TARGET:
					distance = Math.abs(enc1.getDistance());
					
					turnController.enable();
					if (distance < 24) {
						double currentRotationRate = rotateToAngleRate;
						myRobot.mecanumDrive_Cartesian(0, -0.6, currentRotationRate, ahrs.getAngle());
					} else {
						myRobot.mecanumDrive_Cartesian(0, 0, 0, ahrs.getAngle());

						ahrs.reset();
						Timer.delay(5);
						enc1.reset();
						selection = BACKUP;
					}
					break;
				case BACKUP:
					distance = Math.abs(enc1.getDistance());
					if (distance < 24) {
						double currentRotationRate = rotateToAngleRate;
						myRobot.mecanumDrive_Cartesian(0, 0.6, currentRotationRate, ahrs.getAngle());
					} else {
						ahrs.reset();
						enc1.reset();
						selection = TURN_TO_BASELINE;
					}
					break;
				case TURN_TO_BASELINE:
					System.out.println("TURNNING TO BASELINE");
					System.out.println(ahrs.getAngle());
					if (ahrs.getAngle() < 135) {
						myRobot.mecanumDrive_Cartesian(0, 0.0, 0.4, ahrs.getAngle());
					} else {
						turnController.setSetpoint(0.0f);
						ahrs.reset();
						enc1.reset();
						selection = DRIVE_TO_BASELINE;
					}
					break;
				case DRIVE_TO_BASELINE:
					System.out.println("DRIVING TO BASELINE");
					 distance = Math.abs(enc1.getDistance());
					turnController.enable();
					if (distance < 50) {
						double currentRotationRate = rotateToAngleRate;
						myRobot.mecanumDrive_Cartesian(0, -0.6, currentRotationRate, ahrs.getAngle());
					} else {
						ahrs.reset();

						selection = END;
					}
					break;
				case END:
					myRobot.mecanumDrive_Cartesian(0, 0.6, 0, ahrs.getAngle());
					break;
				}
				break;
			case centerStation:
				break;
			case leftStation:
				switch (selection) {
				case DRIVE_FORWARD:
					double distance = Math.abs(enc1.getDistance());
					turnController.enable();
					if (distance < 93) {
						double currentRotationRate = rotateToAngleRate;
						myRobot.mecanumDrive_Cartesian(0, -0.6, currentRotationRate, ahrs.getAngle());
					} else {
						ahrs.reset();

						selection = ROTATE_TO_TARGET;
					}
					break;

				case ROTATE_TO_TARGET:
					System.out.println(ahrs.getAngle());

					if (ahrs.getAngle() < -45) {
						myRobot.mecanumDrive_Cartesian(0, 0.0, -0.4, ahrs.getAngle());
					} else {
						turnController.setSetpoint(0.0f);
						ahrs.reset();
						enc1.reset();
						selection = DRIVE_TO_TARGET;
					}
					break;
				case DRIVE_TO_TARGET:
					distance = Math.abs(enc1.getDistance());
					turnController.enable();
					if (distance < 12) {
						double currentRotationRate = rotateToAngleRate;
						myRobot.mecanumDrive_Cartesian(0, -0.6, currentRotationRate, ahrs.getAngle());
					} else {
						ahrs.reset();

						selection = END;
					}
					break;
				case END:
					myRobot.mecanumDrive_Cartesian(0, 0.0, 0, ahrs.getAngle());
					break;
				}
				break;
			default:
				myRobot.mecanumDrive_Cartesian(0, 0.0, 0, ahrs.getAngle());

				break;
			}
			Timer.delay(0.05);
		}

	}

	@Override
	public void operatorControl() {
		boolean lightOn = false;
		myRobot.setSafetyEnabled(true);
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

			boolean rotateToAngle = false;
			if (xbox.getAButton() == true) {
				ahrs.reset();
			}
			if (opXbox.getRawButton(3)) {
				climbingMotor.set(1.0);
			} else if (opXbox.getRawButton(4)) {
				climbingMotor.set(-1.0);
			} else {
				climbingMotor.set(0.0);
			}

			double currentRotationRate;
			if (rotateToAngle) {
				turnController.enable();
				currentRotationRate = rotateToAngleRate;
			} else {
				turnController.disable();
				currentRotationRate = (xbox.getX(Hand.kLeft));
			}

			if (currentRotationRate == 0 && changed) {
				turnController.enable();
				turnController.setSetpoint(ahrs.getAngle());
				currentRotationRate = rotateToAngleRate;
				changed = !changed;
			} else {
				changed = true;
			}

			/*
			 * if(xbox.getX(Hand.kLeft)!=0){ changed = true;
			 * turnController.disable();
			 * turnController.setSetpoint(ahrs.getAngle()); currentRotationRate
			 * = xbox.getX(Hand.kLeft); }else{ turnController.enable();
			 * currentRotationRate = rotateToAngleRate; }
			 */

			/*
			 * if (xbox.getPOV() == 0) { myRobot.mecanumDrive_Cartesian(0, -1,
			 * currentRotationRate, 0); } else if (xbox.getPOV() == 45) {
			 * myRobot.mecanumDrive_Cartesian(1, -1, currentRotationRate, 0); }
			 * else if (xbox.getPOV() == 90) { myRobot.mecanumDrive_Cartesian(0,
			 * 1, currentRotationRate, 0); } else if (xbox.getPOV() == 135) {
			 * myRobot.mecanumDrive_Cartesian(-1, -1, currentRotationRate, 0); }
			 * else if (xbox.getPOV() == 180) {
			 * myRobot.mecanumDrive_Cartesian(0, 1, currentRotationRate, 0); }
			 * else if (xbox.getPOV() == 225) {
			 * myRobot.mecanumDrive_Cartesian(-1, 1, currentRotationRate, 0); }
			 * else if (xbox.getPOV() == 270) {
			 * myRobot.mecanumDrive_Cartesian(1, 0, currentRotationRate, 0); }
			 * else if (xbox.getPOV() == 315) {
			 * myRobot.mecanumDrive_Cartesian(1, 1, currentRotationRate, 0); }
			 */

			if (xbox.getRawButton(6) && (Timer.getFPGATimestamp() - lastModeSwitchTime) > .3) {
				xMultiplier *= -1;
				yMultiplier *= -1;
				lastModeSwitchTime = Timer.getFPGATimestamp();
			} else if (xbox.getRawButton(6)) {
				lastModeSwitchTime = Timer.getFPGATimestamp();
			}

			if (xbox.getX(Hand.kRight) != 0 || xbox.getY(Hand.kRight) != 0 || xbox.getX(Hand.kLeft) != 0) {
				myRobot.mecanumDrive_Cartesian(xMultiplier * xbox.getX(Hand.kRight),
						yMultiplier * xbox.getY(Hand.kRight), currentRotationRate, 0);
			}
			Timer.delay(.05);

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
