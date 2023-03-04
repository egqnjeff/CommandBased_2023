// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.board.RobotTab;
import frc.constants.OIConstants;
import frc.constants.OIConstants.GamePiece;
import frc.robot.subsystems.ArmProfiledPIDSubsystem;
import frc.robot.subsystems.ClawSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.GroundIntakeSubsystem;
import frc.robot.subsystems.LimeLightSubsystem;
import frc.robot.subsystems.SignalLEDSubsystem;
import frc.robot.subsystems.TelescopePIDSubsystem;
import frc.robot.subsystems.SignalLEDSubsystem.LedMode;
import frc.robot.subsystems.SignalLEDSubsystem.LedPreference;

/** Add your docs here. */
public class RobotShared {

    private static RobotShared instance;

    // The robot's subsystems and commands are defined here...
    protected final DriveSubsystem m_robotDrive = new DriveSubsystem();
    protected final LimeLightSubsystem m_limelight = new LimeLightSubsystem();
    //protected final PhotonVisionSubsystem m_photonVision = new PhotonVisionSubsystem();
    protected final ClawSubsystem m_claw = new ClawSubsystem();
    // protected final ArmPIDSubsystem m_arm = new ArmPIDSubsystem();
    protected final ArmProfiledPIDSubsystem m_armProfiled = new ArmProfiledPIDSubsystem();
    protected final TelescopePIDSubsystem m_telescope = new TelescopePIDSubsystem();
    protected final GroundIntakeSubsystem m_groundIntake = new GroundIntakeSubsystem();
    protected final SignalLEDSubsystem m_signalLEDs = new SignalLEDSubsystem();

    protected final CommandXboxController m_driverController = new CommandXboxController(OIConstants.kDriverControllerPort);
    protected final CommandXboxController m_codriverController = new CommandXboxController(OIConstants.kCoDriverControllerPort);

    private RobotTab m_robotTab;
    
    private RobotShared() {
        m_robotTab = RobotTab.getInstance();
    }

    public static RobotShared getInstance() {
        if(instance == null) {
            instance = new RobotShared();
        }
        return instance;
    }

    public DriveSubsystem getDriveSubsystem() {
        return m_robotDrive;
    }

    public LimeLightSubsystem getLimeLightSubsystem() {
        return m_limelight;
    }

    public ClawSubsystem getClawSubsystem() {
        return m_claw;
    }

    public ArmProfiledPIDSubsystem getArmProfiledPIDSubsystem() {
        return m_armProfiled;
    }

    public TelescopePIDSubsystem getTelescopePIDSubsystem() {
        return m_telescope;
    }

    public GroundIntakeSubsystem getGroundIntakeSubsystem() {
        return m_groundIntake;
    }

    public SignalLEDSubsystem getSignalLEDSubsystem() {
        return m_signalLEDs;
    }

    public CommandXboxController getDriverController() {
        return m_driverController;
    }

    public CommandXboxController getCodriverController() {
        return m_codriverController;
    }

    public RobotTab getRobotTab() {
        return m_robotTab;
    }

    public void toggleGamePiece() {
        GamePiece newGamePiece = m_robotTab.toggleGamePiece();
        setGamePiece(newGamePiece);
      }
    
    public void setGamePiece(OIConstants.GamePiece piece) {
        if (piece == OIConstants.GamePiece.CUBE) {
            m_signalLEDs.setMode(LedMode.CUBE, LedPreference.MAIN, false);
        } else if (piece == OIConstants.GamePiece.CONE) {
            m_signalLEDs.setMode(LedMode.CONE, LedPreference.MAIN, false);
        }
        m_groundIntake.setGamePiece(piece);
        m_claw.setGamePiece(piece);
    }
    
}
