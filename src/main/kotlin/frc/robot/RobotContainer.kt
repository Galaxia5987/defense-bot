package frc.robot

import com.ctre.phoenix6.configs.MotorOutputConfigs
import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.hardware.TalonFX
import com.ctre.phoenix6.signals.NeutralModeValue
import edu.wpi.first.math.MathUtil
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.units.Units.Volts
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.Commands.runOnce
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller
import frc.robot.lib.enableAutoLogOutputFor
import frc.robot.lib.getPose2d
import frc.robot.subsystems.Elevator
import frc.robot.subsystems.drive.DriveCommands
import frc.robot.subsystems.drive.alignToPose
import org.ironmaple.simulation.SimulatedArena
import org.littletonrobotics.junction.AutoLogOutput

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the [Robot] periodic methods (other than the scheduler
 * calls). Instead, the structure of the robot (including subsystems, commands,
 * and trigger mappings) should be declared here.
 */
object RobotContainer {
    private val gripperMotor = TalonFX(16)
    private val gripperConfig = TalonFXConfiguration().apply {
        MotorOutput = MotorOutputConfigs().withNeutralMode(NeutralModeValue.Brake)
    }


    private val driverController = CommandPS5Controller(0)

    private val swerveDrive = frc.robot.swerveDrive

    private val elevator = Elevator()

    private val ELEVATOR_HOLD_VOLTAGE = Volts.of(0.0)

    init {
        gripperMotor.configurator.apply(gripperConfig)

        configureButtonBindings()
        configureDefaultCommands()

        if (CURRENT_MODE == Mode.SIM && USE_MAPLE_SIM)
            SimulatedArena.getInstance().resetFieldForAuto()

        enableAutoLogOutputFor(this)
    }

    @AutoLogOutput(key = "MapleSimPose")
    private fun getMapleSimPose(): Pose2d? =
        driveSimulation?.simulatedDriveTrainPose

    private fun configureDefaultCommands() {
        swerveDrive.defaultCommand =
            DriveCommands.joystickDrive(
                swerveDrive,
                { driverController.leftY },
                { driverController.leftX },
                { -driverController.rightX * 0.8 }
            )
        elevator.defaultCommand =
            elevator.setPower {
                Volts.of(
                    MathUtil.applyDeadband(driverController.rightY, 0.15)
                ) * 4.0 + ELEVATOR_HOLD_VOLTAGE
            }
    }

    private fun stopGripper(): Command = runOnce({ gripperMotor.setVoltage(0.0) })


    private fun configureButtonBindings() {
        driverController
            .options()
            .onTrue(
                runOnce(
                    {
                        swerveDrive.resetGyroBasedOnAlliance(
                            Rotation2d.kZero
                        )
                    },
                    swerveDrive
                )
                    .ignoringDisable(true)
            )

        driverController
            .R1()
            .whileTrue(runOnce({ gripperMotor.setVoltage(4.0) }))
            .onFalse(stopGripper())
        driverController
            .L1()
            .whileTrue(runOnce({ gripperMotor.setVoltage(-3.0) }))
            .onFalse(stopGripper())
        driverController.circle().whileTrue(alignToPose(getPose2d(x = 1.0, y = 1.0)))
        driverController.square().whileTrue(runOnce ({ swerveDrive.resetOdometry(getPose2d(x = 0.0)) }))

    }

    fun getAutonomousCommand(): Command = Commands.none()
}
