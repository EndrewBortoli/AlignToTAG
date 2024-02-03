package frc.robot.subsystems;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.*;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShuffleBoard extends SubsystemBase {

        private final DriveSubsystem m_robotDrive = new DriveSubsystem();

        //------------------------Abas do ShuffleBoard------------------------\\
        final ShuffleboardTab actualYawTab = Shuffleboard.getTab("Teleoperado"); // actualyawTab no Teleoperado        //------------------------Atualização de dados do ShuffleBoard------------------------\\
        final GenericEntry yawEntry = actualYawTab
        .add("actualYaw", 0).getEntry();  // Cria uma janela com o angulo atual do robô


        //__________________________________________________Métodos do ShuffleBoard__________________________________________________\\


            //-------------Ângulo X do Robo-------------\\
        public void actualYaw() {  

                // var yawSignal = m_robotDrive.getHeading(); 
                // yawEntry.setDouble(yawSignal);
                
}
}
