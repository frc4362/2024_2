package com.gemsrobotics.frc2024;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class AutoSelector {

    private static final String kDefaultAuto = "Default";
    private static final String kOnePlusTwoPlusThree = "One Plus Two Plus Three";
    private final SendableChooser<String> m_chooser = new SendableChooser<>();

    public void initializeAutoSelector() {
        m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
        m_chooser.addOption("1 + 2 + 3", kOnePlusTwoPlusThree);
        SmartDashboard.putData("Auto Selector", m_chooser);
    }

    public String getSelectedAuto() {
         return m_chooser.getSelected();
    }
}
