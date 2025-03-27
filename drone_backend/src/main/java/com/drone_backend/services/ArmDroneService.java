package com.drone_backend.services;

import org.springframework.beans.factory.annotation.Autowired;
import org.springframework.stereotype.Service;

import java.nio.file.FileSystems;
import java.nio.file.Path;

@Service
public class ArmDroneService {

    @Autowired
    CommandLineService commandLineService;

    public boolean armDrone(int armStatus){
        // Create the command string
        Path projectRoot = FileSystems.getDefault().getPath("").toAbsolutePath().getParent(); // go up one level
        Path scriptPath = projectRoot.resolve("python_control_scripts/my_arm_script.py");
        String cmdStr = "python3 " + scriptPath + " --arm_status " + armStatus;



        // Call the script from the command line

        return commandLineService.runOnCommandLine(cmdStr);
    }
}
