package com.drone_backend.services;

import org.springframework.beans.factory.annotation.Autowired;
import org.springframework.stereotype.Service;

import java.nio.file.FileSystems;
import java.nio.file.Path;
import java.util.ArrayList;

@Service
public class MissionService {

    @Autowired
    CommandLineService commandLineService;

    public boolean uploadMissionService(ArrayList<Integer> positions, double linearVelocity,
                                     double angularVelocity){
        // Create the command string
        Path currentPath = FileSystems.getDefault().getPath(".").toAbsolutePath();
        String scriptPath = currentPath.toString().substring(0, currentPath.toString().length()-15);
        scriptPath += "python_control_scripts";
        scriptPath += "/my_mission_script.py";
        String cmdStr = "python3 " + scriptPath;
        // add --positions args
        String posStr = " --positions";
        for(int i = 0; i < positions.size(); i++){ //note that order of position elements matters
            posStr += " " + String.valueOf(positions.get(i));
        }
        //add --linear_velocity args
        String linVelStr = " --linear_velocity";
        linVelStr += " " + String.valueOf(linearVelocity);
        //add --angular_velocity args
        String angVelStr = " --angular_velocity";
        angVelStr += " " + String.valueOf(angularVelocity);

        cmdStr += posStr + linVelStr + angVelStr;

        // Call the script from the command line
        return commandLineService.runOnCommandLine(cmdStr);
    }
}
