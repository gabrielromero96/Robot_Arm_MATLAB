% Making DH parameters (theta d a alpha), dimensions in meters
L(1) = Link([0 0.2 0 pi/2]);
L(2) = Link([0 0 0.8 0]);
L(3) = Link([0 0 0.8 0]);
L(4) = Link([0 0 0.2 0]);

% Creating robot named "RobotArm"
RobotArm = SerialLink(L);

% Inserting angles to establish a home or resting position, then using...
%...forward kinematics to have the home coordinates
Angles_Home = ([90*pi/180 90*pi/180 -80*pi/180 -100*pi/180]);
Position_Home = RobotArm.fkine([90*pi/180 90*pi/180 -80*pi/180 -100*pi/180]);

% Creating homogeneous transformation matrix for the following: 
% 1.The side of the conveyor belt furthest from the robot.
% 2.The side of the conveyor belt nearest to the robot.
% 3.The "plastics" bin.
% 4.The "metals" bin.
% 5.The "glass" bin.
Position_ConveyorEnd = transl(0, 1.5, 0) * rpy2tr(0,90,0,'deg');
Position_ConveyorStart = transl(0, 0.5, 0) * rpy2tr(0,90,0,'deg');
Position_PlasticBin = transl(0.6, -0.1, 0.5) * rpy2tr(0,90,0,'deg');
Position_MetalBin = transl(0.8, -0.1, 0.6) * rpy2tr(0,90,0,'deg');
Position_GlassBin = transl(1.0, -0.1, 0.7) * rpy2tr(0,90,0,'deg');

% Creating homogeneous transformation matrix from the previous points...
% ...with 0.2m offset

Position_ConveyorEnd_Offset = transl(0, 1.5, 0.2) * rpy2tr(0,90,0,'deg');
Position_ConveyorStart_Offset = transl(0, 0.5, 0.2) * rpy2tr(0,90,0,'deg');
Position_PlasticBin_Offset = transl(0.6, -0.1, 0.7) * rpy2tr(0,90,0,'deg');
Position_MetalBin_Offset = transl(0.8, -0.1, 0.8) * rpy2tr(0,90,0,'deg');
Position_GlassBin_Offset = transl(1.0, -0.1, 0.9) * rpy2tr(0,90,0,'deg');

% Finding positions using inverse kinematics
AnglesSolution_ConveyorEnd = RobotArm.ikine(Position_ConveyorEnd, [90*pi/180 0 0 -90*pi/180], 'mask', [1 1 1 0 0 1]);
AnglesSolution_ConveyorStart = RobotArm.ikine(Position_ConveyorStart, [90*pi/180 0 0 -90*pi/180], 'mask', [1 1 1 0 0 1]);
AnglesSolution_PlasticBin = RobotArm.ikine(Position_PlasticBin, [-14.4*pi/180 100.3*pi/180 -119*pi/180 -71.3*pi/180], 'mask', [1 1 1 0 0 1]);
AnglesSolution_MetalBin = RobotArm.ikine(Position_MetalBin, [-14.4*pi/180 97*pi/180 -108.1*pi/180 -78.9*pi/180], 'mask', [1 1 1 0 0 1]);
AnglesSolution_GlassBin = RobotArm.ikine(Position_GlassBin, [-14.4*pi/180 86.3*pi/180 -90*pi/180 -86.3*pi/180], 'mask', [1 1 1 0 0 1]);

% Finding offset positions using inverse kinematics
AnglesSolution_ConveyorEnd_Offset = RobotArm.ikine(Position_ConveyorEnd_Offset, [90*pi/180 0 0 -90*pi/180], 'mask', [1 1 1 0 0 1]);
AnglesSolution_ConveyorStart_Offset = RobotArm.ikine(Position_ConveyorStart_Offset, [90*pi/180 0 0 -90*pi/180], 'mask', [1 1 1 0 0 1]);
AnglesSolution_PlasticBin_Offset = RobotArm.ikine(Position_PlasticBin_Offset, [-14.4*pi/180 100.3*pi/180 -119*pi/180 -71.3*pi/180], 'mask', [1 1 1 0 0 1]);
AnglesSolution_MetalBin_Offset = RobotArm.ikine(Position_MetalBin_Offset, [-14.4*pi/180 97*pi/180 -108.1*pi/180 -78.9*pi/180], 'mask', [1 1 1 0 0 1]);
AnglesSolution_GlassBin_Offset = RobotArm.ikine(Position_GlassBin_Offset, [-14.4*pi/180 86.3*pi/180 -90*pi/180 -86.3*pi/180], 'mask', [1 1 1 0 0 1]);

% Creating animated trajectories from the offset points to home and vice versa
TRAJ_OffsetConveyorEndToHome = jtraj(AnglesSolution_ConveyorEnd_Offset, Angles_Home, (0:.05:1));
TRAJ_OffsetConveyorStartToHome = jtraj(AnglesSolution_ConveyorStart_Offset, Angles_Home, (0:.05:1));
TRAJ_OffsetPlasticBinToHome = jtraj(AnglesSolution_PlasticBin_Offset , Angles_Home, (0:.05:1));
TRAJ_OffsetMetalBinToHome = jtraj(AnglesSolution_MetalBin_Offset , Angles_Home, (0:.05:1));
TRAJ_OffsetGlassBinToHome = jtraj(AnglesSolution_GlassBin_Offset , Angles_Home, (0:.05:1));

TRAJ_HomeToOffsetConveyorEnd = jtraj(Angles_Home, AnglesSolution_ConveyorEnd_Offset, (0:.05:1));
TRAJ_HomeToOffsetConveyorStart = jtraj(Angles_Home, AnglesSolution_ConveyorStart_Offset, (0:.05:1));
TRAJ_HomeToOffsetPlasticBin = jtraj(Angles_Home, AnglesSolution_PlasticBin_Offset, (0:.05:1));
TRAJ_HomeToOffsetMetalBin = jtraj(Angles_Home, AnglesSolution_MetalBin_Offset, (0:.05:1));
TRAJ_HomeToOffsetGlassBin = jtraj(Angles_Home, AnglesSolution_GlassBin_Offset, (0:.05:1));

% Creating animated trajectories from the offset points to the main points and vice versa
TRAJ_OffsetConveyorEndToConveyorEnd = jtraj(AnglesSolution_ConveyorEnd_Offset, AnglesSolution_ConveyorEnd, (0:.2:1));
TRAJ_OffsetConveyorStartToConveyorStart = jtraj(AnglesSolution_ConveyorStart_Offset, AnglesSolution_ConveyorStart, (0:.2:1));
TRAJ_OffsetPlasticBinToPlasticBin = jtraj(AnglesSolution_PlasticBin_Offset , AnglesSolution_PlasticBin, (0:.2:1));
TRAJ_OffsetMetalBinToMetalBin = jtraj(AnglesSolution_MetalBin_Offset , AnglesSolution_MetalBin, (0:.2:1));
TRAJ_OffsetGlassBinToGlassBin = jtraj(AnglesSolution_GlassBin_Offset , AnglesSolution_GlassBin, (0:.2:1));

TRAJ_ConveyorEndToOffsetConveyorEnd = jtraj(AnglesSolution_ConveyorEnd, AnglesSolution_ConveyorEnd_Offset, (0:.2:1));
TRAJ_ConveyorStartToOffsetConveyorStart = jtraj(AnglesSolution_ConveyorStart, AnglesSolution_ConveyorStart_Offset, (0:.2:1));
TRAJ_PlasticBinToOffsetPlasticBin = jtraj(AnglesSolution_PlasticBin, AnglesSolution_PlasticBin_Offset, (0:.2:1));
TRAJ_MetalBinToOffsetMetalBin = jtraj(AnglesSolution_MetalBin, AnglesSolution_MetalBin_Offset, (0:.2:1));
TRAJ_GlassBinToOffsetGlassBin = jtraj(AnglesSolution_GlassBin, AnglesSolution_GlassBin_Offset, (0:.2:1));

% Creating a floor
Draw_Floor_x = [-3 -3 3 3];
Draw_Floor_y = [3 -3 -3 3];
Draw_Floor_z = [-0.01 -0.01 -0.01 -0.01];

% Creating the conveyor plane coordinates system
Draw_Conveyor_x = [0 2 2 0];
Draw_Conveyor_y = [1.5 1.5 0.5 0.5];
Draw_Conveyor_z = [0 0 0 0];

% Creating the plastic bin box coordinates system
Draw_PlasticBin_x1 = [0.5 0.5 0.7 0.7];
Draw_PlasticBin_y1 = [0 -0.2 -0.2 0];
Draw_PlasticBin_z1 = [0 0 0 0];

Draw_PlasticBin_x2 = [0.5 0.5 0.5 0.5];
Draw_PlasticBin_y2 = [-0.2 -0.2 0 0];
Draw_PlasticBin_z2 = [0 0.5 0.5 0];

Draw_PlasticBin_x3 = [0.7 0.7 0.7 0.7];
Draw_PlasticBin_y3 = [-0.2 -0.2 0 0];
Draw_PlasticBin_z3 = [0 0.5 0.5 0];

Draw_PlasticBin_x4 = [0.5 0.5 0.7 0.7];
Draw_PlasticBin_y4 = [0 0 0 0];
Draw_PlasticBin_z4 = [0 0.5 0.5 0];

Draw_PlasticBin_x5 = [0.5 0.5 0.7 0.7];
Draw_PlasticBin_y5 = [-0.2 -0.2 -0.2 -0.2];
Draw_PlasticBin_z5 = [0 0.5 0.5 0];

% Creating the metal bin box coordinates system
Draw_MetalBin_x1 = [0.7 0.7 0.9 0.9];
Draw_MetalBin_y1 = [0 -0.2 -0.2 0];
Draw_MetalBin_z1 = [0 0 0 0];

Draw_MetalBin_x2 = [0.7 0.7 0.7 0.7];
Draw_MetalBin_y2 = [-0.2 -0.2 0 0];
Draw_MetalBin_z2 = [0 0.6 0.6 0];

Draw_MetalBin_x3 = [0.9 0.9 0.9 0.9];
Draw_MetalBin_y3 = [-0.2 -0.2 0 0];
Draw_MetalBin_z3 = [0 0.6 0.6 0];

Draw_MetalBin_x4 = [0.7 0.7 0.9 0.9];
Draw_MetalBin_y4 = [0 0 0 0];
Draw_MetalBin_z4 = [0 0.6 0.6 0];

Draw_MetalBin_x5 = [0.7 0.7 0.9 0.9];
Draw_MetalBin_y5 = [-0.2 -0.2 -0.2 -0.2];
Draw_MetalBin_z5 = [0 0.6 0.6 0];

% Creating the glass bin box coordinates system
Draw_GlassBin_x1 = [0.9 0.9 1.1 1.1];
Draw_GlassBin_y1 = [0 -0.2 -0.2 0];
Draw_GlassBin_z1 = [0 0 0 0];

Draw_GlassBin_x2 = [0.9 0.9 0.9 0.9];
Draw_GlassBin_y2 = [-0.2 -0.2 0 0];
Draw_GlassBin_z2 = [0 0.7 0.7 0];

Draw_GlassBin_x3 = [1.1 1.1 1.1 1.1];
Draw_GlassBin_y3 = [-0.2 -0.2 0 0];
Draw_GlassBin_z3 = [0 0.7 0.7 0];

Draw_GlassBin_x4 = [0.9 0.9 1.1 1.1];
Draw_GlassBin_y4 = [0 0 0 0];
Draw_GlassBin_z4 = [0 0.7 0.7 0];

Draw_GlassBin_x5 = [0.9 0.9 1.1 1.1];
Draw_GlassBin_y5 = [-0.2 -0.2 -0.2 -0.2];
Draw_GlassBin_z5 = [0 0.7 0.7 0];

% First we plot the environment before choosing buttons

% Reversing X-axis direction to match diagram           
set(gca, 'XDir','reverse');
           
% Plotting the floor plane coordinates
patch(Draw_Floor_x, Draw_Floor_y, Draw_Floor_z, 'white');

% Plotting the conveyor plane coordinates
patch(Draw_Conveyor_x, Draw_Conveyor_y, Draw_Conveyor_z, 'black');

% Plotting the plastic bin
patch(Draw_PlasticBin_x1, Draw_PlasticBin_y1, Draw_PlasticBin_z1, 'green');
patch(Draw_PlasticBin_x2, Draw_PlasticBin_y2, Draw_PlasticBin_z2, 'green');
patch(Draw_PlasticBin_x3, Draw_PlasticBin_y3, Draw_PlasticBin_z3, 'green');
patch(Draw_PlasticBin_x4, Draw_PlasticBin_y4, Draw_PlasticBin_z4, 'green');
patch(Draw_PlasticBin_x5, Draw_PlasticBin_y5, Draw_PlasticBin_z5, 'green');

% Plotting the metal bin
patch(Draw_MetalBin_x1, Draw_MetalBin_y1, Draw_MetalBin_z1, 'yellow');
patch(Draw_MetalBin_x2, Draw_MetalBin_y2, Draw_MetalBin_z2, 'yellow');
patch(Draw_MetalBin_x3, Draw_MetalBin_y3, Draw_MetalBin_z3, 'yellow');
patch(Draw_MetalBin_x4, Draw_MetalBin_y4, Draw_MetalBin_z4, 'yellow');
patch(Draw_MetalBin_x5, Draw_MetalBin_y5, Draw_MetalBin_z5, 'yellow');

% Plotting the glass bin
patch(Draw_GlassBin_x1, Draw_GlassBin_y1, Draw_GlassBin_z1, 'cyan');
patch(Draw_GlassBin_x2, Draw_GlassBin_y2, Draw_GlassBin_z2, 'cyan');
patch(Draw_GlassBin_x3, Draw_GlassBin_y3, Draw_GlassBin_z3, 'cyan');
patch(Draw_GlassBin_x4, Draw_GlassBin_y4, Draw_GlassBin_z4, 'cyan');
patch(Draw_GlassBin_x5, Draw_GlassBin_y5, Draw_GlassBin_z5, 'cyan');

% Plotting the robot arm at the home position
RobotArm.plot(Angles_Home);

% Menu to give options to user to move the robotic arm
EntryButton = menu('Choose which position would you like to go:','Home','Conveyor End','Conveyor Start','Plastic Bin','Metal Bin', 'Glass Bin', 'All Options');
switch EntryButton
    case 1 % To move the robotic arm in the home position
        RobotArm.plot(Angles_Home);
        msgbox(['Reached Position: X = 0m , Y = 0.7878m , Z = 0.9389m. '...
         'End Defector Angle: ' num2str((sum(Angles_Home(2:4)))*180/pi) '°']);
        
    case 2 % To move the robotic arm in the end of conveyor
        RobotArm.plot(TRAJ_HomeToOffsetConveyorEnd);
        RobotArm.plot(TRAJ_OffsetConveyorEndToConveyorEnd);
        msgbox(['Reached Position: X = ' num2str(Position_ConveyorEnd(1,4)) 'm , Y = ' num2str(Position_ConveyorEnd(2,4)) 'm , Z = ' num2str(Position_ConveyorEnd(3,4)) 'm. '...
         'End Defector Angle: ' num2str((sum(AnglesSolution_ConveyorEnd(2:4)))*180/pi) '°']);
    case 3 % To move the robotic arm in the start of conveyor
        RobotArm.plot(TRAJ_HomeToOffsetConveyorStart);
        RobotArm.plot(TRAJ_OffsetConveyorStartToConveyorStart);
        msgbox(['Reached Position: X = ' num2str(Position_ConveyorStart(1,4)) 'm , Y = ' num2str(Position_ConveyorStart(2,4)) 'm , Z = ' num2str(Position_ConveyorStart(3,4)) 'm. '...
         'End Defector Angle: ' num2str((sum(AnglesSolution_ConveyorStart(2:4)))*180/pi) '°']);
    case 4 % To move the robotic arm in the plastic bin
        RobotArm.plot(TRAJ_HomeToOffsetPlasticBin);
        RobotArm.plot(TRAJ_OffsetPlasticBinToPlasticBin);
        msgbox(['Reached Position: X = ' num2str(Position_PlasticBin(1,4)) 'm , Y = ' num2str(Position_PlasticBin(2,4)) 'm , Z = ' num2str(Position_PlasticBin(3,4)) 'm. '...
         'End Defector Angle: ' num2str((sum(AnglesSolution_PlasticBin(2:4)))*180/pi) '°']);
    case 5 % To move the robotic arm in the metal bin
        RobotArm.plot(TRAJ_HomeToOffsetMetalBin);
        RobotArm.plot(TRAJ_OffsetMetalBinToMetalBin);
        msgbox(['Reached Position: X = ' num2str(Position_MetalBin(1,4)) 'm , Y = ' num2str(Position_MetalBin(2,4)) 'm , Z = ' num2str(Position_MetalBin(3,4)) 'm. '...
         'End Defector Angle: ' num2str((sum(AnglesSolution_MetalBin(2:4)))*180/pi) '°']);
    case 6 % To move the robotic arm in the glass bin
        RobotArm.plot(TRAJ_HomeToOffsetGlassBin);
        RobotArm.plot(TRAJ_OffsetGlassBinToGlassBin);
        msgbox(['Reached Position: X = ' num2str(Position_GlassBin(1,4)) 'm , Y = ' num2str(Position_GlassBin(2,4)) 'm , Z = ' num2str(Position_GlassBin(3,4)) 'm. '...
         'End Defector Angle: ' num2str((sum(AnglesSolution_GlassBin(2:4)))*180/pi) '°']);
    case 7 % To make a full cycly of movement through all the points   
      
        % Plotting the robotic arm movement from home to conveyor end and back
        RobotArm.plot(TRAJ_HomeToOffsetConveyorEnd);
        RobotArm.plot(TRAJ_OffsetConveyorEndToConveyorEnd);
        RobotArm.plot(TRAJ_ConveyorEndToOffsetConveyorEnd);
        RobotArm.plot(TRAJ_OffsetConveyorEndToHome);

        % Plotting the robotic arm movement from home to conveyor start and back
        RobotArm.plot(TRAJ_HomeToOffsetConveyorStart);
        RobotArm.plot(TRAJ_OffsetConveyorStartToConveyorStart);
        RobotArm.plot(TRAJ_ConveyorStartToOffsetConveyorStart);
        RobotArm.plot(TRAJ_OffsetConveyorStartToHome);

        % Plotting the robotic arm movement from home to plastic bin and back
        RobotArm.plot(TRAJ_HomeToOffsetPlasticBin);
        RobotArm.plot(TRAJ_OffsetPlasticBinToPlasticBin);
        RobotArm.plot(TRAJ_PlasticBinToOffsetPlasticBin);
        RobotArm.plot(TRAJ_OffsetPlasticBinToHome);

        % Plotting the robotic arm movement from home to metal bin and back
        RobotArm.plot(TRAJ_HomeToOffsetMetalBin);
        RobotArm.plot(TRAJ_OffsetMetalBinToMetalBin);
        RobotArm.plot(TRAJ_MetalBinToOffsetMetalBin);
        RobotArm.plot(TRAJ_OffsetMetalBinToHome);

        % Plotting the robotic arm movement from home to glass bin and back
        RobotArm.plot(TRAJ_HomeToOffsetGlassBin);
        RobotArm.plot(TRAJ_OffsetGlassBinToGlassBin);
        RobotArm.plot(TRAJ_GlassBinToOffsetGlassBin);
        RobotArm.plot(TRAJ_OffsetGlassBinToHome);   

    otherwise
        msgbox('Simulation aborted');
end


