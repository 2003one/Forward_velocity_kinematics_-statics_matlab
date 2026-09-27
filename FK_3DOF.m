function robot3DOF_POE_interactive
% ROBOT3DOF_POE_INTERACTIVE
% Interactive 3-DOF robot Forward Kinematics using POE.
%
% USER INPUT:
%   Enter theta1, theta2, theta3 in DEGREES.
%
% The program converts degrees to radians internally because MATLAB's
% matrix exponential uses the angle in radians.
%
% POE:
%   T = exp([S1]*theta1) * exp([S2]*theta2) * exp([S3]*theta3) * M

clc;
close all;

%% Robot parameters
l1 = 0.30;
l2 = 0.40;
l3 = 0.30;

% Space screw axes from our current derivation
S1 = [0; 0; 1; 0; 0; 0];
S2 = [1; 0; 0; 0; 0.30; 0];
S3 = [1; 0; 0; 0; 0.30; 0];

% Home configuration M = T(0,0,0)
M = [1 0 0 0.70;
     0 1 0 0;
     0 0 1 0.30;
     0 0 0 1];

%% Create window
fig = figure( ...
    'Name','3-DOF Robot - POE Forward Kinematics', ...
    'NumberTitle','off', ...
    'Color',[0.94 0.94 0.94], ...
    'Position',[80 80 1450 820]);

% Robot plot
ax = axes('Parent',fig,'Position',[0.04 0.10 0.60 0.82]);
grid(ax,'on');
axis(ax,'equal');
axis(ax,[-0.9 1.1 -0.9 0.9 -0.15 0.95]);
xlabel(ax,'X (m)','FontSize',12,'FontWeight','bold');
ylabel(ax,'Y (m)','FontSize',12,'FontWeight','bold');
zlabel(ax,'Z (m)','FontSize',12,'FontWeight','bold');
title(ax,'3-DOF Robot Arm — POE Forward Kinematics', ...
    'FontSize',14,'FontWeight','bold');
view(ax,45,28);

%% Control panel
panel = uipanel(fig, ...
    'Title','Joint Control — Enter Degrees', ...
    'FontSize',12, ...
    'FontWeight','bold', ...
    'BackgroundColor',[0.97 0.97 0.97], ...
    'Position',[0.67 0.08 0.30 0.84]);

uicontrol(panel,'Style','text', ...
    'String','Type a value OR drag the slider (-180° to +180°)', ...
    'Position',[20 595 390 28], ...
    'FontSize',10, ...
    'BackgroundColor',[0.97 0.97 0.97]);

% Theta 1
uicontrol(panel,'Style','text', ...
    'String','Theta 1 — Base rotation (°)', ...
    'Position',[20 545 250 25], ...
    'FontSize',11,'FontWeight','bold', ...
    'HorizontalAlignment','left', ...
    'BackgroundColor',[0.97 0.97 0.97]);

edit1 = uicontrol(panel,'Style','edit','String','0', ...
    'Position',[290 542 90 30], ...
    'FontSize',11,'BackgroundColor','white', ...
    'Callback',@editCallback);

slider1 = uicontrol(panel,'Style','slider', ...
    'Min',-180,'Max',180,'Value',0, ...
    'Position',[20 515 360 22], ...
    'Callback',@sliderCallback);

% Theta 2
uicontrol(panel,'Style','text', ...
    'String','Theta 2 — Shoulder (°)', ...
    'Position',[20 450 250 25], ...
    'FontSize',11,'FontWeight','bold', ...
    'HorizontalAlignment','left', ...
    'BackgroundColor',[0.97 0.97 0.97]);

edit2 = uicontrol(panel,'Style','edit','String','0', ...
    'Position',[290 447 90 30], ...
    'FontSize',11,'BackgroundColor','white', ...
    'Callback',@editCallback);

slider2 = uicontrol(panel,'Style','slider', ...
    'Min',-180,'Max',180,'Value',0, ...
    'Position',[20 420 360 22], ...
    'Callback',@sliderCallback);

% Theta 3
uicontrol(panel,'Style','text', ...
    'String','Theta 3 — Wrist (°)', ...
    'Position',[20 355 250 25], ...
    'FontSize',11,'FontWeight','bold', ...
    'HorizontalAlignment','left', ...
    'BackgroundColor',[0.97 0.97 0.97]);

edit3 = uicontrol(panel,'Style','edit','String','0', ...
    'Position',[290 352 90 30], ...
    'FontSize',11,'BackgroundColor','white', ...
    'Callback',@editCallback);

slider3 = uicontrol(panel,'Style','slider', ...
    'Min',-180,'Max',180,'Value',0, ...
    'Position',[20 325 360 22], ...
    'Callback',@sliderCallback);

% Position
uicontrol(panel,'Style','text','String','END-EFFECTOR POSITION', ...
    'Position',[20 270 360 25], ...
    'FontSize',11,'FontWeight','bold', ...
    'BackgroundColor',[0.90 0.92 0.96]);

textX = uicontrol(panel,'Style','text','String','X = 0.7000 m', ...
    'Position',[20 235 360 25],'FontSize',11, ...
    'HorizontalAlignment','left','BackgroundColor',[0.97 0.97 0.97]);

textY = uicontrol(panel,'Style','text','String','Y = 0.0000 m', ...
    'Position',[20 205 360 25],'FontSize',11, ...
    'HorizontalAlignment','left','BackgroundColor',[0.97 0.97 0.97]);

textZ = uicontrol(panel,'Style','text','String','Z = 0.3000 m', ...
    'Position',[20 175 360 25],'FontSize',11, ...
    'HorizontalAlignment','left','BackgroundColor',[0.97 0.97 0.97]);

% Presets
uicontrol(panel,'Style','text','String','QUICK PRESETS', ...
    'Position',[20 135 360 25],'FontSize',11,'FontWeight','bold', ...
    'BackgroundColor',[0.90 0.92 0.96]);

uicontrol(panel,'Style','pushbutton','String','HOME  (0°, 0°, 0°)', ...
    'Position',[20 95 170 32],'FontSize',10,'FontWeight','bold', ...
    'Callback',@(src,event)setPose(0,0,0));

uicontrol(panel,'Style','pushbutton','String','POSE 1  (45°, -30°, 60°)', ...
    'Position',[205 95 175 32],'FontSize',10, ...
    'Callback',@(src,event)setPose(45,-30,60));

uicontrol(panel,'Style','pushbutton','String','POSE 2  (-60°, 45°, -30°)', ...
    'Position',[20 55 170 32],'FontSize',10, ...
    'Callback',@(src,event)setPose(-60,45,-30));

uicontrol(panel,'Style','pushbutton','String','POSE 3  (90°, -45°, 30°)', ...
    'Position',[205 55 175 32],'FontSize',10, ...
    'Callback',@(src,event)setPose(90,-45,30));

uicontrol(panel,'Style','text', ...
    'String','Move a slider or type an angle, then press Enter/click outside.', ...
    'Position',[20 15 390 30],'FontSize',9, ...
    'HorizontalAlignment','left','BackgroundColor',[0.97 0.97 0.97]);

%% Initial pose
updateRobot(0,0,0);

%% ================================================================
% Nested functions: these can access the GUI variables above.
% ================================================================

    function T = screwExp(S,thetaRad)
        omega = S(1:3);
        v = S(4:6);

        omegaHat = [0 -omega(3) omega(2);
                    omega(3) 0 -omega(1);
                    -omega(2) omega(1) 0];

        se3 = [omegaHat v;
               0 0 0 0];

        T = expm(se3 * thetaRad);
    end

    function T = forwardKinematics(t1Deg,t2Deg,t3Deg)
        % User input is DEGREES.
        % Convert to radians internally.
        th = deg2rad([t1Deg;t2Deg;t3Deg]);

        T1 = screwExp(S1,th(1));
        T2 = screwExp(S2,th(2));
        T3 = screwExp(S3,th(3));

        T = T1*T2*T3*M;
    end

    function updateRobot(t1Deg,t2Deg,t3Deg)

        % Convert degrees to radians for the POE calculation.
        th = deg2rad([t1Deg;t2Deg;t3Deg]);

        T1 = screwExp(S1,th(1));
        T2 = screwExp(S2,th(2));
        T3 = screwExp(S3,th(3));

        T12 = T1*T2;
        T123 = T12*T3;

        % Complete end-effector transformation from POE
        T_ee = T123*M;

        % Joint positions
        P0 = [0;0;0];

        P1 = T1(1:3,1:3)*[0;0;l1] + T1(1:3,4);

        P2 = T12(1:3,1:3)*[l2;0;0] + T12(1:3,4);

        P3 = T123(1:3,1:3)*[l3;0;0] + T123(1:3,4);

        Pee = T_ee(1:3,4);
        R = T_ee(1:3,1:3);

        % Redraw plot
        cla(ax);
        hold(ax,'on');
        grid(ax,'on');
        axis(ax,'equal');
        axis(ax,[-0.9 1.1 -0.9 0.9 -0.15 0.95]);

        % Base
        [bx,by,bz] = cylinder(0.18,40);
        bz = bz*0.12;
        surf(ax,bx,by,bz-0.02, ...
            'FaceColor',[0.35 0.35 0.38], ...
            'EdgeColor','none');

        % Links
        plot3(ax,[P0(1) P1(1)],[P0(2) P1(2)],[P0(3) P1(3)], ...
            'LineWidth',10,'Color',[0.80 0.25 0.25]);

        plot3(ax,[P1(1) P2(1)],[P1(2) P2(2)],[P1(3) P2(3)], ...
            'LineWidth',10,'Color',[0.25 0.55 0.90]);

        plot3(ax,[P2(1) P3(1)],[P2(2) P3(2)],[P2(3) P3(3)], ...
            'LineWidth',10,'Color',[0.25 0.75 0.35]);

        % Joints
        scatter3(ax,P0(1),P0(2),P0(3),230,'o','filled', ...
            'MarkerFaceColor',[0.20 0.20 0.20],'MarkerEdgeColor','k');

        scatter3(ax,P1(1),P1(2),P1(3),180,'o','filled', ...
            'MarkerFaceColor',[0.85 0.15 0.15],'MarkerEdgeColor','k');

        scatter3(ax,P2(1),P2(2),P2(3),180,'o','filled', ...
            'MarkerFaceColor',[0.10 0.35 0.90],'MarkerEdgeColor','k');

        scatter3(ax,P3(1),P3(2),P3(3),180,'o','filled', ...
            'MarkerFaceColor',[0.10 0.65 0.20],'MarkerEdgeColor','k');

        % End-effector body / gripper
        palm = Pee + R*[0.12;0;0];

        plot3(ax,[Pee(1) palm(1)], ...
                 [Pee(2) palm(2)], ...
                 [Pee(3) palm(3)], ...
                 'LineWidth',9,'Color',[0.15 0.15 0.15]);

        spacing = 0.08;
        fingerLength = 0.16;

        f1a = palm + R*[0; spacing/2; 0];
        f1b = f1a + R*[fingerLength;0;0];

        f2a = palm + R*[0;-spacing/2;0];
        f2b = f2a + R*[fingerLength;0;0];

        plot3(ax,[f1a(1) f1b(1)],[f1a(2) f1b(2)],[f1a(3) f1b(3)], ...
            'LineWidth',7,'Color',[0.15 0.15 0.15]);

        plot3(ax,[f2a(1) f2b(1)],[f2a(2) f2b(2)],[f2a(3) f2b(3)], ...
            'LineWidth',7,'Color',[0.15 0.15 0.15]);

        % EE point
        scatter3(ax,Pee(1),Pee(2),Pee(3),150,'s','filled', ...
            'MarkerFaceColor',[1.0 0.75 0.05],'MarkerEdgeColor','k');

        % End-effector frame
        s = 0.14;

        quiver3(ax,Pee(1),Pee(2),Pee(3), ...
            s*R(1,1),s*R(2,1),s*R(3,1), ...
            'Color','r','LineWidth',2.5,'MaxHeadSize',0.7);

        quiver3(ax,Pee(1),Pee(2),Pee(3), ...
            s*R(1,2),s*R(2,2),s*R(3,2), ...
            'Color','g','LineWidth',2.5,'MaxHeadSize',0.7);

        quiver3(ax,Pee(1),Pee(2),Pee(3), ...
            s*R(1,3),s*R(2,3),s*R(3,3), ...
            'Color','b','LineWidth',2.5,'MaxHeadSize',0.7);

        % Labels
        text(ax,P1(1),P1(2),P1(3)+0.06,'J2','FontWeight','bold');
        text(ax,P2(1),P2(2),P2(3)+0.06,'J3','FontWeight','bold');
        text(ax,Pee(1),Pee(2),Pee(3)+0.08,'EE','FontWeight','bold');

        title(ax,sprintf('3-DOF Robot   [%.1f°, %.1f°, %.1f°]', ...
            t1Deg,t2Deg,t3Deg),'FontSize',14,'FontWeight','bold');

        view(ax,45,28);

        % Position display
        set(textX,'String',sprintf('X = %.4f m',Pee(1)));
        set(textY,'String',sprintf('Y = %.4f m',Pee(2)));
        set(textZ,'String',sprintf('Z = %.4f m',Pee(3)));

        drawnow;
    end

    function editCallback(~,~)
        t1 = str2double(get(edit1,'String'));
        t2 = str2double(get(edit2,'String'));
        t3 = str2double(get(edit3,'String'));

        if any(isnan([t1 t2 t3])) || any(~isfinite([t1 t2 t3]))
            msgbox('Please enter valid numbers in DEGREES.','Input Error','error');
            return;
        end

        % Keep values in slider range.
        t1 = max(-180,min(180,t1));
        t2 = max(-180,min(180,t2));
        t3 = max(-180,min(180,t3));

        set(slider1,'Value',t1);
        set(slider2,'Value',t2);
        set(slider3,'Value',t3);

        set(edit1,'String',num2str(t1,'%.1f'));
        set(edit2,'String',num2str(t2,'%.1f'));
        set(edit3,'String',num2str(t3,'%.1f'));

        updateRobot(t1,t2,t3);
    end

    function sliderCallback(~,~)
        t1 = get(slider1,'Value');
        t2 = get(slider2,'Value');
        t3 = get(slider3,'Value');

        set(edit1,'String',num2str(t1,'%.1f'));
        set(edit2,'String',num2str(t2,'%.1f'));
        set(edit3,'String',num2str(t3,'%.1f'));

        updateRobot(t1,t2,t3);
    end

    function setPose(t1,t2,t3)
        set(slider1,'Value',t1);
        set(slider2,'Value',t2);
        set(slider3,'Value',t3);

        set(edit1,'String',num2str(t1,'%.1f'));
        set(edit2,'String',num2str(t2,'%.1f'));
        set(edit3,'String',num2str(t3,'%.1f'));

        updateRobot(t1,t2,t3);
    end
end