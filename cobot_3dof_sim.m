%% 3-DOF COBOT: FK, IK, WORKSPACE, AND PICK-DROP TRAJECTORY
% J1: base yaw (about +Z); J2, J3: planar pitches.
% Edit the parameters in the next block to match your CAD.

clear; clc; close all;

%% ==== USER PARAMETERS (EDIT THESE) =======================================
L1 = 0.22;             % shoulder->elbow   (m)
L2 = 0.20;             % elbow->wrist      (m)
L3 = 0.08;             % wrist->TCP along link (m)

% Joint limits (degrees)
lim.J1 = [-170 170];
lim.J2 = [-150 150];
lim.J3 = [-150 150];

% Home pose (deg)
q_home_deg = [0 10 -10];

% Bin / poses in BASE frame {B} (meters)
pick  = [ 0.30  0.10  0.12];   % where the bread is
view  = [ 0.35  0.00  0.20];   % inspection/vision pose
drop  = [ 0.25 -0.15  0.10];   % disposal bin pose

% Animation
draw_dt   = 0.01;   % s between frames
anim_span = 1.0;    % s per segment
elbow_mode = "down"; % "down" or "up" for IK branch
%% ========================================================================

deg = pi/180; rad = 180/pi;

% Convenience handles
fk = @(q) fk_3dof(q, [L1 L2 L3]);
ik = @(p) ik_3dof(p, [L1 L2 L3], elbow_mode);

%% --- Sanity check: FK at home
q_home = q_home_deg*deg;
[TCP, Jnt] = fk(q_home); % positions of joints
fprintf('Home TCP [m]: [%.3f  %.3f  %.3f]\n', TCP);

%% --- WORKSPACE SWEEP (dense but fast)
figure('Name','Workspace & Arm'); hold on; grid on; axis equal
xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');
title('3-DOF Cobot Workspace & Trajectories');

% Sweep J2,J3 (planar) and a few J1 headings to show reach
samp = 45;
[J2g, J3g] = meshgrid(linspace(lim.J2(1), lim.J2(2), samp), ...
                      linspace(lim.J3(1), lim.J3(2), samp));
for h = linspace(lim.J1(1), lim.J1(2), 9)
    q1 = h*deg; 
    q2 = J2g*deg; q3 = J3g*deg;
    Cx =  cos(q1).*(L1*cos(q2)+L2*cos(q2+q3)+L3*cos(q2+q3));
    Cy =  sin(q1).*(L1*cos(q2)+L2*cos(q2+q3)+L3*cos(q2+q3));
    Cz =  L1*sin(q2)+L2*sin(q2+q3)+L3*sin(q2+q3);
    plot3(Cx(:),Cy(:),Cz(:),'.','MarkerSize',3,'HandleVisibility','off');
end

% Mark task points
scatter3(pick(1),pick(2),pick(3),60,'filled','DisplayName','PICK');
scatter3(view(1),view(2),view(3),60,'filled','DisplayName','VIEW');
scatter3(drop(1),drop(2),drop(3),60,'filled','DisplayName','DROP');

legend('Location','best'); view(35,25);

%% --- Try IK for the three waypoints
[q_pick, ok_pick] = ik(pick);
[q_view, ok_view] = ik(view);
[q_drop, ok_drop] = ik(drop);
assert(ok_pick && ok_view && ok_drop, 'One or more targets not reachable within limits.');

%% --- Animate: home -> pick -> view -> drop -> home
way_q  = [q_home; q_pick; q_view; q_drop; q_home];
way_p  = [fk(q_home); fk(q_pick); fk(q_view); fk(q_drop); fk(q_home)];

% Plot planned TCP path
plot3(way_p(:,1), way_p(:,2), way_p(:,3), 'LineWidth',1.5, 'DisplayName','TCP Path');

% Joint time history buffers
Q = []; T = [];

% Draw once to create arm graphics
[~, jpts] = fk(way_q(1,:));
armPlot = plot3(jpts(:,1), jpts(:,2), jpts(:,3),'-o','LineWidth',2, ...
    'MarkerFaceColor',[0.2 0.2 0.2], 'DisplayName', 'Arm');

% Segment-by-segment quintic interpolation in joint space
t0 = 0;
for seg = 1:size(way_q,1)-1
    qA = way_q(seg,:); qB = way_q(seg+1,:);
    tf = anim_span;
    [t, qtraj] = jtraj_quintic(qA, qB, tf, draw_dt);
    for k = 1:numel(t)
        [~, jpts] = fk(qtraj(k,:));
        set(armPlot,'XData',jpts(:,1),'YData',jpts(:,2),'ZData',jpts(:,3));
        drawnow;
    end
    Q = [Q; qtraj]; %#ok<AGROW>
    T = [T; t0 + t(:)]; %#ok<AGROW>
    t0 = t0 + tf;
end

%% --- Plot joint profiles
figure('Name','Joint Profiles'); 
subplot(3,1,1); plot(T, Q(:,1)*rad); ylabel('J1 (deg)'); grid on;
subplot(3,1,2); plot(T, Q(:,2)*rad); ylabel('J2 (deg)'); grid on;
subplot(3,1,3); plot(T, Q(:,3)*rad); ylabel('J3 (deg)'); grid on; xlabel('Time (s)');

disp('Done. Edit link lengths / waypoints at the top to match your CAD.');

%% ===================== FUNCTIONS ========================================
function [tcp, joints] = fk_3dof(q, L)
% Forward kinematics for 3-DOF: yaw + planar 2R with TCP offset along link
q1=q(1); q2=q(2); q3=q(3); L1=L(1); L2=L(2); L3=L(3);
% Planar reach (in arm plane before yaw)
rx1 =  L1*cos(q2);     rz1 =  L1*sin(q2);
rx2 =  L2*cos(q2+q3);  rz2 =  L2*sin(q2+q3);
rx3 =  L3*cos(q2+q3);  rz3 =  L3*sin(q2+q3);

% Rotate the planar (x,y) by yaw q1 around Z
x = cos(q1)*(rx1+rx2+rx3);
y = sin(q1)*(rx1+rx2+rx3);
z = rz1 + rz2 + rz3;
tcp = [x y z];

% Joint points for plotting (base at [0 0 0])
P0 = [0 0 0];
P1 = [cos(q1)*rx1, sin(q1)*rx1, rz1];
P2 = [cos(q1)*(rx1+rx2), sin(q1)*(rx1+rx2), rz1+rz2];
P3 = tcp;
joints = [P0; P1; P2; P3];
end

function [q, ok] = ik_3dof(p, L, elbow_mode)
% Closed-form IK: aim base at (x,y), solve planar 2R for (r,z)
x=p(1); y=p(2); z=p(3); L1=L(1); L2=L(2); L3=L(3);
q1 = atan2(y,x);
r  = hypot(x,y);               % radial distance to TCP
% Wrist center along planar ray back by L3
rw = r  - L3; 
zw = z  - 0;                   % no vertical offset base->J2
% 2R IK for (rw,zw)
D = (rw^2 + zw^2 - L1^2 - L2^2)/(2*L1*L2);
D = max(min(D,1),-1);          % clamp numerical noise
if elbow_mode == "up"
    q3p = atan2(-sqrt(1-D^2), D);
else
    q3p = atan2( sqrt(1-D^2), D);
end
phi = atan2(zw, rw);
psi = atan2(L2*sin(q3p), L1 + L2*cos(q3p));
q2 = phi - psi;
q3 = q3p;
q  = wrapToPi([q1 q2 q3]);
ok = isfinite(sum(q));
end

function [t, q] = jtraj_quintic(q0, q1, tf, dt)
% Scalar quintic synchronized for each joint
t = (0:dt:tf).';
s = (10*(t/tf).^3 - 15*(t/tf).^4 + 6*(t/tf).^5); % 0->1 smoothstep
q = q0 + (q1 - q0).*s;
end
