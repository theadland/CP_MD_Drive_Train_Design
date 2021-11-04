%% Headland Wood Chipper Drive Shaft Design Calculations
% This script calculates the minimum shaft diameter for given operating
% conditions to acheive infinite fatigue life.

% Required files: fcrit.m, fvonmises.m, step.m, xstep.m

clear variables

%% Constants
yield = 77E3;                % Yield strength of 1045 CD (Psi)
ultimate = 91E3;             % Ultimate strength of 1045 CD (Psi)
theta_allow = 0.0005;        % Allowable angle of shaft at bearings (rad)
ndf = 2;                     % Design factor
stress_allow = yield/ndf;    % Allowable first cycle stress
E = 30E6;                    % Elastic Modulus for carbon steel (Psi)

rho_s = 490;                 % Density of steel (lbm/ft^3)
radi_fw = 10/12;             % Radius of flywheel
radi_c = 0.6483;             % Distance from flywheel axis to center of cutting blade 
n = 1.6;                     % Number of cutting blades on flywheel
blade_a = 37;                % Blade angle in degrees

power_e = 15.4;             % Engine power output (HP)
power_e1 = power_e*550;     % Engine power output (lbf-ft/s)

res = 1001;                 % Subdivisions for iterative analysis

% Rotational Speed Relationship between Engine and Flywheel shaft

omega_e = 3400;             % Desired engine free speed (RPM)
omega_e1 = omega_e*2*pi/60; % Engine shaft speed (1/s)
omega_f = 1000;             % Desired flywheel free speed (RPM)
omega_f1 = omega_f*2*pi/60; % Flywheel shaft speed (1/s)
ratio = omega_f/omega_e;    % Necessary pulley ratio 

% Geometry
geo = ones(8,res);
geo(1,:) = 1.5/12;                      % Distance from pulley to l-bearing (ft)
geo(2,:) = 7.5/12;                      % Distance from l-bearing to flywheel (ft)
geo(3,:) = 1.375/12;                    % Distance from flywheel to r-bearing (ft)
geo(4,:) = geo(1,:)+geo(2,:)+geo(3,:);  % Shaft length (ft)
rad_min = .125/12;               	    % Shaft min radius value (ft)
rad_max = 2/12;                         % Shaft max radius value (ft)
dr = (rad_max-rad_min)/res ;              % Shaft finite difference (ft)
geo(5,:) = linspace(rad_min, rad_max, res)'; % Shaft radius vector
geo(6,:) = pi*geo(5,:).^2;                % Shaft cross sectional area (ft^2)
geo(7,:) = pi*geo(5,:).^4/4;            % Shaft second moment of area (ft^4)
geo(8,:) = pi*geo(5,:).^4/2;            % Shaft polar moment of inertia (ft^4)

thick_f = 1.5/12;            % Thickness of flywheel (ft)

pul_e = 4.5/12;             % Desired engine pulley diameter (ft)
radi_ep = 0.1875;           % Radius of engine pulley (ft)

radi_fp = radi_ep/ratio;    % Radius of flywheel pulley (ft
pul_f = pul_e/ratio;        % Necessary flywheel pulley diameter (ft)

% Mass and Inertia
mai(1,:) = rho_s*geo(4,:).*geo(6,:);   % Mass of shaft (lbm)
mai(2,:) = .5*mai(1,:).*geo(5,:).^2;  % Mass moment of inertia of shaft (lbm-ft^2)

mass_f = rho_s*pi*radi_fw*thick_f;     % Mass of flywheel (lbm)
mai(3,:) = .5*mass_f*radi_fw^2;        % Mass moment of intertia for flywheel (lbm-ft^2)



%% Loads,Forces, Moments

lfm = ones(12,res);

% Engine torque (lbf-ft)
torq_eng_s = power_e1/omega_e1;

% Torque transmitted to flywheel shaft from engine (lbf-ft)
lfm(12,:) = torq_eng_s/ratio; % torq_fly_s      

theta_c = 0.335;             % Angle subtended by midpoint of cutter during cut (rad)
del_tc = theta_c/omega_f1;   % Time elapsed during cut


% This calculates flywheel speed after chip creation
omega_1 = sqrt(omega_f1^2 + (2*(2*pi*lfm(12,1) - 686*n)./(mai(3,:)+mai(2,:))));

% Change in rotational velocity after cut supposing the engine is not
% limited
del_omega = omega_1 - omega_f1;

% This calculates flywheel speed after chip creation supposing there is no 
% driving torque (as when the engine speed is limited)
omega_2 = sqrt(omega_f1^2 - 2*686./(mai(3,:)+mai(2,:)));

% This is the change in rotational velocity after the cut
del_omega2 = omega_2-omega_f1;

omega_3 = sqrt(omega_2.^2 - 2*(2*pi*lfm(12,1))./(mai(3,:)+mai(2,:)));
del_omega3 = omega_3-omega_2;

% The force of the cut using impulse/momentum (lbf)
lfm(1,:) = -0.5*mass_f*radi_fw*del_omega2/del_tc; % force_cy

% Moment on shaft due to cut (lbf-ft)
lfm(2,:) = -mai(2,:).*del_omega2/del_tc; %lfm(1)*radi_c; % moment_c

% Moment on shaft due to pully (lbf-ft) 
lfm(3,:) = (mai(3,:)+mai(2,:)).*(omega_2-omega_f1)/del_tc; % moment_fp

% Weight of shaft (lbf)
lfm(4,:) = (rho_s*geo(4,:).*pi.*geo(5,:).^2); % weight_s

% Weight of  flywheel (lbf)  
lfm(5,:) = rho_s*thick_f*pi*radi_fw; % weight_f

% Weight of flywheel pully (lbf)
lfm(6,:) = 20; % weight_p

% Force on shaft due to flywheel pulley (lbf) 
lfm(7,:) = lfm(12,:)/radi_fp + 4*70; % 70 lbf static tension perstrand per 
                                     % Gates design manual                   

% Bearing reaction forces (lbf)

% right bearing reaction in y-direction
lfm(8,:) = -(lfm(4,:)*0.5.*(geo(4,:)/2 - geo(1,:))./(geo(2,:)+geo(3,:)))...
         + ((lfm(1,:)-lfm(5,:)).*geo(2,:)./(geo(2,:)+geo(3,:)))...
         + (lfm(6,:).*geo(1,:)./(geo(2,:)+geo(3,:))); 

% left bearing reaction in y-direction         
lfm(9,:) = lfm(1,:) - lfm(6,:) - lfm(4,:) - lfm(5,:) - lfm(8,:); 

% left bearing reaction in z-direction
lfm(10,:) = lfm(7,:).*geo(4,:)./(geo(2,:)+geo(3,:)); 

% right bearing reaction in z-direction
lfm(11,:) = lfm(10,:) - lfm(7,:); 


%% Shears, Moments, Stresses

x = linspace(0,geo(4,1),res);

% This section preallocates variable sizes because Matlab likes that.

sms = ones(11,res);
%  sms =
%  [shear_y      
%  [shear_z      
%  [moment_x     
%  [moment_y     
%  [moment_z    
%  [axial_x      
%  [bend_stress_x
%  [shear_xy     
%  [shear_zx     
%  [vonmises_r     
%  [vonmises   

% The while loop is used so that an array of vonMises stress can be
% developed for all radi at the critical location which is found in the for
% loop.
i=1;
while i < length(geo)
  
    % This for loop runs though the shear and moment singularity equations
    % for a single radius value for all x locations. The goal is to find
    % the location of maximum stress so that for all iterations of the
    % loop, the index of n that corresonds with that location will be the
    % only value used for all subsequent iterations.
    sms = fvonmises(lfm,x,i,mai,geo,res,sms);                
    
    % This finds the largest vonMises stress for the first iteration of the
    % stress for loop, and uses the index of that vector to change the
    % x input of all subsequent loops from a range to the critical
    % location.
    max_vonmises = max(sms(11,:));
    k = find(sms(11,:)==max_vonmises);
    sms(10,i)=sms(11,k);

    i=i+1;
   
    
end

% Here the index for the vonMises stress closest to that of the allowable
% stress is found and used to find the corresponding minimum shaft radius
% based on first cicle failure.
[c,I] = min(abs(sms(10,:)-stress_allow));
min_radius_stress = geo(5,I)*12;

% This recalculates the stresses for the mininum radius and finds the x
% location of greatest stress
sms = fvonmises(lfm,x,I,mai,geo,res,sms);
max_vonmises = max(sms(11,:));
k = find(sms(11,:)==max_vonmises);                

%% Stiffness and Deflection

% Super Position Method

theta = ones(res,4);

% This section calculates the angular displacement as a function of
% distance from the left face of the shaft.
for j = 1:length(geo)
    
    % Shaft angle in xz plane due to flywheel pully force at right bearing
    theta(j,1) = (lfm(7,I)*geo(1,1)*(geo(2,1)+geo(3,1)))/(6*E*geo(7,j));
    % Shaft angle in xz plane due to flywheel pully force at left bearing
    theta(j,2) = (-2*lfm(7,I)*geo(1,1)*(geo(2,1)+geo(3,1)))/(6*E*geo(7,j));
    % Shaft angle in xy plane due to flywheel pully force at right bearing
    theta(j,3) = (lfm(1,I)*geo(2,1)^3)/(6*E*geo(7,j)*(geo(2,1)+geo(3,1)));
    % Shaft angle in xy plane due to flywheel pully force at left bearing
    theta(j,4) = (lfm(1,I)*geo(3,1))*(geo(3,1)^2-(geo(2,1)+geo(3,1))^2)/...
                 (6*E*geo(7,j)*(geo(2,1)+geo(3,1)));

end

% Theta minimum index (tmi) finds the row index for the values of theta
% that are closest to the allowable theta, so that it can be related to the
% minimum shaft radius by index.
tmi = ones(4,2);
[tmi(:,1),tmi(:,2)] = min(abs(theta_allow-abs(theta)));

% Finds the the largest of the angular deflects for each plane and bearing
% location, as that will be the limiting factor.
for j1 = 1:4
max_thetas = theta(tmi(j1,2),j1);
end
% Finds the largest deflection of all cases,then finds the indexs of that
% value in the theta matrix, so that that row index can be used to find the
% corresponding minimum shaft radius.
max_theta = max(max_thetas);
[row,col] = find(theta==max_theta);
min_rad_stiff = geo(5,col)*12;


crit = fcrit(lfm,geo,res,I,x,E);


%% Fatigue Analysis

% Fatigue infinite life
sep = 0.5*ultimate;
ka = 2.7*(ultimate*10^-3)^-0.265;
kb = 0.879*(geo(5,I)*2)^-0.107;
se = sep*ka*kb;
nil = se/sms(11,I);

% Fatigue finite life


%% Bearing Loads

% Radial bearing forces at for mininum radius
mag_force_lb = sqrt(lfm(10,I)^2 + lfm(9,I)^2);
mag_force_rb = sqrt(lfm(11,I)^2 + lfm(8,I)^2);



%% Plots
% Turn figures on or off
set(0,'DefaultFigureVisible','on');

xdist = x(1:res)*12;
xliney = zeros([res,1]);
radius = geo(5,:)*12;


figure('pos',[300 100 900 900]);

sp1 = subplot(3,2,1);
plot(sp1,xdist,sms(7,:));
title(sp1,'Bending Stress in x')
ylabel(sp1,'Stress (psi)')
xlabel(sp1,'Distance from left face of shaft (in)')
hold on
plot(sp1,xdist,xliney);
hold off

sp2 = subplot(3,2,3);
plot(sp2,xdist,sms(8,:));
title(sp2,'Shear Stress in xy')
ylabel(sp2,'Stress (psi)')
xlabel(sp2,'Distance from left face of shaft (in)')
hold on
plot(sp2,xdist,xliney);
hold off

sp3 = subplot(3,2,5);
plot(sp3,xdist,sms(9,:));
title(sp3,'Shear Stress in zx')
ylabel(sp3,'Stress (psi)')
xlabel(sp3,'Distance from left face of shaft (in)')
hold on
plot(sp3,xdist,xliney);
hold off

sp4 = subplot(3,2,2);
plot(sp4,xdist,sms(11,:));
title(sp4,'vonMises Stress as X')
ylabel(sp4,'Stress (psi)')
xlabel(sp4,'Distance from left face (in)')
hold on
plot(sp4,xdist,xliney);
hold off

sp5 = subplot(3,2,4);
plot(sp5,radius,sms(10,:));
xlim([.5 1]);
ylim([0 1E5]);
title(sp5,'vonMises Stress at xcrit')
ylabel(sp5,'Stress (psi)')
xlabel(sp5,'Radius of Shaft (in)')

sp6 = subplot(3,2,6);
plot(sp6,xdist,crit(1,:)*12);
xlabel(sp6,'Distance from left face (in)');
ylabel(sp6,'Deflection in y (in)');
hold on
plot(sp6,xdist,xliney)
hold off

%% Print Values

% Units
ft = 'ft';
ft2 = 'ft^2';
ft4 = 'ft^4';
lbf = 'lbf';
lbfft = 'lbf-ft';
psi = 'psi';

% Geometry variable names

units_geo = char(ft,ft,ft,ft,ft,ft2,ft4,ft4);
geo_table = table(geo(:,I),units_geo,'VariableNames',{'GEO' 'Units'},'RowNames',...
            {'a' 'b' 'c' 'l' 'R' 'As' 'Is' 'Js'});

% Loads, Forces, Moments variable names
units_lms = char(lbf,lbfft,lbfft,lbf,lbf,lbf,lbf,lbf,lbf,lbf,lbf,lbfft);
lms_table = table(lfm(:,k),units_lms,'VariableNames',{'LFM' 'Units'},'RowNames',...
            {'Fcy' 'Mc' 'Unused' 'Ws' 'Wf' 'Wp' 'Fp' 'Frby' 'Flby' 'Flbz'...
             'Frbz' 'Mp'});         

% Shear and Moments Vector variable names
units_sms = char(lbf,lbf,lbfft,lbfft,lbfft,psi,psi,psi,psi,psi,psi);
sms_table = table(sms(:,k),units_sms,'VariableNames',{'SMS' 'Units'},'RowNames',...
            {'Shear in y' 'Shear in z' 'Moment about x' 'Moment about y'...
             'Moment about z' 'Axial stress' 'Bending Stress in x'...
             'Shear stress xy' 'Shear stress zx' 'Vonmises for all radi'...
             'Vonmises at Critical location'});



disp('Minimum Radius for Infinite Life (in)');
disp(min_radius_stress);
disp(' ');
disp(geo_table);
disp(' ');
disp(lms_table); 
disp(' ');
disp(sms_table);