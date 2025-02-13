%% estimate_pos_PDC.m
% Estimate position (x) and clock (c) using Pseudorange (P), Doppler (D),
% and Time-differenced carrier phase (C)
% Used factors: PseudorangeFactor_XC, ClockFactor_CC,
%               DopplerFactor_XXCC, TDCPFactor_XXCC
% Author: Taro Suzuki
clear; close all;

%% Path
addpath ./function
if ispc
    addpath C:\'Program Files (x86)'\GTSAM\gtsam_toolbox\
else
    addpath /usr/local/gtsam_toolbox/
end
datapath = "./data/";

%% Read RINEX observation/navigation file
obs = gt.Gobs(datapath+"rover_1Hz.obs");
nav = gt.Gnav(datapath+"base.nav");

% Make the time step of the observation constant (insert NaN)
obs = obs.fixedInterval(obs.dt);

n = obs.n; % Number of epochs
nsat = obs.nsat; % Number of satellites
nsys = 5; % Number of satellite systems, G,R,E,Q,C

%% Read reference position
ref = readmatrix(datapath+"reference.csv");
orgllh = ref(1,3:5); % Origin of the ENU coordinate system

% Creating a gt.Gsol object
time_ref = gt.Gtime(ref(:,1),ref(:,2));
sol_ref = gt.Gsol(time_ref, gt.Gpos(ref(:,3:5),"llh"));
sol_ref = sol_ref.sameTime(obs.time); % 5Hz to 1Hz
sol_ref.setOrg(orgllh, "llh");
assert(sol_ref.n==n)

%% Read single point positioning result
sol_spp = gt.Gsol(datapath+"rover_1Hz_spp.pos");
sol_spp = sol_spp.fixedInterval(sol_spp.dt);
sol_spp.setOrg(orgllh, "llh");
assert(sol_spp.n==n)

%% Initial value of nodes
% Set the result of single point positioning as the initial value of the node
x_ini = fillmissing(sol_spp.pos.xyz,"linear"); % Initial position
c_ini = zeros(n,nsys); % Initial receiver clock

%% Compute residuals
% Compute pseudorange residuals at the initial node
pos_ini = gt.Gpos(x_ini,"xyz");
sat = gt.Gsat(obs, nav);
sat.setRcvPos(pos_ini);
obs = obs.residuals(sat); % Compute residuals

%% Compute Doppler residuals
% Interpolating observation
ti = gt.Gtime(obs.time.t(1:end-1)+seconds(obs.dt/2)); % t+obs.dt/2
obsi = obs.interp(ti);
sati = gt.Gsat(obsi, nav);

% Doppler residuals are calculated to estimate the average velocity between epochs
pos_inii = pos_ini.interp(obs.time.t, ti.t);
vel_inii = gt.Gvel(diff(x_ini)/obs.dt,"xyz"); % Initial average velocity between epochs
sati.setRcvPosVel(pos_inii, vel_inii);
obsi = obsi.residuals(sati); % Compute residuals

%% Select observations for position computation
SNR_TH = 35; % SNR threshold (dBHz)
EL_TH = 15; % Elevation angle threshold (deg)

mask = obs.L1.S<SNR_TH | sat.el<EL_TH;
obs.mask(mask); % Mask observation
obs.maskLLI(); % Mask cycleslip using LLI flag

%% Simple elevation angle dependent model
varP90 = 3.0^2; % Pseudorange variance (m)^2
varD90 = 0.2^2; % Doppler variance (m/s)^2
varC90 = 0.05^2; % TDCP variance (m)^2
sigmaP = sqrt(varP90./sind(sat.el));
sigmaD = sqrt(varD90./sind(sat.el));
sigmaC = sqrt(varC90./sind(sat.el));

%% Check receiver clock jump
% Compute difference of avarage GPS pseudorange
dp = diff(obs.L1.P(:,obs.sys==gt.C.SYS_GPS));
meandP = mean(dp,2,"omitmissing");
clockjump = [false; meandP>1e5]; % Check pseudorange jump

%% Parameters for graph optimization
noise_sigmas = @gtsam.noiseModel.Diagonal.Sigmas;
noise_robust = @gtsam.noiseModel.Robust.Create; % Robust error model (M-estimator)
huber = @gtsam.noiseModel.mEstimator.Huber.Create; % Huber function
sym = @gtsam.symbol;

huber_P = 1.234; % Huber function parameter for pseudorange factor
huber_D = 1.234; % Huber function parameter for Doppler factor
huber_C = 1.234; % Huber function parameter for TDCP

% Initial noise of x (3D position)
sigma_x_ini  = 1e3*ones(3,1);
noise_x_ini = noise_sigmas(sigma_x_ini);

% Initial noise of v (3D velocity)
sigma_v_ini  = 1e3*ones(3,1);
noise_v_ini = noise_sigmas(sigma_v_ini);

% Initial noise of c (clock)
sigma_c_ini  = 1e6*ones(nsys,1);
noise_c_ini = noise_sigmas(sigma_c_ini);

% Between noise of c (clock factor)
sigma_between_c = [100; zeros(nsys-1,1)];
noise_between_c = noise_sigmas(sigma_between_c);
sigma_between_c_jump = [1e6; zeros(nsys-1,1)]; % When clock jumping
noise_between_c_jump = noise_sigmas(sigma_between_c_jump);

%% Graph Construction
% Create a factor graph container
graph = gtsam.NonlinearFactorGraph;

% Initial factor/state
initials = gtsam.Values;

for i=progress(1:n)
    % Insert initial node value
    initials.insert(sym('x',i), x_ini(i,:)');
    initials.insert(sym('c',i), c_ini(i,:)');

    % Add prior factor
    graph.add(gtsam.PriorFactorVector(sym('x',i), x_ini(i,:)', noise_x_ini));
    graph.add(gtsam.PriorFactorVector(sym('c',i), c_ini(i,:)', noise_c_ini));

    % Unary factor
    for j=1:nsat
        losvec = [-sat.ex(i,j) -sat.ey(i,j) -sat.ez(i,j)]'; % LOS vector from initial node to satellite

        % Pseudorange factor
        if ~isnan(obs.L1.resPc(i,j))
            noise = noise_sigmas(sigmaP(i,j)); % Noise model
            noise_robust_P = noise_robust(huber(huber_P), noise); % Robust noise model

            % Add pseudorange factor
            graph.add(gtsam_gnss.PseudorangeFactor_XC(sym('x',i), sym('c',i), losvec, obs.L1.resPc(i,j), sys2sigtype(obs.sys(j)), x_ini(i,:)', noise_robust_P));
        end
    end

    % Binary (Between) factor
    if i>1
        % Add clock factor
        if clockjump(i)
            graph.add(gtsam_gnss.ClockFactor_CC(sym('c',i-1), sym('c',i), noise_between_c_jump)); % When clock jumping
        else
            graph.add(gtsam_gnss.ClockFactor_CC(sym('c',i-1), sym('c',i), noise_between_c));

            % Doppler factor
            % DopplerFactor_XXCC uses Doppler observations as direct constraints on successive positions and clocks
            for j=1:nsat
                losvec = [-sati.ex(i-1,j) -sati.ey(i-1,j) -sati.ez(i-1,j)]'; % Average LOS vector from initial node to satellite
                resD = obsi.L1.resD(i-1,j); % Average Doppler residuals
                if ~isnan(resD)
                    noise = noise_sigmas(sigmaD(i,j)); % Noise model
                    noise_robust_D = noise_robust(huber(huber_D), noise); % Robust noise model

                    % Add Doppler factor
                    graph.add(gtsam_gnss.DopplerFactor_XXCC(sym('x',i-1), sym('x',i), sym('c',i-1), sym('c',i), losvec, resD, obs.dt, x_ini(i-1,:)', x_ini(i,:)', noise_robust_D));
                end
            end

            % TDCP factor
            for j=1:nsat
                losvec = [-sat.ex(i-1,j) -sat.ey(i-1,j) -sat.ez(i-1,j)]'; % LOS vector from initial node to satellite
                tdcp = obs.L1.resL(i,j)-obs.L1.resL(i-1,j); % Time-differenced carrier phase residuals
                if ~isnan(tdcp)
                    noise = noise_sigmas(sigmaC(i,j)); % Noise model
                    noise_robust_C = noise_robust(huber(huber_C), noise); % Robust noise model

                    % Add TDCP factor
                    graph.add(gtsam_gnss.TDCPFactor_XXCC(sym('x',i-1), sym('x',i), sym('c',i-1), sym('c',i), losvec, tdcp, x_ini(i-1,:)', x_ini(i,:)', noise_robust_C));
                end
            end
        end
    end
end

%% Optimization
optparameters = gtsam.LevenbergMarquardtParams;
optparameters.setVerbosity('TERMINATION');
optparameters.setMaxIterations(1000);
optimizer = gtsam.LevenbergMarquardtOptimizer(graph, initials, optparameters);

% Optimize!
disp('optimization... ');
fprintf('Initial Error: %.2f\n',optimizer.error);
tic;
results = optimizer.optimize();
fprintf('Error: %.2f Iter: %d\n',optimizer.error,optimizer.iterations);
toc;

% Retrieving the estimated value
x_est = NaN(n,3);
c_est = NaN(n,nsys);
for i=1:n
    x_est(i,:) = results.atVector(sym('x',i))';
    c_est(i,:) = results.atVector(sym('c',i))';
end

% Set the state that could not be estimated to NaN
x_est(x_est==x_ini) = NaN;
c_est(c_est==c_ini) = NaN;
pos_est = gt.Gpos(x_est,"xyz",orgllh,"llh");

%% Plot position/clock
figure;
plot(sol_spp.pos.east, sol_spp.pos.north,"b.-");
grid on; hold on;
plot(pos_est.east, pos_est.north,"r.-");
legend("SPP","FGO")
axis equal
xlabel("East (m)");
ylabel("North (m)");
set(gca,"FontSize",14);

figure;
nexttile
plot(c_est(:,1),'.-'); grid on;
title('Receiver clock bias');
ylabel('Clock Bias (m)');
legend('GPS L1');
nexttile
plot(c_est(:,2:end),'.-'); grid on;
ylabel('Clock Bias (m)');
legstr = ["GLO L1","GAL L1","QZS L1","BDS L1"];
legstr(all(isnan(c_est(:,2:end)),1)) = "";
legend(legstr)

%% Plot error
% Create gt.Gerr object
err_spp = sol_spp.pos-sol_ref.pos;
err_est = pos_est-sol_ref.pos;

% Compute CDF
cdf = 100*(1:n)' / n;
e3d_spp = sort(err_spp.d3);
e3d_est = sort(err_est.d3);

figure;
plot(e3d_spp, cdf, "b-","LineWidth",2);
grid on; hold on;
plot(e3d_est, cdf, "r-","LineWidth",2);
legend("SPP","FGO")
xlim([0 30])
xlabel("3D position error (m)");
ylabel("Cumulative probability (%)");

% print CDF at 3m, 5m, 10m
idx = ~isnan(e3d_spp);
fprintf("SPP CDF: 3m: %.1f%%, 5m: %.1f%%, 10m: %.1f%%\n", interp1(e3d_spp(idx), cdf(idx), 3.0),...
                                                          interp1(e3d_spp(idx), cdf(idx), 5.0),...
                                                          interp1(e3d_spp(idx), cdf(idx), 10.0));
idx = ~isnan(e3d_est);
fprintf("FGO CDF: 3m: %.1f%%, 5m: %.1f%%, 10m: %.1f%%\n", interp1(e3d_est(idx), cdf(idx), 3.0),...
                                                          interp1(e3d_est(idx), cdf(idx), 5.0),...
                                                          interp1(e3d_est(idx), cdf(idx), 10.0));
