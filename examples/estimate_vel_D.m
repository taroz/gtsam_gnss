%% estimate_vel_D.m
% Estimate velocity (v) and clock drift (d) using Doppler (D)
% Used factors: DopplerFactor_VD
% Author: Taro Suzuki
clear; close all;

%% Path
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

%% Read reference position and velocity
ref = readmatrix(datapath+"reference.csv");
orgllh = ref(1,3:5); % Origin of the ENU coordinate system

% Creating a gt.Gsol object
time_ref = gt.Gtime(ref(:,1),ref(:,2));
sol_ref = gt.Gsol(time_ref, gt.Gpos(ref(:,3:5),"llh"));
sol_ref.vel = gt.Gvel(ref(:,12:14),"enu",orgllh,"llh");
sol_ref.vcov = gt.Gcov(zeros(size(ref,1),6),"xyz");
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
v_ini = zeros(n,3); % Initial velocity
d_ini = zeros(n,1); % Initial receiver clock drift

%% Compute residuals
% Compute pseudorange residuals at the initial node
pos_ini = gt.Gpos(x_ini,"xyz");
vel_ini = gt.Gvel(v_ini,"xyz");
sat = gt.Gsat(obs, nav);
sat.setRcvPosVel(pos_ini, vel_ini);
obs = obs.residuals(sat); % Compute residuals

%% Select observations for position computation
SNR_TH = 35; % SNR threshold (dBHz)
EL_TH = 15; % Elevation angle threshold (deg)

mask = obs.L1.S<SNR_TH | sat.el<EL_TH;
obs.mask(mask); % Mask observation

%% Simple elevation angle dependent model
varD90 = 0.2^2; % Doppler variance (m/s)^2
sigmaD = sqrt(varD90./sind(sat.el));

%% Parameters for graph optimization
noise_sigmas = @gtsam.noiseModel.Diagonal.Sigmas;
noise_robust = @gtsam.noiseModel.Robust.Create; % Robust error model (M-estimator)
huber = @gtsam.noiseModel.mEstimator.Huber.Create; % Huber function
sym = @gtsam.symbol;

huber_D = 1.234; % Huber function parameter for Doppler factor

% Initial noise of v (3D velocity)
sigma_v_ini  = 1e3*ones(3,1);
noise_v_ini = noise_sigmas(sigma_v_ini);

% Initial noise of d (clock drift)
sigma_d_ini  = 1e3;
noise_d_ini = noise_sigmas(sigma_d_ini);

% Between noise of d (constant clock drift)
sigma_between_d = 0.1;
noise_between_d = noise_sigmas(sigma_between_d);

%% Graph Construction
% Create a factor graph container
graph = gtsam.NonlinearFactorGraph;

% Initial factor/state
initials = gtsam.Values;

for i=progress(1:n)
    % Insert initial node value
    initials.insert(sym('v',i), v_ini(i,:)');
    initials.insert(sym('d',i), d_ini(i,:)');

    % Add prior factor
    graph.add(gtsam.PriorFactorVector(sym('v',i), v_ini(i,:)', noise_v_ini));
    graph.add(gtsam.PriorFactorVector(sym('d',i), d_ini(i,:)', noise_d_ini));

    % Unary factor
    for j=1:nsat
        losvec = [-sat.ex(i,j) -sat.ey(i,j) -sat.ez(i,j)]'; % LOS vector from initial node to satellite

        % Doppler factor
        if ~isnan(obs.L1.resD(i,j))
            noise = noise_sigmas(sigmaD(i,j)); % Noise model
            noise_robust_D = noise_robust(huber(huber_D), noise); % Robust noise model

            % Add Doppler factor
            graph.add(gtsam_gnss.DopplerFactor_VD(sym('v',i), sym('d',i), losvec, obs.L1.resD(i,j), v_ini(i,:)', noise_robust_D));
        end
    end

    % Binary (Between) factor
    if i>1
        % Add constraint between clock drift
        graph.add(gtsam.BetweenFactorVector(sym('d',i-1), sym('d',i), 0, noise_between_d));
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
v_est = NaN(n,3);
d_est = NaN(n,1);
for i=1:n
    v_est(i,:) = results.atVector(sym('v',i))';
    d_est(i,:) = results.atVector(sym('d',i))';
end
vel_est = gt.Gvel(v_est,"xyz",orgllh,"llh");

%% Plot velocity/clock drift
figure;
nexttile
plot(d_est,'.-'); grid on;
title('Receiver clock drift');
ylabel('Clock Drift (m/s)');
nexttile
plot(vel_est.v3,'.-'); grid on;
title('3D vellocity');
ylabel('3D vellocity (m/s)');

%% Plot error
% Create gt.Gerr object
err_spp = sol_spp.vel-sol_ref.vel;
err_est = vel_est-sol_ref.vel;

% Compute CDF
cdf = 100*(1:n)' / n;
e3d_spp = sort(err_spp.d3);
e3d_est = sort(err_est.d3);

figure;
plot(e3d_spp, cdf, "b-","LineWidth",2);
grid on; hold on;
plot(e3d_est, cdf, "r-","LineWidth",2);
legend("SPP","FGO")
xlim([0 2])
xlabel("3D velocity error (m/s)");
ylabel("Cumulative probability (%)");

% print CDF at 0.2m/s, 0.5m/s, 1.0m/s
idx = ~isnan(e3d_spp);
fprintf("SPP CDF: 0.2m/s: %.1f%%, 0.5m/s: %.1f%%, 1.0m/s: %.1f%%\n", interp1(e3d_spp(idx), cdf(idx), 0.2),...
                                                          interp1(e3d_spp(idx), cdf(idx), 0.5),...
                                                          interp1(e3d_spp(idx), cdf(idx), 1.0));
idx = ~isnan(e3d_est);
fprintf("FGO CDF: 0.2m/s: %.1f%%, 0.5m/s: %.1f%%, 1.0m/s: %.1f%%\n", interp1(e3d_est(idx), cdf(idx), 0.2),...
                                                          interp1(e3d_est(idx), cdf(idx), 0.5),...
                                                          interp1(e3d_est(idx), cdf(idx), 1.0));
