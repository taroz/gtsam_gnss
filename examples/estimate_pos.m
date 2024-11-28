%% estimate_pos.m
% Estimate position and clock using PseudorangeFactor_XC
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

%% Read reference position
ref = readmatrix(datapath+"reference.csv");
orgllh = ref(1,3:5); % Origin of the ENU coordinate system

% Creating a Gt.Sol object
sol_ref = gt.Gsol(gt.Gtime(ref(:,1),ref(:,2)), gt.Gpos(ref(:,3:5),"llh"));
sol_ref = sol_ref.sameTime(obs.time); % 5Hz to 1Hz
sol_ref.setOrg(orgllh, "llh");
assert(obs.n==sol_ref.n)

%% Read single point positioning result
sol_spp = gt.Gsol(datapath+"rover_1Hz_spp.pos");
sol_spp = sol_spp.fixedInterval(sol_spp.dt);
sol_spp.setOrg(orgllh, "llh");
assert(obs.n==sol_spp.n)

%% Initial position
% Set the result of single point positioning as the initial value of the node
x_ini = fillmissing(sol_spp.pos.xyz,"linear","EndValues","extrap")';

%% Compute residuals
% Compute the pseudorange residual at the initial node 
sat = gt.Gsat(obs, nav);
sat.setRcvPos(gt.Gpos(x_ini',"xyz"));
obs = obs.residuals(sat); % Compute residuals

%% Select observations for position computation
SNR_TH = 35; % SNR threshold (dBHz)
EL_TH = 15; % Elevation angle threshold (deg)

mask = obs.L1.S<SNR_TH | sat.el<EL_TH;
obs.mask(mask); % Mask observation

%% Simple elevation angle dependent model
varP90 = 3.0^2; % Pseudorange variance (m)^2
sigmaP = sqrt(varP90./sind(sat.el));

%% parameters for graph optimization
noise_sigmas = @gtsam.noiseModel.Diagonal.Sigmas;
noise_robust = @gtsam.noiseModel.Robust.Create; % Robust error model (M-estimator)
huber = @gtsam.noiseModel.mEstimator.Huber.Create; % Huber function
sym = @gtsam.symbol;
huber_P = 1.234; % Huber function parameter

% x (3D position)
sigma_x_ini  = 1e3*ones(3,1);
noise_x_ini = noise_sigmas(sigma_x_ini);

% c (clock)
c_ini = zeros(7,obs.n);
sigma_c_ini  = 1e6*ones(7,1);
noise_c_ini = noise_sigmas(sigma_c_ini);

%% Graph Construction
% Create a factor graph container
graph = gtsam.NonlinearFactorGraph;

% Initial factor/state
initials = gtsam.Values;

for i=progress(1:obs.n)
    % Insert initial node value
    initials.insert(sym('x',i), x_ini(:,i));
    initials.insert(sym('c',i), c_ini(:,i));

    % Add prior factor
    graph.add(gtsam.PriorFactorVector(sym('x',i), x_ini(:,i), noise_x_ini));
    graph.add(gtsam.PriorFactorVector(sym('c',i), c_ini(:,i), noise_c_ini));
    
    for j=1:obs.nsat
        losvec = [-sat.ex(i,j) -sat.ey(i,j) -sat.ez(i,j)]'; % LOS vector from initial node to satellite
        if ~isnan(obs.L1.resPc(i,j))
            noise = noise_sigmas(sigmaP(i,j)); % Noise model
            noise_rubust = noise_robust(huber(huber_P), noise); % Robust noise model
            
            % Add pseudorange factor
            graph.add(gtsam_gnss.PseudorangeFactor_XC(sym('x',i), sym('c',i), losvec, obs.L1.resPc(i,j), sys2sigtype(obs.sys(j)), x_ini(:,i), noise_rubust));
        end
    end
end

%% Optimization
optparameters = gtsam.LevenbergMarquardtParams;
optparameters.setVerbosity('TERMINATION');
optparameters.setMaxIterations(1000);
optimizer = gtsam.LevenbergMarquardtOptimizer(graph, initials, optparameters);

% optimize!
disp('optimization... ');
fprintf('Initial Error: %.2f\n',optimizer.error);
tic;
results = optimizer.optimize();
fprintf('Error: %.2f Iter: %d\n',optimizer.error,optimizer.iterations);
toc;

% Retrieving the estimated value
x_est = NaN(obs.n,3);
clk_est = NaN(obs.n,7);
for i=1:obs.n
    x_est(i,:) = results.atVector(sym('x',i))';
    clk_est(i,:) = results.atVector(sym('c',i))';
end
pos_est = gt.Gpos(x_est,"xyz",orgllh,"llh");

%% Plot position
figure;
plot(sol_spp.pos.east, sol_spp.pos.north,"b.-");
grid on; hold on;
plot(pos_est.east, pos_est.north,"r.-");
legend("SPP","FGO")
axis equal
xlabel("East m");
ylabel("North m");
set(gca,"FontSize",14);

%% Plot error
% Create gt.Gerr object
err_spp = sol_spp.pos-sol_ref.pos;
err_est = pos_est-sol_ref.pos;

% Compute CDF
cdf = 100*(1:err_spp.n)' / err_spp.n;
e3d_spp = sort(err_spp.d3);
e3d_est = sort(err_est.d3);

figure;
plot(e3d_spp, cdf, "b-","LineWidth",2);
grid on; hold on;
plot(e3d_est, cdf, "r-","LineWidth",2);
legend("SPP","FGO")
xlim([0 30])
xlabel("3D position error m");
ylabel("Cumulative probability %");

% print CDF at 3m, 5m, 10m
idx = ~isnan(e3d_spp);
fprintf("SPP CDF: 3m: %.1f%%, 5m: %.1f%%, 10m: %.1f%%\n", interp1(e3d_spp(idx), cdf(idx), 3.0),...
                                                          interp1(e3d_spp(idx), cdf(idx), 5.0),...
                                                          interp1(e3d_spp(idx), cdf(idx), 10.0));
idx = ~isnan(e3d_est);
fprintf("FGO CDF: 3m: %.1f%%, 5m: %.1f%%, 10m: %.1f%%\n", interp1(e3d_est(idx), cdf(idx), 3.0),...
                                                          interp1(e3d_est(idx), cdf(idx), 5.0),...
                                                          interp1(e3d_est(idx), cdf(idx), 10.0));
