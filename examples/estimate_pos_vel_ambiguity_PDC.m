%% estimate_pos_ambiguity_PDC.m
% Estimate position (x), velocity (v), and ambiguity (b) using
% DD Pseudorange (P), SD Doppler (D), and DD carrier phase (C)
% Used factors: PseudorangeFactor_X, DopplerFactor_V, CarrierPhaseFactor_XB
%               TDCPFactor_XX, MotionFactor
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
obsb = gt.Gobs(datapath+"base.obs");

% Make the time step of the observation constant (insert NaN)
obs = obs.fixedInterval(obs.dt);

% Exclude observations from GLONASS, QZSS, SBAS
exsys = [gt.C.SYS_GLO gt.C.SYS_QZS gt.C.SYS_SBS];
obs = obs.selectSat(~ismember(obs.sys,exsys));
obsb = obsb.selectSat(~ismember(obsb.sys,exsys));
[obs,obsb] = obs.commonSat(obsb);

n = obs.n; % Number of epochs
nsat = obs.nsat; % Number of satellites

%% Read reference position
ref = readmatrix(datapath+"reference.csv");
orgllh = ref(1,3:5); % Origin of the ENU coordinate system

% Creating a gt.Gsol object
time_ref = gt.Gtime(ref(:,1),ref(:,2));
sol_ref = gt.Gsol(time_ref, gt.Gpos(ref(:,3:5),"llh"));
sol_ref = sol_ref.sameTime(obs.time); % 5Hz to 1Hz
sol_ref.setOrg(orgllh, "llh");
assert(sol_ref.n==n)

% Base station position
basepos = readmatrix(datapath+"base_position.txt");

%% Read single point positioning result
sol_spp = gt.Gsol(datapath+"rover_1Hz_spp.pos");
sol_spp = sol_spp.fixedInterval(sol_spp.dt);
sol_spp.setOrg(orgllh, "llh");
assert(sol_spp.n==n)

%% Initial value of nodes
% Set the result of single point positioning as the initial value of the node
x_ini = fillmissing(sol_spp.pos.xyz,"linear"); % Initial position

%% Compute residuals
% Compute observation residuals at the initial node
pos_ini = gt.Gpos(x_ini,"xyz");
vel_ini = gt.Gvel(zeros(n,3),"xyz"); % Initial average velocity between epochs
sat = gt.Gsat(obs, nav);
sat.setRcvPosVel(pos_ini,vel_ini);
obs = obs.residuals(sat); % Compute residuals

% Compute observatrion residuals for base station
pos_base = gt.Gpos(basepos(2,:),"llh");
satb = gt.Gsat(obsb, nav);
satb.setRcvPos(pos_base);
obsb = obsb.residuals(satb); % Compute residuals

% Compute single-differenced observation with base station
obs = obs-obsb;

% Reference satellite
refsatidx = satb.referenceSat();

% Compute Double-differenced observation
obs = obs.doubleDifference(refsatidx);
exdd = sat.ex-sat.ex(:,refsatidx);
eydd = sat.ey-sat.ey(:,refsatidx);
ezdd = sat.ez-sat.ez(:,refsatidx);

%% Select observations for position computation
SNR_TH = 35; % SNR threshold (dBHz)
EL_TH = 15; % Elevation angle threshold (deg)

mask = obs.L1.S<SNR_TH | sat.el<EL_TH;
obs.mask(mask); % Observation masking using SNR and elevation angle
obs.maskLLI(); % Carrier phase masking using LLI (Loop Lock Indicator) flag
obsb.maskLLI();

% Compute Doppler difference from reference satellite to eliminate satellite clock drift
resDsd = obs.L1.resD-obs.L1.resD(:,refsatidx);

% Compute carrier phase difference from reference satellite to eliminate satellite clock
resLsd = obs.L1.resL-obs.L1.resL(:,refsatidx);

% Compute initial float ambiguities
b_ini = (obs.L1.resLdd-obs.L1.resPdd)./obs.L1.lam; % L1 float ambiguity
b_ini(isnan(b_ini)) = 0;

%% Simple elevation angle dependent model
varP90 = 1.0^2; % Pseudorange variance (m)^2
sigmaP = sqrt(varP90./sind(sat.el));
varL90 = 0.003^2; % Pseudorange variance (m)^2
sigmaL = sqrt(varL90./sind(sat.el));
varD90 = 0.2^2; % Pseudorange variance (m)^2
sigmaD = sqrt(varD90./sind(sat.el));

%% Parameters for graph optimization
noise_sigmas = @gtsam.noiseModel.Diagonal.Sigmas;
noise_robust = @gtsam.noiseModel.Robust.Create; % Robust error model (M-estimator)
huber = @gtsam.noiseModel.mEstimator.Huber.Create; % Huber function
sym = @gtsam.symbol;

huber_P = 1.234; % Huber function parameter for pseudorange factor
huber_L = 1.234; % Huber function parameter for carrier phase factor
huber_D = 1.234; % Huber function parameter for Doppler factor
huber_TDCP = 1.234; % Huber function parameter for TDCP factor

% Initial noise of x (3D position)
sigma_x_ini  = 1e2*ones(3,1);
noise_x_ini = noise_sigmas(sigma_x_ini);

% Initial noise of v (3D velocity)
sigma_v_ini  = 1e2*ones(3,1);
noise_v_ini = noise_sigmas(sigma_v_ini);

% Initial noise of b (ambiguity)
sigma_b_ini = Inf(nsat,n);
sigma_b_ini(b_ini'==0) = 1; % If the carrier phase cannot be obtained, its ambiguity is fixed to 0

% Between noise of x (motion factor)
sigma_motion = 0.01*ones(3,1);
noise_motion = noise_sigmas(sigma_motion);

% Between noise of b (constant ambiguity constraint)
sigma_between_b = 0.001;

%% Graph Optimization
% Create a factor graph container
graph = gtsam.NonlinearFactorGraph;

% Initial factor/state
initials = gtsam.Values;

for i=progress(1:n)
    % Insert initial node value
    initials.insert(sym('x',i), x_ini(i,:)');
    initials.insert(sym('b',i), b_ini(i,:)');
    initials.insert(sym('v',i), vel_ini.xyz(i,:)');

    % Add prior factor
    graph.add(gtsam.PriorFactorVector(sym('x',i), x_ini(i,:)', noise_x_ini));
    graph.add(gtsam.PriorFactorVector(sym('b',i), b_ini(i,:)', noise_sigmas(sigma_b_ini(:,i))));
    graph.add(gtsam.PriorFactorVector(sym('v',i), vel_ini.xyz(i,:)', noise_v_ini));

    % Unary factor
    for j=1:nsat
        losvecDD = [-exdd(i,j) -eydd(i,j) -ezdd(i,j)]'; % LOS vector from initial node to satellite

        % Pseudorange factor
        if ~isnan(obs.L1.resPdd(i,j))
            noise = noise_sigmas(sigmaP(i,j)); % Noise model
            noise_robust_P = noise_robust(huber(huber_P), noise); % Robust noise model

            % Add pseudorange factor
            graph.add(gtsam_gnss.PseudorangeFactor_X(sym('x',i), losvecDD, obs.L1.resPdd(i,j), x_ini(i,:)', noise_robust_P));
        end

        % Carrier phase factor
        if ~isnan(obs.L1.resLdd(i,j))
            noise = noise_sigmas(sigmaL(i,j));
            noise_robust_L = noise_robust(huber(huber_L), noise);

            % Add carrier phase factor
            graph.add(gtsam_gnss.CarrierPhaseFactor_XB(sym('x',i), sym('b',i), losvecDD, obs.L1.resLdd(i,j), j-1, -1, obs.L1.lam(j), x_ini(i,:)', noise_robust_L));
        end

        % Doppler factor
        if ~isnan(resDsd(i,j))
            noise = noise_sigmas(sigmaD(i,j)); % Noise model
            noise_robust_D = noise_robust(huber(huber_D), noise); % Robust noise model

            % Add Doppler factor
            graph.add(gtsam_gnss.DopplerFactor_V(sym('v',i), losvecDD, resDsd(i,j), vel_ini.xyz(i,:)', noise_robust_D));
        end
    end

    % Binary (Between) factor
    if i>1
        % Motion factor
        graph.add(gtsam_gnss.MotionFactor_XXVV(sym('x',i-1), sym('x',i), sym('v',i-1), sym('v',i), obs.dt, noise_motion));

        % Between ambiguity constraint
        idx = ~isnan(obs.L1.resLdd(i-1,:)) & ~isnan(obs.L1.resLdd(i,:));
        sig = Inf(nsat,1);
        sig(idx) = sigma_between_b;
        noise_between_b = noise_sigmas(sig); % Noise model

        % Add between ambiguity factor
        graph.add(gtsam.BetweenFactorVector(sym('b',i-1), sym('b',i), zeros(nsat,1), noise_between_b));

        for j=1:nsat
            losvecDD = [-exdd(i,j) -eydd(i,j) -ezdd(i,j)]'; % LOS vector from initial node to satellite
            tdcp = resLsd(i,j)-resLsd(i-1,j); % Time-differenced carrier phase residuals
            if ~isnan(tdcp) && tdcp~=0
                noise = noise_sigmas(sigmaL(i,j)); % Noise model
                noise_robust_TDCP = noise_robust(huber(huber_TDCP), noise); % Robust noise model

                % Add TDCP factor
                graph.add(gtsam_gnss.TDCPFactor_XX(sym('x',i-1), sym('x',i), losvecDD, tdcp, x_ini(i-1,:)', x_ini(i,:)', noise_robust_TDCP));
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

% Marginalization for estimating covariance
marginals = gtsam.Marginals(graph, results);

% Retrieving the estimated value
x_est = NaN(n,3);
b_est = NaN(n,nsat);
covxb_est = zeros(nsat+3,nsat+3,n);
for i=1:n
    x_est(i,:) = results.atVector(sym('x',i))';
    b_est(i,:) = results.atVector(sym('b',i))';

    % Covariance (X+B)
    kv = gtsam.KeyVector();
    kv.push_back(sym('b',i));
    kv.push_back(sym('x',i));
    covxb_est(:,:,i) = marginals.jointMarginalCovariance(kv).fullMatrix;
end

% Set the state that could not be estimated to NaN
x_est(x_est==x_ini) = NaN; % Float solution
pos_est = gt.Gpos(x_est,"xyz",orgllh,"llh");
err_est = pos_est-sol_ref.pos; % Float solution error

%% Integer ambiguity resolution
minobs_th = 5; % Minimum number of observations for ambiguity resolution
ratio_th = 2.0; % Threshold for ratio test

% Integer Least Squares (ILS) based on LAMBDA method
[xa_est, ba_est, ratio] = ils_lambda(x_est, covxb_est, b_est, ratio_th, minobs_th);

idxfix = ratio>ratio_th; % Index of fixed solution
posa_est = gt.Gpos(xa_est,"xyz",orgllh,"llh"); % Fixed and float solution
erra_est = posa_est-sol_ref.pos;  % Fixed and float solution error

%% Plot
% Fixed rate
nfix = nnz(idxfix);
fixratestr = sprintf('Fixed rate: %.1f%% (%d/%d)',nfix/n*100,nfix,n);
fprintf('%s\n',fixratestr);

% Ratio
figure;
plot(1:n,ratio,'.-r'); grid on; hold on
plot([1 n],[ratio_th ratio_th],'k-');
xlim([1 n])
title("Ratio, "+fixratestr)
ylabel('Ratio');
fontsize(12,"points");

% Compute CDF
cdf = 100*(1:n)' / n;
e3d_estf = sort(err_est.d3);  % Float solution 3D error
e3d_esta = sort(erra_est.d3); % Fixed solution 3D error

figure;
plot(e3d_estf, cdf, "b-","LineWidth",2);
grid on; hold on;
plot(e3d_esta, cdf, "r-","LineWidth",2);
legend("Float solution","Fixed solution")
xlim([0.1 10])
ylim([0 100])
xlabel("3D position error (m)");
ylabel("Cumulative probability (%)");
fontsize(12,"points");

% Print CDF at 0.5m, 1m, 5m
idx = ~isnan(e3d_estf);
fprintf("Float solution: CDF: 0.5m: %.1f%%, 1m: %.1f%%, 5m: %.1f%%\n", interp1(e3d_estf(idx), cdf(idx), 0.5),...
                                                                       interp1(e3d_estf(idx), cdf(idx), 1.0),...
                                                                       interp1(e3d_estf(idx), cdf(idx), 5.0));
idx = ~isnan(e3d_esta);
fprintf("Fixed solution: CDF: 0.5m: %.1f%%, 1m: %.1f%%, 5m: %.1f%%\n", interp1(e3d_esta(idx), cdf(idx), 0.5),...
                                                                       interp1(e3d_esta(idx), cdf(idx), 1.0),...
                                                                       interp1(e3d_esta(idx), cdf(idx), 5.0));