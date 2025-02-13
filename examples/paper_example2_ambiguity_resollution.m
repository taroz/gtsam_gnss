%% paper_example2_ambiguity_resolution.m
% Example 2 in PLANS paper (https://arxiv.org/abs/2502.08158)
%
% Compare the rate of resolution of integer ambiguity in the carrier phase
% between the two models
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

%% Graph Optimization
% k=1: Model 1, k=2: Model 2
for k=1:2
    % Create a factor graph container
    graph{k} = gtsam.NonlinearFactorGraph;

    % Initial factor/state
    initials{k} = gtsam.Values;

    for i=progress(1:n)
        % Insert initial node value
        initials{k}.insert(sym('x',i), x_ini(i,:)');
        initials{k}.insert(sym('b',i), b_ini(i,:)');
        initials{k}.insert(sym('v',i), vel_ini.xyz(i,:)');

        % Add prior factor
        graph{k}.add(gtsam.PriorFactorVector(sym('x',i), x_ini(i,:)', noise_x_ini));
        graph{k}.add(gtsam.PriorFactorVector(sym('b',i), b_ini(i,:)', noise_sigmas(sigma_b_ini(:,i))));
        graph{k}.add(gtsam.PriorFactorVector(sym('v',i), vel_ini.xyz(i,:)', noise_v_ini));

        % Unary factor
        for j=1:nsat
            losvecDD = [-exdd(i,j) -eydd(i,j) -ezdd(i,j)]'; % LOS vector from initial node to satellite

            % Pseudorange factor
            if ~isnan(obs.L1.resPdd(i,j))
                noise = noise_sigmas(sigmaP(i,j)); % Noise model
                noise_robust_P = noise_robust(huber(huber_P), noise); % Robust noise model

                % Add pseudorange factor
                graph{k}.add(gtsam_gnss.PseudorangeFactor_X(sym('x',i), losvecDD, obs.L1.resPdd(i,j), x_ini(i,:)', noise_robust_P));
            end

            % Carrier phase factor
            if ~isnan(obs.L1.resLdd(i,j))
                noise = noise_sigmas(sigmaL(i,j));
                noise_robust_L = noise_robust(huber(huber_L), noise);

                % Add carrier phase factor
                graph{k}.add(gtsam_gnss.CarrierPhaseFactor_XB(sym('x',i), sym('b',i), losvecDD, obs.L1.resLdd(i,j), j-1, obs.L1.lam(j), x_ini(i,:)', noise_robust_L));
            end

            if k==2
                % Doppler factor
                if ~isnan(resDsd(i,j))
                    noise = noise_sigmas(sigmaD(i,j)); % Noise model
                    noise_robust_D = noise_robust(huber(huber_D), noise); % Robust noise model

                    % Add Doppler factor
                    graph{k}.add(gtsam_gnss.DopplerFactor_V(sym('v',i), losvecDD, resDsd(i,j), vel_ini.xyz(i,:)', noise_robust_D));
                end
            end
        end

        % Binary (Between) factor
        if i>1
            if k==2
                % Motion factor
                graph{k}.add(gtsam_gnss.MotionFactor_XXVV(sym('x',i-1), sym('x',i), sym('v',i-1), sym('v',i), obs.dt, noise_motion));
            end
        end
    end

    %% Optimization
    optparameters = gtsam.LevenbergMarquardtParams;
    optparameters.setVerbosity('TERMINATION');
    optparameters.setMaxIterations(1000);
    optimizer = gtsam.LevenbergMarquardtOptimizer(graph{k}, initials{k}, optparameters);

    % Optimize!
    disp('optimization... ');
    fprintf('Initial Error: %.2f\n',optimizer.error);
    tic;
    results{k} = optimizer.optimize();
    fprintf('Error: %.2f Iter: %d\n',optimizer.error,optimizer.iterations);
    toc;

    % Marginalization for estimating covariance
    marginals{k} = gtsam.Marginals(graph{k}, results{k});

    % Retrieving the estimated value
    x_est{k} = NaN(n,3);
    b_est{k} = NaN(n,nsat);
    covxb_est{k} = zeros(nsat+3,nsat+3,n);
    for i=1:n
        x_est{k}(i,:) = results{k}.atVector(sym('x',i))';
        b_est{k}(i,:) = results{k}.atVector(sym('b',i))';

        % Covariance (X+B)
        kv = gtsam.KeyVector();
        kv.push_back(sym('b',i));
        kv.push_back(sym('x',i));
        covxb_est{k}(:,:,i) = marginals{k}.jointMarginalCovariance(kv).fullMatrix;
    end

    % Set the state that could not be estimated to NaN
    x_est{k}(x_est{k}==x_ini) = NaN; % Float solution
    pos_est{k} = gt.Gpos(x_est{k},"xyz",orgllh,"llh");
    err_est{k} = pos_est{k}-sol_ref.pos; % Float solution error

    %% Integer ambiguity resolution
    minobs_th = 5; % Minimum number of observations for ambiguity resolution
    ratio_th = 2.0; % Threshold for ratio test

    % Integer Least Squares (ILS) based on LAMBDA method
    [xa_est{k}, ba_est{k}, ratio{k}] = ils_lambda(x_est{k}, covxb_est{k}, b_est{k}, ratio_th, minobs_th);

    idxfix{k} = ratio{k}>ratio_th; % Index of fixed solution
    posa_est{k} = gt.Gpos(xa_est{k},"xyz",orgllh,"llh"); % Fixed and float solution
    erra_est{k} = posa_est{k}-sol_ref.pos;  % Fixed and float solution error

end

%% Plot
% Fixed rate
nfix1 = nnz(idxfix{1});
fixratestr = sprintf('Model 1, Fix rate: %.1f%% (%d/%d)',nfix1/n*100,nfix1,n);
fprintf('%s\n',fixratestr);

nfix2 = nnz(idxfix{2});
fixratestr = sprintf('Model 2, Fix rate: %.1f%% (%d/%d)',nfix2/n*100,nfix2,n);
fprintf('%s\n',fixratestr);

% Ratio
figure;
plot(1:n,ratio{1},'.-b'); grid on; hold on
plot(1:n,ratio{2},'.-r');
plot([1 n],[ratio_th ratio_th],'k-');
xlim([1 n])
title("Ratio, "+fixratestr)
ylabel('Ratio');
legend("Model 1 (w/o velocity)","Model 2 (w/ velocity)")
fontsize(12,"points");

% Compute CDF
cdf = 100*(1:n)' / n;
e3d_est1 = sort(erra_est{1}.d3);
e3d_est2 = sort(erra_est{2}.d3);

figure;
plot(e3d_est1, cdf, "b-","LineWidth",2);
grid on; hold on;
plot(e3d_est2, cdf, "r-","LineWidth",2);
legend("Model 1 (w/o velocity)","Model 2 (w/ velocity)")
xlim([0.1 10])
ylim([0 100])
xlabel("3D position error (m)");
ylabel("Cumulative probability (%)");
fontsize(12,"points");

% Print CDF at 0.5m, 1m, 5m
idx = ~isnan(e3d_est1);
fprintf("Model1: CDF: 0.5m: %.1f%%, 1m: %.1f%%, 5m: %.1f%%\n", interp1(e3d_est1(idx), cdf(idx), 0.5),...
                                                               interp1(e3d_est1(idx), cdf(idx), 1.0),...
                                                               interp1(e3d_est1(idx), cdf(idx), 5.0));
idx = ~isnan(e3d_est2);
fprintf("Model2: CDF: 0.5m: %.1f%%, 1m: %.1f%%, 5m: %.1f%%\n", interp1(e3d_est2(idx), cdf(idx), 0.5),...
                                                               interp1(e3d_est2(idx), cdf(idx), 1.0),...
                                                               interp1(e3d_est2(idx), cdf(idx), 5.0));