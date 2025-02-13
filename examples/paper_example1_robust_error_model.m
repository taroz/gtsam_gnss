%% paper_example1_robust_error_model.m
% Example 1 in PLANS paper (https://arxiv.org/abs/2502.08158)
%
% Estimate position (x) and clock (c) using Pseudorange (P) and
% compare with and without robust error model
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

%% Select observations for position computation
SNR_TH = 35; % SNR threshold (dBHz)
EL_TH = 15; % Elevation angle threshold (deg)

mask = obs.L1.S<SNR_TH | sat.el<EL_TH;
obs.mask(mask); % Mask observation

%% Simple elevation angle dependent model
varP90 = 2.0^2; % Pseudorange variance (m)^2
sigmaP = sqrt(varP90./sind(sat.el));

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

% Initial noise of x (3D position)
sigma_x_ini  = Inf*ones(3,1);
noise_x_ini = noise_sigmas(sigma_x_ini);

% Initial noise of c (clock)
sigma_c_ini  = 1e6*ones(nsys,1);
noise_c_ini = noise_sigmas(sigma_c_ini);

% Between noise of c (clock factor)
sigma_between_c = [100; zeros(nsys-1,1)];
noise_between_c = noise_sigmas(sigma_between_c);
sigma_between_c_jump = [1e6; zeros(nsys-1,1)]; % When clock jumping
noise_between_c_jump = noise_sigmas(sigma_between_c_jump);

%% Graph Optimization
% k=1: witout robust model, k=2 with robust error model
for k=1:2
    % Create a factor graph container
    graph{k} = gtsam.NonlinearFactorGraph;

    % Initial factor/state
    initials{k} = gtsam.Values;

    for i=progress(1:n)
        % Insert initial node value
        initials{k}.insert(sym('x',i), x_ini(i,:)');
        initials{k}.insert(sym('c',i), c_ini(i,:)');

        % Add prior factor
        graph{k}.add(gtsam.PriorFactorVector(sym('x',i), x_ini(i,:)', noise_x_ini));
        graph{k}.add(gtsam.PriorFactorVector(sym('c',i), c_ini(i,:)', noise_c_ini));

        % Unary factor
        for j=1:nsat
            losvec = [-sat.ex(i,j) -sat.ey(i,j) -sat.ez(i,j)]'; % LOS vector from initial node to satellite

            % Pseudorange factor
            if ~isnan(obs.L1.resPc(i,j))
                noise = noise_sigmas(sigmaP(i,j)); % Noise model
                noise_robust_P = noise_robust(huber(huber_P), noise); % Robust noise model

                % Add pseudorange factor
                if k==1
                    % Without robust error model
                    graph{k}.add(gtsam_gnss.PseudorangeFactor_XC(sym('x',i), sym('c',i), losvec, obs.L1.resPc(i,j), sys2sigtype(obs.sys(j)), x_ini(i,:)', noise));
                else
                    % With robust error model
                    graph{k}.add(gtsam_gnss.PseudorangeFactor_XC(sym('x',i), sym('c',i), losvec, obs.L1.resPc(i,j), sys2sigtype(obs.sys(j)), x_ini(i,:)', noise_robust_P));
                end
            end
        end

        % Binary (Between) factor
        if i>1
            % Add clock factor
            if clockjump(i)
                graph{k}.add(gtsam_gnss.ClockFactor_CC(sym('c',i-1), sym('c',i), noise_between_c_jump)); % When clock jumping
            else
                graph{k}.add(gtsam_gnss.ClockFactor_CC(sym('c',i-1), sym('c',i), noise_between_c));
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
    results = optimizer.optimize();
    fprintf('Error: %.2f Iter: %d\n',optimizer.error,optimizer.iterations);
    toc;

    % Retrieving the estimated value
    x_est{k} = NaN(n,3);
    c_est{k} = NaN(n,nsys);
    for i=1:n
        x_est{k}(i,:) = results.atVector(sym('x',i))';
        c_est{k}(i,:) = results.atVector(sym('c',i))';
    end

    % Set the state that could not be estimated to NaN
    x_est{k}(x_est{k}==x_ini) = NaN;
    c_est{k}(c_est{k}==c_ini) = NaN;
    pos_est{k} = gt.Gpos(x_est{k},"xyz",orgllh,"llh");
end

%% Plot error
% Create gt.Gerr object
err_normal = pos_est{1}-sol_ref.pos;
err_robust = pos_est{2}-sol_ref.pos;

% Compute CDF
cdf = 100*(1:n)' / n;
e3d_normal = sort(err_normal.d3);
e3d_robust = sort(err_robust.d3);

figure;
plot(e3d_normal, cdf, "b-","LineWidth",2);
grid on; hold on;
plot(e3d_robust, cdf, "r-","LineWidth",2);
legend("w/o Robust Error Model","w/ Robust Error Model")
xlim([0 10])
xlabel("3D position error (m)");
ylabel("Cumulative probability (%)");
fontsize(12,"points");

% Print CDF at 3m, 5m, 10m
idx = ~isnan(e3d_normal);
fprintf("w/o Robust Error Model CDF: 3m: %.1f%%, 5m: %.1f%%, 10m: %.1f%%\n", interp1(e3d_normal(idx), cdf(idx), 3.0),...
                                                                             interp1(e3d_normal(idx), cdf(idx), 5.0),...
                                                                             interp1(e3d_normal(idx), cdf(idx), 10.0));
idx = ~isnan(e3d_robust);
fprintf("w/  Robust Error Model CDF: 3m: %.1f%%, 5m: %.1f%%, 10m: %.1f%%\n", interp1(e3d_robust(idx), cdf(idx), 3.0),...
                                                                             interp1(e3d_robust(idx), cdf(idx), 5.0),...
                                                                             interp1(e3d_robust(idx), cdf(idx), 10.0));

%% Plot position/clock
figure;
tiledlayout(3,1,"TileSpacing","tight")
nexttile([2 1])
plot(pos_est{1}.east, pos_est{1}.north,"b.-");
grid on; hold on;
plot(pos_est{2}.east, pos_est{2}.north,"r.-");
legend("w/o Robust Error Model","w/ Robust Error Model")
axis equal
xlabel("East (m)");
ylabel("North (m)");
set(gca,"FontSize",14);
nexttile
plot(err_normal.d3,"b.-");
grid on; hold on;
plot(err_robust.d3,"r.-");
ylabel("3D error (m)");
ylim([0 100])
xlim([0 err_robust.n])
fontsize(12,"points");