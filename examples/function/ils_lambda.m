function [xaest, baest, ratio] = ils_lambda(xest, covest, best, ratio_th, minobs_th)
% Integer least-squares by LAMBDA method
% Author: Taro Suzuki
arguments
    xest double                 % Float solution (nepoch x 3)
    covest double               % Covariance matrix ((3+nobs) x (3+nobs) x nepoch )
    best double                 % Float ambiguities (nepoch x nobs)
    ratio_th (1,1) double = 2.0 % Ratio test thresholds
    minobs_th (1,1) double = 5  % Minimum number of observation for AR
end

nobs = size(covest,1)-3; % Number of observations
nepoch = size(xest,1);   % Number of epochs
assert(nobs==size(best,2));
assert(nepoch==size(covest,3) && nepoch==size(best,1));

xaest = xest; % Fixed position
baest = NaN(nepoch,nobs); % Fixed integer ambiguity
ratio = zeros(nepoch,1);  % Ratio

% Integer ambiguity resolusiton
for i=1:nepoch
    j = find(best(i,:)~=0); % Index of valid ambiguities
    if length(j)<=minobs_th
        continue;
    end
    Qfafa = covest(j,j,i); % Covariance between ambiguity
    Qxfa = covest(end-2:end,j,i); % Covariance between position and ambiguity

    % Call lambda in RTKLIB
    [a,s] = rtklib.lambda(2,best(i,j),Qfafa);

    % Compute ratio
    ratio(i,1) = s(2)/s(1);

    % Ratio test
    if ratio(i)>ratio_th
        aa = (best(i,j)-a(1,:))';
        xaest(i,:) = xest(i,:)-(Qxfa/Qfafa*aa)'; % Fixed positon
        baest(i,j) = a(1,:);  % Fixed ambiguity
    end
end
