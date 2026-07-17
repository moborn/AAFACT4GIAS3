function varargout = rigidreg(data, target)
% fitRigid fits rigid transform parameters [tx,ty,tz,rx,ry,rz] to map data->target
% Minimal port of the provided Python implementation.


% Ensure inputs are double
data = double(data);
target = double(target);
xtol = 1e-12;
rotcentre = mean(data,1);
sample = 10000;
verbose = false;
output_errors = 0;
maxfev = 5000;
maxfun = 0;
epsfcn = 1e-9;
% initial translation guess: difference of centroids (Python: target_pts.mean(0) - source_pts.mean(0))
t0 = mean(target,1) - mean(data,1);
r0 = [0, 0, 0];
% combine initial translation and rotation guesses into single parameter vector
t0 = [t0, r0];
% Sampling helper
    function S = sampleData(X, n)
        idx = randperm(size(X,1), n);
        S = X(idx, :);
    end

% Apply sampling logic
if ~isempty(sample) && sample < size(data,1) && sample < size(target,1)
    D = sampleData(data, sample);
    T = sampleData(target, sample);
elseif ~isempty(sample) && sample >= size(data,1) || sample >= size(target,1)
    if size(data,1) > size(target,1)
        D = sampleData(data, size(target,1));
        T = target;
    else
        D = data;
        T = sampleData(target, size(data,1));
    end
else
    D = data;
    T = target;
end

% Initial parameters
if isempty(t0)
    t0 = zeros(1,6);
else
    t0 = double(t0(:))';
end

% rotation centre
if isempty(rotcentre)
    rotcentre = mean(D,1);
else
    rotcentre = double(rotcentre(:))';
end

% Objective: return residuals per point or scalar depending on sizes
if size(data,1) >= numel(t0)
    obj = @(x) residuals_per_point(D, T, x, rotcentre);
else
    obj = @(x) sum(residuals_per_point(D, T, x, rotcentre));
end

% initial RMS
r0 = obj(t0);
rms0 = sqrt(mean(r0(:)));
if verbose, fprintf('initial RMS: %g\n', rms0); end

% Optimization
if size(data,1) >= numel(t0)
    % use lsqnonlin if available, otherwise lsqcurvefit style via lsqnonlin
    % Create options to display iteration output
    opts = optimoptions('lsqnonlin','Display','iter','FunctionTolerance',xtol, ...
        'FiniteDifferenceStepSize', epsfcn, ...
        'FiniteDifferenceType', 'central', ...
        'Algorithm', 'levenberg-marquardt');
    opts.Display = "iter";
    if ~isempty(maxfev), opts.MaxFunctionEvaluations = maxfev; end

    lb = []; ub = [];
    x_opt = lsqnonlin(obj, t0, lb, ub, opts);
else
    % use fminsearch to minimize scalar objective
    opts = optimset('TolX', xtol, 'MaxFunEvals', maxfun, 'Display', 'off');
    if verbose, opts.Display = 'iter'; end
    x_opt = fminsearch(obj, t0, opts);
end

r_opt = obj(x_opt);
rms_opt = sqrt(mean(r_opt(:)));
if verbose, fprintf('final RMS: %g\n', rms_opt); end

data_fitted = transformRigid3DAboutP(data, x_opt, rotcentre);

if output_errors
    varargout{1} = x_opt;
    varargout{2} = data_fitted;
    varargout{3} = [rms0, rms_opt];
else
    varargout{1} = x_opt;
    varargout{2} = data_fitted;
end
end

function r = residuals_per_point(D, T, x, rotcentre)
% x = [tx ty tz rx ry rz]
dt = transformRigid3DAboutP(D, x, rotcentre);

d2 = sum((dt - T).^2, 2);
r = d2;
end

function Xr = transformRigid3DAboutP(X, x, P)
% Applies rigid transform about point P.
% x = [tx ty tz rx ry rz], rotations in radians about x,y,z (intrinsic)
t = x(1:3);
r = x(4:6);

% shift to rotation centre
Xc = bsxfun(@minus, X, P);

% rotation matrices
Rx = [1 0 0; 0 cos(r(1)) -sin(r(1)); 0 sin(r(1)) cos(r(1))];
Ry = [cos(r(2)) 0 sin(r(2)); 0 1 0; -sin(r(2)) 0 cos(r(2))];
Rz = [cos(r(3)) -sin(r(3)) 0; sin(r(3)) cos(r(3)) 0; 0 0 1];

R = Rz * Ry * Rx; % apply Rx then Ry then Rz (adjust if different convention)

Xr = (R * Xc')' + P + repmat(t, size(X,1), 1);
end