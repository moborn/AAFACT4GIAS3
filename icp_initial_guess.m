function [R, T, nodes2_aligned] = icp_initial_guess(nodes1, nodes2, name)
%ALIGNMESHESBYPCA Align two point meshes by matching principal axes and normal vector slopes.
%   Handles sign ambiguity without relying on unstable skewness metrics.
rng('default'); % Freezes random states so every run behaves identically
clear('pcregistericp'); % Forces MATLAB to refresh the ICP internal MEX memory
if size(nodes1,2) ~= 3 || size(nodes2,2) ~= 3
    error('Input node arrays must be Nx3.');
end
prevThreads = maxNumCompThreads(1);
% 1. Compute centroids and center points
c1 = mean(nodes1,1);
c2 = mean(nodes2,1);
X1 = bsxfun(@minus, nodes1, c1);
X2 = bsxfun(@minus, nodes2, c2);

% 2. Covariance and basic PCA
C1 = (X1' * X1) / max(1, size(X1,1)-1);
C2 = (X2' * X2) / max(1, size(X2,1)-1);
[V1, D1] = eig(C1);
[V2, D2] = eig(C2);

[eigvals1, idx1] = sort(diag(D1), 'descend');
[eigvals2, idx2] = sort(diag(D2), 'descend');
V1 = V1(:, idx1);
V2 = V2(:, idx2);

% Enforce baseline right-handed frames
if det(V1) < 0, V1(:,3) = -V1(:,3); end
if det(V2) < 0, V2(:,3) = -V2(:,3); end

% 3. Pre-compute Native Surface Normals
cloud1 = pointCloud(nodes1);
cloud2 = pointCloud(nodes2);
normals1 = pcnormals(cloud1, 30);
normals2 = pcnormals(cloud2, 30);
cloud1.Normal = normals1;
cloud2.Normal = normals2;
% CRITICAL FIX: Force all calculated surface normals to point OUTWARD from the bone center.
% This stops MATLAB's native pcnormals from flipping directions randomly.
for i = 1:size(X1,1)
    if sum(normals1(i,:) .* X1(i,:)) < 0, normals1(i,:) = -normals1(i,:); end
end
for i = 1:size(X2,1)
    if sum(normals2(i,:) .* X2(i,:)) < 0, normals2(i,:) = -normals2(i,:); end
end
cloud1.Normal = normals1;
cloud2.Normal = normals2;
% 4. Explicitly define the 4 valid non-reflective quadrant sign permutations
% All axis permutations
axis_permutations = perms(1:3);

% All non-reflective sign choices
sign_flips = [
     1  1  1;
     1 -1 -1;
    -1  1 -1;
    -1 -1  1
];

bestNormalScore = -Inf; % Initialize to negative infinity to find the absolute maximum
bestR = V1 * V2';     
bestT = c1' - bestR * c2';
ptCloud1 = cloud1;
ptCloud2 = cloud2;

for p = 1:size(axis_permutations,1)
    V2_perm = V2(:,axis_permutations(p,:));
    for s = 1:size(sign_flips,1)
        V2_corrected = V2_perm .* sign_flips(s,:);
        R_cand = V1 * V2_corrected';
        if det(R_cand) < 0
            continue;
        end
        T_cand = c1' - R_cand*c2';

        % Apply candidate as initial transform (rigid3d wants R, then translation)
        tform_init = rigid3d(R_cand', T_cand'); % note: rigid3d expects R', see below

        % Move source cloud into candidate pose, then refine with a few ICP iterations
        ptCloud2_init = pctransform(ptCloud2, tform_init);
        % tform_icp = []; movingReg = []; rmse = Inf;
        attempt = 0;
        while attempt < 1
            attempt = attempt + 1;
            try
                [tform_icp, movingReg, rmse] = pcregistericp(ptCloud2_init, ptCloud1, ...
                    'Metric','pointToPlane', ...
                    'MaxIterations', 20, ...
                    'Tolerance',[0.001, 0.01]);
                break;
            catch ME
                if contains(ME.message, 'Invalid transformation matrix')
                    if mod(attempt, 1) == 0
                        warning('icp_initial_guess:icpskipped', ...
                            '%s: pcregistericp failing after %d attempts on perm=%d sign=%d, continuing to retry', ...
                            name, attempt, p, s);
                        tform_icp.Rotation = eye(3);
                        tform_icp.Translation = [0,0,0];
                    end
                    continue;
                else
                    rethrow(ME);
                end
            end
        end

        % Compose: final = icp_refinement * initial_candidate
        R_ref = (tform_init.Rotation * tform_icp.Rotation)';
        T_ref = (tform_init.Translation * tform_icp.Rotation + tform_icp.Translation)';

        % Score using the refined pose
        testNodes = movingReg.Location;
        idx = knnsearch(nodes1, testNodes, 'K', 1);
        matchedTargetNodes = nodes1(idx,:);
        rmseVal = rmse; % pcregistericp already returns this

        % If you want normal agreement too, transform normals with R_ref and recompute
        testNormals = normals2 * R_ref';
        matchedTargetNormals = normals1(idx,:);
        dotProducts = sum(testNormals .* matchedTargetNormals, 2);
        normalScore = mean(dotProducts);

        currentScore = normalScore - rmseVal/max(range(nodes1));
        if currentScore > bestNormalScore
            bestNormalScore = currentScore;
            bestR = R_ref;
            bestT = T_ref;
        end
    end
end
% 6. Assign the absolute winning values to your variables
R = bestR;
T = bestT;
nodes2_aligned = (nodes2 * R') + T';

% 7. Visualize ONLY the final, optimal alignment choice
% figure;
% fixedPointcloud = pointCloud(nodes1);
% ptCloudTformed = pointCloud(nodes2_aligned);
% pcshowpair(fixedPointcloud, ptCloudTformed);
% axis on;
% grid on;
% title(strcat("Optimal Baseline Alignment for ", name, " (Ready for local ICP)"));

end
