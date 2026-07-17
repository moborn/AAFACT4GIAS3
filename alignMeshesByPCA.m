function [R, nodes2_aligned] = alignMeshesByPCA(nodes1, nodes2)
%ALIGNMESHESBYPCA Align two point meshes by matching principal axes.
%   [R, t, nodes2_aligned] = ALIGNMESHESBYPCA(nodes1, nodes2)
%   computes a rotation matrix R (3x3) and translation vector t (3x1)
%   that aligns nodes2 to nodes1 by matching centroids and principal
%   axes (PCA). nodes1 and nodes2 are N1x3 and N2x3 matrices of points.
%   nodes2_aligned = nodes2*R' + t' (each row transformed).
%
%   The function handles reflection ambiguity by enforcing a right-handed
%   coordinate frame for the rotation.

% Validate inputs
if size(nodes1,2) ~= 3 || size(nodes2,2) ~= 3
    error('Input node arrays must be Nx3.');
end

% Compute centroids
c1 = mean(nodes1,1);
c2 = mean(nodes2,1);

% Center points
X1 = bsxfun(@minus, nodes1, c1);
X2 = bsxfun(@minus, nodes2, c2);

% Covariance matrices
C1 = (X1' * X1) / max(1, size(X1,1)-1);
C2 = (X2' * X2) / max(1, size(X2,1)-1);

% PCA via eig (cov symmetric)
[V1, D1] = eig(C1);
[V2, D2] = eig(C2);

% Sort eigenvectors by descending eigenvalue
[~, idx1] = sort(diag(D1), 'descend');
[~, idx2] = sort(diag(D2), 'descend');
V1 = V1(:, idx1);
V2 = V2(:, idx2);

% Ensure right-handed frames
if det(V1) < 0
    V1(:,3) = -V1(:,3);
end
if det(V2) < 0
    V2(:,3) = -V2(:,3);
end

% Initial rotation to align principal axes: R = V1 * V2'
R = V1 * V2';

% Fix reflection if needed (ensure det(R)=+1)
if det(R) < 0
    % Flip third column of V1 and recompute
    V1(:,3) = -V1(:,3);
    R = V1 * V2';
end
% Use skewness of projected points to resolve sign ambiguity of first two axes
S_centered = X2; % source is nodes2 centered
T_centered = X1; % target is nodes1 centered
V_source = V2;
V_target = V1;
for i = 1:2
    skewS = sum((S_centered * V_source(:,i)).^3);
    skewT = sum((T_centered * V_target(:,i)).^3);
    if sign(skewS) ~= sign(skewT)
        V_source(:,i) = -V_source(:,i);
    end
end
V2 = V_source;
% Recompute rotation with possibly flipped axes
R = V1 * V2';
% Ensure proper rotation (no reflection)
if det(R) < 0
    V1(:,3) = -V1(:,3);
    R = V1 * V2';
end
% Translation: map centroid of nodes2 after rotation to centroid of nodes1
t = c1' - R * c2';

% Apply transform to nodes2
nodes2_aligned = (nodes2 * R');
end