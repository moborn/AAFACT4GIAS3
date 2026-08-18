function [R, T, aligned_nodes] = alignmeshbyCPD(nodes1, nodes2)

fixedPointcloud = pointCloud(nodes1);
movingPointcloud = pointCloud(nodes2);
% Downsample heavily first to speed up the optimization
movingDown = pcdownsample(movingPointcloud, 'gridAverage', 1.5); % 1.5mm voxels
fixedDown = pcdownsample(fixedPointcloud, 'gridAverage', 1.5);

% Run CPD with a strict rigid transformation constraint
[tformCPD, ~] = pcregistercpd(movingDown, fixedDown, ...
    'TransformationType', 'Rigid', ...
    'MaxIterations', 100, ...
    'Tolerance', 1e-5);

% Pass this excellent initial guess into your local plane-to-plane ICP
[tformFinal, aligned_nodes] = pcregistericp(movingPointcloud, fixedPointcloud, ...
    'Metric', 'planeToPlane', ...
    'InitialTransform', tformCPD);
ptCloudTformed = pctransform(movingPointcloud,invert(tformFinal));

pcshowpair(fixedPointcloud,ptCloudTformed)
axis on
xlim([-50 50])
ylim([-40 60])
title("Aligned Point Clouds")
end
end