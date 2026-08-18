function [R, T, aligned_nodes] = alignmeshbyCPD(nodes1, nodes2)

fixedPointcloud = pointCloud(nodes1);
movingPointcloud = pointCloud(nodes2);
% Downsample heavily first to speed up the optimization
movingDown = pcdownsample(movingPointcloud, 'gridAverage', 1.5); % 1.5mm voxels
fixedDown = pcdownsample(fixedPointcloud, 'gridAverage', 1.5);

% Run CPD with a strict rigid transformation constraint
[tformCPD, ~] = pcregistercpd(movingDown, fixedDown, ...
    'Transform', 'Rigid', ...
    'MaxIterations', 100, ...
    'Tolerance', 1e-5);
R = tformCPD.R;
T = tformCPD.Translation;

ptCloudTformed = pctransform(movingPointcloud,invert(tformCPD));
aligned_nodes = ptCloudTformed.Location;
pcshowpair(fixedPointcloud,ptCloudTformed)
axis on
xlim([-50 50])
ylim([-40 60])
title("Aligned Point Clouds")
end
