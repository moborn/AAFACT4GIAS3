function [R, rot_nodes] = alignmeshbyfeatures(nodes1, nodes2)
ptcloud1 = pointCloud(nodes1);
ptcloud2 = pointCloud(nodes2);


% ptcloud1 = pcdownsample(ptcloud1,gridAverage=2);
% ptcloud2 = pcdownsample(ptcloud2,gridAverage=2);
fixedFeature = extractFPFHFeatures(ptcloud1, Radius=16);
movingFeature = extractFPFHFeatures(ptcloud2, Radius = 16);

[matchingPairs,scores] = pcmatchfeatures(fixedFeature,movingFeature, ...
    ptcloud1,ptcloud2,Method="Exhaustive", MatchThreshold=0.5);
length(matchingPairs)
% 1. Set a score threshold (lower = stricter/higher confidence)


fixedPts = select(ptcloud1,matchingPairs(:,1));
matchingPts = select(ptcloud2,matchingPairs(:,2));
estimatedTform = estgeotform3d(fixedPts.Location, ...
    matchingPts.Location,"rigid", "Confidence", 99, "MaxNumTrials",5000, "MaxDistance",8);
disp(estimatedTform.A)
R = estimatedTform.R;
rot_nodes = nodes2 * R';
ptCloudTformed = pctransform(ptcloud2,invert(estimatedTform));
pcshowpair(ptcloud1,ptCloudTformed)
axis on
xlim([-50 50])
ylim([-40 60])
title("Aligned Point Clouds")
end