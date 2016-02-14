function processDepthMapToPCD()
M = loadpcd('../../test/test_data/generated/mapper_point_cloud_loaded.pcd');
pclviewer(M);
scatter3( M(1,:), M(3,:), M(2,:), 2, M(3,:) );