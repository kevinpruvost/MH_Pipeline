alice_vision_path=./AliceVision/install/bin/

# Camera Initialization
${alice_vision_path}aliceVision_cameraInit --imageFolder /home/user/images --sensorDatabase ../src/aliceVision/sensorDB/cameraSensors.db --output cameraInit.sfm

# Feature Extraction
${alice_vision_path}aliceVision_featureExtraction --input cameraInit.sfm --output features

# Image Matching
${alice_vision_path}aliceVision_imageMatching --input cameraInit.sfm --featuresFolders features --output matches

# Feature Matching
${alice_vision_path}aliceVision_featureMatching --input cameraInit.sfm --featuresFolders features --output matches

# Structure from Motion
${alice_vision_path}aliceVision_structureFromMotion --input cameraInit.sfm --featuresFolders features --matchesFolders matches --output sfm_output

# Prepare Dense Scene
${alice_vision_path}aliceVision_prepareDenseScene --input sfm_output/sfmData.sfm --output densePrep

# Depth Map Estimation
${alice_vision_path}aliceVision_depthMapEstimation --input sfm_output/sfmData.sfm --imagesFolder /home/user/images --downscale 2 --output depthMaps

# Depth Map Filtering
${alice_vision_path}aliceVision_depthMapFiltering --input sfm_output/sfmData.sfm --depthMapFolder depthMaps --output filteredDepthMaps

# Meshing
${alice_vision_path}aliceVision_meshing --input sfm_output/sfmData.sfm --depthMapFolder filteredDepthMaps --output mesh.obj

# Texturing
${alice_vision_path}aliceVision_texturing --input sfm_output/sfmData.sfm --imagesFolder /home/user/images --inputMesh mesh.obj --output texturedMesh