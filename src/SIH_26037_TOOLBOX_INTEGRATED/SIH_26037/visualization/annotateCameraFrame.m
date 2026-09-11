function img = annotateCameraFrame(veh, cameraDetections, params)
% annotateCameraFrame  Computer Vision Toolbox demo visual: renders a
% synthetic forward-camera-style frame with bounding boxes and type
% labels for the current virtualCamera.m detections.
%
% *** TOOLBOX ADD-ON -- see TOOLBOX_INTEGRATION.md ***
% Purely additive/cosmetic: this has no effect on planning, tracking,
% or any metric. It exists to give a demo/pitch a recognizable "camera
% view with detections" visual, built with a real, stable Computer
% Vision Toolbox function (insertObjectAnnotation) rather than raw
% plotting -- useful for showing a judge "yes, this is Computer Vision
% Toolbox, doing something plotting alone doesn't do (drawing
% correctly-scaled boxes + text labels baked into image pixel data)".
%
%   img = annotateCameraFrame(veh, cameraDetections, params)
%
% cameraDetections: output of sensors/virtualCamera.m (fields .bearing,
%   .type, .confidence, .trueId) -- this function does NOT re-run the
%   camera model, so call it right after virtualCamera.m with that same
%   output.
%
% params: .imgWidth (default 640), .imgHeight (default 360),
%   .fov (default matches virtualCamera.m's default, 100 deg),
%   .boxSize (default 60 px, fixed box size -- this simplified camera
%   model has no metric range/size, only bearing, so box size cannot
%   be derived honestly from real depth; a fixed size is a deliberate,
%   flagged simplification, not an attempt to fake distance-based
%   scaling).
%
% Output: img (imgHeight x imgWidth x 3 uint8), or [] with a warning if
% Computer Vision Toolbox's insertObjectAnnotation isn't available.
%
% CONFIDENCE: insertObjectAnnotation is a long-standing, very commonly
% used Computer Vision Toolbox function; I'm confident in this call
% shape. It could not be executed in this sandbox (no Computer Vision
% Toolbox in this Octave install), so, like the other toolbox add-ons,
% treat this as untested -- but it is also low-risk: this file cannot
% affect anything else in the project even if it silently does
% nothing, since nothing downstream consumes its output.

if nargin < 3, params = struct(); end
imgWidth  = getOrDefault(params,'imgWidth',640);
imgHeight = getOrDefault(params,'imgHeight',360);
fov       = getOrDefault(params,'fov',deg2rad(100));
boxSize   = getOrDefault(params,'boxSize',60);

canvas = uint8(zeros(imgHeight, imgWidth, 3));
canvas(:,:,3) = 40; % faint dark-blue "road ahead" background, purely cosmetic

if isempty(cameraDetections)
    img = canvas;
    return;
end

boxes = zeros(numel(cameraDetections), 4);
labels = cell(numel(cameraDetections), 1);
for i = 1:numel(cameraDetections)
    d = cameraDetections(i);
    % Map bearing in [-fov/2, fov/2] to a horizontal pixel position.
    px = imgWidth/2 + (d.bearing / (fov/2)) * (imgWidth/2);
    px = max(boxSize/2, min(imgWidth-boxSize/2, px));
    py = imgHeight/2; % no elevation info in this simplified model

    boxes(i,:) = [px-boxSize/2, py-boxSize/2, boxSize, boxSize];
    labels{i} = sprintf('%s (%.0f%%)', d.type, 100*d.confidence);
end

try
    img = insertObjectAnnotation(canvas, 'rectangle', boxes, labels, ...
        'Color', 'yellow', 'TextColor', 'black', 'FontSize', 14);
catch ME
    warning('annotateCameraFrame:toolboxUnavailable', ...
        'insertObjectAnnotation failed (%s: %s) -- Computer Vision Toolbox may not be licensed here. Returning the unannotated canvas instead.', ...
        ME.identifier, ME.message);
    img = canvas;
end
end

function v = getOrDefault(s,f,default)
if isfield(s,f) && ~isempty(s.(f)), v = s.(f); else, v = default; end
end
