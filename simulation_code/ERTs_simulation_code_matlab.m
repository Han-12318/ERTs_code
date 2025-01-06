%   ERTs
% Inputs:
%   start_status, goal_status - designated starting/ending point position
%   ~.png - map
% Output:
%   path - multi-robot movement path
clc;
close all;
clear all;

%%  Parameter setting
n = 3;  %   number of robots
numpoints = 500;   %   number of samples
d = 2;  %   planning dimension
start_status = [215,76; %   start status for each robot
    113,505;
    63,788];
goal_status = [650,785; %   goal status for each robot
    888,501;
    501,925];
heupath = [];   % heuristic path
path = [];  %   final path
speed = 10; %   robot move speed

%%  Workplace preprocessing
[g2, obs2, m2] = mapinit();
obspoints = getobspoints(obs2);
gvgpoints = getgvgpoints(g2);
points = [gvgpoint,zeros(size(gvgpoint,1),4)];
%   get domination relationship about gvg and obs
points = getdomrela(obspoints,gvgpoints,points);    

%%  Implicit cost tree searching & Conflict pre-resolution
isconflict = 1;
while heupath == [] || isconflict == 1
    %   constructing implicit cost tree for searching
    heupath = impcosttree(start_status, goal_status, gvgpoints, points);
    isconflict = checkPathConflict(heupath);
    if isconflict == 1
        [gvgpoints,points] = conpreresolution(heupath,gvgpoints,points);
    end
end

%%  Hierarchical decoupling
groups = groupRobotsByConflict(compositePaths);

%%  Construction of Composite Roadmap
points = samplesfree(points, n, map);    %   samplesfree
[G, samplePoints] = buildPRMGraph(env, n, k);

%%  Path searching
path = drrtStarMain(G, samplePoints);
drawpath(path);

%%  Functions
function [g2, obs2, m2] = mapinit()
    maprgb = imread('/maps/map.png');
    m2 = rgb2gray(maprgb);
    g2rgb = imread('/maps/gvg.png');
    g2 = rgb2gray(g2rgb);
    obs2rgb = imread('/maps/obs.png');
    obs2 = rgb2gray(obs2rgb);
end
function obspoints = getobspoints(obs2)
    obspoints =[];
    for i = 1:size(obs2,1)
        for j = 1:size(obs2,2)
            if obs2(i,j) == 0
                obspoints = [obspoints; j,i];
            end
        end
    end
end
function gvgpoints = getgvgpoints(g2)
    gvgpoints = [];
    for i = 1:size(g2, 1)
        for j = 1:size(g2, 2)
            if g2(i,j)==0
                gvgpoints = [gvgpoints; j,i];
            end
        end
    end
end
function points = getdomrela(obspoints,gvgpoints,points)
    for i = 1:size(gvgpoints,1)
        dmax = inf;
        dz = 0;
        for j = 1:size(obspoints,1)
            d = sqrt((gvgpoints(i,1)-obspoints(j,1))^2+((gvgpoints(i,2)-obspoints(j,2))^2));
            if d<dmax
                dmax = d;
                dz = j;
            end
        end
        points(i,5:6) = [dz,dmax];
    end
end
function [pathinit, points] = impcosttree(start, goal, gvgpoint, points)
    a = find(gvgpoint(:,1)==start(1,1));
    b = find(gvgpoint(:,2)==start(1,2));
    si = intersect(a,b);
    c = find(gvgpoint(:,1)==goal(1,1));
    d = find(gvgpoint(:,2)==goal(1,2));
    gi = intersect(c,d);
    pindex = [1:1:size(gvgpoint)]';
    vop = [si];
    vun = pindex;
    vun(si)=[];
    vcl = [];
    z = si;
    while z ~= gi
        near = [];
        for i = 1:size(vun,1)
            d = sqrt((points(z,1)-points(vun(i,1),1))^2+(points(z,2)-points(vun(i,1),2))^2);
            if d<2
                near = [near;vun(i,1)];
                points(vun(i,1),3:4)=[z,d+points(z,4)];
            end
        end
        vcl = [vcl;z];
        vop(find(vop==z))=[];
        vop = [vop;near];
        for i = 1:size(near,1)
            vun(find(vun==near(i,1))) = [];
        end
        gmax = inf;
        for i = 1:size(vop,1)
            if points(vop(i,1),4)<gmax
                gmax = points(vop(i,1),4);
                z = vop(i,1);
            end
        end
    end
    pathinit = [];
    while z~=si
        pathinit = [pathinit;points(z,1:2),z];
        j = points(z,3);
        z = j;
    end
end
function isconflict = checkPathConflict(path1, path2)
    isconflict = false; % Assume no conflict initially
    
    % Iterate over all segments of path1
    for i = 1:size(path1, 1) - 1
        seg1_start = path1(i, :);
        seg1_end = path1(i + 1, :);
        
        % Iterate over all segments of path2
        for j = 1:size(path2, 1) - 1
            seg2_start = path2(j, :);
            seg2_end = path2(j + 1, :);
            
            % Check if the current segments intersect
            if doSegmentsIntersect(seg1_start, seg1_end, seg2_start, seg2_end)
                isconflict = true;
                return; % Exit function as soon as a conflict is found
            end
        end
    end
end
function path = drrtStarMain(G, samplePoints)
    env = createEnvironment(); 
    robots = initializeRobots(env); 
    commRadius = 10; 
    maxIterations = 1000;
    stepSize = 0.5; 
    for iter = 1:maxIterations
        for i = 1:length(robots)
            robot = robots(i);
            randomSample = generateRandomSample(env);
            nearest = nearestNode(robot.tree, randomSample);
            newNode = steer(nearest, randomSample, stepSize);
            if ~isInCollision(newNode, env.obstacles)
                addNodeToTree(robot.tree, newNode);
                rewire(robot.tree, newNode, env.obstacles);
                for j = 1:length(robots)
                    if i ~= j && distanceBetweenTrees(robot.tree, robots(j).tree) < commRadius
                        connectTrees(robot.tree, robots(j).tree);
                    end
                end
                if isGoalReached(robot.tree, env.goal)
                    disp('Path found!');
                    return;
                end
            end
        end
    end
    path = robot.tree;
    disp('Failed to find a path within the maximum number of iterations.');
end
function tra = path2tra(speed,start_status,path)
    x = start_status(1,1);
    y = start_status(1,2);
    m = 0;
    d = 0;
    tra = [];
    for i = 2:size(path,1)
        thetax = (path(i,1)-path(i-1,1))/(sqrt((path(i-1,1)-path(i,1))^2+(path(i-1,2)-path(i,2))^2));
        thetay = (path(i,2)-path(i-1,2))/(sqrt((path(i-1,1)-path(i,1))^2+(path(i-1,2)-path(i,2))^2));
        if d>0
             m = speed - d;
            x = path(i-1,1)+m*thetax;
            y = path(i-1,2)+m*thetay;
            tra = [tra;x, y];
        end
        d = sqrt((path(i-1,1)-path(i,1))^2+(path(i-1,2)-path(i,2))^2)-m;
        k = floor(d/speed);
        for j = 1:k
            x = path(i-1,1)+j*speed*thetax;
            y = path(i-1,2)+j*speed*thetay;
            tra = [tra;x, y];
        end
        d = d-k*speed;
    end
    tra = [tra;g2];
end
function groups = groupRobotsByConflict(compositePaths)
    numRobots = length(compositePaths); % Number of robots
    conflictMatrix = zeros(numRobots, numRobots); % Initialize conflict matrix

    % Fill the conflict matrix
    for i = 1:numRobots
        for j = i+1:numRobots
            if checkPathConflict(compositePaths{i}, compositePaths{j})
                conflictMatrix(i, j) = 1;
                conflictMatrix(j, i) = 1; % Since conflict is bidirectional
            end
        end
    end

    % Find connected components (groups) using graph theory
    G = graph(conflictMatrix);
    bins = conncomp(G, 'OutputForm', 'cell'); % Get the connected components as cells

    % Convert the connected components into groups of robot indices
    groups = cell(0); % Initialize output cell array
    for i = 1:length(bins)
        if ~isempty(bins{i}) % Ignore empty groups
            groups{end+1} = bins{i}; % Add non-empty groups to the output
        end
    end
end
function intersects = doSegmentsIntersect(p1, q1, p2, q2)
    % Find the four orientations needed for general and special cases
    o1 = orientation(p1, q1, p2);
    o2 = orientation(p1, q1, q2);
    o3 = orientation(p2, q2, p1);
    o4 = orientation(p2, q2, q1);

    % General case
    if (o1 ~= o2) && (o3 ~= o4)
        intersects = true;
        return;
    end

    % Special Cases
    % p1, q1 and p2 are collinear and p2 lies on segment p1q1
    if (o1 == 0) && onSegment(p1, p2, q1)
        intersects = true;
        return;
    end
    % p1, q1 and q2 are collinear and q2 lies on segment p1q1
    if (o2 == 0) && onSegment(p1, q2, q1)
        intersects = true;
        return;
    end
    % p2, q2 and p1 are collinear and p1 lies on segment p2q2
    if (o3 == 0) && onSegment(p2, p1, q2)
        intersects = true;
        return;
    end
    % p2, q2 and q1 are collinear and q1 lies on segment p2q2
    if (o4 == 0) && onSegment(p2, q1, q2)
        intersects = true;
        return;
    end

    intersects = false;
end
function orient = orientation(p, q, r)
    val = (q(2) - p(2)) * (r(1) - q(1)) - (q(1) - p(1)) * (r(2) - q(2));
    
    if val == 0
        orient = 0; % Collinear
    elseif val > 0
        orient = 1; % Clockwise
    else
        orient = 2; % Counterclockwise
    end
end
function bool = onSegment(p, q, r)
    if (min(p(1), r(1)) <= q(1) && q(1) <= max(p(1), r(1))) && ...
       (min(p(2), r(2)) <= q(2) && q(2) <= max(p(2), r(2)))
        bool = true;
    else
        bool = false;
    end
end
function result = edgecollision(p1, p2, map)
    result = 0;
    n = 30;
    a = (p2(1,1)-p1(1,1))/n;
    b = (p2(1,2)-p1(1,2))/n;
    for i = 1:n
        c = p1(1,1)+i*a;
        d = p1(1,2)+i*b;
        if map(ceil(d), ceil(c)) ~= 255 || map(floor(d), ceil(c)) ~= 255 || map(ceil(d), floor(c)) ~= 255 || map(floor(d), floor(c)) ~= 255
            result = 1;
            break;
        end
    end
end
function drawpath(path)
    tra = path2tra(speed,start_status,path);
    robot1 = scatter(s1(1,1), s1(1,2), 50, 'b', 'filled');
    robot2 = scatter(s2(1,1), s2(1,2), 50, 'r', 'filled');
    robot3 = scatter(s3(1,1), s3(1,2), 50, 'm', 'filled');
    v = VideoWriter('circle_animation.mp4', 'MPEG-4');
    open(v);

    for i = 1:100
        if i <= size(pa1,1)
            robot1.XData = pa1(i,1);
            robot1.YData = pa1(i,2);
        end
        if i <= size(pa2,1)
            robot2.XData = pa2(i,1);
            robot2.YData = pa2(i,2);
        end
        if i <= size(pa3,1)
            robot3.XData = pa3(i,1);
            robot3.YData = pa3(i,2);
        end
        frame = getframe(gcf);
        writeVideo(v, frame);
        pause(0.1);  
    end
    close(v);
    hold off;    
end
function points = samplesfree(points, n, map)
    while n > 0
        xrand = [size(map,2)*rand,size(map,1)*rand];
        if map(ceil(xrand(1,2)),ceil(xrand(1,1)))==255 && map(ceil(xrand(1,2)),floor(xrand(1,1))) == 255 && map(floor(xrand(1,2)),ceil(xrand(1,1)))==255 && map(floor(xrand(1,2)),floor(xrand(1,1)))==255
            points = [points;xrand];
            n = n-1;
        end
    end
end
function [G, samplePoints] = buildPRMGraph(env, n, k)
    G = graph();
    samplePoints = [env.start; env.goal];
    for i = 1:n-2
        newPoint = generateRandomSample(env);
        if ~isInObstacle(newPoint, env.obstacles)
            samplePoints = [samplePoints; newPoint];
        end
    end
    while size(samplePoints, 1) < n
        newPoint = generateRandomSample(env);
        if ~isInObstacle(newPoint, env.obstacles)
            samplePoints = [samplePoints; newPoint];
        end
    end
    numSamples = size(samplePoints, 1);
    for i = 1:numSamples
        distances = pdist2(samplePoints(i,:), samplePoints, 'euclidean');
        [~, sortedIndices] = sort(distances);
        nearestNeighbors = sortedIndices(2:min(k+1, numSamples));
        for j = nearestNeighbors
            if i ~= j && canConnect(samplePoints(i,:), samplePoints(j,:), env.obstacles)
                G = addnode(G, 0); % Add nodes if not already present
                G = addedge(G, i, j, {'Weight', norm(samplePoints(i,:) - samplePoints(j,:))});
            end
        end
    end
    if canConnect(env.start, env.goal, env.obstacles)
        startIndex = find(ismember(samplePoints, env.start, 'rows'), 1);
        goalIndex = find(ismember(samplePoints, env.goal, 'rows'), 1);
        if ~isempty(startIndex) && ~isempty(goalIndex)
            G = addedge(G, startIndex, goalIndex, {'Weight', norm(env.start - env.goal)});
        end
    end
end
function point = generateRandomSample(env)
    minX = min([env.obstacles{:}(:,1)]);
    maxX = max([env.obstacles{:}(:,1)]);
    minY = min([env.obstacles{:}(:,2)]);
    maxY = max([env.obstacles{:}(:,2)]);
    point = [minX + (maxX - minX) * rand, minY + (maxY - minY) * rand];
end
function inObstacle = isInObstacle(point, obstacles)
    inObstacle = false;
    for i = 1:length(obstacles)
        if inpolygon(point(1), point(2), obstacles{i}(:,1), obstacles{i}(:,2))
            inObstacle = true;
            return;
        end
    end
end
function canConnect = canConnect(p1, p2, obstacles)
    canConnect = true;
    x = [p1(1), p2(1)];
    y = [p1(2), p2(2)];
    for i = 1:length(obstacles)
        poly = obstacles{i};
        for j = 1:size(poly, 1)
            edgeStart = poly(j,:);
            edgeEnd = poly(mod(j, size(poly, 1)) + 1,:);
            if doSegmentsIntersect(p1, p2, edgeStart, edgeEnd)
                canConnect = false;
                return;
            end
        end
    end
end
