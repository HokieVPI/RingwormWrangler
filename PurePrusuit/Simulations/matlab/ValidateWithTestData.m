function ValidateWithTestData()
% ValidateWithTestData
% Compare robot PuTTY-log data against the Pure Pursuit MATLAB simulation.
%
% This script is designed around the repo's test logs in:
%   PurePrusuit/Test data/
%
% It currently validates:
%   - test_3_31_night13: LA=150 cm, V=150 cm/s, WP_R=100 cm
%   - test_3_31_night11: LA=200 cm, V=150 cm/s, WP_R=100 cm
%
% To run:
%   >> ValidateWithTestData

    %% Constants (match firmware assumptions)
    TRACK_WIDTH      = 43.18;        % cm
    WHEEL_RADIUS     = 15.24;        % cm
    ENDPOINT_RADIUS  = 200;          % cm
    DT               = 0.10;         % s
    MAX_STEPS        = 20000;
    N_RUNS           = 10;

    POS_NOISE_STD    = 35.34;        % cm
    HEAD_NOISE_STD   = deg2rad(10);  % rad

    MAP_AREA = [
        0 0
        70*2.54*12 0
        70*2.54*12 32.4*2.54*12
        (70-2)*2.54*12  (32.4)*2.54*12
        (70-2)*2.54*12  (32.4+19.81)*2.54*12
        (70-2-12.5)*2.54*12 (32.4+19.81)*2.54*12
        (70-2-12.5)*2.54*12 (32.4+19.81+14.75)*2.54*12
        (70-2-12.5-4.45)*2.54*12 (32.4+19.81+14.75)*2.54*12
        (70-2-12.5-4.45)*2.54*12 (32.4+19.81+14.75+20)*2.54*12
        0 (32.4+19.81+14.75+20)*2.54*12
        0 0
    ];
    OUTER_WALLS = polyshape(MAP_AREA(:,1), MAP_AREA(:,2));

    thisDir = fileparts(mfilename('fullpath'));
    testDir = fullfile(thisDir, '..', '..', 'Test data');

    %% Test configurations
    cfg13.name      = 'night13';
    cfg13.file      = fullfile(testDir, 'test_3_31_night13');
    cfg13.lookahead = 150;
    cfg13.speed     = 150;
    cfg13.wpRadius  = 100;
    cfg13.waypoints = [
        200    502.4
        200   2252.4
        442.5 2252.4
        442.5  200
        792.5  200
        792.5 2252.4
       1142.5 2252.4
       1142.5  200
       1300    200
       1300   2252.4
    ];

    cfg11.name      = 'night11';
    cfg11.file      = fullfile(testDir, 'test_3_31_night11');
    cfg11.lookahead = 200;
    cfg11.speed     = 150;
    cfg11.wpRadius  = 100;
    cfg11.waypoints = [
        200    502.4
        200   2252.4
        442.5 2252.4
        442.5  152.4
        792.5  152.4
        792.5 2252.4
       1142.5 2252.4
       1142.5  152.4
       1492.5  152.4
       1492.5 2252.4
    ];

    configs = {cfg13, cfg11};

    for ci = 1:numel(configs)
        cfg = configs{ci};
        fprintf('\n====== Validating %s (LA=%g  V=%g  WP_R=%g) ======\n', ...
            cfg.name, cfg.lookahead, cfg.speed, cfg.wpRadius);

        [rd.goalX, rd.goalY, rd.desHead, rd.azimuth, rd.curX, rd.curY, rd.pathCompleteIdx] = ...
            parseTestData(cfg.file);
        fprintf('  Parsed %d samples from %s\n', numel(rd.curX), cfg.file);

        % Monte Carlo
        allCT = cell(N_RUNS,1);
        allXT = cell(N_RUNS,1);
        allYT = cell(N_RUNS,1);
        completed = false(N_RUNS,1);

        for iRun = 1:N_RUNS
            [xt,yt,~,~,~,~,wallFail] = simulateRun(cfg.waypoints, cfg.speed, cfg.lookahead, cfg.wpRadius, ENDPOINT_RADIUS, ...
                POS_NOISE_STD, HEAD_NOISE_STD, TRACK_WIDTH, WHEEL_RADIUS, DT, MAX_STEPS, iRun-1, OUTER_WALLS);
            allXT{iRun} = xt; allYT{iRun} = yt;
            allCT{iRun} = crossTrackSeries(xt, yt, cfg.waypoints);
            dg = hypot(xt(end)-cfg.waypoints(end,1), yt(end)-cfg.waypoints(end,2));
            completed(iRun) = (dg < ENDPOINT_RADIUS) && ~wallFail;
        end

        [repXT,repYT,repXM,repYM] = simulateRun(cfg.waypoints, cfg.speed, cfg.lookahead, cfg.wpRadius, ENDPOINT_RADIUS, ...
            POS_NOISE_STD, HEAD_NOISE_STD, TRACK_WIDTH, WHEEL_RADIUS, DT, MAX_STEPS, 0, OUTER_WALLS);

        realCT = crossTrackSeries(rd.curX, rd.curY, cfg.waypoints);
        [simGoalX, simGoalY] = recomputeGoals(rd.curX, rd.curY, cfg.waypoints, cfg.lookahead, cfg.wpRadius);

        plotTrajectoryOverlay(cfg, rd, MAP_AREA, allXT, allYT, repXT, repYT, repXM, repYM);
        plotCrossTrackComparison(cfg, realCT, allCT);
        plotGoalDiagnostic(cfg, rd, simGoalX, simGoalY);
        plotHeadingDiagnostic(cfg, rd);
        printMetrics(cfg, rd, realCT, allCT, completed, N_RUNS);
    end

    fprintf('\n====== Validation complete ======\n');
end

function [goalX,goalY,desHead,azimuth,curX,curY,pcIdx] = parseTestData(filepath)
    fid = fopen(filepath, 'r');
    if fid == -1
        error('Cannot open %s', filepath);
    end
    lines = {};
    while ~feof(fid)
        lines{end+1} = fgetl(fid); %#ok<AGROW>
    end
    fclose(fid);

    goalX = []; goalY = [];
    desHead = []; azimuth = [];
    curX = []; curY = [];
    pcIdx = NaN;

    idx = 2; % skip PuTTY header
    while idx + 5 <= numel(lines)
        v1 = str2double(lines{idx});
        if isnan(v1)
            pcIdx = numel(curX);
            break;
        end
        goalX(end+1,1)   = v1;
        goalY(end+1,1)   = str2double(lines{idx+1});
        desHead(end+1,1) = str2double(lines{idx+2});
        azimuth(end+1,1) = str2double(lines{idx+3});
        curX(end+1,1)    = str2double(lines{idx+4});
        curY(end+1,1)    = str2double(lines{idx+5});
        idx = idx + 6;
    end

    valid = curY >= 0;
    goalX   = goalX(valid);   goalY   = goalY(valid);
    desHead = desHead(valid); azimuth = azimuth(valid);
    curX    = curX(valid);    curY    = curY(valid);

    if ~isnan(pcIdx)
        cumV = cumsum(valid);
        tmp = find(cumV == pcIdx, 1, 'first');
        if ~isempty(tmp), pcIdx = tmp; else, pcIdx = numel(curX); end
    end
end

function [gxArr, gyArr] = recomputeGoals(curX, curY, path, lookahead, wpRadius)
    n = numel(curX);
    gxArr = zeros(n,1);
    gyArr = zeros(n,1);
    segIdx = 1;
    for i = 1:n
        segIdx = advanceSegment(curX(i), curY(i), path, segIdx, wpRadius);
        [gx, gy, ~] = findLookaheadGoal(curX(i), curY(i), path, segIdx, lookahead);
        gxArr(i) = gx;
        gyArr(i) = gy;
    end
end

function plotTrajectoryOverlay(cfg, rd, mapArea, allXT, allYT, repXT, repYT, repXM, repYM)
    figure('Name', sprintf('Trajectory - %s', cfg.name), 'Position', [60 60 900 700]);
    hold on;
    plot(mapArea(:,1), mapArea(:,2), 'k-', 'LineWidth', 1.2);
    plot(cfg.waypoints(:,1), cfg.waypoints(:,2), 'g--', 'LineWidth', 1.5);
    plot(cfg.waypoints(:,1), cfg.waypoints(:,2), 'go', 'MarkerSize', 7, 'LineWidth', 1.2);
    for i = 1:numel(allXT)
        plot(allXT{i}, allYT{i}, '-', 'Color', [0.8 0.8 1], 'LineWidth', 0.4);
    end
    plot(rd.curX, rd.curY, 'r-', 'LineWidth', 1.4);
    plot(repXT, repYT, 'b-', 'LineWidth', 1.4);
    plot(repXM, repYM, 'c.', 'MarkerSize', 3);
    plot(rd.curX(1), rd.curY(1), 'ms', 'MarkerSize', 12, 'LineWidth', 2);
    if ~isnan(rd.pathCompleteIdx)
        plot(rd.curX(rd.pathCompleteIdx), rd.curY(rd.pathCompleteIdx), 'r^', 'MarkerSize', 12, 'LineWidth', 2);
    end
    legend({'Map boundary','Waypoint path','Waypoints','Sim MC runs','Real trajectory','Sim representative','Sim measured','Real start','Real end'}, ...
        'Location','bestoutside');
    title(sprintf('%s  (LA=%g  V=%g  WP\\_R=%g)', cfg.name, cfg.lookahead, cfg.speed, cfg.wpRadius));
    xlabel('x (cm)'); ylabel('y (cm)');
    axis equal; grid on;
    hold off;
end

function plotCrossTrackComparison(cfg, realCT, allCT)
    simCTall = vertcat(allCT{:});
    figure('Name', sprintf('Cross-Track - %s', cfg.name), 'Position', [80 80 900 400]);
    subplot(1,2,1);
    histogram(realCT, 50, 'Normalization','probability', 'FaceColor','r', 'FaceAlpha',0.5); hold on;
    histogram(simCTall, 50, 'Normalization','probability', 'FaceColor','b', 'FaceAlpha',0.5);
    legend('Real','Simulation');
    xlabel('Cross-track error (cm)'); ylabel('Probability');
    title(sprintf('%s: CT error distribution', cfg.name));
    grid on;

    subplot(1,2,2);
    nSamples = min(numel(realCT), 2000);
    idx = round(linspace(1, numel(realCT), nSamples));
    plot(idx, realCT(idx), 'r-', 'LineWidth', 1.0); hold on;
    simMeanCT = zeros(size(realCT));
    for k = 1:numel(allCT)
        ct = allCT{k};
        nk = min(numel(ct), numel(realCT));
        simMeanCT(1:nk) = simMeanCT(1:nk) + ct(1:nk);
    end
    simMeanCT = simMeanCT / numel(allCT);
    plot(1:numel(simMeanCT), simMeanCT, 'b-', 'LineWidth', 1.0);
    legend('Real CT','Sim mean CT');
    xlabel('Sample'); ylabel('Cross-track (cm)');
    title(sprintf('%s: CT over time', cfg.name));
    grid on;
end

function plotGoalDiagnostic(cfg, rd, simGoalX, simGoalY)
    figure('Name', sprintf('Goal Diagnostic - %s', cfg.name), 'Position', [100 100 1000 500]);
    n = numel(rd.goalX);
    subplot(2,2,1);
    plot(1:n, rd.goalX, 'r.', 'MarkerSize', 2); hold on;
    plot(1:n, simGoalX(1:n), 'b.', 'MarkerSize', 2);
    legend('Firmware','MATLAB re-compute'); ylabel('Goal X (cm)'); xlabel('Sample');
    title(sprintf('%s: Goal X', cfg.name)); grid on;

    subplot(2,2,2);
    plot(1:n, rd.goalY, 'r.', 'MarkerSize', 2); hold on;
    plot(1:n, simGoalY(1:n), 'b.', 'MarkerSize', 2);
    legend('Firmware','MATLAB re-compute'); ylabel('Goal Y (cm)'); xlabel('Sample');
    title(sprintf('%s: Goal Y', cfg.name)); grid on;

    subplot(2,2,3);
    histogram(rd.goalX - simGoalX(1:n), 50, 'FaceColor', [0.5 0 0.5]);
    xlabel('Goal X error (cm)'); ylabel('Count'); title('Goal X: firmware - MATLAB'); grid on;

    subplot(2,2,4);
    histogram(rd.goalY - simGoalY(1:n), 50, 'FaceColor', [0.5 0 0.5]);
    xlabel('Goal Y error (cm)'); ylabel('Count'); title('Goal Y: firmware - MATLAB'); grid on;
end

function plotHeadingDiagnostic(cfg, rd)
    figure('Name', sprintf('Heading - %s', cfg.name), 'Position', [120 120 900 350]);
    n = numel(rd.desHead);
    plot(1:n, rd.desHead, 'b-', 'LineWidth', 0.8); hold on;
    plot(1:n, rd.azimuth, 'r-', 'LineWidth', 0.8);
    legend('Desired heading (deg)','Global azimuth (deg)');
    xlabel('Sample'); ylabel('Angle (deg)'); title(sprintf('%s: heading tracking', cfg.name));
    grid on;
end

function printMetrics(cfg, rd, realCT, allCT, completed, nRuns)
    fprintf('\n  --- %s summary ---\n', cfg.name);
    fprintf('  Real samples       : %d\n', numel(rd.curX));
    if ~isnan(rd.pathCompleteIdx)
        fprintf('  Path Complete at   : sample %d\n', rd.pathCompleteIdx);
    else
        fprintf('  Path Complete      : NOT reached (truncated log)\n');
    end
    fprintf('  Real CT  mean=%.1f  p95=%.1f  max=%.1f cm\n', mean(realCT), prctile(realCT,95), max(realCT));
    simMeans = cellfun(@mean, allCT);
    simP95s  = cellfun(@(c)prctile(c,95), allCT);
    simMaxs  = cellfun(@max, allCT);
    fprintf('  Sim CT   mean=%.1f +/- %.1f  p95=%.1f +/- %.1f  max=%.1f +/- %.1f cm\n', ...
        mean(simMeans), std(simMeans), mean(simP95s), std(simP95s), mean(simMaxs), std(simMaxs));
    fprintf('  Sim completion     : %d / %d (%.0f%%)\n', sum(completed), nRuns, 100*mean(completed));
    realPathLen = sum(sqrt(diff(rd.curX).^2 + diff(rd.curY).^2));
    fprintf('  Real path length   : %.0f cm\n', realPathLen);
    wpPathLen = sum(sqrt(sum(diff(cfg.waypoints).^2, 2)));
    fprintf('  Waypoint path len  : %.0f cm\n', wpPathLen);
    fprintf('  Ratio (real/wp)    : %.2f\n', realPathLen / wpPathLen);
end

%% ==== Simulation core (extracted from TenPointPurePursuitGUI) ====
function [xt,yt,xm,ym,pathEnded,hitMaxSteps,wallFail] = simulateRun( ...
        path, velocity, lookahead, wpRadius, endRadius, ...
        posStd, headStd, trackW, wheelR, dt, maxSteps, seed, outerWalls)
    rng(seed);

    x = path(1,1); y = path(1,2);
    if size(path,1) >= 2
        theta = atan2(path(2,2)-y, path(2,1)-x);
    else
        theta = 0;
    end

    xm_i = x + randn()*posStd;
    ym_i = y + randn()*posStd;
    thm_i = wrapPi(theta + randn()*headStd);

    segIdx = 1;
    xt = zeros(maxSteps,1); yt = zeros(maxSteps,1);
    xm = zeros(maxSteps,1); ym = zeros(maxSteps,1);
    xt(1)=x; yt(1)=y; xm(1)=xm_i; ym(1)=ym_i;

    pathEnded = false; hitMaxSteps = false; wallFail = false;

    for k = 2:maxSteps
        if segIdx >= size(path,1)
            pathEnded = true;
            xt = xt(1:k-1); yt = yt(1:k-1); xm = xm(1:k-1); ym = ym(1:k-1);
            return;
        end

        segIdx = advanceSegment(xm_i, ym_i, path, segIdx, wpRadius);
        [gx, gy, foundGoal] = findLookaheadGoal(xm_i, ym_i, path, segIdx, lookahead);
        if ~foundGoal || ~isinterior(outerWalls, gx, gy)
            wallFail = true;
            xt = xt(1:k-1); yt = yt(1:k-1); xm = xm(1:k-1); ym = ym(1:k-1);
            return;
        end

        [wl, wr, valid] = purePursuitCommand(xm_i, ym_i, thm_i, path, segIdx, lookahead, velocity, trackW, wheelR);
        if ~valid
            wl = 0.5*velocity/wheelR;
            wr = wl;
        end

        vL = wl*wheelR; vR = wr*wheelR;
        v  = (vL+vR)/2;
        omega = (vR-vL)/trackW;
        x = x + v*cos(theta)*dt;
        y = y + v*sin(theta)*dt;
        theta = wrapPi(theta + omega*dt);

        if ~isinterior(outerWalls, x, y)
            wallFail = true;
            xt(k)=x; yt(k)=y; xm(k)=xm_i; ym(k)=ym_i;
            xt = xt(1:k); yt = yt(1:k); xm = xm(1:k); ym = ym(1:k);
            return;
        end

        xm_i = x + randn()*posStd;
        ym_i = y + randn()*posStd;
        thm_i = wrapPi(theta + randn()*headStd);
        xt(k)=x; yt(k)=y; xm(k)=xm_i; ym(k)=ym_i;

        if hypot(x - path(end,1), y - path(end,2)) < endRadius
            xt = xt(1:k); yt = yt(1:k); xm = xm(1:k); ym = ym(1:k);
            return;
        end
    end

    hitMaxSteps = true;
    xt = xt(:); yt = yt(:); xm = xm(:); ym = ym(:);
end

function segIdx = advanceSegment(curx, cury, path, segIdx, wpRadius)
    while segIdx < size(path,1)
        next = path(segIdx+1,:);
        dx = next(1)-curx;
        dy = next(2)-cury;
        if (dx*dx + dy*dy) <= wpRadius^2
            segIdx = segIdx + 1;
            if segIdx >= size(path,1), break; end
        else
            break;
        end
    end
end

function [wl, wr, valid] = purePursuitCommand(curx, cury, heading, path, segIdx, lookahead, velocity, trackW, wheelR)
    [gx, gy, found] = findLookaheadGoal(curx, cury, path, segIdx, lookahead);
    if ~found, wl=0; wr=0; valid=false; return; end
    dx = gx - curx;  dy = gy - cury;
    L = hypot(dx, dy);
    if L < 1.0, wl=0; wr=0; valid=false; return; end
    alpha = wrapPi(atan2(dy,dx) - heading);
    K = 2*sin(alpha)/L;
    omega = K*velocity;
    wl = (velocity - omega*trackW/2) / wheelR;
    wr = (velocity + omega*trackW/2) / wheelR;
    valid = true;
end

function [gx, gy, found] = findLookaheadGoal(curx, cury, path, segIdx, lookahead)
    found = false;
    gx = NaN; gy = NaN;
    Lsq = lookahead^2;
    for seg = segIdx:(size(path,1)-1)
        a = path(seg,:);
        b = path(seg+1,:);
        ds = b - a;
        fx = a(1)-curx;
        fy = a(2)-cury;
        qa = ds(1)^2 + ds(2)^2;
        qb = 2*(fx*ds(1) + fy*ds(2));
        qc = (fx^2 + fy^2) - Lsq;
        disc = qb^2 - 4*qa*qc;
        if disc < 0, continue; end
        sd = sqrt(disc);
        t1 = (-qb - sd)/(2*qa);
        t2 = (-qb + sd)/(2*qa);
        bestT = -1;
        if t2 >= 0 && t2 <= 1
            bestT = t2;
        elseif t1 >= 0 && t1 <= 1
            bestT = t1;
        end
        if bestT >= 0
            g = a + bestT*ds;
            gx = g(1); gy = g(2);
            found = true;
            return;
        end
    end
    if segIdx < size(path,1)
        gx = path(segIdx+1,1);
        gy = path(segIdx+1,2);
        found = true;
    end
end

function ct = crossTrackSeries(x, y, path)
    n = numel(x);
    ct = zeros(n,1);
    for i = 1:n
        ct(i) = minDistToPolyline(x(i), y(i), path);
    end
end

function dmin = minDistToPolyline(px, py, path)
    dmin = inf;
    for seg = 1:(size(path,1)-1)
        a = path(seg,:);
        b = path(seg+1,:);
        ab = b-a;
        ap = [px-a(1), py-a(2)];
        denom = (ab(1)^2 + ab(2)^2) + 1e-10;
        t = max(0, min(1, (ap(1)*ab(1) + ap(2)*ab(2)) / denom));
        proj = a + t*ab;
        d = hypot(px-proj(1), py-proj(2));
        if d < dmin, dmin = d; end
    end
end

function a = wrapPi(a)
    while a > pi,  a = a - 2*pi; end
    while a < -pi, a = a + 2*pi; end
end

