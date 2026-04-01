function TenPointPurePursuitGUI()
% TenPointPurePursuitGUI
% MATLAB UI to run Monte Carlo pure-pursuit sims on the 10-corner path.
%
% Sliders:
%   - Speed (cm/s): 50..250
%   - Lookahead (cm): 50..300
%   - Waypoint radius (cm): 75..125
%
% Button:
%   - Run (computes completion %, cross-track metrics, failure counts)
%
% To run:
%   >> TenPointPurePursuitGUI
%
% Notes:
% - This is a uifigure-based app (no .mlapp needed).
% - Noise parameters match the Python work: position noise std = 35.34 cm,
%   heading noise std = 10 deg.

    %% Defaults (match your simulation assumptions)
    POSITION_NOISE_STD_DEFAULT = 35.34;      % cm (per-axis)
    HEADING_NOISE_STD_DEFAULT  = deg2rad(10); % rad
    DT = 0.10;                       % s
    MAX_STEPS = 20000;
    N_RUNS = 10;
    SEED_REP = 0;                    % representative run seed
    TRACK_WIDTH = 43.18;             % cm
    WHEEL_RADIUS = 15.24;            % cm
    ENDPOINT_RADIUS = 150;           % cm (fixed completion radius for final waypoint)
    MOP_FAIL_DIST = 6*12*2.54;       % cm (6 feet). Fail if within this distance of map boundary polyline.

    % Mop / mat area boundary polyline (from PurePrusuit/Test data/PurePursuitdata.m)
    MAP_AREA = [
        0 0
        70*2.54*12 0
        70*2.54*12 32.4*2.54*12
        (70-2)*2.54*12 (32.4)*2.54*12
        (70-2)*2.54*12 (32.4+19.81)*2.54*12
        (70-2-12.5)*2.54*12 (32.4+19.81)*2.54*12
        (70-2-12.5)*2.54*12 (32.4+19.81+14.75)*2.54*12
        (70-2-12.5-4.45)*2.54*12 (32.4+19.81+14.75)*2.54*12
        (70-2-12.5-4.45)*2.54*12 (32.4+19.81+14.75+20)*2.54*12
        0 (32.4+19.81+14.75+20)*2.54*12
        0 0
    ];

    % Build a mowing (boustrophedon) waypoint path inside the mop area.
    % Each new sweep line in Y is spaced 8 ft apart.
    Y_STEP = 8*12*2.54; % cm
    safePoly = polybuffer(polyshape(MAP_AREA(:,1), MAP_AREA(:,2)), -MOP_FAIL_DIST);
    PATH = generateMowingWaypoints(safePoly, Y_STEP);

    %% UI
    fig = uifigure('Name','10-point Pure Pursuit Simulator', 'Position',[100 100 1200 720]);

    % Left control panel
    pnl = uipanel(fig, 'Title','Parameters', 'Position',[15 15 360 690]);

    y = 615;
    lblSpeed = uilabel(pnl,'Position',[15 y 200 22],'Text','Speed (cm/s)');
    sldSpeed = uislider(pnl, ...
        'Position',[15 y-10 330 3], ...
        'Limits',[50 250], 'Value',100, 'MajorTicks',50:50:250);
    txtSpeed = uilabel(pnl,'Position',[240 y 110 22],'HorizontalAlignment','right','Text','100');

    y = y - 90;
    lblLA = uilabel(pnl,'Position',[15 y 250 22],'Text','Lookahead distance (cm)');
    sldLA = uislider(pnl, ...
        'Position',[15 y-10 330 3], ...
        'Limits',[50 300], 'Value',150, 'MajorTicks',50:50:300);
    txtLA = uilabel(pnl,'Position',[240 y 110 22],'HorizontalAlignment','right','Text','150');

    y = y - 90;
    lblRad = uilabel(pnl,'Position',[15 y 250 22],'Text','Waypoint radius (cm)');
    sldRad = uislider(pnl, ...
        'Position',[15 y-10 330 3], ...
        'Limits',[75 125], 'Value',100, 'MajorTicks',75:25:125);
    txtRad = uilabel(pnl,'Position',[240 y 110 22],'HorizontalAlignment','right','Text','100');

    y = y - 90;
    uilabel(pnl,'Position',[15 y 250 22],'Text','Position noise std (cm)');
    sldPosNoise = uislider(pnl, ...
        'Position',[15 y-10 330 3], ...
        'Limits',[0 100], 'Value',POSITION_NOISE_STD_DEFAULT, 'MajorTicks',0:25:100);
    txtPosNoise = uilabel(pnl,'Position',[240 y 110 22],'HorizontalAlignment','right','Text',sprintf('%.1f', POSITION_NOISE_STD_DEFAULT));

    y = y - 90;
    uilabel(pnl,'Position',[15 y 250 22],'Text','Heading noise std (deg)');
    sldHeadNoise = uislider(pnl, ...
        'Position',[15 y-10 330 3], ...
        'Limits',[0 30], 'Value',rad2deg(HEADING_NOISE_STD_DEFAULT), 'MajorTicks',0:5:30);
    txtHeadNoise = uilabel(pnl,'Position',[240 y 110 22],'HorizontalAlignment','right','Text',sprintf('%.1f', rad2deg(HEADING_NOISE_STD_DEFAULT)));

    % Runs / steps controls (optional)
    y = y - 90;
    uilabel(pnl,'Position',[15 y 250 22],'Text','Monte Carlo runs');
    spnRuns = uispinner(pnl,'Position',[240 y 110 22], 'Limits',[1 200], 'Value',N_RUNS);

    y = y - 45;
    uilabel(pnl,'Position',[15 y 250 22],'Text','Max steps');
    spnSteps = uispinner(pnl,'Position',[240 y 110 22], 'Limits',[500 200000], 'Value',MAX_STEPS, 'Step',500);

    % Run button
    btnRun = uibutton(pnl,'push', ...
        'Text','Run simulation', ...
        'Position',[15 80 330 40], ...
        'ButtonPushedFcn', @onRun);

    % Status text
    txtStatus = uitextarea(pnl, 'Position',[15 15 330 55], 'Editable','off');
    txtStatus.Value = { ...
        'Ready.'; ...
        sprintf('Endpoint radius: %d cm', ENDPOINT_RADIUS); ...
        sprintf('Mop boundary fail dist: %.2f cm', MOP_FAIL_DIST); ...
        sprintf('Y step: %.2f cm (8 ft)', Y_STEP); ...
        sprintf('Path points: %d', size(PATH,1)) ...
    };

    % Right plots
    axTraj = uiaxes(fig,'Position',[390 375 790 330]);
    title(axTraj,'Trajectory (representative run)');
    xlabel(axTraj,'x (cm)'); ylabel(axTraj,'y (cm)');
    grid(axTraj,'on'); axis(axTraj,'equal');

    axCT = uiaxes(fig,'Position',[390 40 790 300]);
    title(axCT,'Cross-track error (all runs)');
    xlabel(axCT,'Time (s)'); ylabel(axCT,'Cross-track (cm)');
    grid(axCT,'on');

    % Slider callbacks for live numeric label updates
    sldSpeed.ValueChangingFcn = @(~,e)set(txtSpeed,'Text',sprintf('%.0f',e.Value));
    sldLA.ValueChangingFcn    = @(~,e)set(txtLA,'Text',sprintf('%.0f',e.Value));
    sldRad.ValueChangingFcn   = @(~,e)set(txtRad,'Text',sprintf('%.0f',e.Value));
    sldPosNoise.ValueChangingFcn  = @(~,e)set(txtPosNoise,'Text',sprintf('%.1f',e.Value));
    sldHeadNoise.ValueChangingFcn = @(~,e)set(txtHeadNoise,'Text',sprintf('%.1f',e.Value));

    %% Helpers
    function onRun(~,~)
        % Read UI values
        V = sldSpeed.Value;
        LA = sldLA.Value;
        WP_R = sldRad.Value;
        POS_STD = sldPosNoise.Value;
        HEAD_STD = deg2rad(sldHeadNoise.Value);
        nRuns = spnRuns.Value;
        maxSteps = spnSteps.Value;

        % Patch display
        txtStatus.Value = { ...
            'Running...'; ...
            sprintf('Speed: %.0f cm/s', V); ...
            sprintf('Lookahead: %.0f cm', LA); ...
            sprintf('Waypoint radius: %.0f cm', WP_R); ...
            sprintf('Pos noise std: %.1f cm', POS_STD); ...
            sprintf('Head noise std: %.1f deg', rad2deg(HEAD_STD)); ...
            sprintf('Runs: %d', nRuns); ...
            sprintf('Max steps: %d', maxSteps) ...
        };
        drawnow;

        % Run Monte Carlo
        allCT = cell(nRuns,1);
        allT = cell(nRuns,1);
        allXT = cell(nRuns,1);
        allYT = cell(nRuns,1);
        completed = false(nRuns,1);
        failMode = strings(nRuns,1);
        distGoal = zeros(nRuns,1);

        rep = struct('xt',[],'yt',[],'xm',[],'ym',[]);

        for seed = 0:(nRuns-1)
            [xt,yt,xm,ym,pathEnded,hitMaxSteps,boundaryHit] = simulateRun(PATH, V, LA, WP_R, ENDPOINT_RADIUS, ...
                POS_STD, HEAD_STD, TRACK_WIDTH, WHEEL_RADIUS, DT, maxSteps, seed, MAP_AREA, MOP_FAIL_DIST);

            ct = crossTrackSeries(xt, yt, PATH);
            t = (0:numel(ct)-1) * DT;

            allCT{seed+1} = ct;
            allT{seed+1} = t;
            allXT{seed+1} = xt;
            allYT{seed+1} = yt;

            dg = hypot(xt(end)-PATH(end,1), yt(end)-PATH(end,2));
            distGoal(seed+1) = dg;
            completed(seed+1) = (dg < ENDPOINT_RADIUS) && ~boundaryHit;

            if completed(seed+1)
                failMode(seed+1) = "completed";
            elseif boundaryHit
                failMode(seed+1) = "hit_mop_boundary";
            elseif hitMaxSteps
                failMode(seed+1) = "timeout";
            elseif pathEnded
                failMode(seed+1) = "path_ended_not_within_radius";
            else
                failMode(seed+1) = "other";
            end

            if seed == SEED_REP
                rep.xt = xt; rep.yt = yt; rep.xm = xm; rep.ym = ym;
            end
        end

        % Summary metrics
        meanCT = mean(cellfun(@mean, allCT));
        p95CT  = mean(cellfun(@(c)prctile(c,95), allCT));
        maxCT  = mean(cellfun(@max, allCT));
        compPct = 100*mean(completed);
        meanGoal = mean(distGoal);

        % Failure mode counts (excluding completed for "most common failure")
        modes = categorical(failMode);
        cats = categories(modes);
        counts = countcats(modes);
        modeMap = containers.Map(cats, counts);
        failCounts = containers.Map({'hit_mop_boundary','timeout','path_ended_not_within_radius','other'}, [0 0 0 0]);
        for k = 1:numel(cats)
            if ~strcmp(cats{k}, 'completed')
                if isKey(failCounts, cats{k})
                    failCounts(cats{k}) = failCounts(cats{k}) + modeMap(cats{k});
                else
                    failCounts('other') = failCounts('other') + modeMap(cats{k});
                end
            end
        end
        % Most common failure
        keysF = keys(failCounts);
        valsF = cellfun(@(k)failCounts(k), keysF);
        [~,idxMax] = max(valsF);
        mostCommonFail = keysF{idxMax};

        % Update status text
        txtStatus.Value = { ...
            'Done.'; ...
            sprintf('Completion: %.1f%%', compPct); ...
            sprintf('Mean CT: %.1f cm', meanCT); ...
            sprintf('95th CT: %.1f cm', p95CT); ...
            sprintf('Mean Max CT: %.1f cm', maxCT); ...
            sprintf('Mean dist-to-goal: %.1f cm', meanGoal); ...
            sprintf('Most common failure: %s', mostCommonFail); ...
            ''; ...
            sprintf('Counts: completed=%d, mop_boundary=%d, timeout=%d, path_end=%d, other=%d', ...
                sum(failMode=="completed"), sum(failMode=="hit_mop_boundary"), ...
                sum(failMode=="timeout"), sum(failMode=="path_ended_not_within_radius"), ...
                sum(failMode=="other")) ...
        };

        % Plot trajectory
        cla(axTraj);
        plot(axTraj, MAP_AREA(:,1), MAP_AREA(:,2), 'k-', 'LineWidth',1.2); hold(axTraj,'on');
        % Show the safe area boundary (inset by 6 ft) so you can see where waypoints live
        [bx, by] = boundary(safePoly);
        plot(axTraj, bx, by, 'k--', 'LineWidth',1.0);
        plot(axTraj, PATH(:,1), PATH(:,2), 'g--', 'LineWidth',1.5); hold(axTraj,'on');
        plot(axTraj, PATH(:,1), PATH(:,2), 'go', 'MarkerSize',5, 'LineWidth',1.0);
        for i = 1:nRuns
            plot(axTraj, allXT{i}, allYT{i}, 'r--', 'LineWidth',0.5);
        end
        plot(axTraj, rep.xm, rep.ym, 'r.', 'MarkerSize',6);
        plot(axTraj, rep.xt, rep.yt, 'b-', 'LineWidth',1.4);
        plot(axTraj, rep.xt(1), rep.yt(1), 'ms', 'MarkerSize',10, 'LineWidth',1.5);
        plot(axTraj, rep.xt(end), rep.yt(end), 'r^', 'MarkerSize',10, 'LineWidth',1.5);
        legend(axTraj, {'Map area','Safe area (>=6 ft from boundary)','Waypoints','Waypoints','All runs','Measured','Representative truth','Start','End'}, 'Location','best');
        title(axTraj, sprintf('Representative trajectory (seed=%d)', SEED_REP));
        axis(axTraj,'equal'); grid(axTraj,'on');

        % Plot cross-track error time series (all runs)
        cla(axCT);
        hold(axCT,'on');
        for i = 1:nRuns
            plot(axCT, allT{i}, allCT{i}, 'Color',[0.2 0.4 0.8 0.25]);
        end
        yline(axCT, WP_R, '--', 'Color',[0.3 0.3 0.3], 'LineWidth',1.0);
        title(axCT, sprintf('Cross-track error (all runs). WP radius=%.0f cm', WP_R));
        grid(axCT,'on');
        hold(axCT,'off');
    end

    function [xt,yt,xm,ym,pathEnded,hitMaxSteps,boundaryHit] = simulateRun(path, velocity, lookahead, wpRadius, endRadius, posStd, headStd, trackW, wheelR, dt, maxSteps, seed, mapArea, mopFailDist)
        rng(seed);

        % Truth state
        x = path(1,1); y = path(1,2);
        theta = atan2(path(1,2)-y, path(1,1)-x); %#ok<NASGU>
        if size(path,1) >= 2
            theta = atan2(path(2,2)-y, path(2,1)-x);
        else
            theta = 0;
        end

        % Noisy measurement used by controller
        xm_i = x + randn()*posStd;
        ym_i = y + randn()*posStd;
        thm_i = wrapPi(theta + randn()*headStd);

        segIdx = 1;

        xt = zeros(maxSteps,1); yt = zeros(maxSteps,1);
        xm = zeros(maxSteps,1); ym = zeros(maxSteps,1);
        xt(1)=x; yt(1)=y; xm(1)=xm_i; ym(1)=ym_i;

        pathEnded = false;
        hitMaxSteps = false;
        boundaryHit = false;

        for k = 2:maxSteps
            if segIdx >= size(path,1)
                pathEnded = true;
                xt = xt(1:k-1); yt = yt(1:k-1); xm = xm(1:k-1); ym = ym(1:k-1);
                return;
            end

            % Advance segment if within radius or overshot
            segIdx = advanceSegment(xm_i, ym_i, path, segIdx, wpRadius);

            % Compute pure pursuit command
            [wl, wr, valid] = purePursuitCommand(xm_i, ym_i, thm_i, path, segIdx, lookahead, velocity, trackW, wheelR);
            if ~valid
                wl = 0.5*velocity/wheelR;
                wr = wl;
            end

            % Kinematics (truth)
            vL = wl*wheelR;
            vR = wr*wheelR;
            v = (vL+vR)/2;
            omega = (vR-vL)/trackW;
            x = x + v*cos(theta)*dt;
            y = y + v*sin(theta)*dt;
            theta = wrapPi(theta + omega*dt);

            % Mop area boundary failure check (truth position)
            if minDistToPolyline(x, y, mapArea) <= mopFailDist
                boundaryHit = true;
                xt(k)=x; yt(k)=y;
                xm(k)=xm_i; ym(k)=ym_i;
                xt = xt(1:k); yt = yt(1:k); xm = xm(1:k); ym = ym(1:k);
                return;
            end

            % Measurement
            xm_i = x + randn()*posStd;
            ym_i = y + randn()*posStd;
            thm_i = wrapPi(theta + randn()*headStd);

            xt(k)=x; yt(k)=y; xm(k)=xm_i; ym(k)=ym_i;

            % Optional early break if close to final
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
            dist2 = dx*dx + dy*dy;
            inRadius = dist2 <= wpRadius^2;

            if inRadius
                segIdx = segIdx + 1;
                if segIdx >= size(path,1)
                    break;
                end
            else
                break;
            end
        end
    end

    function [wl, wr, valid] = purePursuitCommand(curx, cury, heading, path, segIdx, lookahead, velocity, trackW, wheelR)
        % find lookahead goal intersection with segments >= segIdx
        [gx, gy, found] = findLookaheadGoal(curx, cury, path, segIdx, lookahead);
        if ~found
            wl = 0; wr = 0; valid = false; return;
        end
        dx = gx - curx;
        dy = gy - cury;
        L = hypot(dx,dy);
        if L < 1.0
            wl = 0; wr = 0; valid = false; return;
        end
        angleToGoal = atan2(dy,dx);
        alpha = wrapPi(angleToGoal - heading);
        K = 2*sin(alpha)/L;
        omega = K*velocity;
        wl = (velocity - omega*trackW/2)/wheelR;
        wr = (velocity + omega*trackW/2)/wheelR;
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
            if disc < 0
                continue;
            end
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
        % fallback: aim at next waypoint
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
            if d < dmin
                dmin = d;
            end
        end
    end

    function a = wrapPi(a)
        while a > pi
            a = a - 2*pi;
        end
        while a < -pi
            a = a + 2*pi;
        end
    end

    function waypoints = generateMowingWaypoints(poly, yStep)
        % Generate a boustrophedon (mowing) path inside polyshape `poly`.
        % Uses horizontal sweep lines spaced by yStep (cm).
        %
        % For each sweep line y = const:
        % - Compute intersections with polygon boundary
        % - Use [minX, maxX] inside the polygon as the sweep segment
        % - Alternate direction each line

        if poly.NumRegions < 1
            error('Safe area polygon is empty. Reduce MOP_FAIL_DIST or check MAP_AREA.');
        end

        % Use the polygon bounding box in Y
        v = poly.Vertices;
        yMin = min(v(:,2));
        yMax = max(v(:,2));

        ys = yMin:yStep:yMax;
        if numel(ys) < 2
            ys = linspace(yMin, yMax, max(2, round((yMax-yMin)/yStep)+1));
        end

        wps = [];
        dir = 1;

        for i = 1:numel(ys)
            y0 = ys(i);

            % Intersections of horizontal line with polygon boundary
            [bx, by] = boundary(poly);
            xLine = [min(v(:,1))-1000, max(v(:,1))+1000];
            yLine = [y0, y0];
            [xi, yi] = polyxpoly(bx, by, xLine, yLine);

            if numel(xi) < 2
                continue;
            end

            xi = sort(xi(:));

            % Use the widest interior span on this scanline
            spans = reshape(xi(1:2*floor(numel(xi)/2)), 2, []).';
            widths = spans(:,2) - spans(:,1);
            [~, idx] = max(widths);
            x1 = spans(idx,1);
            x2 = spans(idx,2);

            if dir > 0
                wps = [wps; x1 y0; x2 y0]; %#ok<AGROW>
            else
                wps = [wps; x2 y0; x1 y0]; %#ok<AGROW>
            end
            dir = -dir;
        end

        if size(wps,1) < 2
            error('Generated mowing waypoints are empty. Check MAP_AREA and yStep.');
        end

        % Remove consecutive duplicates (can happen if spans collapse)
        d = sqrt(sum(diff(wps,1,1).^2,2));
        keep = [true; d > 1e-6];
        waypoints = wps(keep,:);
    end
end

