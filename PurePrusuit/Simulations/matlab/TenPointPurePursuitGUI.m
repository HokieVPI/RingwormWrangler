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
    OUTER_WALLS = polyshape();       % initialized below from MAP_AREA

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
    OUTER_WALLS = polyshape(MAP_AREA(:,1), MAP_AREA(:,2));

    % Build a mowing (boustrophedon) waypoint path inside the mop area.
    % Each new sweep line in X is spaced 8 ft apart.
    X_STEP = 8*12*2.54; % cm
    SAFE_MARGIN_CM = 6*12*2.54; % cm (default waypoint inset from boundary; does NOT cause failure)
    safePoly = polybuffer(polyshape(MAP_AREA(:,1), MAP_AREA(:,2)), -SAFE_MARGIN_CM);
    PATH = generateMowingWaypoints(safePoly, X_STEP);

    %% UI
    fig = uifigure('Name','Pure Pursuit Simulator', 'Position',[100 100 1200 900]);

    % Left control panel
    pnl = uipanel(fig, 'Title','Parameters', 'Position',[15 15 360 870]);

    cancelRequested = false;

    y = 795;
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

    % Waypoint generation controls
    y = y - 90;
    uilabel(pnl,'Position',[15 y 250 22],'Text','Sweep spacing (ft in X)');
    spnXStepFt = uispinner(pnl,'Position',[240 y 110 22], 'Limits',[1 30], 'Value',8, 'Step',1);

    y = y - 45;
    uilabel(pnl,'Position',[15 y 250 22],'Text','Boundary margin (ft)');
    spnMarginFt = uispinner(pnl,'Position',[240 y 110 22], 'Limits',[0 12], 'Value',6, 'Step',0.5);

    y = y - 55;
    btnRegen = uibutton(pnl,'push', ...
        'Text','Regenerate waypoints', ...
        'Position',[15 y 330 32], ...
        'ButtonPushedFcn', @onRegenWaypoints);

    y = y - 70;
    lblWpStats = uilabel(pnl,'Position',[15 y 330 60], 'Text','Waypoints:');

    % Seed controls
    y = y - 70;
    chkRandomize = uicheckbox(pnl,'Position',[15 y 330 22],'Text','Randomize seeds (non-repeatable)','Value',false);

    % Runs / steps controls (optional)
    y = y - 55;
    uilabel(pnl,'Position',[15 y 250 22],'Text','Monte Carlo runs');
    spnRuns = uispinner(pnl,'Position',[240 y 110 22], 'Limits',[1 200], 'Value',N_RUNS);

    y = y - 45;
    uilabel(pnl,'Position',[15 y 250 22],'Text','Max steps');
    spnSteps = uispinner(pnl,'Position',[240 y 110 22], 'Limits',[500 200000], 'Value',MAX_STEPS, 'Step',500);

    % Run / stop buttons
    btnRun = uibutton(pnl,'push', ...
        'Text','Run simulation', ...
        'Position',[15 70 240 40], ...
        'ButtonPushedFcn', @onRun);
    btnStop = uibutton(pnl,'push', ...
        'Text','Stop', ...
        'Position',[265 70 80 40], ...
        'Enable','off', ...
        'ButtonPushedFcn', @onStop);

    % Status label (kept small so it won't cover controls)
    lblStatus = uilabel(pnl, 'Position',[15 25 330 22], 'Text','Ready.');

    % Right plots
    axTraj = uiaxes(fig,'Position',[390 505 790 380]);
    title(axTraj,'Trajectory (representative run)');
    xlabel(axTraj,'x (cm)'); ylabel(axTraj,'y (cm)');
    grid(axTraj,'on'); axis(axTraj,'equal');

    axHead = uiaxes(fig,'Position',[390 270 790 220]);
    title(axHead,'Heading vs Desired Heading (representative run)');
    xlabel(axHead,'Time (s)');
    ylabel(axHead,'Angle (deg)');
    grid(axHead,'on');

    % Bottom row: results next to a smaller CT summary
    txtResults = uitextarea(fig, 'Position',[390 40 300 210], 'Editable','off');
    txtResults.Value = {'Results will appear here.'};

    axCTSum = uiaxes(fig,'Position',[700 40 480 210]);
    title(axCTSum,'Cross-track summary (per-run)');
    ylabel(axCTSum,'Cross-track (cm)');
    grid(axCTSum,'on');

    % Slider callbacks for live numeric label updates
    sldSpeed.ValueChangingFcn = @(~,e)set(txtSpeed,'Text',sprintf('%.0f',e.Value));
    sldLA.ValueChangingFcn    = @(~,e)set(txtLA,'Text',sprintf('%.0f',e.Value));
    sldRad.ValueChangingFcn   = @(~,e)set(txtRad,'Text',sprintf('%.0f',e.Value));
    sldPosNoise.ValueChangingFcn  = @(~,e)set(txtPosNoise,'Text',sprintf('%.1f',e.Value));
    sldHeadNoise.ValueChangingFcn = @(~,e)set(txtHeadNoise,'Text',sprintf('%.1f',e.Value));

    %% Helpers
    refreshWaypointStats();
    lblStatus.Text = 'Ready.';

    function onStop(~,~)
        cancelRequested = true;
        btnStop.Enable = 'off';
    end

    function onRegenWaypoints(~,~)
        % Rebuild PATH + safePoly from current UI controls.
        X_STEP = spnXStepFt.Value * 12 * 2.54;
        SAFE_MARGIN_CM = spnMarginFt.Value * 12 * 2.54;
        safePoly = polybuffer(polyshape(MAP_AREA(:,1), MAP_AREA(:,2)), -SAFE_MARGIN_CM);
        PATH = generateMowingWaypoints(safePoly, X_STEP);
        refreshWaypointStats();

        % Quick redraw of geometry (no sim)
        cla(axTraj);
        plot(axTraj, MAP_AREA(:,1), MAP_AREA(:,2), 'k-', 'LineWidth',1.2); hold(axTraj,'on');
        [bx, by] = boundary(safePoly);
        plot(axTraj, bx, by, 'k--', 'LineWidth',1.0);
        plot(axTraj, PATH(:,1), PATH(:,2), 'g--', 'LineWidth',1.5);
        plot(axTraj, PATH(:,1), PATH(:,2), 'go', 'MarkerSize',5, 'LineWidth',1.0);
        title(axTraj,'Waypoints regenerated (no simulation yet)');
        axis(axTraj,'equal'); grid(axTraj,'on');
        hold(axTraj,'off');
        lblStatus.Text = 'Waypoints regenerated.';
    end

    function refreshWaypointStats()
        % Stats: # sweeps, # waypoints, path length
        nWps = size(PATH,1);
        if nWps >= 2
            seg = diff(PATH,1,1);
            pathLen = sum(sqrt(sum(seg.^2,2)));
        else
            pathLen = 0;
        end
        xStepCm = X_STEP;
        nSweeps = max(0, floor(nWps/2));
        marginCm = SAFE_MARGIN_CM;
        marginFt = SAFE_MARGIN_CM/(12*2.54);
        lblWpStats.Text = sprintf([ ...
            'Waypoints stats\n' ...
            '  # sweeps: %d\n' ...
            '  # waypoints: %d\n' ...
            '  total path length: %.1f cm (%.1f m)\n' ...
            '  X step: %.2f cm\n' ...
            '  boundary margin: %.2f cm (%.2f ft)'], ...
            nSweeps, nWps, pathLen, pathLen/100, xStepCm, marginCm, marginFt);
    end

    function onRun(~,~)
        cancelRequested = false;
        btnStop.Enable = 'on';

        % Read UI values
        V = sldSpeed.Value;
        LA = sldLA.Value;
        WP_R = sldRad.Value;
        POS_STD = sldPosNoise.Value;
        HEAD_STD = deg2rad(sldHeadNoise.Value);
        nRuns = spnRuns.Value;
        maxSteps = spnSteps.Value;
        randomize = chkRandomize.Value;

        if randomize
            seeds = randi([0 1e9], nRuns, 1);
            repSeed = seeds(1);
        else
            seeds = (0:(nRuns-1)).';
            repSeed = 0;
        end

        % Patch display
        lblStatus.Text = sprintf('Running... (runs=%d, maxSteps=%d)', nRuns, maxSteps);
        drawnow;

        dlg = uiprogressdlg(fig, ...
            'Title','Running Monte Carlo', ...
            'Message','Starting...', ...
            'Cancelable','on', ...
            'Value',0);

        % Run Monte Carlo
        allCT = cell(nRuns,1);
        allT = cell(nRuns,1);
        allXT = cell(nRuns,1);
        allYT = cell(nRuns,1);
        completed = false(nRuns,1);
        failMode = strings(nRuns,1);
        distGoal = zeros(nRuns,1);

        rep = struct('xt',[],'yt',[],'xm',[],'ym',[]);

        for iRun = 1:nRuns
            seed = seeds(iRun);

            if cancelRequested || dlg.CancelRequested
                break;
            end

            dlg.Value = (iRun-1)/max(1,nRuns);
            dlg.Message = sprintf('Run %d / %d (seed=%d)', iRun, nRuns, seed);
            drawnow;

            [xt,yt,xm,ym,pathEnded,hitMaxSteps,wallFail] = simulateRun(PATH, V, LA, WP_R, ENDPOINT_RADIUS, ...
                POS_STD, HEAD_STD, TRACK_WIDTH, WHEEL_RADIUS, DT, maxSteps, seed, OUTER_WALLS);

            ct = crossTrackSeries(xt, yt, PATH);
            t = (0:numel(ct)-1) * DT;

            allCT{iRun} = ct;
            allT{iRun} = t;
            allXT{iRun} = xt;
            allYT{iRun} = yt;

            dg = hypot(xt(end)-PATH(end,1), yt(end)-PATH(end,2));
            distGoal(iRun) = dg;
            completed(iRun) = (dg < ENDPOINT_RADIUS) && ~wallFail;

            if completed(iRun)
                failMode(iRun) = "completed";
            elseif wallFail
                failMode(iRun) = "outside_outer_walls";
            elseif hitMaxSteps
                failMode(iRun) = "timeout";
            elseif pathEnded
                failMode(iRun) = "path_ended_not_within_radius";
            else
                failMode(iRun) = "other";
            end
        end

        % Representative run (always computed so plots are repeatable)
        dlg.Message = sprintf('Representative run (seed=%d)', repSeed);
        drawnow;
        [xtR,ytR,xmR,ymR,~,~,~] = simulateRun(PATH, V, LA, WP_R, ENDPOINT_RADIUS, ...
            POS_STD, HEAD_STD, TRACK_WIDTH, WHEEL_RADIUS, DT, maxSteps, repSeed, OUTER_WALLS);
        rep.xt = xtR; rep.yt = ytR; rep.xm = xmR; rep.ym = ymR;

        dlg.Value = 1;
        dlg.Message = 'Done';
        pause(0.05);
        close(dlg);
        btnStop.Enable = 'off';

        % Summary metrics
        ranMask = ~cellfun(@isempty, allCT);
        nRan = sum(ranMask);
        if any(ranMask)
            meanCT = mean(cellfun(@mean, allCT(ranMask)));
            p95CT  = mean(cellfun(@(c)prctile(c,95), allCT(ranMask)));
            maxCT  = mean(cellfun(@max, allCT(ranMask)));
        else
            meanCT = NaN; p95CT = NaN; maxCT = NaN;
        end
        if nRan > 0
            compPct = 100*mean(completed(ranMask));
            meanGoal = mean(distGoal(ranMask));
        else
            compPct = 0;
            meanGoal = NaN;
        end

        % Average completion time (completed runs only)
        avgTimeToComplete_s = NaN;
        if nRan > 0
            doneMask = ranMask & completed;
            if any(doneMask)
                idxDone = find(doneMask);
                tDone = zeros(numel(idxDone),1);
                for jj = 1:numel(idxDone)
                    iIdx = idxDone(jj);
                    % time ~= (#samples-1)*DT since the first sample is t=0
                    tDone(jj) = (numel(allXT{iIdx}) - 1) * DT;
                end
                avgTimeToComplete_s = mean(tDone);
            end
        end

        % Failure mode counts (excluding completed for "most common failure")
        modes = categorical(failMode(ranMask));
        cats = categories(modes);
        counts = countcats(modes);
        modeMap = containers.Map(cats, counts);
        failCounts = containers.Map({'outside_outer_walls','timeout','path_ended_not_within_radius','other'}, [0 0 0 0]);
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
        lblStatus.Text = sprintf('Done. Completion=%.1f%%', compPct);

        % Results panel (right side)
        if isnan(avgTimeToComplete_s)
            avgTimeLine = 'Avg completion time: n/a';
        else
            avgTimeLine = sprintf('Avg completion time: %.1f s', avgTimeToComplete_s);
        end

        % Failure mode breakdown lines
        modeNames = {'completed','outside_outer_walls','timeout','path_ended_not_within_radius','other'};
        if nRan > 0
            mAll = categorical(failMode(ranMask), modeNames, 'Ordinal',true);
            cAll = countcats(mAll);
            pctAll = 100 * (cAll / max(1, sum(cAll)));
            failLines = {
                'Failure modes (count, %):'
                sprintf('  completed: %d (%.1f%%)', cAll(1), pctAll(1))
                sprintf('  outside_outer_walls: %d (%.1f%%)', cAll(2), pctAll(2))
                sprintf('  timeout: %d (%.1f%%)', cAll(3), pctAll(3))
                sprintf('  path_ended_not_within_radius: %d (%.1f%%)', cAll(4), pctAll(4))
                sprintf('  other: %d (%.1f%%)', cAll(5), pctAll(5))
            };
        else
            failLines = {'Failure modes: n/a'};
        end

        txtResults.Value = [ ...
            { ...
                sprintf('Completion (avg): %.1f%%', compPct); ...
                avgTimeLine; ...
                sprintf('Mean CT: %.1f cm', meanCT); ...
                sprintf('P95 CT: %.1f cm', p95CT); ...
                sprintf('Mean Max CT: %.1f cm', maxCT); ...
                sprintf('Most common failure: %s', mostCommonFail); ...
                sprintf('Ran: %d/%d', nRan, nRuns); ...
                '' ...
            }; ...
            failLines ...
        ];

        % Plot trajectory
        cla(axTraj);
        plot(axTraj, MAP_AREA(:,1), MAP_AREA(:,2), 'k-', 'LineWidth',1.2); hold(axTraj,'on');
        % Show the safe area boundary so you can see where waypoints live
        [bx, by] = boundary(safePoly);
        plot(axTraj, bx, by, 'k--', 'LineWidth',1.0);
        plot(axTraj, PATH(:,1), PATH(:,2), 'g--', 'LineWidth',1.5); hold(axTraj,'on');
        plot(axTraj, PATH(:,1), PATH(:,2), 'go', 'MarkerSize',5, 'LineWidth',1.0);
        for i = 1:nRuns
            if isempty(allXT{i})
                continue;
            end
            % Actual (truth) trajectory for each Monte Carlo run
            plot(axTraj, allXT{i}, allYT{i}, 'r-', 'LineWidth',0.5);
        end
        plot(axTraj, rep.xm, rep.ym, 'r.', 'MarkerSize',6);
        plot(axTraj, rep.xt, rep.yt, 'b-', 'LineWidth',1.4);
        plot(axTraj, rep.xt(1), rep.yt(1), 'ms', 'MarkerSize',10, 'LineWidth',1.5);
        plot(axTraj, rep.xt(end), rep.yt(end), 'r^', 'MarkerSize',10, 'LineWidth',1.5);
        marginFt = SAFE_MARGIN_CM/(12*2.54);
        legend(axTraj, {'Map area',sprintf('Safe area (>=%.1f ft from boundary)', marginFt),'Waypoints','Waypoints','All runs','Measured','Representative truth','Start','End'}, 'Location','best');
        title(axTraj, sprintf('Representative trajectory (seed=%d)', repSeed));
        axis(axTraj,'equal'); grid(axTraj,'on');

        % Heading vs desired heading over time (representative run)
        cla(axHead);
        [~,~,~,~,~,~,~,tH,headDeg,desDeg] = simulateRun(PATH, V, LA, WP_R, ENDPOINT_RADIUS, ...
            POS_STD, HEAD_STD, TRACK_WIDTH, WHEEL_RADIUS, DT, maxSteps, repSeed, OUTER_WALLS);
        if ~isempty(tH)
            plot(axHead, tH, headDeg, 'b-', 'LineWidth',1.2); hold(axHead,'on');
            plot(axHead, tH, desDeg, 'r--', 'LineWidth',1.2);
            legend(axHead, {'Heading','Desired heading'}, 'Location','best');
            grid(axHead,'on'); hold(axHead,'off');
            title(axHead, sprintf('Heading vs Desired Heading (seed=%d)', repSeed));
            xlabel(axHead,'Time (s)'); ylabel(axHead,'Angle (deg)');
        else
            title(axHead,'Heading vs Desired Heading');
        end

        % Cross-track summary chart (boxplot of per-run stats)
        cla(axCTSum);
        if nRan > 0
            ctMean = zeros(nRan,1);
            ctP95  = zeros(nRan,1);
            ctMax  = zeros(nRan,1);
            idxs = find(ranMask);
            for j = 1:nRan
                c = allCT{idxs(j)};
                ctMean(j) = mean(c);
                ctP95(j)  = prctile(c,95);
                ctMax(j)  = max(c);
            end
            boxplot(axCTSum, [ctMean, ctP95, ctMax], 'Labels',{'Mean','P95','Max'});
            ylabel(axCTSum,'Cross-track (cm)');
            title(axCTSum, sprintf('Cross-track summary (WP radius=%.0f cm)', WP_R));
            grid(axCTSum,'on');
        else
            title(axCTSum,'Cross-track summary (per-run)');
        end
    end

    function [xt,yt,xm,ym,pathEnded,hitMaxSteps,wallFail,tHist,headingDeg,desiredHeadingDeg] = simulateRun(path, velocity, lookahead, wpRadius, endRadius, posStd, headStd, trackW, wheelR, dt, maxSteps, seed, outerWalls)
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
        wallFail = false;

        % Optional history for heading plot (only used when requested)
        recordHeading = (nargout >= 10);
        if recordHeading
            tHist = zeros(maxSteps,1);
            headingDeg = zeros(maxSteps,1);
            desiredHeadingDeg = zeros(maxSteps,1);
            tHist(1) = 0;
            headingDeg(1) = rad2deg(theta);
            desiredHeadingDeg(1) = rad2deg(theta);
        else
            tHist = [];
            headingDeg = [];
            desiredHeadingDeg = [];
        end

        for k = 2:maxSteps
            if segIdx >= size(path,1)
                pathEnded = true;
                xt = xt(1:k-1); yt = yt(1:k-1); xm = xm(1:k-1); ym = ym(1:k-1);
                return;
            end

            % Advance segment if within radius or overshot
            segIdx = advanceSegment(xm_i, ym_i, path, segIdx, wpRadius);

            % Compute lookahead goal (for control and wall constraint)
            [gx, gy, foundGoal] = findLookaheadGoal(xm_i, ym_i, path, segIdx, lookahead);

            % Fail if the controller goal leaves the outer walls
            if ~foundGoal || ~isinterior(outerWalls, gx, gy)
                wallFail = true;
                xt = xt(1:k-1); yt = yt(1:k-1); xm = xm(1:k-1); ym = ym(1:k-1);
                if recordHeading
                    tHist = tHist(1:k-1);
                    headingDeg = headingDeg(1:k-1);
                    desiredHeadingDeg = desiredHeadingDeg(1:k-1);
                end
                return;
            end

            if recordHeading
                tHist(k) = (k-1)*dt;
                headingDeg(k) = rad2deg(theta);
                desiredHeadingDeg(k) = rad2deg(atan2(gy - ym_i, gx - xm_i));
            end

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

            % Fail only if the robot truth position exceeds the outer walls
            if ~isinterior(outerWalls, x, y)
                wallFail = true;
                xt(k)=x; yt(k)=y;
                xm(k)=xm_i; ym(k)=ym_i;
                xt = xt(1:k); yt = yt(1:k); xm = xm(1:k); ym = ym(1:k);
                if recordHeading
                    tHist = tHist(1:k);
                    headingDeg = headingDeg(1:k);
                    desiredHeadingDeg = desiredHeadingDeg(1:k);
                end
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
                if recordHeading
                    tHist = tHist(1:k);
                    headingDeg = headingDeg(1:k);
                    desiredHeadingDeg = desiredHeadingDeg(1:k);
                end
                return;
            end
        end

        hitMaxSteps = true;
        xt = xt(:); yt = yt(:); xm = xm(:); ym = ym(:);
        if recordHeading
            tHist = tHist(:);
            headingDeg = headingDeg(:);
            desiredHeadingDeg = desiredHeadingDeg(:);
        end
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

    function waypoints = generateMowingWaypoints(poly, xStep)
        % Generate a boustrophedon (mowing) path inside polyshape `poly`.
        % Uses vertical sweep lines spaced by xStep (cm).
        %
        % For each sweep line x = const:
        % - Compute intersections with polygon boundary
        % - Use [minY, maxY] inside the polygon as the sweep segment
        % - Alternate direction each line

        if poly.NumRegions < 1
            error('Safe area polygon is empty. Reduce boundary margin or check MAP_AREA.');
        end

        % Use the polygon bounding box in X
        v = poly.Vertices;
        xMin = min(v(:,1));
        xMax = max(v(:,1));

        xs = xMin:xStep:xMax;
        if numel(xs) < 2
            xs = linspace(xMin, xMax, max(2, round((xMax-xMin)/xStep)+1));
        end

        wps = [];
        dir = 1;

        for i = 1:numel(xs)
            x0 = xs(i);

            % Intersections of vertical line with polygon boundary
            [bx, by] = boundary(poly);
            yLine = [min(v(:,2))-1000, max(v(:,2))+1000];
            xLine = [x0, x0];
            [xi, yi] = polyxpoly(bx, by, xLine, yLine);

            if numel(yi) < 2
                continue;
            end

            yi = sort(yi(:));

            % Use the widest interior span on this scanline
            spans = reshape(yi(1:2*floor(numel(yi)/2)), 2, []).';
            widths = spans(:,2) - spans(:,1);
            [~, idx] = max(widths);
            y1 = spans(idx,1);
            y2 = spans(idx,2);

            if dir > 0
                wps = [wps; x0 y1; x0 y2]; %#ok<AGROW>
            else
                wps = [wps; x0 y2; x0 y1]; %#ok<AGROW>
            end
            dir = -dir;
        end

        if size(wps,1) < 2
            error('Generated mowing waypoints are empty. Check MAP_AREA and xStep.');
        end

        % Remove consecutive duplicates (can happen if spans collapse)
        d = sqrt(sum(diff(wps,1,1).^2,2));
        keep = [true; d > 1e-6];
        waypoints = wps(keep,:);
    end
end

