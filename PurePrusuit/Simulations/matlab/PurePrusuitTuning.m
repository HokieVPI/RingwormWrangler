function PurePrusuitTuning()
% PurePrusuitTuning
% MATLAB UI to tune pure pursuit and plan obstacle-aware mowing waypoints.
%
% Features:
% - Hard obstacles (circle/rect): path segments must not intersect.
% - Soft obstacles (circle/rect): path planning avoids with weight-scaled inflation.
% - Inflation: applied to outer walls + obstacles for collision checks.
% - Segment validation: every waypoint-to-waypoint segment is verified to stay
%   inside the allowed zone (outer wall minus inflated obstacles).
% - Export C struct: outputs waypoints in Arduino `Waypoint` struct format.
% - Monte Carlo simulation with soft-avoidance rate metric.
%
% To run:
%   >> PurePrusuitTuning

    %% Defaults
    POSITION_NOISE_STD_DEFAULT = 35.34;       % cm (per-axis)
    HEADING_NOISE_STD_DEFAULT  = deg2rad(10); % rad
    DT = 0.10;                                % s
    MAX_STEPS = 20000;
    N_RUNS = 10;
    TRACK_WIDTH = 43.18;                      % cm
    WHEEL_RADIUS = 15.24;                     % cm
    ENDPOINT_RADIUS = 150;                    % cm (completion radius for final waypoint)
    OUTER_WALLS = polyshape();
    OBSTACLES = struct('type',{},'hard',{},'params',{},'weight',{});
    INFLATION_CM = 100;                       % cm default inflation

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

    X_STEP = 8*12*2.54; % cm
    PATH = rebuildPath();

    %% UI
    fig = uifigure('Name','Pure Pursuit Tuning', 'Position',[50 30 1430 900]);
    cancelRequested = false;

    pnl = uipanel(fig, 'Title','Parameters', 'Position',[15 15 360 870]);

    y = 795;
    uilabel(pnl,'Position',[15 y 200 22],'Text','Speed (cm/s)');
    sldSpeed = uislider(pnl, 'Position',[15 y-10 330 3], ...
        'Limits',[50 250], 'Value',100, 'MajorTicks',50:50:250);
    txtSpeed = uilabel(pnl,'Position',[240 y 110 22],'HorizontalAlignment','right','Text','100');

    y = y - 90;
    uilabel(pnl,'Position',[15 y 250 22],'Text','Lookahead distance (cm)');
    sldLA = uislider(pnl, 'Position',[15 y-10 330 3], ...
        'Limits',[50 300], 'Value',150, 'MajorTicks',50:50:300);
    txtLA = uilabel(pnl,'Position',[240 y 110 22],'HorizontalAlignment','right','Text','150');

    y = y - 90;
    uilabel(pnl,'Position',[15 y 250 22],'Text','Waypoint radius (cm)');
    sldRad = uislider(pnl, 'Position',[15 y-10 330 3], ...
        'Limits',[75 125], 'Value',100, 'MajorTicks',75:25:125);
    txtRad = uilabel(pnl,'Position',[240 y 110 22],'HorizontalAlignment','right','Text','100');

    y = y - 90;
    uilabel(pnl,'Position',[15 y 250 22],'Text','Position noise std (cm)');
    sldPosNoise = uislider(pnl, 'Position',[15 y-10 330 3], ...
        'Limits',[0 100], 'Value',POSITION_NOISE_STD_DEFAULT, 'MajorTicks',0:25:100);
    txtPosNoise = uilabel(pnl,'Position',[240 y 110 22],'HorizontalAlignment','right','Text',sprintf('%.1f', POSITION_NOISE_STD_DEFAULT));

    y = y - 90;
    uilabel(pnl,'Position',[15 y 250 22],'Text','Heading noise std (deg)');
    sldHeadNoise = uislider(pnl, 'Position',[15 y-10 330 3], ...
        'Limits',[0 30], 'Value',rad2deg(HEADING_NOISE_STD_DEFAULT), 'MajorTicks',0:5:30);
    txtHeadNoise = uilabel(pnl,'Position',[240 y 110 22],'HorizontalAlignment','right','Text',sprintf('%.1f', rad2deg(HEADING_NOISE_STD_DEFAULT)));

    y = y - 90;
    uilabel(pnl,'Position',[15 y 250 22],'Text','Sweep spacing (ft in X)');
    spnXStepFt = uispinner(pnl,'Position',[240 y 110 22], 'Limits',[1 30], 'Value',8, 'Step',1);

    y = y - 60;
    uibutton(pnl,'push', 'Text','Regenerate waypoints', ...
        'Position',[15 y 330 32], 'ButtonPushedFcn', @onRegenWaypoints);

    y = y - 90;
    lblWpStats = uilabel(pnl,'Position',[15 y 330 70], 'Text','Waypoints:');

    y = y - 70;
    chkRandomize = uicheckbox(pnl,'Position',[15 y 330 22],'Text','Randomize seeds (non-repeatable)','Value',false);

    y = y - 55;
    uilabel(pnl,'Position',[15 y 250 22],'Text','Monte Carlo runs');
    spnRuns = uispinner(pnl,'Position',[240 y 110 22], 'Limits',[1 200], 'Value',N_RUNS);

    y = y - 45;
    uilabel(pnl,'Position',[15 y 250 22],'Text','Max steps');
    spnSteps = uispinner(pnl,'Position',[240 y 110 22], 'Limits',[500 200000], 'Value',MAX_STEPS, 'Step',500);

    uibutton(pnl,'push', 'Text','Run simulation', ...
        'Position',[15 70 240 40], 'ButtonPushedFcn', @onRun);
    btnStop = uibutton(pnl,'push', 'Text','Stop', ...
        'Position',[265 70 80 40], 'Enable','off', 'ButtonPushedFcn', @onStop);
    lblStatus = uilabel(pnl, 'Position',[15 25 330 22], 'Text','Ready.');

    %% Middle panel: Obstacles / Export
    pnlDZ = uipanel(fig, 'Title','Obstacles / Export', 'Position',[385 15 225 870]);

    dzy = 800;
    uibutton(pnlDZ,'push', 'Text','Export C struct to clipboard', ...
        'Position',[10 dzy 205 34], 'ButtonPushedFcn', @onExportCStruct);
    dzy = dzy - 45;
    uibutton(pnlDZ,'push', 'Text','Save trajectory figure', ...
        'Position',[10 dzy 205 34], 'ButtonPushedFcn', @onSaveFigures);

    dzy = dzy - 50;
    uilabel(pnlDZ,'Position',[10 dzy 205 22], 'Text','Obstacles', 'FontWeight','bold');

    dzy = dzy - 30;
    uilabel(pnlDZ,'Position',[10 dzy 90 22], 'Text','Inflation (cm)');
    spnInflation = uispinner(pnlDZ,'Position',[110 dzy 105 22], ...
        'Limits',[0 500], 'Value',INFLATION_CM, 'Step',5);

    dzy = dzy - 30;
    uilabel(pnlDZ,'Position',[10 dzy 40 22], 'Text','Type');
    ddObsType = uidropdown(pnlDZ,'Position',[55 dzy 160 22], ...
        'Items',{'Circle','Rectangle (axis-aligned)'}, 'Value','Circle');

    dzy = dzy - 30;
    chkHardObs = uicheckbox(pnlDZ,'Position',[10 dzy 60 22], 'Text','Hard', 'Value',true);
    uilabel(pnlDZ,'Position',[75 dzy 55 22], 'Text','Weight');
    spnSoftW = uispinner(pnlDZ,'Position',[135 dzy 80 22], 'Limits',[0 100], 'Value',50, 'Step',5);

    dzy = dzy - 25;
    uilabel(pnlDZ,'Position',[10 dzy 205 16], 'Text','circle: cx,cy,r  |  rect: xmin,ymin,w,h');
    dzy = dzy - 25;
    edtObsParams = uieditfield(pnlDZ,'text','Position',[10 dzy 205 22], 'Value','500,500,100');

    dzy = dzy - 35;
    uibutton(pnlDZ,'push','Text','Add obstacle', 'Position',[10 dzy 100 28], 'ButtonPushedFcn', @onAddObstacle);
    uibutton(pnlDZ,'push','Text','Remove last',  'Position',[115 dzy 100 28], 'ButtonPushedFcn', @onRemoveObstacle);

    dzy = dzy - 10;
    txtObsList = uitextarea(pnlDZ,'Position',[10 15 205 dzy-15], 'Editable','off', 'Value',{'(no obstacles)'});

    %% Right plots
    axTraj = uiaxes(fig,'Position',[625 505 790 380]);
    title(axTraj,'Trajectory'); xlabel(axTraj,'x (cm)'); ylabel(axTraj,'y (cm)');
    grid(axTraj,'on'); axis(axTraj,'equal');

    axHead = uiaxes(fig,'Position',[625 270 790 220]);
    title(axHead,'Heading vs Desired Heading');
    xlabel(axHead,'Time (s)'); ylabel(axHead,'Angle (deg)'); grid(axHead,'on');

    txtResults = uitextarea(fig, 'Position',[625 40 300 210], 'Editable','off');
    txtResults.Value = {'Results will appear here.'};

    axCTSum = uiaxes(fig,'Position',[935 40 480 210]);
    title(axCTSum,'Cross-track summary'); ylabel(axCTSum,'Cross-track (cm)'); grid(axCTSum,'on');

    sldSpeed.ValueChangingFcn    = @(~,e)set(txtSpeed,'Text',sprintf('%.0f',e.Value));
    sldLA.ValueChangingFcn       = @(~,e)set(txtLA,'Text',sprintf('%.0f',e.Value));
    sldRad.ValueChangingFcn      = @(~,e)set(txtRad,'Text',sprintf('%.0f',e.Value));
    sldPosNoise.ValueChangingFcn = @(~,e)set(txtPosNoise,'Text',sprintf('%.1f',e.Value));
    sldHeadNoise.ValueChangingFcn= @(~,e)set(txtHeadNoise,'Text',sprintf('%.1f',e.Value));

    refreshWaypointStats();
    refreshObstacleList();
    redrawPreview();

    %% ===== Callbacks =====
    function onStop(~,~)
        cancelRequested = true;
        btnStop.Enable = 'off';
    end

    function onAddObstacle(~,~)
        typ = ddObsType.Value;
        hard = chkHardObs.Value;
        w = spnSoftW.Value;
        nums = parseCsvNumbers(edtObsParams.Value);
        if strcmp(typ, 'Circle')
            if numel(nums) ~= 3
                uialert(fig, 'Circle params: cx,cy,r', 'Error'); return;
            end
            o.type = 'circle'; o.params = nums(:).';
        else
            if numel(nums) ~= 4
                uialert(fig, 'Rect params: xmin,ymin,w,h', 'Error'); return;
            end
            o.type = 'rect'; o.params = nums(:).';
        end
        o.hard = logical(hard);
        o.weight = w;
        OBSTACLES(end+1) = o;
        refreshObstacleList();
        onRegenWaypoints();
    end

    function onRemoveObstacle(~,~)
        if ~isempty(OBSTACLES)
            OBSTACLES(end) = [];
            refreshObstacleList();
            onRegenWaypoints();
        end
    end

    function refreshObstacleList()
        if isempty(OBSTACLES)
            txtObsList.Value = {'(no obstacles)'}; return;
        end
        lines = cell(numel(OBSTACLES),1);
        for i = 1:numel(OBSTACLES)
            o = OBSTACLES(i);
            hs = 'soft'; if o.hard, hs = 'HARD'; end
            if strcmp(o.type,'circle')
                lines{i} = sprintf('#%d %s circ cx=%.0f cy=%.0f r=%.0f w=%d', i, hs, o.params(1), o.params(2), o.params(3), o.weight);
            else
                lines{i} = sprintf('#%d %s rect x=%.0f y=%.0f w=%.0f h=%.0f w=%d', i, hs, o.params(1), o.params(2), o.params(3), o.params(4), o.weight);
            end
        end
        txtObsList.Value = lines;
    end

    function onExportCStruct(~,~)
        nWp = size(PATH,1);
        cLines = cell(nWp + 4, 1);
        cLines{1} = sprintf('static constexpr int PATH_LENGTH = %d;', nWp);
        cLines{2} = 'static Waypoint path[PATH_LENGTH] = {';
        for ii = 1:nWp
            comma = ','; if ii == nWp, comma = ''; end
            cLines{ii+2} = sprintf('  {%.1f, %.1f}%s', PATH(ii,1), PATH(ii,2), comma);
        end
        cLines{nWp+3} = '};'; cLines{nWp+4} = '';
        codeStr = strjoin(cLines, newline);
        clipboard('copy', codeStr);
        fprintf('\n--- Waypoints copied to clipboard ---\n%s\n', codeStr);
        lblStatus.Text = sprintf('Exported %d waypoints.', nWp);
    end

    function onSaveFigures(~,~)
        [fname, fpath] = uiputfile('*.png', 'Save trajectory figure');
        if isequal(fname,0), return; end
        exportgraphics(axTraj, fullfile(fpath, fname), 'Resolution', 200);
        lblStatus.Text = sprintf('Saved: %s', fname);
    end

    function onRegenWaypoints(~,~)
        INFLATION_CM = spnInflation.Value;
        X_STEP = spnXStepFt.Value * 12 * 2.54;
        PATH = rebuildPath();
        refreshWaypointStats();
        redrawPreview();
        lblStatus.Text = sprintf('Waypoints regenerated (%d pts).', size(PATH,1));
    end

    %% ===== Path building =====
    function path = rebuildPath()
        planPoly = polybuffer(OUTER_WALLS, -INFLATION_CM);

        [softPolys, hardPoly, ~] = buildObstaclePolys(OBSTACLES, INFLATION_CM);
        if hardPoly.NumRegions > 0
            planPoly = subtract(planPoly, hardPoly);
        end
        for si = 1:numel(softPolys)
            w = softPolys{si}.weight;
            scaledInfl = INFLATION_CM * (w / 100);
            p = obstaclePolyFromStruct(softPolys{si}.obs, scaledInfl);
            if p.NumRegions > 0
                planPoly = subtract(planPoly, p);
            end
        end

        if planPoly.NumRegions < 1
            warning('Planning area empty after obstacle subtraction.');
            path = [0 0; 100 100]; return;
        end

        path = generateMowingWaypoints(planPoly, X_STEP);
        path = validateAndFixSegments(path, planPoly);
    end

    function redrawPreview()
        cla(axTraj);
        plot(axTraj, MAP_AREA(:,1), MAP_AREA(:,2), 'k-', 'LineWidth',1.2); hold(axTraj,'on');
        drawObstacles(axTraj);
        if ~isempty(PATH) && size(PATH,1) >= 2
            plot(axTraj, PATH(:,1), PATH(:,2), 'g--o', 'LineWidth',1.5, 'MarkerSize',5, ...
                'MarkerFaceColor',[0 0.6 0], 'MarkerEdgeColor',[0 0.5 0]);
        end
        title(axTraj,'Waypoint preview');
        axis(axTraj,'equal'); grid(axTraj,'on'); hold(axTraj,'off');
    end

    function drawObstacles(ax)
        if isempty(OBSTACLES), return; end
        tt = linspace(0,2*pi,80);
        for i = 1:numel(OBSTACLES)
            o = OBSTACLES(i);
            if o.hard
                face = [1 0.3 0.3]; edge = 'r';
            else
                face = [1 0.7 0.2]; edge = [0.9 0.5 0];
            end
            if strcmp(o.type,'circle')
                cx=o.params(1); cy=o.params(2); r=o.params(3);
                fill(ax, cx + r*cos(tt), cy + r*sin(tt), face, 'FaceAlpha',0.15, 'EdgeColor',edge, 'LineWidth',1.2, 'HandleVisibility','off');
                if INFLATION_CM > 0
                    rInfl = r + INFLATION_CM; if ~o.hard, rInfl = r + INFLATION_CM*(o.weight/100); end
                    plot(ax, cx + rInfl*cos(tt), cy + rInfl*sin(tt), '--', 'Color',edge, 'LineWidth',0.8, 'HandleVisibility','off');
                end
            else
                xmin=o.params(1); ymin=o.params(2); w=o.params(3); h=o.params(4);
                patch(ax, [xmin xmin+w xmin+w xmin], [ymin ymin ymin+h ymin+h], face, 'FaceAlpha',0.15, 'EdgeColor',edge, 'LineWidth',1.2, 'HandleVisibility','off');
                if INFLATION_CM > 0
                    infl = INFLATION_CM; if ~o.hard, infl = INFLATION_CM*(o.weight/100); end
                    patch(ax, [xmin-infl xmin+w+infl xmin+w+infl xmin-infl], ...
                              [ymin-infl ymin-infl ymin+h+infl ymin+h+infl], ...
                          [1 1 1], 'FaceColor','none', 'EdgeColor',edge, 'LineWidth',0.8, ...
                          'LineStyle','--', 'HandleVisibility','off');
                end
            end
        end
    end

    function refreshWaypointStats()
        nWps = size(PATH,1);
        if nWps >= 2
            seg = diff(PATH,1,1);
            pathLen = sum(sqrt(sum(seg.^2,2)));
        else
            pathLen = 0;
        end
        nSweeps = max(0, floor(nWps/2));
        lblWpStats.Text = sprintf([ ...
            'Waypoints: %d  |  sweeps: %d\n' ...
            'Path length: %.1f cm (%.1f m)\n' ...
            'X step: %.1f cm  |  Inflation: %.0f cm'], ...
            nWps, nSweeps, pathLen, pathLen/100, X_STEP, INFLATION_CM);
    end

    %% ===== Simulation =====
    function onRun(~,~)
        cancelRequested = false;
        btnStop.Enable = 'on';

        V = sldSpeed.Value;
        LA = sldLA.Value;
        WP_R = sldRad.Value;
        POS_STD = sldPosNoise.Value;
        HEAD_STD = deg2rad(sldHeadNoise.Value);
        nRuns = spnRuns.Value;
        maxSteps = spnSteps.Value;
        randomize = chkRandomize.Value;

        if randomize
            seeds = randi([0 1e9], nRuns, 1); repSeed = seeds(1);
        else
            seeds = (0:(nRuns-1)).'; repSeed = 0;
        end

        lblStatus.Text = sprintf('Running... (runs=%d)', nRuns); drawnow;
        dlg = uiprogressdlg(fig, 'Title','Running Monte Carlo', ...
            'Message','Starting...', 'Cancelable','on', 'Value',0);

        allCT = cell(nRuns,1);
        allXT = cell(nRuns,1); allYT = cell(nRuns,1);
        completed = false(nRuns,1);
        failMode = strings(nRuns,1);
        distGoal = zeros(nRuns,1);
        softAvoidRate = zeros(nRuns,1);

        [inflatedSoftPolys, inflatedHardPoly, baseHardPoly] = buildObstaclePolys(OBSTACLES, INFLATION_CM);
        softPolyList = cellfun(@(s) s.poly, inflatedSoftPolys, 'UniformOutput', false);

        for iRun = 1:nRuns
            if cancelRequested || dlg.CancelRequested, break; end
            dlg.Value = (iRun-1)/max(1,nRuns);
            dlg.Message = sprintf('Run %d / %d', iRun, nRuns); drawnow;

            [xt,yt,xm,ym,pathEnded,hitMaxSteps,wallFail,softFrac] = simulateRun( ...
                PATH, V, LA, WP_R, ENDPOINT_RADIUS, POS_STD, HEAD_STD, ...
                TRACK_WIDTH, WHEEL_RADIUS, DT, maxSteps, seeds(iRun), ...
                OUTER_WALLS, inflatedHardPoly, baseHardPoly, softPolyList);

            ct = crossTrackSeries(xt, yt, PATH);
            allCT{iRun} = ct;
            allXT{iRun} = xt; allYT{iRun} = yt;

            dg = hypot(xt(end)-PATH(end,1), yt(end)-PATH(end,2));
            distGoal(iRun) = dg;
            completed(iRun) = (dg < ENDPOINT_RADIUS) && ~wallFail;
            softAvoidRate(iRun) = 1 - softFrac;

            if completed(iRun), failMode(iRun) = "completed";
            elseif wallFail,    failMode(iRun) = "wall_or_obstacle";
            elseif hitMaxSteps, failMode(iRun) = "timeout";
            elseif pathEnded,   failMode(iRun) = "path_ended_not_within_radius";
            else,               failMode(iRun) = "other";
            end
        end

        dlg.Message = sprintf('Representative run (seed=%d)', repSeed); drawnow;
        [xtR,ytR,xmR,ymR,~,~,~,~] = simulateRun( ...
            PATH, V, LA, WP_R, ENDPOINT_RADIUS, POS_STD, HEAD_STD, ...
            TRACK_WIDTH, WHEEL_RADIUS, DT, maxSteps, repSeed, ...
            OUTER_WALLS, inflatedHardPoly, baseHardPoly, softPolyList);

        dlg.Value = 1; dlg.Message = 'Done'; pause(0.05); close(dlg);
        btnStop.Enable = 'off';

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
            meanSoftAvoid = mean(softAvoidRate(ranMask));
        else
            compPct = 0; meanSoftAvoid = NaN;
        end

        avgTime = NaN;
        if nRan > 0
            doneMask = ranMask & completed;
            if any(doneMask)
                idxDone = find(doneMask);
                tDone = arrayfun(@(j)(numel(allXT{j})-1)*DT, idxDone);
                avgTime = mean(tDone);
            end
        end

        modeNames = {'completed','wall_or_obstacle','timeout','path_ended_not_within_radius','other'};
        if nRan > 0
            mAll = categorical(failMode(ranMask), modeNames, 'Ordinal',true);
            cAll = countcats(mAll); pctAll = 100*(cAll/max(1,sum(cAll)));
            failLines = { 'Failure modes:'
                sprintf('  completed: %d (%.1f%%)', cAll(1), pctAll(1))
                sprintf('  wall/obstacle: %d (%.1f%%)', cAll(2), pctAll(2))
                sprintf('  timeout: %d (%.1f%%)', cAll(3), pctAll(3))
                sprintf('  path_ended: %d (%.1f%%)', cAll(4), pctAll(4))
                sprintf('  other: %d (%.1f%%)', cAll(5), pctAll(5))
            };
        else
            failLines = {'Failure modes: n/a'};
        end

        lblStatus.Text = sprintf('Done. Completion=%.1f%%', compPct);

        if isnan(avgTime), avgTimeLine = 'Avg time: n/a';
        else, avgTimeLine = sprintf('Avg time: %.1f s', avgTime); end

        txtResults.Value = [{ sprintf('Completion: %.1f%%', compPct); avgTimeLine;
            sprintf('Mean CT: %.1f cm', meanCT); sprintf('P95 CT: %.1f cm', p95CT);
            sprintf('Max CT: %.1f cm', maxCT); sprintf('Soft avoid: %.1f%%', 100*meanSoftAvoid);
            sprintf('Ran: %d/%d', nRan, nRuns); '' }; failLines];

        % Trajectory plot
        cla(axTraj);
        hMap = plot(axTraj, MAP_AREA(:,1), MAP_AREA(:,2), 'k-', 'LineWidth',1.2); hold(axTraj,'on');
        drawObstacles(axTraj);
        hPlan = plot(axTraj, PATH(:,1), PATH(:,2), 'g--o', 'LineWidth',1.5, 'MarkerSize',5, ...
            'MarkerFaceColor',[0 0.6 0], 'MarkerEdgeColor',[0 0.5 0]);
        for i = 1:nRuns
            if ~isempty(allXT{i})
                plot(axTraj, allXT{i}, allYT{i}, 'r-', 'LineWidth',0.5, 'HandleVisibility','off');
            end
        end
        hMC = plot(axTraj, NaN, NaN, 'r-', 'LineWidth',0.5);
        hMeas = plot(axTraj, xmR, ymR, 'r.', 'MarkerSize',6);
        hTruth = plot(axTraj, xtR, ytR, 'b-', 'LineWidth',1.4);
        hStart = plot(axTraj, xtR(1), ytR(1), 'ms', 'MarkerSize',10, 'LineWidth',1.5);
        hGoal = plot(axTraj, xtR(end), ytR(end), 'r^', 'MarkerSize',10, 'LineWidth',1.5);
        legend(axTraj, [hMap, hPlan, hMC, hMeas, hTruth, hStart, hGoal], ...
            {'Map','Plan','MC','Meas','Truth','Start','Goal'}, 'Location','best');
        title(axTraj, sprintf('Trajectory (seed=%d)', repSeed));
        axis(axTraj,'equal'); grid(axTraj,'on');

        % Heading plot
        cla(axHead);
        [~,~,~,~,~,~,~,~,tH,headDeg,desDeg] = simulateRun( ...
            PATH, V, LA, WP_R, ENDPOINT_RADIUS, POS_STD, HEAD_STD, ...
            TRACK_WIDTH, WHEEL_RADIUS, DT, maxSteps, repSeed, ...
            OUTER_WALLS, inflatedHardPoly, baseHardPoly, softPolyList);
        if ~isempty(tH)
            plot(axHead, tH, headDeg, 'b-', 'LineWidth',1.2); hold(axHead,'on');
            plot(axHead, tH, desDeg, 'r--', 'LineWidth',1.2);
            legend(axHead, {'Heading','Desired'},'Location','best');
            grid(axHead,'on'); hold(axHead,'off');
            title(axHead, sprintf('Heading (seed=%d)', repSeed));
            xlabel(axHead,'Time (s)'); ylabel(axHead,'Angle (deg)');
        end

        % Cross-track boxplot
        cla(axCTSum);
        if nRan > 0
            idxs = find(ranMask);
            ctStats = zeros(nRan,3);
            for j = 1:nRan
                c = allCT{idxs(j)};
                ctStats(j,:) = [mean(c) prctile(c,95) max(c)];
            end
            boxplot(axCTSum, ctStats, 'Labels',{'Mean','P95','Max'});
            ylabel(axCTSum,'Cross-track (cm)');
            title(axCTSum, sprintf('Cross-track (WP_R=%.0f)', WP_R));
            grid(axCTSum,'on');
        end
    end

    %% ===== Simulation core =====
    function [xt,yt,xm,ym,pathEnded,hitMaxSteps,wallFail,softFrac,tHist,headingDeg,desiredHeadingDeg] = simulateRun( ...
            path, velocity, lookahead, wpRadius, endRadius, posStd, headStd, trackW, wheelR, dt, maxSteps, seed, ...
            outerWalls, inflatedHardPoly, baseHardPoly, softPolyList)
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
        softHits = 0;

        recordHeading = (nargout >= 11);
        if recordHeading
            tHist = zeros(maxSteps,1); headingDeg = zeros(maxSteps,1); desiredHeadingDeg = zeros(maxSteps,1);
            tHist(1) = 0; headingDeg(1) = rad2deg(theta); desiredHeadingDeg(1) = rad2deg(theta);
        else
            tHist = []; headingDeg = []; desiredHeadingDeg = [];
        end

        for k = 2:maxSteps
            if segIdx >= size(path,1)
                pathEnded = true;
                xt = xt(1:k-1); yt = yt(1:k-1); xm = xm(1:k-1); ym = ym(1:k-1);
                if recordHeading, tHist=tHist(1:k-1); headingDeg=headingDeg(1:k-1); desiredHeadingDeg=desiredHeadingDeg(1:k-1); end
                softFrac = softHits / max(1, k-1); return;
            end

            segIdx = advanceSegment(xm_i, ym_i, path, segIdx, wpRadius);
            [gx, gy, foundGoal] = findLookaheadGoal(xm_i, ym_i, path, segIdx, lookahead);

            % Goal sanity check: only fail if goal is outside the PHYSICAL outer
            % walls or inside a HARD obstacle's base polygon. The goal is always on
            % the planned path so it should be well inside the inflated boundary;
            % checking against inflatedOuter caused false failures due to
            % isinterior returning false for boundary points.
            if ~foundGoal || ~isinterior(outerWalls, gx, gy) || ...
                    (baseHardPoly.NumRegions > 0 && isinterior(baseHardPoly, gx, gy))
                wallFail = true;
                xt = xt(1:k-1); yt = yt(1:k-1); xm = xm(1:k-1); ym = ym(1:k-1);
                if recordHeading, tHist=tHist(1:k-1); headingDeg=headingDeg(1:k-1); desiredHeadingDeg=desiredHeadingDeg(1:k-1); end
                softFrac = softHits / max(1, k-1); return;
            end

            if recordHeading
                tHist(k) = (k-1)*dt;
                headingDeg(k) = rad2deg(theta);
                desiredHeadingDeg(k) = rad2deg(atan2(gy - ym_i, gx - xm_i));
            end

            [wl, wr, valid] = purePursuitCommand(xm_i, ym_i, thm_i, path, segIdx, lookahead, velocity, trackW, wheelR);
            if ~valid, wl = 0.5*velocity/wheelR; wr = wl; end

            vL = wl*wheelR; vR = wr*wheelR;
            v = (vL+vR)/2; omega = (vR-vL)/trackW;
            x = x + v*cos(theta)*dt;
            y = y + v*sin(theta)*dt;
            theta = wrapPi(theta + omega*dt);

            % Robot position check: fail if truth position exits the physical
            % outer walls or enters a hard obstacle (base, un-inflated).
            % Inflation is for PLANNING (keeping waypoints away), not for runtime
            % failure of the actual robot which only fails on real collision.
            if ~isinterior(outerWalls, x, y) || ...
                    (baseHardPoly.NumRegions > 0 && isinterior(baseHardPoly, x, y))
                wallFail = true;
                xt(k)=x; yt(k)=y; xm(k)=xm_i; ym(k)=ym_i;
                xt = xt(1:k); yt = yt(1:k); xm = xm(1:k); ym = ym(1:k);
                if recordHeading, tHist=tHist(1:k); headingDeg=headingDeg(1:k); desiredHeadingDeg=desiredHeadingDeg(1:k); end
                softFrac = softHits / max(1, k); return;
            end

            for si = 1:numel(softPolyList)
                pS = softPolyList{si};
                if pS.NumRegions > 0 && isinterior(pS, x, y)
                    softHits = softHits + 1; break;
                end
            end

            xm_i = x + randn()*posStd; ym_i = y + randn()*posStd;
            thm_i = wrapPi(theta + randn()*headStd);
            xt(k)=x; yt(k)=y; xm(k)=xm_i; ym(k)=ym_i;

            if hypot(x - path(end,1), y - path(end,2)) < endRadius
                xt = xt(1:k); yt = yt(1:k); xm = xm(1:k); ym = ym(1:k);
                if recordHeading, tHist=tHist(1:k); headingDeg=headingDeg(1:k); desiredHeadingDeg=desiredHeadingDeg(1:k); end
                softFrac = softHits / max(1, k); return;
            end
        end

        hitMaxSteps = true;
        xt = xt(:); yt = yt(:); xm = xm(:); ym = ym(:);
        if recordHeading, tHist=tHist(:); headingDeg=headingDeg(:); desiredHeadingDeg=desiredHeadingDeg(:); end
        softFrac = softHits / max(1, numel(xt));
    end

    %% ===== Pure pursuit helpers =====
    function segIdx = advanceSegment(curx, cury, path, segIdx, wpRadius)
        while segIdx < size(path,1)
            next = path(segIdx+1,:);
            dx = next(1)-curx; dy = next(2)-cury;
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
        dx = gx-curx; dy = gy-cury;
        L = hypot(dx,dy);
        if L < 1.0, wl=0; wr=0; valid=false; return; end
        alpha = wrapPi(atan2(dy,dx) - heading);
        K = 2*sin(alpha)/L;
        omega = K*velocity;
        wl = (velocity - omega*trackW/2)/wheelR;
        wr = (velocity + omega*trackW/2)/wheelR;
        valid = true;
    end

    function [gx, gy, found] = findLookaheadGoal(curx, cury, path, segIdx, lookahead)
        found = false; gx = NaN; gy = NaN;
        Lsq = lookahead^2;
        for seg = segIdx:(size(path,1)-1)
            a = path(seg,:); b = path(seg+1,:);
            ds = b - a;
            fx = a(1)-curx; fy = a(2)-cury;
            qa = ds(1)^2 + ds(2)^2;
            qb = 2*(fx*ds(1) + fy*ds(2));
            qc = (fx^2 + fy^2) - Lsq;
            disc = qb^2 - 4*qa*qc;
            if disc < 0, continue; end
            sd = sqrt(disc);
            t1 = (-qb - sd)/(2*qa);
            t2 = (-qb + sd)/(2*qa);
            bestT = -1;
            if t2 >= 0 && t2 <= 1, bestT = t2;
            elseif t1 >= 0 && t1 <= 1, bestT = t1; end
            if bestT >= 0
                g = a + bestT*ds; gx = g(1); gy = g(2); found = true; return;
            end
        end
        if segIdx < size(path,1)
            gx = path(segIdx+1,1); gy = path(segIdx+1,2); found = true;
        end
    end

    function ct = crossTrackSeries(x, y, path)
        n = numel(x); ct = zeros(n,1);
        for i = 1:n, ct(i) = minDistToPolyline(x(i), y(i), path); end
    end

    function dmin = minDistToPolyline(px, py, path)
        dmin = inf;
        for seg = 1:(size(path,1)-1)
            a = path(seg,:); b = path(seg+1,:);
            ab = b-a; ap = [px-a(1), py-a(2)];
            denom = (ab(1)^2 + ab(2)^2) + 1e-10;
            t = max(0, min(1, (ap(1)*ab(1)+ap(2)*ab(2)) / denom));
            proj = a + t*ab;
            d = hypot(px-proj(1), py-proj(2));
            if d < dmin, dmin = d; end
        end
    end

    function a = wrapPi(a)
        while a > pi, a = a - 2*pi; end
        while a < -pi, a = a + 2*pi; end
    end

    %% ===== Obstacle polygon builders =====
    function [softList, hardPoly, baseHardPoly] = buildObstaclePolys(obstacles, inflationCm)
        softList = {};
        baseHardPoly = polyshape();
        hardPoly = polyshape();
        for i = 1:numel(obstacles)
            o = obstacles(i);
            pI = obstaclePolyFromStruct(o, inflationCm);
            if o.hard
                baseHardPoly = union(baseHardPoly, obstaclePolyFromStruct(o, 0));
                hardPoly = union(hardPoly, pI);
            else
                softList{end+1} = struct('poly', pI, 'weight', o.weight, 'obs', o); %#ok<AGROW>
            end
        end
    end

    function p = obstaclePolyFromStruct(o, inflationCm)
        if strcmp(o.type,'circle')
            cx=o.params(1); cy=o.params(2); r=max(0, o.params(3)+inflationCm);
            p = nsidedpoly(60, 'Center',[cx cy], 'Radius',r);
        else
            xmin=o.params(1); ymin=o.params(2); w=o.params(3); h=o.params(4);
            p = polyshape([xmin-inflationCm xmin+w+inflationCm xmin+w+inflationCm xmin-inflationCm], ...
                          [ymin-inflationCm ymin-inflationCm ymin+h+inflationCm ymin+h+inflationCm]);
        end
    end

    function nums = parseCsvNumbers(s)
        s = strrep(string(s), ';', ',');
        parts = split(s, ',');
        nums = [];
        for i = 1:numel(parts)
            t = strtrim(parts(i));
            if t == "", continue; end
            v = str2double(t);
            if isnan(v), nums = []; return; end
            nums(end+1) = v; %#ok<AGROW>
        end
    end

    %% ===== Waypoint generation =====
    function waypoints = generateMowingWaypoints(poly, xStep)
        % Boustrophedon mowing path inside polyshape, with waypoints inset from
        % the boundary so they are strictly interior (avoids isinterior == false
        % for points exactly on the boundary).
        BOUNDARY_INSET = 15; % cm inset from polygon boundary for each waypoint

        if poly.NumRegions < 1
            waypoints = [0 0; 100 100]; return;
        end
        verts = poly.Vertices;
        xMin = min(verts(:,1)); xMax = max(verts(:,1));

        xs = (xMin + BOUNDARY_INSET) : xStep : (xMax - BOUNDARY_INSET);
        if numel(xs) < 2
            xs = linspace(xMin + BOUNDARY_INSET, xMax - BOUNDARY_INSET, max(2, round((xMax-xMin)/xStep)+1));
        end

        wps = [];
        dir = 1;
        [bx, by] = boundary(poly);

        for i = 1:numel(xs)
            x0 = xs(i);
            yRange = [min(verts(:,2))-1000, max(verts(:,2))+1000];
            [~, yi] = polyxpoly(bx, by, [x0 x0], yRange);
            if numel(yi) < 2, continue; end
            yi = sort(yi(:));
            nPairs = floor(numel(yi)/2);
            spans = reshape(yi(1:2*nPairs),2,[]).';
            % Inset from boundary
            spans(:,1) = spans(:,1) + BOUNDARY_INSET;
            spans(:,2) = spans(:,2) - BOUNDARY_INSET;
            spans = spans(spans(:,2) - spans(:,1) > 10, :);
            if isempty(spans), continue; end
            if dir > 0
                for s = 1:size(spans,1)
                    wps = [wps; x0 spans(s,1); x0 spans(s,2)]; %#ok<AGROW>
                end
            else
                for s = size(spans,1):-1:1
                    wps = [wps; x0 spans(s,2); x0 spans(s,1)]; %#ok<AGROW>
                end
            end
            dir = -dir;
        end

        if isempty(wps) || size(wps,1) < 2
            waypoints = [0 0; 100 100]; return;
        end
        d = sqrt(sum(diff(wps,1,1).^2,2));
        waypoints = wps([true; d > 1e-6], :);
    end

    %% ===== Segment validation =====
    function path = validateAndFixSegments(path, poly)
        % Ensure every segment A->B stays inside poly. For any segment that exits
        % (transition between mowing sweeps crossing a concavity), insert
        % intermediate waypoints routed along the polygon interior.
        SAMPLE_DIST = 40;   % cm between sample points along each segment
        INSET = 20;         % cm inset from boundary for inserted points
        MAX_PASSES = 8;

        for pass = 1:MAX_PASSES
            changed = false;
            i = 1;
            while i < size(path,1)
                A = path(i,:);
                B = path(i+1,:);
                segLen = norm(B - A);
                nSamp = max(5, ceil(segLen / SAMPLE_DIST));
                ts = linspace(0, 1, nSamp);
                pts = A + ts(:) * (B - A);
                inside = isinterior(poly, pts(:,1), pts(:,2));

                if all(inside)
                    i = i + 1; continue;
                end

                % Find the midpoint of the exterior portion
                extIdx = find(~inside);
                midExtIdx = extIdx(ceil(numel(extIdx)/2));
                extPt = pts(midExtIdx, :);

                newPt = findInteriorDetourPoint(extPt, poly, INSET);
                if isempty(newPt)
                    i = i + 1; continue;
                end

                path = [path(1:i,:); newPt; path(i+1:end,:)];
                changed = true;
            end
            if ~changed, break; end
        end
    end

    function pt = findInteriorDetourPoint(target, poly, inset)
        % Find a point inside poly near target by searching along boundary
        % normals. Returns [] if nothing found.
        [bx, by] = boundary(poly);
        nB = numel(bx);
        dists = hypot(bx - target(1), by - target(2));
        [~, sortIdx] = sort(dists);

        for trial = 1:min(12, nB)
            idx = sortIdx(trial);
            iPrev = mod(idx-2, nB) + 1;
            iNext = mod(idx, nB) + 1;
            tangent = [bx(iNext)-bx(iPrev), by(iNext)-by(iPrev)];
            nrm = [-tangent(2), tangent(1)];
            nLen = norm(nrm);
            if nLen < 1e-10, continue; end
            nrm = nrm / nLen;

            for sign = [1 -1]
                cand = [bx(idx) + sign*inset*nrm(1), by(idx) + sign*inset*nrm(2)];
                if isinterior(poly, cand(1), cand(2))
                    pt = cand; return;
                end
            end
        end
        pt = [];
    end
end
