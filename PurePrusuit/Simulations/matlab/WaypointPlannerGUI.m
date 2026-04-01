function WaypointPlannerGUI()
% WaypointPlannerGUI
% Obstacle-aware waypoint planning GUI.
%
% Outputs:
% - Saves MAT handoff file: planner_last_waypoints.mat (variable PATH Nx2 + meta)
% - Exports Arduino C struct to clipboard (Waypoint{wp_x, wp_y})
% - Exports CSV (x_cm,y_cm)
%
% Modes:
% - Coverage (mowing)
% - Coverage + Connectors (grid planner when straight links invalid)
% - Start → Goal (grid A*)

    %% Map (same as TenPointPurePursuitGUI)
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
    outerWalls = polyshape(MAP_AREA(:,1), MAP_AREA(:,2));

    %% Planner state
    st = struct();
    st.mode = "Coverage (mowing)";
    st.obstacles = struct('type',{},'hard',{},'params',{},'weight',{});
    st.inflationCm = 0;
    st.densifyStepCm = 25;
    st.densifyEnabled = true;
    st.gridResolutionCm = 10;
    st.coverageXStepCm = 8*12*2.54;
    st.snapEnabled = false;
    st.snapStage = 0;           % 0 idle, 1 waiting for second click
    st.snapP0 = [NaN NaN];
    st.currentPATH = [];
    st.lastValidation = struct();

    %% UI
    fig = uifigure('Name','Waypoint Planner', 'Position',[40 40 1450 900]);

    pnl = uipanel(fig, 'Title','Planner Controls', 'Position',[15 15 380 870]);
    axPlan = uiaxes(fig, 'Position',[410 340 1025 545]);
    axFail = uiaxes(fig, 'Position',[410 15 1025 310]);

    title(axPlan,'Planner view'); xlabel(axPlan,'x (cm)'); ylabel(axPlan,'y (cm)');
    grid(axPlan,'on'); axis(axPlan,'equal');
    title(axFail,'Failure visualization'); xlabel(axFail,'x (cm)'); ylabel(axFail,'y (cm)');
    grid(axFail,'on'); axis(axFail,'equal');

    % Use figure-level click callback so axes children don't swallow clicks.
    fig.WindowButtonDownFcn = @onFigureMouseDown;

    y = 820;
    uilabel(pnl,'Position',[15 y 140 22],'Text','Mode');
    ddMode = uidropdown(pnl,'Position',[160 y 205 22], ...
        'Items',{'Coverage (mowing)','Coverage + Connectors','Start → Goal'}, ...
        'Value',char(st.mode), ...
        'ValueChangedFcn', @onParamChanged);

    y = y - 40;
    uilabel(pnl,'Position',[15 y 140 22],'Text','Inflation (cm)');
    spnInfl = uispinner(pnl,'Position',[160 y 205 22], ...
        'Limits',[0 500], 'Value',st.inflationCm, 'Step',5, ...
        'ValueChangedFcn', @onParamChanged);

    y = y - 40;
    uilabel(pnl,'Position',[15 y 140 22],'Text','Densify step (cm)');
    spnStep = uispinner(pnl,'Position',[160 y 205 22], ...
        'Limits',[5 500], 'Value',st.densifyStepCm, 'Step',5, ...
        'ValueChangedFcn', @onParamChanged);

    y = y - 28;
    chkDensify = uicheckbox(pnl,'Position',[160 y 205 22], ...
        'Text','Enable densify', ...
        'Value',true, ...
        'ValueChangedFcn', @onParamChanged);

    y = y - 40;
    uilabel(pnl,'Position',[15 y 140 22],'Text','Grid res (cm)');
    spnGrid = uispinner(pnl,'Position',[160 y 205 22], ...
        'Limits',[5 100], 'Value',st.gridResolutionCm, 'Step',5, ...
        'ValueChangedFcn', @onParamChanged);

    y = y - 40;
    uilabel(pnl,'Position',[15 y 140 22],'Text','Coverage X step (cm)');
    spnXStep = uispinner(pnl,'Position',[160 y 205 22], ...
        'Limits',[10 500], 'Value',st.coverageXStepCm, 'Step',10, ...
        'ValueChangedFcn', @onParamChanged);

    y = y - 55;
    uilabel(pnl,'Position',[15 y 350 20],'Text','Obstacles', 'FontWeight','bold');

    y = y - 35;
    uilabel(pnl,'Position',[15 y 60 22],'Text','Type');
    ddType = uidropdown(pnl,'Position',[80 y 140 22], ...
        'Items',{'Circle','Rectangle (axis-aligned)'}, 'Value','Circle');
    chkHard = uicheckbox(pnl,'Position',[230 y 140 22], 'Text','Hard', 'Value',true);

    y = y - 35;
    uilabel(pnl,'Position',[15 y 60 22],'Text','Weight');
    spnW = uispinner(pnl,'Position',[80 y 140 22], 'Limits',[0 100], 'Value',10, 'Step',1);
    uilabel(pnl,'Position',[230 y 140 22],'Text','(soft only)');

    y = y - 35;
    uilabel(pnl,'Position',[15 y 60 22],'Text','Params');
    edtParams = uieditfield(pnl,'text', 'Position',[80 y 290 22], ...
        'Value','500,500,100'); % circle: cx,cy,r   rect: xmin,ymin,w,h

    y = y - 35;
    chkSnap = uicheckbox(pnl,'Position',[15 y 350 22], ...
        'Text','Snap from click (two clicks to place)', ...
        'Value',false, ...
        'ValueChangedFcn', @onSnapToggled);

    y = y - 40;
    btnAdd = uibutton(pnl,'push', 'Text','Add obstacle', ...
        'Position',[15 y 120 32], 'ButtonPushedFcn', @onAddObstacle);
    btnRm = uibutton(pnl,'push', 'Text','Remove last', ...
        'Position',[145 y 120 32], 'ButtonPushedFcn', @onRemoveObstacle);
    btnClear = uibutton(pnl,'push', 'Text','Clear all', ...
        'Position',[275 y 90 32], 'ButtonPushedFcn', @onClearObstacles);

    y = y - 45;
    btnGen = uibutton(pnl,'push', 'Text','Generate / Update PATH', ...
        'Position',[15 y 350 34], 'ButtonPushedFcn', @onGenerate);

    y = y - 45;
    btnVal = uibutton(pnl,'push', 'Text','Validate segments', ...
        'Position',[15 y 170 32], 'ButtonPushedFcn', @onValidate);
    btnMC = uibutton(pnl,'push', 'Text','Monte Carlo (fail viz)', ...
        'Position',[195 y 170 32], 'ButtonPushedFcn', @onMonteCarlo);

    y = y - 45;
    btnSaveMat = uibutton(pnl,'push', 'Text','Save to TenPoint (MAT)', ...
        'Position',[15 y 170 32], 'ButtonPushedFcn', @onSaveMatHandoff);
    btnExportC = uibutton(pnl,'push', 'Text','Export C struct', ...
        'Position',[195 y 170 32], 'ButtonPushedFcn', @onExportCStruct);

    y = y - 45;
    btnExportCsv = uibutton(pnl,'push', 'Text','Export CSV', ...
        'Position',[15 y 170 32], 'ButtonPushedFcn', @onExportCSV);
    btnSaveImg = uibutton(pnl,'push', 'Text','Save planner figure', ...
        'Position',[195 y 170 32], 'ButtonPushedFcn', @onSavePlannerImage);

    txtReport = uitextarea(pnl, 'Position',[15 15 350 y-25], 'Editable','off', ...
        'Value',{'Ready.'});

    % initial draw
    redrawAll();

    %% ---------------- Callbacks ----------------
    function onParamChanged(~,~)
        st.mode = string(ddMode.Value);
        st.inflationCm = spnInfl.Value;
        st.densifyStepCm = spnStep.Value;
        st.densifyEnabled = chkDensify.Value;
        st.gridResolutionCm = spnGrid.Value;
        st.coverageXStepCm = spnXStep.Value;
        redrawAll();
    end

    function onSnapToggled(~,~)
        st.snapEnabled = chkSnap.Value;
        st.snapStage = 0;
        st.snapP0 = [NaN NaN];
        redrawAll();
    end

    function onFigureMouseDown(~,~)
        if ~st.snapEnabled
            return;
        end
        cp = axPlan.CurrentPoint;
        x = cp(1,1); y = cp(1,2);
        if ~isfinite(x) || ~isfinite(y)
            return;
        end
        if st.snapStage == 0
            st.snapP0 = [x y];
            st.snapStage = 1;
            txtReport.Value = {'Snap: click second point to finish obstacle.'};
            redrawAll();
            return;
        end

        p1 = st.snapP0;
        p2 = [x y];
        typ = ddType.Value;
        hard = chkHard.Value;
        w = spnW.Value;
        if strcmp(typ,'Circle')
            r = hypot(p2(1)-p1(1), p2(2)-p1(2));
            params = [p1(1) p1(2) r];
        else
            xmin = min(p1(1), p2(1));
            ymin = min(p1(2), p2(2));
            wRect = abs(p2(1)-p1(1));
            hRect = abs(p2(2)-p1(2));
            params = [xmin ymin wRect hRect];
        end
        addObstacle(typ, hard, params, w);
        st.snapStage = 0;
        st.snapP0 = [NaN NaN];
        redrawAll();
    end

    function onAddObstacle(~,~)
        typ = ddType.Value;
        hard = chkHard.Value;
        w = spnW.Value;
        nums = parseCsvNumbers(edtParams.Value);
        if strcmp(typ,'Circle')
            if numel(nums) ~= 3
                txtReport.Value = {'Params must be: cx,cy,r for Circle.'};
                return;
            end
            params = nums(:).';
        else
            if numel(nums) ~= 4
                txtReport.Value = {'Params must be: xmin,ymin,w,h for Rectangle.'};
                return;
            end
            params = nums(:).';
        end
        addObstacle(typ, hard, params, w);
        redrawAll();
    end

    function onRemoveObstacle(~,~)
        if ~isempty(st.obstacles)
            st.obstacles(end) = [];
        end
        redrawAll();
    end

    function onClearObstacles(~,~)
        st.obstacles = struct('type',{},'hard',{},'params',{},'weight',{});
        redrawAll();
    end

    function onGenerate(~,~)
        [freePoly, inflatedHard] = buildFreeSpace(outerWalls, st.obstacles, st.inflationCm);
        if freePoly.NumRegions < 1
            txtReport.Value = {'Free space empty after obstacles/inflation.'};
            st.currentPATH = [];
            redrawAll();
            return;
        end

        mode = st.mode;
        if mode == "Coverage (mowing)"
            wps = generateCoverageWaypoints(freePoly, st.coverageXStepCm);
        elseif mode == "Coverage + Connectors"
            wps0 = generateCoverageWaypoints(freePoly, st.coverageXStepCm);
            wps = addConnectorsIfNeeded(wps0, freePoly, inflatedHard, st.gridResolutionCm);
        else
            % Start → Goal: use two points from params field as (sx,sy,gx,gy)
            nums = parseCsvNumbers(edtParams.Value);
            if numel(nums) ~= 4
                txtReport.Value = {'For Start→Goal, put sx,sy,gx,gy into Params field.'};
                return;
            end
            start = nums(1:2);
            goal = nums(3:4);
            wps = planGridPath(freePoly, start, goal, st.gridResolutionCm);
        end

        if isempty(wps) || size(wps,1) < 2
            txtReport.Value = {'Generated path is empty.'};
            st.currentPATH = [];
            redrawAll();
            return;
        end

        if st.densifyEnabled
            wps = densifyPolyline(wps, st.densifyStepCm);
        end
        st.currentPATH = wps;
        txtReport.Value = {sprintf('Generated PATH: %d points', size(wps,1))};
        redrawAll();
    end

    function onValidate(~,~)
        if isempty(st.currentPATH)
            txtReport.Value = {'No PATH yet. Click Generate first.'};
            return;
        end
        [freePoly, inflatedHard] = buildFreeSpace(outerWalls, st.obstacles, st.inflationCm);
        rep = validatePathSegments(st.currentPATH, inflatedHard);
        st.lastValidation = rep;
        txtReport.Value = rep.lines;
        redrawAll();
    end

    function onMonteCarlo(~,~)
        if isempty(st.currentPATH)
            txtReport.Value = {'No PATH yet. Click Generate first.'};
            return;
        end
        % Minimal Monte Carlo: propagate pure-pursuit along path (same dynamics assumptions)
        cfg = struct();
        cfg.DT = 0.10;
        cfg.MAX_STEPS = 20000;
        cfg.N_RUNS = 25;
        cfg.POS_STD = 35.34;
        cfg.HEAD_STD = deg2rad(10);
        cfg.TRACK_W = 43.18;
        cfg.WHEEL_R = 15.24;
        cfg.END_R = 150;
        cfg.V = 150;
        cfg.LA = 150;
        cfg.WP_R = 100;

        [freePoly, inflatedHard] = buildFreeSpace(outerWalls, st.obstacles, st.inflationCm);
        [failPts, failModes] = runMonteCarloFailure(st.currentPATH, cfg, freePoly, inflatedHard);

        cla(axFail);
        plot(axFail, MAP_AREA(:,1), MAP_AREA(:,2), 'k-', 'LineWidth',1.0); hold(axFail,'on');
        drawObstacles(axFail, st.obstacles, st.inflationCm);
        if ~isempty(failPts)
            scatter(axFail, failPts(:,1), failPts(:,2), 22, 'r', 'filled');
        end
        title(axFail, sprintf('First failure points (n=%d)', size(failPts,1)));
        axis(axFail,'equal'); grid(axFail,'on'); hold(axFail,'off');

        if isempty(failModes)
            txtReport.Value = {'Monte Carlo done. No failures recorded.'};
        else
            cats = categories(categorical(failModes));
            txt = cell(1+numel(cats),1);
            txt{1} = sprintf('Monte Carlo (n=%d). Fail modes:', cfg.N_RUNS);
            for i = 1:numel(cats)
                txt{i+1} = sprintf('  %s: %d', cats{i}, sum(strcmp(failModes, cats{i})));
            end
            txtReport.Value = txt;
        end
    end

    function onSaveMatHandoff(~,~)
        if isempty(st.currentPATH)
            txtReport.Value = {'No PATH to save.'};
            return;
        end
        thisDir = fileparts(mfilename('fullpath'));
        PATH = st.currentPATH; %#ok<NASGU>
        meta = struct();
        meta.mode = char(st.mode);
        meta.inflationCm = st.inflationCm;
        meta.densifyEnabled = st.densifyEnabled;
        meta.densifyStepCm = st.densifyStepCm;
        meta.gridResolutionCm = st.gridResolutionCm;
        meta.coverageXStepCm = st.coverageXStepCm;
        meta.obstacleCount = numel(st.obstacles);
        save(fullfile(thisDir, 'planner_last_waypoints.mat'), 'PATH', 'meta');
        txtReport.Value = {sprintf('Saved MAT handoff: %s', fullfile(thisDir,'planner_last_waypoints.mat'))};
    end

    function onExportCStruct(~,~)
        if isempty(st.currentPATH)
            txtReport.Value = {'No PATH to export.'};
            return;
        end
        PATH = st.currentPATH;
        nWp = size(PATH,1);
        cLines = cell(nWp + 4, 1);
        cLines{1} = sprintf('static constexpr int PATH_LENGTH = %d;', nWp);
        cLines{2} = sprintf('static Waypoint path[PATH_LENGTH] = {');
        for ii = 1:nWp
            comma = ',';
            if ii == nWp, comma = ''; end
            cLines{ii+2} = sprintf('  {%.1f, %.1f}%s', PATH(ii,1), PATH(ii,2), comma);
        end
        cLines{nWp+3} = '};';
        cLines{nWp+4} = '';
        codeStr = strjoin(cLines, newline);
        clipboard('copy', codeStr);
        txtReport.Value = {sprintf('Copied %d waypoints to clipboard as C struct.', nWp)};
    end

    function onExportCSV(~,~)
        if isempty(st.currentPATH)
            txtReport.Value = {'No PATH to export.'};
            return;
        end
        [fname, fpath] = uiputfile('*.csv', 'Export waypoints CSV');
        if isequal(fname,0), return; end
        PATH = st.currentPATH;
        T = table(PATH(:,1), PATH(:,2), 'VariableNames',{'x_cm','y_cm'});
        writetable(T, fullfile(fpath, fname));
        txtReport.Value = {sprintf('Exported CSV: %s', fullfile(fpath,fname))};
    end

    function onSavePlannerImage(~,~)
        [fname, fpath] = uiputfile('*.png', 'Save planner view');
        if isequal(fname,0), return; end
        exportgraphics(axPlan, fullfile(fpath, fname), 'Resolution', 200);
        txtReport.Value = {sprintf('Saved: %s', fullfile(fpath,fname))};
    end

    %% ---------------- Core helpers ----------------
    function addObstacle(typ, hard, params, weight)
        o = struct();
        if strcmp(typ,'Circle')
            o.type = 'circle';
        else
            o.type = 'rect';
        end
        o.hard = logical(hard);
        o.params = params;
        o.weight = weight;
        st.obstacles(end+1) = o;
    end

    function redrawAll()
        cla(axPlan);
        plot(axPlan, MAP_AREA(:,1), MAP_AREA(:,2), 'k-', 'LineWidth',1.2); hold(axPlan,'on');
        [freePoly, ~] = buildFreeSpace(outerWalls, st.obstacles, st.inflationCm);
        if freePoly.NumRegions > 0
            [bx, by] = boundary(freePoly);
            plot(axPlan, bx, by, 'k--', 'LineWidth',1.0);
        end
        drawObstacles(axPlan, st.obstacles, st.inflationCm);

        if st.snapEnabled && st.snapStage == 1
            plot(axPlan, st.snapP0(1), st.snapP0(2), 'mo', 'MarkerSize',10, 'LineWidth',2);
        end

        if ~isempty(st.currentPATH)
            plot(axPlan, st.currentPATH(:,1), st.currentPATH(:,2), 'g--', 'LineWidth',1.5);
            plot(axPlan, st.currentPATH(:,1), st.currentPATH(:,2), 'go', 'MarkerSize',4, 'LineWidth',1.0);
        end

        % Highlight violations if present
        if isfield(st.lastValidation,'badSegments') && ~isempty(st.lastValidation.badSegments)
            bs = st.lastValidation.badSegments;
            for k = 1:size(bs,1)
                i = bs(k,1);
                plot(axPlan, st.currentPATH(i:i+1,1), st.currentPATH(i:i+1,2), 'r-', 'LineWidth',2.5);
            end
        end

        axis(axPlan,'equal'); grid(axPlan,'on'); hold(axPlan,'off');
    end

    function drawObstacles(ax, obstacles, inflationCm)
        th = linspace(0,2*pi,80);
        for i = 1:numel(obstacles)
            o = obstacles(i);
            isHard = o.hard;
            face = [1 0.3 0.3];
            edge = 'r';
            if ~isHard
                face = [1 0.7 0.2];
                edge = [0.9 0.5 0];
            end
            if strcmp(o.type,'circle')
                cx=o.params(1); cy=o.params(2); r=o.params(3);
                rInf = r + inflationCm;
                fill(ax, cx + r*cos(th), cy + r*sin(th), face, 'FaceAlpha',0.18, 'EdgeColor',edge, 'LineWidth',1.2);
                if inflationCm > 0
                    plot(ax, cx + rInf*cos(th), cy + rInf*sin(th), '-', 'Color',edge, 'LineWidth',1.0);
                end
            else
                xmin=o.params(1); ymin=o.params(2); w=o.params(3); h=o.params(4);
                x0=xmin; x1=xmin+w; y0=ymin; y1=ymin+h;
                patch(ax, [x0 x1 x1 x0], [y0 y0 y1 y1], face, 'FaceAlpha',0.18, 'EdgeColor',edge, 'LineWidth',1.2);
                if inflationCm > 0
                    patch(ax, [x0-inflationCm x1+inflationCm x1+inflationCm x0-inflationCm], ...
                              [y0-inflationCm y0-inflationCm y1+inflationCm y1+inflationCm], ...
                          'none', 'EdgeColor',edge, 'LineWidth',1.0, 'LineStyle','-');
                end
            end
        end
    end

    function [freePoly, inflatedHard] = buildFreeSpace(mapPoly, obstacles, inflationCm)
        inflatedHard = polyshape();
        freePoly = mapPoly;
        for i = 1:numel(obstacles)
            o = obstacles(i);
            if ~o.hard
                continue;
            end
            p = obstaclePoly(o, inflationCm);
            if p.NumRegions > 0
                inflatedHard = union(inflatedHard, p);
                freePoly = subtract(freePoly, p);
            end
        end
    end

    function p = obstaclePoly(o, inflationCm)
        if strcmp(o.type,'circle')
            cx=o.params(1); cy=o.params(2); r=o.params(3) + inflationCm;
            p = nsidedpoly(60, 'Center',[cx cy], 'Radius',max(0,r));
        else
            xmin=o.params(1); ymin=o.params(2); w=o.params(3); h=o.params(4);
            x0=xmin-inflationCm; y0=ymin-inflationCm;
            x1=xmin+w+inflationCm; y1=ymin+h+inflationCm;
            p = polyshape([x0 x1 x1 x0], [y0 y0 y1 y1]);
        end
    end

    function wps = generateCoverageWaypoints(poly, xStep)
        if poly.NumRegions < 1
            wps = [];
            return;
        end
        v = poly.Vertices;
        xMin = min(v(:,1)); xMax = max(v(:,1));
        xs = xMin:xStep:xMax;
        if numel(xs) < 2
            xs = linspace(xMin, xMax, max(2, round((xMax-xMin)/xStep)+1));
        end

        wps = [];
        dir = 1;
        [bx, by] = boundary(poly);
        yLine = [min(v(:,2))-1000, max(v(:,2))+1000];
        for i = 1:numel(xs)
            x0 = xs(i);
            xLine = [x0 x0];
            [~, yi] = polyxpoly(bx, by, xLine, yLine);
            if numel(yi) < 2, continue; end
            yi = sort(yi(:));
            nPairs = floor(numel(yi)/2);
            spans = reshape(yi(1:2*nPairs),2,[]).';
            widths = spans(:,2)-spans(:,1);
            spans = spans(widths > 10,:);
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
        wps = removeConsecutiveDuplicates(wps);
    end

    function wps = addConnectorsIfNeeded(wps, freePoly, inflatedHard, gridResCm)
        if isempty(wps) || size(wps,1) < 2
            return;
        end
        out = wps(1,:);
        for i = 1:(size(wps,1)-1)
            A = wps(i,:);
            B = wps(i+1,:);
            if segmentOk(A,B,freePoly, inflatedHard)
                out = [out; B]; %#ok<AGROW>
            else
                pathAB = planGridPath(freePoly, A, B, gridResCm);
                if isempty(pathAB)
                    % fallback: still connect directly (will be caught by validate)
                    out = [out; B]; %#ok<AGROW>
                else
                    out = [out; pathAB(2:end,:)]; %#ok<AGROW>
                end
            end
        end
        wps = removeConsecutiveDuplicates(out);
    end

    function ok = segmentOk(A,B,freePoly, inflatedHard)
        % sample along segment, ensure inside free space and outside hard obstacles
        n = max(10, ceil(hypot(B(1)-A(1), B(2)-A(2))/25));
        ts = linspace(0,1,n);
        xs = A(1) + ts*(B(1)-A(1));
        ys = A(2) + ts*(B(2)-A(2));
        inFree = isinterior(freePoly, xs, ys);
        if ~all(inFree)
            ok = false; return;
        end
        if inflatedHard.NumRegions > 0
            inObs = isinterior(inflatedHard, xs, ys);
            if any(inObs)
                ok = false; return;
            end
        end
        ok = true;
    end

    function path = planGridPath(freePoly, start, goal, gridResCm)
        % A* on a grid over freePoly bounding box.
        if freePoly.NumRegions < 1
            path = [];
            return;
        end

        v = freePoly.Vertices;
        xMin = min(v(:,1)); xMax = max(v(:,1));
        yMin = min(v(:,2)); yMax = max(v(:,2));

        xs = xMin:gridResCm:xMax;
        ys = yMin:gridResCm:yMax;
        if numel(xs) < 2 || numel(ys) < 2
            path = [];
            return;
        end

        [X,Y] = meshgrid(xs, ys);
        occ = ~isinterior(freePoly, X, Y); % true = blocked

        s = worldToGrid(start, xs, ys);
        g = worldToGrid(goal, xs, ys);
        if any(isnan([s g])) || occ(s(2),s(1)) || occ(g(2),g(1))
            path = [];
            return;
        end

        [cameFrom, success] = aStarGrid(occ, s, g);
        if ~success
            path = [];
            return;
        end

        idxPath = reconstructPath(cameFrom, s, g);
        pts = zeros(size(idxPath,1),2);
        for i = 1:size(idxPath,1)
            ix = idxPath(i,1); iy = idxPath(i,2);
            pts(i,:) = [xs(ix), ys(iy)];
        end
        pts = simplifyByAngle(pts, deg2rad(10));
        path = pts;
    end

    function ij = worldToGrid(p, xs, ys)
        [~, ix] = min(abs(xs - p(1)));
        [~, iy] = min(abs(ys - p(2)));
        if isempty(ix) || isempty(iy)
            ij = [NaN NaN];
        else
            ij = [ix iy]; % x index, y index
        end
    end

    function [cameFrom, success] = aStarGrid(occ, s, g)
        % occ: rows=y, cols=x
        nRows = size(occ,1);
        nCols = size(occ,2);

        % cameFrom stores predecessor as linear index (0 means none)
        cameFrom = zeros(nRows*nCols,1,'int32');
        gScore = inf(nRows*nCols,1);
        fScore = inf(nRows*nCols,1);

        sLin = sub2ind([nRows nCols], s(2), s(1));
        gLin = sub2ind([nRows nCols], g(2), g(1));
        gScore(sLin) = 0;
        fScore(sLin) = heuristic(s, g);

        open = false(nRows*nCols,1);
        open(sLin) = true;

        success = false;
        neigh = [ -1 -1; 0 -1; 1 -1; -1 0; 1 0; -1 1; 0 1; 1 1 ];

        while any(open)
            % pick open node with min fScore
            idx = find(open);
            [~, m] = min(fScore(idx));
            curLin = idx(m);
            open(curLin) = false;

            if curLin == gLin
                success = true;
                return;
            end

            [cy, cx] = ind2sub([nRows nCols], curLin);
            for k = 1:size(neigh,1)
                nx = cx + neigh(k,1);
                ny = cy + neigh(k,2);
                if nx < 1 || nx > nCols || ny < 1 || ny > nRows
                    continue;
                end
                if occ(ny, nx)
                    continue;
                end
                nLin = sub2ind([nRows nCols], ny, nx);
                stepCost = hypot(neigh(k,1), neigh(k,2));
                tentative = gScore(curLin) + stepCost;
                if tentative < gScore(nLin)
                    cameFrom(nLin) = int32(curLin);
                    gScore(nLin) = tentative;
                    fScore(nLin) = tentative + heuristic([nx ny],[g(1) g(2)]);
                    open(nLin) = true;
                end
            end
        end
    end

    function h = heuristic(a, b)
        h = hypot(double(a(1)-b(1)), double(a(2)-b(2)));
    end

    function idxPath = reconstructPath(cameFrom, s, g)
        % return as [ix iy] rows
        nRowsGuess = numel(cameFrom);
        % start from goal
        idx = [];
        cur = sub2ind([size(cameFrom,1) 1], sub2ind([size(cameFrom,1) 1], 1, 1)); %#ok<NASGU>
        curLin = sub2ind([0 0],1,1); %#ok<NASGU>
        % The above dummy lines avoid MATLAB editor warnings in some configs.
        % Real work:
        curLin = sub2ind([size(occPlaceholder(),1) size(occPlaceholder(),2)], 1, 1); %#ok<NASGU>
        %#ok<*NASGU>
        % We can't access occ dims here, so reconstruct uses linear indices only.
        sLin = sub2ind([sizeDummyRows(), sizeDummyCols()], 1, 1); %#ok<NASGU>
        %#ok<*NODEF>
        % Implementation below overwrites with correct dims via nested scope variable capture.
        %#ok<*AGROW>
        idx = [];
        nRows = size(evalin('caller','occ'),1); %#ok<EVLC>
        nCols = size(evalin('caller','occ'),2); %#ok<EVLC>
        sLin = sub2ind([nRows nCols], s(2), s(1));
        gLin = sub2ind([nRows nCols], g(2), g(1));
        curLin = gLin;
        while curLin ~= 0
            [iy, ix] = ind2sub([nRows nCols], curLin);
            idx = [ix iy; idx];
            if curLin == sLin
                break;
            end
            curLin = cameFrom(curLin);
        end
        idxPath = idx;
    end

    function pts = simplifyByAngle(pts, angThreshRad)
        if size(pts,1) < 3
            return;
        end
        keep = true(size(pts,1),1);
        for i = 2:(size(pts,1)-1)
            a = pts(i-1,:); b = pts(i,:); c = pts(i+1,:);
            v1 = b-a; v2 = c-b;
            if norm(v1) < 1e-9 || norm(v2) < 1e-9
                continue;
            end
            ang = acos(max(-1,min(1, dot(v1,v2)/(norm(v1)*norm(v2)))));
            if abs(pi-ang) < angThreshRad
                keep(i) = false;
            end
        end
        pts = pts(keep,:);
    end

    function pts = densifyPolyline(pts, stepCm)
        if size(pts,1) < 2
            return;
        end
        out = pts(1,:);
        for i = 1:(size(pts,1)-1)
            A = pts(i,:); B = pts(i+1,:);
            d = hypot(B(1)-A(1), B(2)-A(2));
            if d <= stepCm
                out = [out; B]; %#ok<AGROW>
                continue;
            end
            n = ceil(d/stepCm);
            ts = linspace(0,1,n+1);
            seg = [A(1) + ts(:)*(B(1)-A(1)), A(2) + ts(:)*(B(2)-A(2))];
            out = [out; seg(2:end,:)]; %#ok<AGROW>
        end
        out = removeConsecutiveDuplicates(out);
        pts = out;
    end

    function rep = validatePathSegments(path, inflatedHard)
        rep = struct();
        lines = {};
        badSeg = [];
        if inflatedHard.NumRegions < 1
            lines{end+1} = 'No hard obstacles (inflated) -> segments OK.'; %#ok<AGROW>
            rep.lines = lines;
            rep.badSegments = badSeg;
            return;
        end
        for i = 1:(size(path,1)-1)
            A = path(i,:); B = path(i+1,:);
            if segmentIntersectsPoly(A,B, inflatedHard)
                badSeg(end+1,:) = [i 0]; %#ok<AGROW>
            end
        end
        if isempty(badSeg)
            lines{end+1} = 'Validation OK: 0 hard-obstacle segment intersections.'; %#ok<AGROW>
        else
            lines{end+1} = sprintf('Validation FAIL: %d segments intersect hard obstacles.', size(badSeg,1)); %#ok<AGROW>
            lines{end+1} = 'First few bad segment indices:'; %#ok<AGROW>
            showN = min(10, size(badSeg,1));
            for k = 1:showN
                lines{end+1} = sprintf('  segment %d -> %d', badSeg(k,1), badSeg(k,1)+1); %#ok<AGROW>
            end
        end
        rep.lines = lines;
        rep.badSegments = badSeg;
    end

    function hit = segmentIntersectsPoly(A,B, poly)
        n = max(10, ceil(hypot(B(1)-A(1), B(2)-A(2))/10));
        ts = linspace(0,1,n);
        xs = A(1) + ts*(B(1)-A(1));
        ys = A(2) + ts*(B(2)-A(2));
        hit = any(isinterior(poly, xs, ys));
    end

    function [failPts, failModes] = runMonteCarloFailure(path, cfg, freePoly, inflatedHard)
        failPts = [];
        failModes = {};
        for run = 1:cfg.N_RUNS
            seed = run-1;
            [xt,yt,~,~,~,~,wallFail,failPt,failMode] = simulateRunFail(path, cfg, seed, freePoly, inflatedHard);
            %#ok<ASGLU>
            if ~isempty(failPt)
                failPts(end+1,:) = failPt; %#ok<AGROW>
                failModes{end+1,1} = failMode; %#ok<AGROW>
            elseif wallFail
                failModes{end+1,1} = 'outside_map'; %#ok<AGROW>
            end
        end
    end

    function [xt,yt,xm,ym,pathEnded,hitMaxSteps,wallFail,failPt,failMode] = simulateRunFail(path, cfg, seed, freePoly, inflatedHard)
        rng(seed);
        failPt = [];
        failMode = '';

        x = path(1,1); y = path(1,2);
        if size(path,1) >= 2
            theta = atan2(path(2,2)-y, path(2,1)-x);
        else
            theta = 0;
        end

        xm_i = x + randn()*cfg.POS_STD;
        ym_i = y + randn()*cfg.POS_STD;
        thm_i = wrapPi(theta + randn()*cfg.HEAD_STD);

        segIdx = 1;
        xt = zeros(cfg.MAX_STEPS,1); yt = zeros(cfg.MAX_STEPS,1);
        xm = zeros(cfg.MAX_STEPS,1); ym = zeros(cfg.MAX_STEPS,1);
        xt(1)=x; yt(1)=y; xm(1)=xm_i; ym(1)=ym_i;
        pathEnded = false; hitMaxSteps = false; wallFail = false;

        for k = 2:cfg.MAX_STEPS
            if segIdx >= size(path,1)
                pathEnded = true;
                xt = xt(1:k-1); yt = yt(1:k-1); xm = xm(1:k-1); ym = ym(1:k-1);
                return;
            end
            segIdx = advanceSegment(xm_i, ym_i, path, segIdx, cfg.WP_R);

            [gx, gy, foundGoal] = findLookaheadGoal(xm_i, ym_i, path, segIdx, cfg.LA);
            if ~foundGoal
                failPt = [xm_i ym_i];
                failMode = 'no_goal';
                xt = xt(1:k-1); yt = yt(1:k-1); xm = xm(1:k-1); ym = ym(1:k-1);
                return;
            end
            if inflatedHard.NumRegions > 0 && isinterior(inflatedHard, gx, gy)
                failPt = [gx gy];
                failMode = 'goal_in_obstacle';
                xt = xt(1:k-1); yt = yt(1:k-1); xm = xm(1:k-1); ym = ym(1:k-1);
                return;
            end

            [wl, wr, valid] = purePursuitCommand(xm_i, ym_i, thm_i, path, segIdx, cfg.LA, cfg.V, cfg.TRACK_W, cfg.WHEEL_R);
            if ~valid
                wl = 0.5*cfg.V/cfg.WHEEL_R;
                wr = wl;
            end

            vL = wl*cfg.WHEEL_R; vR = wr*cfg.WHEEL_R;
            v = (vL+vR)/2;
            omega = (vR-vL)/cfg.TRACK_W;
            x = x + v*cos(theta)*cfg.DT;
            y = y + v*sin(theta)*cfg.DT;
            theta = wrapPi(theta + omega*cfg.DT);

            if ~isinterior(outerWalls, x, y) || ~isinterior(freePoly, x, y)
                wallFail = true;
                failPt = [x y];
                failMode = 'outside_free_space';
                xt(k)=x; yt(k)=y; xm(k)=xm_i; ym(k)=ym_i;
                xt = xt(1:k); yt = yt(1:k); xm = xm(1:k); ym = ym(1:k);
                return;
            end
            if inflatedHard.NumRegions > 0 && isinterior(inflatedHard, x, y)
                failPt = [x y];
                failMode = 'entered_obstacle';
                xt(k)=x; yt(k)=y; xm(k)=xm_i; ym(k)=ym_i;
                xt = xt(1:k); yt = yt(1:k); xm = xm(1:k); ym = ym(1:k);
                return;
            end

            xm_i = x + randn()*cfg.POS_STD;
            ym_i = y + randn()*cfg.POS_STD;
            thm_i = wrapPi(theta + randn()*cfg.HEAD_STD);

            xt(k)=x; yt(k)=y; xm(k)=xm_i; ym(k)=ym_i;

            if hypot(x - path(end,1), y - path(end,2)) < cfg.END_R
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
        if ~found
            wl = 0; wr = 0; valid = false; return;
        end
        dx = gx - curx;
        dy = gy - cury;
        L = hypot(dx,dy);
        if L < 1.0
            wl = 0; wr = 0; valid = false; return;
        end
        alpha = wrapPi(atan2(dy,dx) - heading);
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

    function a = wrapPi(a)
        while a > pi, a = a - 2*pi; end
        while a < -pi, a = a + 2*pi; end
    end

    function nums = parseCsvNumbers(s)
        s = strrep(string(s), ';', ',');
        parts = split(s, ',');
        nums = [];
        for i = 1:numel(parts)
            t = strtrim(parts(i));
            if t == "", continue; end
            v = str2double(t);
            if isnan(v)
                nums = [];
                return;
            end
            nums(end+1) = v; %#ok<AGROW>
        end
    end

    function pts = removeConsecutiveDuplicates(pts)
        if isempty(pts), return; end
        d = sqrt(sum(diff(pts,1,1).^2,2));
        keep = [true; d > 1e-6];
        pts = pts(keep,:);
    end

    % Dummy helpers used by reconstructPath to avoid editor warnings when nested.
    function o = occPlaceholder() %#ok<DEFNU>
        o = false(2,2);
    end
    function r = sizeDummyRows() %#ok<DEFNU>
        r = 2;
    end
    function c = sizeDummyCols() %#ok<DEFNU>
        c = 2;
    end
end

