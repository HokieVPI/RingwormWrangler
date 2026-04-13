function SimpleWaypointPlanner()
% SimpleWaypointPlanner  Simplified mowing-pattern waypoint planner.
%
% Generates vertical-pass mowing paths inside the MAP_AREA polygon, inset
% 250 cm from all walls.  Two sliders control x-spacing between passes and
% the number of intermediate waypoints per vertical segment (enforcing a
% 300 cm minimum spacing).  The path always closes back to the first point.
% The last segment is aligned parallel to the first segment so the robot
% arrives at the final waypoint with the same heading as segment 0->1.
%
% Export copies a C struct to the clipboard that can be pasted directly
% into RW_Tag_PF_v2.ino.

    %% -------- Map definition (identical to WaypointPlannerGUI) --------
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

    WALL_MARGIN    = 250;   % cm – minimum distance from any wall
    MIN_WP_SPACING = 300;   % cm – minimum distance between consecutive waypoints on vertical segments

    insetVerts = computeInsetPolygon(MAP_AREA, WALL_MARGIN);
    insetPoly  = polyshape(insetVerts(:,1), insetVerts(:,2));

    %% -------- State --------
    st.xStepCm      = 350;
    st.nIntermediate = 0;
    st.currentPATH   = [];

    %% -------- UI --------
    fig = uifigure('Name','Simple Waypoint Planner','Position',[50 50 1200 800]);

    ax = uiaxes(fig,'Position',[20 270 1160 510]);
    title(ax,'Waypoint Planner'); xlabel(ax,'x (cm)'); ylabel(ax,'y (cm)');
    grid(ax,'on'); axis(ax,'equal');

    % --- X-spacing slider ---
    yPos = 230;
    uilabel(fig,'Position',[20 yPos 130 22],'Text','X Spacing (cm):');
    sldXStep = uislider(fig,'Position',[160 yPos+12 520 3], ...
        'Limits',[100 400],'Value',st.xStepCm, ...
        'MajorTicks',100:50:400, ...
        'ValueChangedFcn',@onSliderChanged);
    lblXStep = uilabel(fig,'Position',[700 yPos 80 22], ...
        'Text',sprintf('%.0f',st.xStepCm));

    % --- Intermediate-points slider ---
    yPos = yPos - 55;
    uilabel(fig,'Position',[20 yPos 150 22],'Text','Intermediate Pts:');
    sldIntermediate = uislider(fig,'Position',[160 yPos+12 520 3], ...
        'Limits',[0 20],'Value',st.nIntermediate, ...
        'MajorTicks',0:2:20, ...
        'ValueChangedFcn',@onSliderChanged);
    lblInter = uilabel(fig,'Position',[700 yPos 80 22], ...
        'Text',sprintf('%d',st.nIntermediate));

    % --- Buttons ---
    yPos = yPos - 50;
    uibutton(fig,'push','Text','Generate Path', ...
        'Position',[20 yPos 150 34],'ButtonPushedFcn',@onGenerate);
    uibutton(fig,'push','Text','Copy to Clipboard', ...
        'Position',[185 yPos 170 34],'ButtonPushedFcn',@onCopyToClipboard);

    % --- Status area ---
    txtStatus = uitextarea(fig,'Position',[20 15 1160 yPos-25], ...
        'Editable','off', ...
        'Value',{'Ready.  Adjust sliders and click Generate Path.'});

    redrawAll();

    %% ==================== Callbacks ====================
    function onSliderChanged(~,~)
        st.xStepCm      = round(sldXStep.Value);
        st.nIntermediate = round(sldIntermediate.Value);
        lblXStep.Text = sprintf('%.0f',st.xStepCm);
        lblInter.Text = sprintf('%d',st.nIntermediate);
    end

    function onGenerate(~,~)
        st.xStepCm      = round(sldXStep.Value);
        st.nIntermediate = round(sldIntermediate.Value);

        wps = generateMowingPath(insetPoly, st.xStepCm, ...
                                 st.nIntermediate, MIN_WP_SPACING);
        if isempty(wps) || size(wps,1) < 2
            txtStatus.Value = {'No valid path generated.  Try different parameters.'};
            st.currentPATH = [];
        else
            st.currentPATH = wps;
            totalDist = sum(sqrt(sum(diff(wps).^2, 2)));
            txtStatus.Value = {
                sprintf('Generated %d waypoints.  Path length: %.0f cm (%.1f m).', ...
                        size(wps,1), totalDist, totalDist/100)
                'Click "Copy to Clipboard" to export.'
            };
        end
        redrawAll();
    end

    function onCopyToClipboard(~,~)
        if isempty(st.currentPATH)
            txtStatus.Value = {'No path to export.  Generate one first.'};
            return;
        end
        P   = st.currentPATH;
        nWp = size(P,1);
        cLines = cell(nWp+4, 1);
        cLines{1} = sprintf('static constexpr int PATH_LENGTH = %d;', nWp);
        cLines{2} = 'static Waypoint path[PATH_LENGTH] = {';
        for ii = 1:nWp
            if ii == nWp, comma = ''; else, comma = ','; end
            cLines{ii+2} = sprintf('  {%.1f, %.1f}%s', P(ii,1), P(ii,2), comma);
        end
        cLines{nWp+3} = '};';
        cLines{nWp+4} = '';
        clipboard('copy', strjoin(cLines, newline));
        txtStatus.Value = {sprintf('Copied %d waypoints to clipboard as C struct.', nWp)};
    end

    %% ==================== Drawing ====================
    function redrawAll()
        cla(ax); hold(ax,'on');

        plot(ax, MAP_AREA(:,1), MAP_AREA(:,2), 'k-', 'LineWidth',1.5);
        plot(ax, insetVerts(:,1), insetVerts(:,2), 'k--', 'LineWidth',1.0);

        if ~isempty(st.currentPATH)
            plot(ax, st.currentPATH(:,1), st.currentPATH(:,2), ...
                 'g-','LineWidth',1.5);
            plot(ax, st.currentPATH(:,1), st.currentPATH(:,2), ...
                 'go','MarkerSize',5,'MarkerFaceColor','g');
            for k = 1:size(st.currentPATH,1)
                text(ax, st.currentPATH(k,1)+20, st.currentPATH(k,2)+20, ...
                     num2str(k),'FontSize',7,'Color',[0 0.5 0]);
            end
        end

        axis(ax,'equal'); grid(ax,'on'); hold(ax,'off');
    end

    %% ==================== Path generation ====================
    function wps = generateMowingPath(iPoly, xStep, nInter, minSpc)
        wps = [];
        if iPoly.NumRegions < 1, return; end

        vv = iPoly.Vertices;
        xMin = min(vv(:,1));  xMax = max(vv(:,1));

        % Fixed start point (requested)
        startX = 250;
        startY = 500;

        xs = xMin:xStep:xMax;
        if isempty(xs), return; end
        % Force a column exactly on startX so the first waypoint is exact.
        xs = unique([startX, xs], 'stable');
        % Keep only columns within the inset polygon x-range.
        xs = xs(xs >= xMin - 1e-6 & xs <= xMax + 1e-6);
        if isempty(xs), return; end

        [bx, by] = boundary(iPoly);
        yBounds  = [min(by)-100, max(by)+100];

        cols = struct('x',{},'ymin',{},'ymax',{});
        for ci = 1:numel(xs)
            x0 = xs(ci);
            [~, yi] = polyxpoly(bx, by, [x0 x0], yBounds);
            if numel(yi) < 2, continue; end
            ybot = min(yi);  ytop = max(yi);
            if ytop - ybot < minSpc, continue; end
            cols(end+1) = struct('x',x0,'ymin',ybot,'ymax',ytop); %#ok<AGROW>
        end
        if isempty(cols), return; end

        % Use the fixed start point on the first column (assumed to be startX).
        % If startX didn't produce a valid column, fall back to the first available.
        [~, startColIdx] = min(abs([cols.x] - startX));
        if startColIdx ~= 1
            cols = cols([startColIdx, 1:startColIdx-1, startColIdx+1:end]);
        end
        startX = cols(1).x;  % snapped to actual first column x
        startY = max(startY, cols(1).ymin + 1e-3); % keep above min-y line

        nCols    = numel(cols);
        goingUp  = true;
        connectY = NaN;   % y-level carried from a top-connecting hop

        for ci = 1:nCols
            x      = cols(ci).x;
            ymin_c = cols(ci).ymin;
            ymax_c = cols(ci).ymax;

            if goingUp
                effYmax = ymax_c;
                if ci < nCols
                    effYmax = min(ymax_c, cols(ci+1).ymax);
                end

                yStart = ymin_c;
                if ci == 1
                    % First segment should start at the fixed point and go UP.
                    yStart = startY;
                end
                seg = makeVertSeg(x, yStart, effYmax, nInter, minSpc);
                wps = appendWps(wps, seg);

                if ci < nCols
                    xN   = cols(ci+1).x;
                    xMid = (x + xN) / 2;
                    wps = appendWps(wps, [xMid, effYmax]);
                    wps = appendWps(wps, [xN,   effYmax]);
                    connectY = effYmax;
                end
            else
                % Start from the connecting y-level, not ymax_c, to
                % avoid a short UP-then-DOWN reversal when ymax_c >
                % the hop level set by the preceding UP column.
                yTop = ymax_c;
                if ~isnan(connectY)
                    yTop     = connectY;
                    connectY = NaN;
                end

                seg = makeVertSeg(x, yTop, ymin_c, nInter, minSpc);
                wps = appendWps(wps, seg);

                if ci < nCols
                    xN   = cols(ci+1).x;
                    xMid = (x + xN) / 2;
                    wps = appendWps(wps, [xMid, ymin_c]);
                    wps = appendWps(wps, [xN,   ymin_c]);
                end
            end
            goingUp = ~goingUp;
        end

        % ----- Close loop (run along min-y line, then turn into start) -----
        if size(wps,1) < 2, return; end
        endPt = wps(end,:);

        % Desired explicit start point
        startPt = [startX, startY];

        % Travel down to the min-y line of the first column, run along it to x=start,
        % then turn up into the start point. This avoids 180-deg reversals and
        % makes the final approach heading +Y (matching the first segment which is +Y).
        yMinLine = cols(1).ymin;

        if abs(endPt(2) - yMinLine) > 1
            wps = appendWps(wps, makeVertSeg(endPt(1), endPt(2), yMinLine, nInter, minSpc));
        end
        if abs(wps(end,1) - startX) > 1
            xMid = (wps(end,1) + startX) / 2;
            wps = appendWps(wps, [xMid, yMinLine]);
            wps = appendWps(wps, [startX, yMinLine]);
        end
        if abs(wps(end,2) - startY) > 1
            wps = appendWps(wps, makeVertSeg(startX, yMinLine, startY, nInter, minSpc));
        end
        if norm(wps(end,:) - startPt) > 1
            wps = appendWps(wps, startPt);
        end
    end

    %% ==================== Helpers ====================
    function seg = makeVertSeg(x, yStart, yEnd, nInter, minSpc)
        span = abs(yEnd - yStart);
        if span < 1
            seg = [x yStart; x yEnd];
            return;
        end
        nSeg = nInter + 1;
        if span / nSeg < minSpc
            nSeg = max(1, floor(span / minSpc));
        end
        ys  = linspace(yStart, yEnd, nSeg+1);
        seg = [repmat(x, numel(ys), 1), ys(:)];
    end

    function wps = appendWps(wps, pts)
        if isempty(wps)
            wps = pts; return;
        end
        if size(pts,1) >= 1 && norm(wps(end,:) - pts(1,:)) < 1
            pts = pts(2:end,:);
        end
        if ~isempty(pts)
            wps = [wps; pts];
        end
    end

    function iv = computeInsetPolygon(mapVerts, margin)
        n = size(mapVerts,1) - 1;          % number of edges (exclude closing vertex)
        edgeVal = zeros(n,1);
        isHorz  = false(n,1);

        for ei = 1:n
            j  = mod(ei, n) + 1;
            dx = mapVerts(j,1) - mapVerts(ei,1);
            dy = mapVerts(j,2) - mapVerts(ei,2);

            if abs(dy) < 1e-6                              % horizontal edge
                isHorz(ei)  = true;
                edgeVal(ei) = mapVerts(ei,2) + sign(dx)*margin;
            else                                           % vertical edge
                isHorz(ei)  = false;
                edgeVal(ei) = mapVerts(ei,1) - sign(dy)*margin;
            end
        end

        iv = zeros(n, 2);
        for ei = 1:n
            p = mod(ei-2, n) + 1;                         % previous edge index
            if isHorz(p) && ~isHorz(ei)
                iv(ei,:) = [edgeVal(ei), edgeVal(p)];
            elseif ~isHorz(p) && isHorz(ei)
                iv(ei,:) = [edgeVal(p),  edgeVal(ei)];
            else
                iv(ei,:) = mapVerts(ei,:);                 % fallback (shouldn't happen)
            end
        end
        iv = [iv; iv(1,:)];                               % close polygon
    end
end
