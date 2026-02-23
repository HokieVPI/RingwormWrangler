# Test harness for findLookaheadGoal() from RW_Tag_PF_v2.ino
# Run:  powershell -ExecutionPolicy Bypass -File test_findLookaheadGoal.ps1

$script:passed = 0
$script:failed = 0

function Check($cond, $msg) {
    if ($cond) {
        Write-Host "  PASS: $msg" -ForegroundColor Green
        $script:passed++
    } else {
        Write-Host "  FAIL: $msg" -ForegroundColor Red
        $script:failed++
    }
}

function ApproxEq($a, $b, $tol) {
    if ($null -eq $tol) { $tol = 1.0 }
    return ([Math]::Abs($a - $b) -le $tol)
}

function DistToGoal($gx, $gy, $rx, $ry) {
    $dx = $gx - $rx; $dy = $gy - $ry
    return [Math]::Sqrt($dx*$dx + $dy*$dy)
}

# Port of findLookaheadGoal() - verbatim logic from the .ino
# Returns hashtable: @{ gx; gy; found }
function FindLookaheadGoal($path, $pathLength, $pathSegIdx, $curX, $curY, $lookAhead) {
    $Lsq = $lookAhead * $lookAhead
    $missWp = $true
    $guard = 0
    $maxIter = 50

    while ($missWp) {
        $guard++
        if ($guard -gt $maxIter) {
            return @{ gx = 0; gy = 0; found = $false }
        }

        for ($seg = $pathSegIdx; $seg -lt ($pathLength - 1); $seg++) {
            $dsx = $path[$seg+1].x - $path[$seg].x
            $dsy = $path[$seg+1].y - $path[$seg].y

            $fx = $path[$seg].x - $curX
            $fy = $path[$seg].y - $curY

            $qa = $dsx*$dsx + $dsy*$dsy
            $qb = 2.0 * ($fx*$dsx + $fy*$dsy)
            $qc = ($fx*$fx + $fy*$fy) - $Lsq

            $discriminant = $qb*$qb - 4.0*$qa*$qc

            if ($discriminant -lt 0) {
                $missWp = $false
            } else {
                $sqrtDisc = [Math]::Sqrt($discriminant)

                $t1 = (-$qb - $sqrtDisc) / (2.0 * $qa)
                $t2 = (-$qb + $sqrtDisc) / (2.0 * $qa)

                $bestT = -1.0
                if ($t2 -ge 0 -and $t2 -le 1) {
                    $bestT = $t2
                } elseif ($t1 -ge 0 -and $t1 -le 1) {
                    $bestT = $t1
                }

                if ($bestT -ge 0) {
                    $gx = $path[$seg].x + $bestT * $dsx
                    $gy = $path[$seg].y + $bestT * $dsy
                    return @{ gx = $gx; gy = $gy; found = $true }
                }
            }
        }
    }

    return @{ gx = 0; gy = 0; found = $true }
}

# Helper to build a waypoint
function WP($x, $y) { return @{ x = $x; y = $y } }

# Default path from the .ino
$defaultPath = @( (WP 200 200), (WP 792 1321), (WP 792 1864), (WP 792 2441) )

Write-Host "====== findLookaheadGoal() Test Suite ======" -ForegroundColor Cyan
Write-Host ""

# --- Test 1 ---
Write-Host "Test 1: Robot at first waypoint"
$g = FindLookaheadGoal $defaultPath 4 0 200.0 200.0 100.0
Check $g.found "intersection found"
$d = DistToGoal $g.gx $g.gy 200.0 200.0
Check (ApproxEq $d 100.0 2.0) "goal is ~100 cm away (got $([Math]::Round($d,2)))"
Check ($g.gx -ge 200 -and $g.gx -le 792) "goal x within segment x range ($([Math]::Round($g.gx,1)))"
Check ($g.gy -ge 200 -and $g.gy -le 1321) "goal y within segment y range ($([Math]::Round($g.gy,1)))"
Write-Host ""

# --- Test 2 ---
Write-Host "Test 2: Robot midway along first segment"
$g = FindLookaheadGoal $defaultPath 4 0 496.0 760.5 100.0
Check $g.found "intersection found"
$d = DistToGoal $g.gx $g.gy 496.0 760.5
Check (ApproxEq $d 100.0 2.0) "goal is ~100 cm away (got $([Math]::Round($d,2)))"
Write-Host ""

# --- Test 3 ---
Write-Host "Test 3: Robot near second waypoint, segment 1 active"
$g = FindLookaheadGoal $defaultPath 4 1 792.0 1321.0 100.0
Check $g.found "intersection found"
Check (ApproxEq $g.gx 792.0 2.0) "goal x on vertical segment (got $([Math]::Round($g.gx,1)))"
Check (ApproxEq $g.gy 1421.0 2.0) "goal y ~1421 (got $([Math]::Round($g.gy,1)))"
Write-Host ""

# --- Test 4 ---
Write-Host "Test 4: Robot offset from path, circle still reaches segment"
$g = FindLookaheadGoal $defaultPath 4 1 700.0 1321.0 100.0
Check $g.found "intersection found"
$d = DistToGoal $g.gx $g.gy 700.0 1321.0
Check (ApproxEq $d 100.0 2.0) "goal is ~100 cm away (got $([Math]::Round($d,2)))"
Check (ApproxEq $g.gx 792.0 2.0) "goal x on vertical segment (got $([Math]::Round($g.gx,1)))"
Write-Host ""

# --- Test 5 ---
# BUG: result.found is initialized to true (line 146) and never set to false.
#      When the circle misses all segments, the function still returns found=true
#      with uninitialized gx/gy.  The correct behavior would be found=false.
Write-Host "Test 5: Circle too small to reach path (exposes found-flag bug)"
$g = FindLookaheadGoal $defaultPath 4 0 0.0 0.0 10.0
Check $g.found "found=true returned (BUG: should be false -- result.found never set to false)"
$d = DistToGoal $g.gx $g.gy 0.0 0.0
Check (-not (ApproxEq $d 10.0 2.0)) "goal distance != lookahead (confirms no real intersection)"
Write-Host ""

# --- Test 6 ---
Write-Host "Test 6: Large lookahead from far away"
$g = FindLookaheadGoal $defaultPath 4 0 0.0 0.0 500.0
Check $g.found "intersection found"
$d = DistToGoal $g.gx $g.gy 0.0 0.0
Check (ApproxEq $d 500.0 2.0) "goal is ~500 cm away (got $([Math]::Round($d,2)))"
Write-Host ""

# --- Test 7 ---
Write-Host "Test 7: Last segment of the path"
$g = FindLookaheadGoal $defaultPath 4 2 792.0 1864.0 100.0
Check $g.found "intersection found on last segment"
Check (ApproxEq $g.gx 792.0 2.0) "goal x = 792 (got $([Math]::Round($g.gx,1)))"
Check (ApproxEq $g.gy 1964.0 2.0) "goal y ~1964 (got $([Math]::Round($g.gy,1)))"
Write-Host ""

# --- Test 8 ---
Write-Host "Test 8: Simple horizontal path"
$hPath = @( (WP 0 0), (WP 1000 0) )
$g = FindLookaheadGoal $hPath 2 0 100.0 0.0 200.0
Check $g.found "intersection found"
Check (ApproxEq $g.gx 300.0 1.0) "goal x = 300 (got $([Math]::Round($g.gx,2)))"
Check (ApproxEq $g.gy 0.0 1.0) "goal y = 0 (got $([Math]::Round($g.gy,2)))"
Write-Host ""

# --- Test 9 ---
Write-Host "Test 9: Robot perpendicular to horizontal segment"
$g = FindLookaheadGoal $hPath 2 0 500.0 60.0 100.0
Check $g.found "intersection found"
$expectedX = 500.0 + [Math]::Sqrt(100*100 - 60*60)
Check (ApproxEq $g.gx $expectedX 1.0) "goal x = $([Math]::Round($expectedX,1)) (got $([Math]::Round($g.gx,1)))"
Check (ApproxEq $g.gy 0.0 1.0) "goal y = 0 (got $([Math]::Round($g.gy,2)))"
Write-Host ""

# --- Test 10 ---
Write-Host "Test 10: Circle tangent to segment (discriminant ~ 0)"
$g = FindLookaheadGoal $hPath 2 0 500.0 100.0 100.0
Check $g.found "intersection found (tangent)"
Check (ApproxEq $g.gx 500.0 2.0) "goal x ~ 500 (got $([Math]::Round($g.gx,2)))"
Check (ApproxEq $g.gy 0.0 2.0) "goal y ~ 0 (got $([Math]::Round($g.gy,2)))"
Write-Host ""

# --- Test 11 ---
Write-Host "Test 11: Intersection near segment end (t2 > 1, falls back to t1)"
$g = FindLookaheadGoal $hPath 2 0 950.0 0.0 200.0
Check $g.found "intersection found via t1 fallback"
Check (ApproxEq $g.gx 750.0 2.0) "goal x ~ 750 (got $([Math]::Round($g.gx,2)))"
Check (ApproxEq $g.gy 0.0 1.0) "goal y = 0 (got $([Math]::Round($g.gy,2)))"
Write-Host ""

# --- Test 12 ---
Write-Host "Test 12: Circle skips short segment, finds next"
$sPath = @( (WP 0 0), (WP 100 0), (WP 100 1000) )
$g = FindLookaheadGoal $sPath 3 0 50.0 0.0 80.0
Check $g.found "intersection found on next segment"
$d = DistToGoal $g.gx $g.gy 50.0 0.0
Check (ApproxEq $d 80.0 2.0) "goal is ~80 cm away (got $([Math]::Round($d,2)))"
Check (ApproxEq $g.gx 100.0 2.0) "goal x on vertical segment (got $([Math]::Round($g.gx,1)))"
Write-Host ""

# --- Test 13 ---
Write-Host "Test 13: Diagonal path (3-4-5 triangle)"
$dPath = @( (WP 0 0), (WP 300 400) )
$g = FindLookaheadGoal $dPath 2 0 0.0 0.0 100.0
Check $g.found "intersection found"
Check (ApproxEq $g.gx 60.0 1.0) "goal x = 60 (got $([Math]::Round($g.gx,2)))"
Check (ApproxEq $g.gy 80.0 1.0) "goal y = 80 (got $([Math]::Round($g.gy,2)))"
Write-Host ""

# --- Test 14 ---
Write-Host "Test 14: Path already complete (no segments left)"
$g = FindLookaheadGoal $defaultPath 4 3 792.0 2441.0 100.0
Check (-not $g.found) "no intersection (path complete, no segments)"
Write-Host ""

# --- Test 15 ---
Write-Host "Test 15: Very short first segment, intersection on next"
$tPath = @( (WP 100 100), (WP 101 100), (WP 101 500) )
$g = FindLookaheadGoal $tPath 3 0 100.5 100.0 50.0
Check $g.found "intersection found (on longer segment)"
$d = DistToGoal $g.gx $g.gy 100.5 100.0
Check (ApproxEq $d 50.0 2.0) "goal is ~50 cm away (got $([Math]::Round($d,2)))"
Write-Host ""

# --- Summary ---
Write-Host "====== Results: $($script:passed) passed, $($script:failed) failed ======" -ForegroundColor Cyan
if ($script:failed -gt 0) { exit 1 } else { exit 0 }
