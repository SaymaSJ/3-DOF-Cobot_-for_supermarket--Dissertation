
function cycle_time_check(way_q_deg, speed_caps_deg, target_cycles_s)
% way_q_deg:    Nx3 matrix of waypoints in DEGREES
% speed_caps_deg: scalar or 1x3 per-joint caps (deg/s)
% target_cycles_s: array of target total cycle times (s)

    way_q = deg2rad(way_q_deg);
    seg_dq = abs(diff(way_q,1,1));           % (N-1)x3
    Nseg   = size(seg_dq,1);

    if isscalar(speed_caps_deg)
        cap_rad = deg2rad(repmat(speed_caps_deg,1,3));
    else
        cap_rad = deg2rad(speed_caps_deg(:).');
    end

    fprintf('\n=== Cycle-time feasibility ===\n');

    all_req_deg = [];  % store for plotting
    labels = {};

    for tc = target_cycles_s
        Tseg = tc / Nseg;
        req_avg = seg_dq ./ Tseg;             % per joint, rad/s
        req_avg_deg = rad2deg(req_avg);

        all_req_deg(:,:,end+1) = req_avg_deg; %#ok<AGROW>
        labels{end+1} = sprintf('%.1f s', tc); %#ok<AGROW>

        fprintf('\nTarget cycle %.1f s:\n', tc);
        disp(array2table(req_avg_deg, ...
            'VariableNames',{'q1_req','q2_req','q3_req'}));
    end

    % --- Plotting ---
    figure('Name','Cycle Time Analysis','Color','w');
    for j=1:3
        subplot(1,3,j);
        hold on; grid on;
        bar(squeeze(all_req_deg(:,j,:)));
        title(sprintf('Joint %d',j));
        xlabel('Segment'); ylabel('Speed (deg/s)');
        xticklabels(1:Nseg);
        % plot cap line
        yline(speed_caps_deg(min(end,j)),'r--','Cap');
        legend(labels,'Location','northoutside','Orientation','horizontal');
    end
end
