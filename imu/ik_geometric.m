function ik_geometric()
    % --- Robot Physical Constants ---
    a1 = 52.50; 
    a2 = 108.00; 
    a3 = 130.775; % The exact distance to the spherical nub
    
    % --- RECALIBRATED Motor Limits (Degrees) ---
    T1_MIN = -20; T1_MAX = 20;   % Coxa 
    T2_MIN = -45; T2_MAX = 90;   % Femur 
    T3_MIN = -45; T3_MAX = 90;   % Tibia 
    
    % --- Custom Project Color Palette ---
    bg_color = [49, 54, 71] / 255;        % #313647
    panel_color = [67, 86, 99] / 255;     % #435663
    accent_color = [163, 176, 135] / 255; % #A3B087
    text_color = [255, 248, 212] / 255;   % #FFF8D4
    
    % --- Create UI Figure ---
    fig = uifigure('Name', 'Robotic Leg 2-Way Kinematics (Pure Geometric)', 'Position', [100, 100, 800, 600]);
    fig.Color = bg_color;
    
    % --- 3D Plot Area ---
    ax = uiaxes(fig, 'Position', [20, 100, 400, 480]);
    ax.BackgroundColor = panel_color;
    ax.XColor = text_color; ax.YColor = text_color; ax.ZColor = text_color;
    ax.GridColor = text_color;
    xlabel(ax, 'X (mm)'); ylabel(ax, 'Y (mm)'); zlabel(ax, 'Z (mm)');
    axis(ax, 'equal'); grid(ax, 'on');
    view(ax, 3);
    ax.XLim = [50 250]; ax.YLim = [-100 100]; ax.ZLim = [-250 50];
    hold(ax, 'on');
    
    % Initialize plot objects
    legPlot = plot3(ax, [0 0 0 0], [0 0 0 0], [0 0 0 0], '-o', 'LineWidth', 3, 'Color', accent_color, 'MarkerFaceColor', accent_color);
    targetPlot = plot3(ax, 0, 0, 0, 'x', 'Color', text_color, 'MarkerSize', 10, 'LineWidth', 2);
    fkPlot = plot3(ax, 0, 0, 0, 'o', 'Color', text_color, 'MarkerSize', 8, 'LineWidth', 1);
    
    % --- Control Area ---
    pnl = uipanel(fig, 'Position', [440, 100, 340, 480], 'Title', 'Kinematic Controls');
    pnl.BackgroundColor = bg_color; pnl.ForegroundColor = text_color;
    
    % MODE SELECTOR
    uilabel(pnl, 'Position', [20, 430, 60, 22], 'Text', 'Mode:', 'FontWeight', 'bold', 'FontColor', text_color);
    dd_mode = uidropdown(pnl, 'Position', [80, 430, 240, 22], ...
        'Items', {'Inverse Kinematics (XYZ -> Angles)', 'Forward Kinematics (Angles -> XYZ)'}, ...
        'BackgroundColor', panel_color, 'FontColor', text_color);
        
    % --- IK SLIDERS ---
    lbl_ik_x = uilabel(pnl, 'Position', [20, 380, 100, 22], 'Text', 'Target X:', 'FontColor', text_color);
    sld_x = uislider(pnl, 'Position', [120, 390, 200, 3], 'Limits', [50, 250], 'Value', 150, 'FontColor', text_color);
    val_x = uilabel(pnl, 'Position', [120, 365, 200, 22], 'Text', '150.00', 'FontColor', text_color);
    
    lbl_ik_y = uilabel(pnl, 'Position', [20, 340, 100, 22], 'Text', 'Target Y:', 'FontColor', text_color);
    sld_y = uislider(pnl, 'Position', [120, 350, 200, 3], 'Limits', [-100, 100], 'Value', 0, 'FontColor', text_color);
    val_y = uilabel(pnl, 'Position', [120, 325, 200, 22], 'Text', '0.00', 'FontColor', text_color);
    
    lbl_ik_z = uilabel(pnl, 'Position', [20, 300, 100, 22], 'Text', 'Target Z:', 'FontColor', text_color);
    sld_z = uislider(pnl, 'Position', [120, 310, 200, 3], 'Limits', [-250, 20], 'Value', -150, 'FontColor', text_color);
    val_z = uilabel(pnl, 'Position', [120, 285, 200, 22], 'Text', '-150.00', 'FontColor', text_color);
    
    % --- FK SLIDERS (Updated Limits) ---
    lbl_fk_t1 = uilabel(pnl, 'Position', [20, 380, 100, 22], 'Text', 'Theta 1:', 'FontColor', text_color, 'Visible', 'off');
    sld_t1 = uislider(pnl, 'Position', [120, 390, 200, 3], 'Limits', [T1_MIN, T1_MAX], 'Value', 0, 'FontColor', text_color, 'Visible', 'off');
    val_t1 = uilabel(pnl, 'Position', [120, 365, 200, 22], 'Text', '0.00°', 'FontColor', text_color, 'Visible', 'off');
    
    lbl_fk_t2 = uilabel(pnl, 'Position', [20, 340, 100, 22], 'Text', 'Theta 2:', 'FontColor', text_color, 'Visible', 'off');
    sld_t2 = uislider(pnl, 'Position', [120, 350, 200, 3], 'Limits', [T2_MIN, T2_MAX], 'Value', 0, 'FontColor', text_color, 'Visible', 'off');
    val_t2 = uilabel(pnl, 'Position', [120, 325, 200, 22], 'Text', '0.00°', 'FontColor', text_color, 'Visible', 'off');
    
    lbl_fk_t3 = uilabel(pnl, 'Position', [20, 300, 100, 22], 'Text', 'Theta 3:', 'FontColor', text_color, 'Visible', 'off');
    sld_t3 = uislider(pnl, 'Position', [120, 310, 200, 3], 'Limits', [T3_MIN, T3_MAX], 'Value', 45, 'FontColor', text_color, 'Visible', 'off');
    val_t3 = uilabel(pnl, 'Position', [120, 285, 200, 22], 'Text', '45.00°', 'FontColor', text_color, 'Visible', 'off');
    
    % --- Universal Output Readouts ---
    lbl_out1_title = uilabel(pnl, 'Position', [20, 240, 150, 22], 'Text', 'Theta1 (CAD):', 'FontWeight', 'bold', 'FontColor', text_color);
    txt_out1 = uilabel(pnl, 'Position', [150, 240, 150, 22], 'Text', 'N/A', 'FontColor', accent_color);
    
    lbl_out2_title = uilabel(pnl, 'Position', [20, 200, 150, 22], 'Text', 'Theta2 (CAD):', 'FontWeight', 'bold', 'FontColor', text_color);
    txt_out2 = uilabel(pnl, 'Position', [150, 200, 150, 22], 'Text', 'N/A', 'FontColor', accent_color);
    
    lbl_out3_title = uilabel(pnl, 'Position', [20, 160, 150, 22], 'Text', 'Theta3 (CAD):', 'FontWeight', 'bold', 'FontColor', text_color);
    txt_out3 = uilabel(pnl, 'Position', [150, 160, 150, 22], 'Text', 'N/A', 'FontColor', accent_color);
    
    % --- Verification Panel ---
    cmp_pnl = uipanel(fig, 'Position', [20, 20, 760, 60], 'Title', 'Kinematic Math Verification');
    cmp_pnl.BackgroundColor = panel_color; cmp_pnl.ForegroundColor = text_color;
    
    lbl_cmp_target_title = uilabel(cmp_pnl, 'Position', [20, 10, 100, 22], 'Text', 'IK Target:', 'FontWeight', 'bold', 'FontColor', text_color);
    lbl_cmp_target = uilabel(cmp_pnl, 'Position', [100, 10, 200, 22], 'Text', '[150, 0, -150]', 'FontColor', text_color);
    
    lbl_cmp_output_title = uilabel(cmp_pnl, 'Position', [320, 10, 100, 22], 'Text', 'FK Output:', 'FontWeight', 'bold', 'FontColor', text_color);
    lbl_cmp_output = uilabel(cmp_pnl, 'Position', [400, 10, 200, 22], 'Text', 'N/A', 'FontColor', text_color);
    
    lbl_error = uilabel(cmp_pnl, 'Position', [580, 10, 160, 22], 'Text', 'Verification Error: 0.000', 'FontWeight', 'bold', 'FontColor', accent_color);
    
    % --- Callbacks ---
    dd_mode.ValueChangedFcn = @(src, event) switchMode(event.Value);
    
    sld_x.ValueChangingFcn = @(src, event) updateKinematics('IK', event.Value, sld_y.Value, sld_z.Value);
    sld_y.ValueChangingFcn = @(src, event) updateKinematics('IK', sld_x.Value, event.Value, sld_z.Value);
    sld_z.ValueChangingFcn = @(src, event) updateKinematics('IK', sld_x.Value, sld_y.Value, event.Value);
    
    sld_t1.ValueChangingFcn = @(src, event) updateKinematics('FK', event.Value, sld_t2.Value, sld_t3.Value);
    sld_t2.ValueChangingFcn = @(src, event) updateKinematics('FK', sld_t1.Value, event.Value, sld_t3.Value);
    sld_t3.ValueChangingFcn = @(src, event) updateKinematics('FK', sld_t1.Value, sld_t2.Value, event.Value);
    
    % Initialize the view
    updateKinematics('IK', 150, 0, -150);
    
    % =========================================================
    % --- Mode Switch Logic ---
    function switchMode(mode_str)
        isIK = contains(mode_str, 'Inverse');
        
        lbl_ik_x.Visible = isIK; sld_x.Visible = isIK; val_x.Visible = isIK;
        lbl_ik_y.Visible = isIK; sld_y.Visible = isIK; val_y.Visible = isIK;
        lbl_ik_z.Visible = isIK; sld_z.Visible = isIK; val_z.Visible = isIK;
        
        lbl_fk_t1.Visible = ~isIK; sld_t1.Visible = ~isIK; val_t1.Visible = ~isIK;
        lbl_fk_t2.Visible = ~isIK; sld_t2.Visible = ~isIK; val_t2.Visible = ~isIK;
        lbl_fk_t3.Visible = ~isIK; sld_t3.Visible = ~isIK; val_t3.Visible = ~isIK;
        
        if isIK
            lbl_out1_title.Text = 'Theta1 (CAD):'; lbl_out2_title.Text = 'Theta2 (CAD):'; lbl_out3_title.Text = 'Theta3 (CAD):';
            lbl_cmp_target_title.Text = 'IK Target:'; lbl_cmp_output_title.Text = 'FK Output:';
            updateKinematics('IK', sld_x.Value, sld_y.Value, sld_z.Value);
        else
            lbl_out1_title.Text = 'Result X (mm):'; lbl_out2_title.Text = 'Result Y (mm):'; lbl_out3_title.Text = 'Result Z (mm):';
            lbl_cmp_target_title.Text = 'FK Angles:'; lbl_cmp_output_title.Text = 'IK Verified:';
            updateKinematics('FK', sld_t1.Value, sld_t2.Value, sld_t3.Value);
        end
    end

    % =========================================================
    % --- Main Kinematics and Update Function ---
    function updateKinematics(mode, val1, val2, val3)
        if strcmp(mode, 'IK')
            x = val1; y = val2; z = val3;
            val_x.Text = sprintf('%.2f', x); val_y.Text = sprintf('%.2f', y); val_z.Text = sprintf('%.2f', z);
            
            [t1, t2, t3, reachable] = computeIK_CAD(x, y, z);
            lbl_cmp_target.Text = sprintf('[%.1f, %.1f, %.1f] mm', x, y, z);
            
            if ~reachable
                setLegErrorState(); return;
            end
            
            txt_out1.Text = sprintf('%.2f°', t1); txt_out2.Text = sprintf('%.2f°', t2); txt_out3.Text = sprintf('%.2f°', t3);
            
            [fk_x, fk_y, fk_z] = computeFK_CAD(t1, t2, t3);
            lbl_cmp_output.Text = sprintf('[%.1f, %.1f, %.1f] mm', fk_x, fk_y, fk_z);
            
            drawLeg(t1, t2, t3, x, y, z);
            err = sqrt((x-fk_x)^2 + (y-fk_y)^2 + (z-fk_z)^2);
            lbl_error.Text = sprintf('Math Error: %.5f mm', err);
            
        else
            t1 = val1; t2 = val2; t3 = val3;
            val_t1.Text = sprintf('%.2f°', t1); val_t2.Text = sprintf('%.2f°', t2); val_t3.Text = sprintf('%.2f°', t3);
            
            [fk_x, fk_y, fk_z] = computeFK_CAD(t1, t2, t3);
            
            lbl_cmp_target.Text = sprintf('[%.1f°, %.1f°, %.1f°]', t1, t2, t3);
            txt_out1.Text = sprintf('%.2f', fk_x); txt_out2.Text = sprintf('%.2f', fk_y); txt_out3.Text = sprintf('%.2f', fk_z);
            
            [ik_t1, ik_t2, ik_t3, reachable] = computeIK_CAD(fk_x, fk_y, fk_z);
            
            if ~reachable
                lbl_cmp_output.Text = 'IK Invalid State';
                lbl_error.Text = 'IK Solver Failed Match';
            else
                lbl_cmp_output.Text = sprintf('[%.1f°, %.1f°, %.1f°]', ik_t1, ik_t2, ik_t3);
                err = sqrt((t1-ik_t1)^2 + (t2-ik_t2)^2 + (t3-ik_t3)^2);
                lbl_error.Text = sprintf('Angle Match Error: %.3f°', err);
            end
            
            drawLeg(t1, t2, t3, fk_x, fk_y, fk_z);
        end
    end

    function setLegErrorState()
        legPlot.Color = [0.8, 0.2, 0.2]; legPlot.MarkerFaceColor = [0.8, 0.2, 0.2];
        txt_out1.Text = 'Out of Reach'; txt_out2.Text = 'Out of Reach'; txt_out3.Text = 'Out of Reach';
        lbl_cmp_output.Text = 'N/A'; lbl_error.Text = 'N/A';
    end

    function drawLeg(t1_cad, t2_cad, t3_cad, target_x, target_y, target_z)
        legPlot.Color = accent_color; legPlot.MarkerFaceColor = accent_color; 
        d2r = pi / 180;
        t1_math = t1_cad * d2r;
        t2_math = -t2_cad * d2r;            
        t3_math = (t3_cad - 90) * d2r;      
        
        x1 = a1 * cos(t1_math); y1 = a1 * sin(t1_math); z1 = 0;
        x2 = x1 + a2 * cos(t1_math) * cos(t2_math);
        y2 = y1 + a2 * sin(t1_math) * cos(t2_math);
        z2 = z1 + a2 * sin(t2_math);
        
        x3 = x2 + a3 * cos(t1_math) * cos(t2_math + t3_math);
        y3 = y2 + a3 * sin(t1_math) * cos(t2_math + t3_math);
        z3 = z2 + a3 * sin(t2_math + t3_math);
        
        legPlot.XData = [0 x1 x2 x3]; legPlot.YData = [0 y1 y2 y3]; legPlot.ZData = [0 z1 z2 z3];
        targetPlot.XData = target_x; targetPlot.YData = target_y; targetPlot.ZData = target_z;
        fkPlot.XData = x3; fkPlot.YData = y3; fkPlot.ZData = z3;
    end

    % --- PURE GEOMETRIC IK LOGIC (100% STABLE) ---
    function [t1_cad, t2_cad, t3_cad, isValid] = computeIK_CAD(px, py, pz)
        isValid = false; t1_cad = 0; t2_cad = 0; t3_cad = 0;
        
        % 1. Coxa Angle (Yaw)
        t1_cad = atan2(py, px) * (180 / pi);
        
        % 2. Horizontal distance from hip to target
        R = sqrt(px^2 + py^2);
        
        % 3. Distances for the Femur-Tibia 2D Plane
        r = R - a1;
        z = pz;
        
        % 4. Law of Cosines for the Knee Angle
        L_squared = r^2 + z^2;
        
        % Check if target is perfectly on the coxa joint (singularity)
        if L_squared == 0
            return;
        end
            
        c3 = (L_squared - a2^2 - a3^2) / (2 * a2 * a3);
        
        % Precision Clamping
        if c3 < -1.0; c3 = -1.0; end
        if c3 > 1.0; c3 = 1.0; end
        
        % Negative root forces the knee to bend backwards (Mammalian)
        s3 = -sqrt(1 - c3^2); 
        
        t3_math = atan2(s3, c3);
        t2_math = atan2(z, r) - atan2(a3 * s3, a2 + a3 * c3);
        
        % 5. Map Math to CAD conventions
        t2_cad_d = -(t2_math * (180 / pi));
        t3_cad_d = (t3_math * (180 / pi)) + 90.0;
        
        % Angle Wrap Normalization (-180 to 180)
        t2_cad_d = mod(t2_cad_d + 180, 360) - 180;
        t3_cad_d = mod(t3_cad_d + 180, 360) - 180;
        
        % 6. Validate against limits with tolerance
        tol = 0.5;
        if (t1_cad >= T1_MIN-tol && t1_cad <= T1_MAX+tol && ...
            t2_cad_d >= T2_MIN-tol && t2_cad_d <= T2_MAX+tol && ...
            t3_cad_d >= T3_MIN-tol && t3_cad_d <= T3_MAX+tol)
            
            t2_cad = t2_cad_d;
            t3_cad = t3_cad_d;
            isValid = true;
        end
    end

    function [x, y, z] = computeFK_CAD(t1_cad_d, t2_cad_d, t3_cad_d)
        d2r = pi / 180;
        t1_math = t1_cad_d * d2r;
        t2_math = -t2_cad_d * d2r;
        t3_math = (t3_cad_d - 90) * d2r;
        
        t23_math = t2_math + t3_math;
        x = cos(t1_math) * (a3 * cos(t23_math) + a2 * cos(t2_math) + a1);
        y = sin(t1_math) * (a3 * cos(t23_math) + a2 * cos(t2_math) + a1);
        z = a3 * sin(t23_math) + a2 * sin(t2_math);
    end
end