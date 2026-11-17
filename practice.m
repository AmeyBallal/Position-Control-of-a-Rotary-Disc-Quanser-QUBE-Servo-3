
%% identify_qube_hw.m
% QUBE-Servo 3: Open-loop system identification
%
% Required MAT-file variables:
%   t       : Time (s)
%   u       : Input voltage (V)
%   w_meas  : Measured angular velocity (rad/s)
%
% Supports:
%   step_3V.mat, step_4V.mat, step_5V.mat
%   staircase.mat
%
% Reference values from EE302 report:
%   K   = 16.294 rad/s/V
%   tau = 0.0983 s

%% 1. Load MAT files

mat_files = dir('*.mat');

assert(~isempty(mat_files), ...
    'No MAT files found in current directory.');

step_results = struct([]);
stair_data = [];

for k = 1:length(mat_files)

    fname = mat_files(k).name;
    raw = load(fname);

    % Check required variables
    if ~all(isfield(raw, {'t','u','w_meas'}))
        warning('Skipping %s: required variables missing.',fname);
        continue;
    end

    t = double(raw.t(:));
    u = double(raw.u(:));
    w = double(raw.w_meas(:));

    assert(length(t)==length(u) && length(t)==length(w), ...
        'Signal lengths do not match in %s.',fname);

    % Remove duplicate timestamps
    [t,idx] = unique(t,'stable');

    u = u(idx);
    w = w(idx);

    % Sort according to time
    [t,idx] = sort(t);

    u = u(idx);
    w = w(idx);

    % Separate staircase and step experiments
    if contains(lower(fname),'stair')

        stair_data.name = fname;
        stair_data.t = t;
        stair_data.u = u;
        stair_data.w = w;

    else

        res = fit_step_trial(t,u,w,fname);

        step_results = [step_results; res]; %#ok<AGROW>

    end

end

assert(~isempty(step_results), ...
    'No valid step-response files found.');

%% 2. Average Identified Parameters

K_values = [step_results.K_ls];
tau_values = [step_results.tau_ls];

K_avg = mean(K_values);
tau_avg = mean(tau_values);

%% 3. Identification Summary

fprintf('\n');
fprintf('============================================================\n');
fprintf('       QUBE-SERVO 3 SYSTEM IDENTIFICATION RESULTS\n');
fprintf('============================================================\n');

fprintf('%-18s | %9s | %10s | %10s | %10s\n', ...
    'File','dV(V)','K_LS','Tau(s)','RMSE');

fprintf('------------------------------------------------------------\n');

for i = 1:length(step_results)

    r = step_results(i);

    fprintf('%-18s | %9.3f | %10.4f | %10.4f | %10.4f\n', ...
        r.name,r.dV,r.K_ls,r.tau_ls,r.rmse);

end

fprintf('------------------------------------------------------------\n');

fprintf('\nAVERAGE IDENTIFIED MODEL:\n');

fprintf('K_hw   = %.4f rad/s/V\n',K_avg);
fprintf('tau_hw = %.4f seconds\n',tau_avg);

fprintf('\nSpeed Transfer Function:\n');
fprintf('G(s) = %.4f / (%.4f*s + 1)\n',K_avg,tau_avg);

fprintf('\nPosition Transfer Function:\n');
fprintf('G(s) = %.4f / (s*(%.4f*s + 1))\n', ...
    K_avg,tau_avg);

%% 4. Compare With Report Values

K_report = 16.294;
tau_report = 0.0983;

K_error = abs(K_avg-K_report)/abs(K_report)*100;
tau_error = abs(tau_avg-tau_report)/abs(tau_report)*100;

fprintf('\n============================================================\n');
fprintf('               REPORT VALUE COMPARISON\n');
fprintf('============================================================\n');

fprintf('Parameter       Identified       Report       Error(%%)\n');

fprintf('K               %.4f          %.4f       %.3f\n', ...
    K_avg,K_report,K_error);

fprintf('Tau             %.4f          %.4f       %.3f\n', ...
    tau_avg,tau_report,tau_error);

%% 5. Staircase Validation

if ~isempty(stair_data)

    t_s = stair_data.t;
    u_s = stair_data.u;
    w_s = stair_data.w;

    w_pred_s = zeros(size(t_s));

    w_pred_s(1) = w_s(1);

    for n = 1:length(t_s)-1

        dt = t_s(n+1)-t_s(n);

        a = exp(-dt/tau_avg);

        % Zero-order hold input model
        w_pred_s(n+1) = a*w_pred_s(n) + ...
            K_avg*(1-a)*u_s(n);

    end

    rmse_val = sqrt(mean((w_s-w_pred_s).^2));

    fprintf('\nSTAIRCASE VALIDATION:\n');
    fprintf('File = %s\n',stair_data.name);
    fprintf('RMSE = %.4f rad/s\n',rmse_val);

else

    fprintf('\nNo staircase file found. Validation skipped.\n');

end

%% 6. Calculate P and PI Controller Gains
% Matching parameters.m:
% zeta = 0.80
% wn   = 40 rad/s

zeta = 0.80;
wn = 40;

Kp_only = (2*zeta*wn*tau_avg-1)/K_avg;

Kp_pi = (2*zeta*wn*tau_avg-1)/K_avg;

Ki_pi = (tau_avg*wn^2)/K_avg;

fprintf('\n============================================================\n');
fprintf('             CONTROLLER PARAMETERS\n');
fprintf('============================================================\n');

fprintf('K_dc    = %.4f;\n',K_avg);
fprintf('tau     = %.4f;\n',tau_avg);

fprintf('Kp_only = %.4f;\n',Kp_only);

fprintf('Kp      = %.4f;\n',Kp_pi);
fprintf('Ki      = %.4f;\n',Ki_pi);

%% 7. Plot Step Responses and Fitted Models

figure('Name','QUBE Identification Results', ...
    'Position',[100 100 1100 450]);

subplot(1,2,1);

hold on;
grid on;

colors = lines(length(step_results));

for i = 1:length(step_results)

    r = step_results(i);

    plot(r.t,r.w, ...
        'Color',colors(i,:), ...
        'LineWidth',1.2, ...
        'DisplayName',[r.name ' Measured']);

    plot(r.t,r.w_pred,'--', ...
        'Color',colors(i,:), ...
        'LineWidth',1.7, ...
        'DisplayName',sprintf('Fit K=%.3f, tau=%.4f', ...
        r.K_ls,r.tau_ls));

end

xlabel('Time (s)');
ylabel('Angular velocity (rad/s)');

title('Step Response Identification');

legend('Location','best');

%% 8. Plot Staircase Validation

subplot(1,2,2);

hold on;
grid on;

if ~isempty(stair_data)

    plot(t_s,w_s,'b','LineWidth',1.2, ...
        'DisplayName','Measured Speed');

    plot(t_s,w_pred_s,'r--','LineWidth',1.7, ...
        'DisplayName','Identified Model');

    title(sprintf('Staircase Validation RMSE = %.3f', ...
        rmse_val));

    legend('Location','best');

else

    title('Staircase Data Not Available');

end

xlabel('Time (s)');
ylabel('Angular velocity (rad/s)');

%% 9. Plot Individual Identification Errors

figure('Name','Identification Error');

hold on;
grid on;

for i = 1:length(step_results)

    r = step_results(i);

    plot(r.t,r.w-r.w_pred, ...
        'LineWidth',1.2, ...
        'DisplayName',r.name);

end

xlabel('Time (s)');
ylabel('Error (rad/s)');
title('Measured vs Identified Model Error');

legend('Location','best');

fprintf('\nIdentification completed successfully.\n');

%% ============================================================
%              LOCAL IDENTIFICATION FUNCTION
% =============================================================

function res = fit_step_trial(t,u,w,fname)

    %% A. Initial and final conditions

    N = length(t);

    n_initial = max(1,floor(0.05*N));

    u_init = mean(u(1:n_initial));

    %% B. Detect input step

    idx_step = find(abs(u-u_init)>0.1,1,'first');

    assert(~isempty(idx_step), ...
        'No voltage step detected in %s.',fname);

    t_step = t(idx_step);

    %% C. Calculate steady-state values

    t_ss_start = t_step + 0.80*(t(end)-t_step);

    idx_ss = t>=t_ss_start;

    idx_initial = t<t_step;

    assert(any(idx_ss) && any(idx_initial), ...
        'Insufficient initial or steady-state samples in %s.', ...
        fname);

    u_ss = mean(u(idx_ss));

    w_init = mean(w(idx_initial));

    w_ss = mean(w(idx_ss));

    dV = u_ss-u_init;

    dw = w_ss-w_init;

    assert(abs(dV)>0.1 && abs(dw)>1e-6, ...
        'Invalid step response in %s.',fname);

    %% D. Calculate open-loop gain

    K_63 = dw/dV;

    %% E. Calculate initial time constant

    target = w_init+0.632*dw;

    if dw>=0

        idx63 = find(t>=t_step & w>=target,1,'first');

    else

        idx63 = find(t>=t_step & w<=target,1,'first');

    end

    assert(~isempty(idx63), ...
        '63.2%% crossing not found in %s.',fname);

    tau_63 = t(idx63)-t_step;

    %% F. Prepare data for least-squares fitting

    idx_fit = t>=t_step;

    t_rel = t(idx_fit)-t_step;

    w_data = w(idx_fit);

    %% G. First-order model

    % Model:
    % w(t) = w_init + K*dV*(1-exp(-t/tau))

    K0 = K_63;

    tau0 = max(tau_63,1e-5);

    model = @(p) w_init + ...
        sign(K0)*exp(p(1))*dV.* ...
        (1-exp(-t_rel/exp(p(2))));

    %% H. Nonlinear least-squares optimization

    % Logarithmic parameters ensure positive magnitudes

    p0 = [log(max(abs(K0),1e-8)), ...
          log(tau0)];

    cost_fn = @(p) mean((w_data-model(p)).^2);

    options = optimset( ...
        'Display','off', ...
        'TolX',1e-10, ...
        'TolFun',1e-10, ...
        'MaxIter',5000, ...
        'MaxFunEvals',10000);

    p_best = fminsearch(cost_fn,p0,options);

    %% I. Extract final parameters

    K_ls = sign(K0)*exp(p_best(1));

    tau_ls = exp(p_best(2));

    %% J. Generate fitted response

    w_pred = w_init*ones(size(t));

    w_pred(idx_fit) = w_init + ...
        K_ls*dV.*(1-exp(-t_rel/tau_ls));

    %% K. Calculate fitting error

    rmse = sqrt(mean((w-w_pred).^2));

    %% L. Return results

    res.name = fname;

    res.t = t;

    res.w = w;

    res.dV = dV;

    res.K_63 = K_63;

    res.tau_63 = tau_63;

    res.K_ls = K_ls;

    res.tau_ls = tau_ls;

    res.w_pred = w_pred;

    res.rmse = rmse;

end
