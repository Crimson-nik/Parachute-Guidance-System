%% ================= PARAMETERS =================
V  = 5;                 % Airspeed (m/s)
dt = 0.5;               % Time step (s)
N  = 1500;              % Max simulation steps
SeparationFromWaypoint = 20;
maxTurnRate = deg2rad(25);   % Physical yaw rate limit (rad/s)

%% ================= WAYPOINTS ==================
wps = [
    0   100;
    50  0;
    200 300;
    300 200
];
wp_idx = 1;

%% ================= INITIAL STATE ==============
x   = wps(1,1);
y   = wps(1,2);
psi = -pi/2;

%% ================= LOGGING ====================
X = nan(N,1);
Y = nan(N,1);
idx = 0;

%% ================= SIMULATION =================
for k = 1:N

    if wp_idx >= size(wps,1)
        break;
    end

    t_now = (k-1)*dt;

    %% Path segment
    A = wps(wp_idx,:);
    B = wps(wp_idx+1,:);
    t_vec = B - A;
    L_seg = norm(t_vec);
    t_hat = t_vec / L_seg;

    r = [x - A(1); y - A(2)];
    s = max(0, min(dot(r, t_hat), L_seg));

    %% ================= WIND FIELD =================
    wind = [0.01*x - t_now/100;
            -0.01*y + 5*t_now/2000];

    %% Velocities
    v_air    = V * [cos(psi); sin(psi)];
    v_ground = v_air + wind;
    Vg = max(norm(v_ground), 0.5);   % protect from low speed

    %% ================= ADAPTIVE L1 =================
    L1 = max(20, 5 * Vg);

    %% L1 reference point
    s_ref = min(s + L1, L_seg);
    ref   = A + s_ref * t_hat;

    dx = ref(1) - x;
    dy = ref(2) - y;

    %% ================= GUIDANCE =================
    chi_ref = atan2(dy, dx);                 % desired ground track
    chi     = atan2(v_ground(2), v_ground(1));
    eta     = wrapToPi(chi_ref - chi);

    psi_dot = 2 * Vg / L1 * sin(eta);
    psi_dot = max(min(psi_dot, maxTurnRate), -maxTurnRate);

    psi = psi + psi_dot * dt;

    %% ================= STATE UPDATE =================
    x = x + v_ground(1) * dt;
    y = y + v_ground(2) * dt;

    %% Logging
    idx = idx + 1;
    X(idx) = x;
    Y(idx) = y;

    %% Waypoint capture
    if norm([x - B(1), y - B(2)]) < SeparationFromWaypoint
        wp_idx = wp_idx + 1;
    end
end

%% ================= PLOT =======================
figure; hold on; grid on; axis equal;
plot(X(1:idx), Y(1:idx), 'b', 'LineWidth', 2);
plot(wps(:,1), wps(:,2), 'r--', 'LineWidth', 1.5);
scatter(wps(:,1), wps(:,2), 80, 'r', 'filled');
xlabel('X (m)');
ylabel('Y (m)');
title('Robust L1 Guidance with Strong Wind');
legend('Vehicle Path','Waypoint Path','Waypoints');
