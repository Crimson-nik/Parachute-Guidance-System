%% ================= PARAMETERS =================
V  = 5;          % Airspeed (m/s)
L1 = 50;        % L1 lookahead distance (m)
dt = 1;          % Time step (s)
N  = 1000;       % Max simulation steps
SeparationFromWaypoint=20;% max dist from waypoint to be success

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

    % Time
    t_now = (k-1) * dt;

    % Segment endpoints
    A = wps(wp_idx,:);
    B = wps(wp_idx+1,:);

    % Path unit vector
    t = B - A;
    L_seg = norm(t);
    t_hat = t / L_seg;

    % Position relative to A
    r = [x - A(1); y - A(2)];
    s = dot(r, t_hat);
    s = max(0, min(s, L_seg));

    % L1 reference point
    s_ref = min(s + L1, L_seg);
    ref = A + s_ref * t_hat;

    % Vector to reference
    dx = ref(1) - x;
    dy = ref(2) - y;

    % ================= WIND FIELD =================
    % Time and space varying wind
    wind = [0.01*x-t_now/100; -0.01*y+5*t_now/2000];

    % Air-relative velocity
    v_air = V * [cos(psi); sin(psi)];

    % Ground-relative velocity
    v_ground = v_air + wind;
    Vg = norm(v_ground);

    % ================= L1 GUIDANCE =================
    eta   = wrapToPi(atan2(dy, dx) - psi);
    a_lat = 2 * Vg^2 / L1 * sin(eta);

    % Heading update
    psi = psi + (a_lat / V) * dt;

    % Position update
    x = x + v_ground(1) * dt;
    y = y + v_ground(2) * dt;

    % Log
    idx = idx + 1;
    X(idx) = x;
    Y(idx) = y;

    % ================= WAYPOINT CAPTURE 
    if norm([x - B(1), y - B(2)]) < SeparationFromWaypoint
        wp_idx = wp_idx + 1;
    end
end


%% ================= PLOT =======================
figure; hold on; grid on; axis equal;

% Vehicle path
plot(X(1:idx), Y(1:idx), 'b', 'LineWidth', 2);

% Waypoint path
plot(wps(:,1), wps(:,2), 'r--', 'LineWidth', 1.5);

% Waypoints
scatter(wps(:,1), wps(:,2), 80, 'r', 'filled');

xlabel('X (m)');
ylabel('Y (m)');
title('L1 Guidance with Time-Varying Wind');
legend('Vehicle Path','Waypoint Path','Waypoints');
