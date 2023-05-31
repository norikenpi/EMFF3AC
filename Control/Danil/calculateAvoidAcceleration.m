% 衝突回避制御
function u = calculateAvoidAcceleration(relative_position)
    u = -0.01*relative_position/norm(relative_position);
end