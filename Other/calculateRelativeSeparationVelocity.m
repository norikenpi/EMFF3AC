function relative_separation_velocity = calculateRelativeSeparationVelocity(satellites, param)
    relative_separation_velocity = 0;
    for i = 1:param.N
        if dot(satellites{i}.velocity, satellites{i}.position) > 0
            reative_velocity_square = norm(satellites{i}.velocity)^2;
            if reative_velocity_square > relative_separation_velocity
                relative_separation_velocity = reative_velocity_square;
            end
        end
    end
end