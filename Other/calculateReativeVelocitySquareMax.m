function reative_velocity_square_max = calculateReativeVelocitySquareMax(satellites, param)
    reative_velocity_square_max = 0;
    for i = 1:param.N
        reative_velocity_square = norm(satellites{i}.velocity)^2;
        if reative_velocity_square > reative_velocity_square_max
            reative_velocity_square_max = reative_velocity_square;
        end
    end
end