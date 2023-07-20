function reative_velocity_square_mean = calculateReativeVelocitySquareMean(satellites, param)
    reative_velocity_square_mean = 0;
    for i = 1:param.N
        reative_velocity_square_mean = reative_velocity_square_mean + norm(satellites{i}.velocity)^2/param.N;
    end
end