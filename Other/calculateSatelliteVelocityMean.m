function velocity_mean = calculateSatelliteVelocityMean(satellites)
    velocity_mean = 0;
    for i = 1:size(satellites,2)
        velocity_mean = velocity_mean + norm(satellites{1}.velocity)/size(satellites,2);
    end
end