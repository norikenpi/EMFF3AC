classdef Satellite
    properties
        position
        velocity
        orientation
        angular_velocity
        magnetic_moment
        mass
        moment_of_inertia
        max_magnetic_moment
        radius
    end
    
    methods
        function obj = Satellite(position, velocity, orientation, angular_velocity, magnetic_moment, mass, moment_of_inertia, max_magnetic_moment, radius)
            obj.position = position;
            obj.velocity = velocity;
            obj.orientation = orientation;
            obj.angular_velocity = angular_velocity;
            obj.magnetic_moment = magnetic_moment;
            obj.mass = mass;
            obj.moment_of_inertia = moment_of_inertia;
            obj.max_magnetic_moment = max_magnetic_moment;
            obj.radius = radius;
        end
    end
end
