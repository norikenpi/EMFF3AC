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
        position_d
        velocity_d
    end
    
    methods
        function obj = Satellite(position, velocity, orientation, angular_velocity, magnetic_moment, mass, ...
                moment_of_inertia, max_magnetic_moment, radius, position_d, velocity_d)
            obj.position = position;
            obj.velocity = velocity;
            obj.orientation = orientation;
            obj.angular_velocity = angular_velocity;
            obj.magnetic_moment = magnetic_moment;
            obj.mass = mass;
            obj.moment_of_inertia = moment_of_inertia;
            obj.max_magnetic_moment = max_magnetic_moment;
            obj.radius = radius;
            obj.position_d = position_d;
            obj.velocity_d = velocity_d;
        end
    end
end
