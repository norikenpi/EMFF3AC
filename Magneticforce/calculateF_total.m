function [magnetic_forces, magnetic_torques] = calculateF_total(satellites, param)
        % 各衛星に働く磁気トルクと磁気力を計算
    magnetic_torques = cell(1, param.N);
    magnetic_forces = cell(1, param.N);

    % 1からNまでの衛星に対して、他のすべての衛星の磁気ダイポールによる磁場を計算し、それらの磁場の合計から各衛星に働く磁気トルクと磁気力を求める。
    % 1からNまでの衛星に対して、Far Fieldモデルで電磁力を計算。
    for satellite_i = 1:param.N
        F_total = zeros(3, 1);  
        % それぞれの衛星がある場所での磁力を計算

        if param.current_type == "DC"
            % 稼働している衛星のみ計算。
            for index = 1:size(param.set{satellite_i},2)
                satellite_j = param.set{satellite_i}(index);
                r = satellites{satellite_j}.position - satellites{satellite_i}.position;
                F = magneticForce(satellites{satellite_i}.magnetic_moment, satellites{satellite_j}.magnetic_moment, r, param);
                F_total = F_total + F;
            end
        elseif param.current_type == "AC"
            %交流の場合の特定の衛星に加わる力を計算する。
            for index = 1:size(param.N,2)-1
                satellite_j = param.set{satellite_i}(index);
                r = satellites{satellite_j}.position - satellites{satellite_i}.position;
                F = magneticForce(satellites{satellite_i}.magnetic_moment, satellites{satellite_j}.magnetic_moment, r, param);
                
                F_total = F_total + F;
            end
        end


        % 全ての衛星と計算
        %{
        for j = 1:param.N
            if satellite_i ~= j
                r = satellites{j}.position - satellites{satellite_i}.position;
                F = magneticForce(satellites{satellite_i}.magnetic_moment, satellites{j}.magnetic_moment, r, param);
                F_total = F_total + F;

            end
        end
        %}
        
        

        % 磁気トルクを計算
        %magnetic_torques{i} = cross(satellites{i}.magnetic_moment, B_total);

        % 磁気力を計算
        magnetic_forces{satellite_i} = F_total;
        
    end
end
