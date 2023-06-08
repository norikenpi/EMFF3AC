function [magnetic_forces, magnetic_torques] = calculateF_total(satellites, param)
        % 各衛星に働く磁気トルクと磁気力を計算
    magnetic_torques = cell(1, param.N);
    magnetic_forces = cell(1, param.N);

    % 1からNまでの衛星に対して、他のすべての衛星の磁気ダイポールによる磁場を計算し、それらの磁場の合計から各衛星に働く磁気トルクと磁気力を求める。
    % 1からNまでの衛星に対して、Far Fieldモデルで電磁力を計算。
    for i = 1:param.N
        F_total = zeros(3, 1);  
        % それぞれの衛星がある場所での磁力を計算
        for j = 1:param.N
            if i ~= j
                
                r = satellites{j}.position - satellites{i}.position;
                F = magneticForce(satellites{i}.magnetic_moment, satellites{j}.magnetic_moment, r, param);
                F_total = F_total + F;

            end
        end
        

        % 磁気トルクを計算
        %magnetic_torques{i} = cross(satellites{i}.magnetic_moment, B_total);

        % 磁気力を計算
        magnetic_forces{i} = F_total;
        
    end
end
