%紐の長さと相対距離を比較して修正
function satellites = updateSatelliteStatesString(satellites, param)
    % それぞれの衛星とつながっている衛星との距離を計算する。
    
    for satellite_i = 1
        param_set = param.set{satellite_i};
        for j = 1:length(param_set)
            satellite_j = param_set(j);
            r = satellites{satellite_j}.position - satellites{satellite_i}.position;
            disp("r")
            disp(norm(r))
            if norm(r) > param.length
                %disp('r')
                %disp(r)
                v = satellites{satellite_j}.velocity - satellites{satellite_i}.velocity;
                disp("v")
                disp(v)
                [~, T] = baseTransformation(r, v);
                v_trans = T \ v;
                if v_trans(1) > 0
                    v_trans_bounced = [-param.cof * v_trans(1); v_trans(2); v_trans(3)];
                    disp("reverse")
                    disp(v_trans)
                    disp("v_trans_bounced")
                    disp(v_trans_bounced)
                    v_bounced = T*v_trans_bounced;
                    disp("v_bounced")
                    disp(v_bounced)
                    satellites{satellite_i}.velocity = -v_bounced/2;
                    satellites{satellite_j}.velocity = v_bounced/2;
                else
                    
                    disp("nonreverse")
                    disp(v_trans)
                end
                
                
            end
        end
    end
end


