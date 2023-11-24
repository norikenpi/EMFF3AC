% 原点距離の4乗の逆数を計算
function vec = RelativeDistanceFourthPower(s, s0, num)
    s = [s(1+6*num:end);s0];
    pos_vec = extract_3elements(s);
    rel_pos_vec = 
    vec = dist_fourth(rel_pos_vec);
    


    function extracted = extract_3elements(vector)
        n = length(vector); % ベクトルの長さを取得
        extracted = []; % 抽出された要素を格納するための空のベクトル
    
        i = 1;
        while i <= n
            % 現在の位置から3要素を抽出（範囲外にならないようにチェック）
            end_index = min(i+2, n);
            extracted = [extracted; vector(i:end_index)];
    
            % 6要素分進める（3要素抽出して3要素スキップ）
            i = i + 6;
        end
    end

    function dist_fourth_vec = dist_fourth(pos_vec)
        dist_fourth_vec = [];
        for i = 1:3:length(pos_vec) % ステップサイズを3に設定
            subset = pos_vec(i:i+2); % 3つの要素を抽出
            magnitude = norm(subset)^4; % 部分ベクトルの大きさを計算
            dist_fourth_vec = [dist_fourth_vec; magnitude*ones(3,1)]; % 結果を配列に追加
        end
    end


    function rel_pos = calc_rel_pos(pos_vec)
        n = length(vector); % ベクトルの長さを取得
        rel_pos = []; % 抽出された要素を格納するための空のベクトル
        i = 1;
        while i <= n
            end_index = min(i+2, n);
            
            rel_pos = [rel_pos;(i:end_index)];
        end
    end
end