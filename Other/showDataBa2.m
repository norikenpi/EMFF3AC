%5×100の配列を棒グラフにして図示する．

function showDataBa2(data, result01)

    %for i = 1:size(data,1)
      %  data(i,:) = rescaleTo01(data(i,:));
    %end

    % 画像を作成するFigureを生成
    figure;
    
    % グラフのタイトルや軸ラベルなどを設定する
    title('5つの棒グラフ');
    xlabel('X軸');
    ylabel('Y軸');
    %legend('重心からの相対速度最大', '重心からのエネルギー最大', '重心からのC1最大', '重心からの距離最大', '目標相対地位誤差最大');
    
    %title_list = ["重心からのC1最大の衛星のC1" "分離方向の速度を持つ衛星の衛星の中で，最大の速度を持つ衛星の速度の大きさ" "重心からのC1最大の衛星に最も近い衛星までの距離" "重心から見たエネルギー最大の衛星とペアを組んでいる衛星までの重み付き平均距離" "重心から見た運動エネルギー最大の衛星とペアを組んでいる衛星までの重み付き平均距離" "Skylab B" "ISS"];

        
    % データを全て青色でプロット
    bar(data, 'FaceColor', "#0072BD");
    
    % 1つ目、4番目、10番目のデータを赤色に変更
    hold on;
    %minus_index = [1,3,4,5,6,7,9,10,11,12,13,14,15,16,17,18,19,20,21,22,23,24,25,26,27,28,30,31,39,43,79,80,90];
    zero_index = find(result01 == 0);

    for j = 1:size(data,2)
        disp(j)
        disp(zero_index(j))
        bar(zero_index(j), data(zero_index(j)), 'r'); % 1つ目のデータを赤でプロット
    end

    hold off;
        
    % 各グラフのタイトルを設定
    % （適宜変更可能）
    %title(sprintf('グラフ%d', i));
    %title(title_list(i));

    xlabel('Seed');
    
    % X軸とY軸の範囲を調整（必要に応じてコメントアウトを解除）
    %xlim([0, 100]); % X軸の範囲を0から100に設定
    %ylim([0, 1]);   % Y軸の範囲を0から1に設定


end