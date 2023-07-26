%5×100の配列を棒グラフにして図示する．

function showDataBar(data)

    for i = 1:size(data,1)
        data(i,:) = rescaleTo01(data(i,:));
    end

    % 画像を作成するFigureを生成
    figure;
    
    % グラフのタイトルや軸ラベルなどを設定する
    title('5つの棒グラフ');
    xlabel('X軸');
    ylabel('Y軸');
    legend('重心からの相対速度最大', '重心からのエネルギー最大', '重心からのC1最大', '重心からの距離最大', '目標相対地位誤差最大');
    
    graph_num = 2;
    title_list = ["重心からのC1最大の衛星のC1" "重心からのC1最大の衛星に最も近い衛星までの距離" "Apollo" "Skylab" "Skylab B" "ISS"];

    % 5つの棒グラフを縦に並べる
    for i = 1:graph_num
        % グラフの位置とサイズを設定
        subplot(graph_num, 1, i);
        
        % データを全て青色でプロット
        bar(data(i, :), 'FaceColor', "#0072BD");
        
        % 1つ目、4番目、10番目のデータを赤色に変更
        hold on;
        bar(3, data(i, 3), 'r'); % 1つ目のデータを赤でプロット
        bar(6, data(i, 6), 'r'); % 6番目のデータを赤でプロット
        bar(13, data(i, 13), 'r'); % 13番目のデータを赤でプロット
        bar(16, data(i, 16), 'r'); % 16番目のデータを赤でプロット
        bar(17, data(i, 17), 'r'); % 17番目のデータを赤でプロット

        bar(1, data(i, 1), 'y'); % 1つ目のデータを黄でプロット
        bar(4, data(i, 4), 'y'); % 4番目のデータを黄でプロット
        bar(7, data(i, 7), 'y'); % 7番目のデータを黄でプロット
        bar(9, data(i, 9), 'y'); % 9番目のデータを黄でプロット
        bar(10, data(i, 10), 'y'); % 10番目のデータを黄でプロット
        bar(15, data(i, 15), 'y'); % 15番目のデータを黄でプロット
        bar(18, data(i, 18), 'y'); % 18番目のデータを黄でプロット
        bar(20, data(i, 20), 'y'); % 20番目のデータを黄でプロット
        bar(55, data(i, 55), 'y'); % 55番目のデータを黄でプロット
        bar(84, data(i, 84), 'y'); % 84番目のデータを黄でプロット
        bar(87, data(i, 87), 'y'); % 87番目のデータを黄でプロット
        hold off;
            
        % 各グラフのタイトルを設定
        % （適宜変更可能）
        %title(sprintf('グラフ%d', i));
        title(title_list(i));

        xlabel('Seed');
        
        % X軸とY軸の範囲を調整（必要に応じてコメントアウトを解除）
        %xlim([0, 100]); % X軸の範囲を0から100に設定
        %ylim([0, 1]);   % Y軸の範囲を0から1に設定
    end
    
    % サブプロット間のスペースを調整

end
