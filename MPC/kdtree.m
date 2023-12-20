% 100個のランダムな3次元座標を生成
X = rand(100, 3)*5;

% 各点に番号を割り当て（1から100）
pointNumbers = (1:100)';

% KDツリーを構築
tree = KDTreeSearcher(X);

% クエリの中心点（例としてランダムに選ぶ）
queryIndex = randi(100);
queryPoint = X(queryIndex, :);

% 検索範囲を設定（例：0.2）
range = 1;

% 最小距離を設定（例：0.05）
minDistance = 0.05*6;

% 検索範囲内の点のインデックスを取得
idx = rangesearch(tree, queryPoint, range);

% 最小距離よりも遠い点をフィルタリング
filteredIdx = idx{1}(vecnorm(X(idx{1}, :) - queryPoint, 2, 2) > minDistance);

% クエリの中心点の番号と、その周囲にある点の番号を表示
fprintf('Query Point Number: %d\n', queryIndex);
fprintf('Points within range: ');
fprintf('%d ', pointNumbers(idx{1}));
fprintf('\n');
fprintf('Points within range but not too close: ');
fprintf('%d ', pointNumbers(filteredIdx));
fprintf('\n')

% 全ての点をプロット（基本色：青）
scatter3(X(:, 1), X(:, 2), X(:, 3), 'b');

% グラフにホールドオン
hold on;

% 近すぎる点をプロット（色：緑）
scatter3(X(idx{1}, 1), X(idx{1}, 2), X(idx{1}, 3), 'g', 'filled');

% 近すぎない点をプロット（色：黒）
scatter3(X(filteredIdx, 1), X(filteredIdx, 2), X(filteredIdx, 3), 'k', 'filled');

% クエリの中心点をプロット（色：赤）
scatter3(queryPoint(1), queryPoint(2), queryPoint(3), 'r', 'filled');

% グラフのタイトルと軸ラベルを設定
title('3D Point Visualization with KD Tree');
xlabel('X-axis');
ylabel('Y-axis');
zlabel('Z-axis');

% グラフにホールドオフ
hold off;