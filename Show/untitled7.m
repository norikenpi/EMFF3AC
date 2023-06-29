assignin('base', 'force1216', -histories.force_histories(:,:,12));

matrix = (1:15000)';



% 抽出する要素のインデックスを計算
startIndex = 26; % 最初の要素のインデックス
stepSize = 21*10; % 抽出する要素の間隔
indices1 = startIndex:stepSize:length(matrix(:,:));

indices2 = matrix(indices1, :);
% 各要素に連続した9つの数字を付け足す
result = [];
for i = 1:length(indices2)
    new_elements = indices2(i):(indices2(i)+9);
    result = [result, new_elements];
end

force12 = force1216(result, 2);
data = 1:length(force12);
figure

plot(data, force12)

% x軸のラベルとy軸のラベルを設定
xlabel('データ');
ylabel('力(N)');

title('衛星12と衛星16の間に働くy軸方向の力')

grid on