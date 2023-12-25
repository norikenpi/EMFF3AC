for b = 1:5
   F(b,:) = b*ones(1,2);
   F(b)% ここでエラー
   F(b,:) % これはok
end