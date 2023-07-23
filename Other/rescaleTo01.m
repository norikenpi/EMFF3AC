%データを0~1に縮尺を変更する．
function list = rescaleTo01(list)
    max_list = max(list);
    min_list = min(list);
    list = (list - min_list)/(max_list - min_list);
end