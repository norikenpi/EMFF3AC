function outputStructToTextFile(structure, filename)
    fid = fopen(filename, 'w');
    fields = fieldnames(structure);
    for i = 1:numel(fields)
        fieldName = fields{i};
        fieldValue = structure.(fieldName);
        if isnumeric(fieldValue) || ischar(fieldValue)
            fieldValueStr = num2str(fieldValue);
        elseif islogical(fieldValue)
            fieldValueStr = num2str(double(fieldValue));
        else
            fieldValueStr = 'Cannot output field value';
        end
        fprintf(fid, '%s = %s\n', fieldName, fieldValueStr);
    end
    fclose(fid);
end
