function Linear_Index = Convert_Index_to_Linear(Row,Column,Num_Columns_in_Array)
%CONVERT_INDEX_TO_LINEAR: Converts a row and column index to a linear index

Linear_Index=(Row-1)*Num_Columns_in_Array+Column;
end

