function t = readmatrixtable(table_name, varargin)

	t = readtable(table_name, varargin{1:end});
	for col = t.Properties.VariableNames
		col = cell2mat(col);
		t.(col) = cell2mat(arrayfun(@(x) str2num(cell2mat(x)), t.(col), 'UniformOutput', false));
	end
end