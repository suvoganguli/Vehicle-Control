function y_sum = my_cumsum(y)

y_sum = [];

if ~isempty(y)
    y_sum = cumsum(y) - y(1);
end

end