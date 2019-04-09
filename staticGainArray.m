function K = staticGainArray(tf)
dimensions = size(tf.Numerator);
K = zeros(min(dimensions));
for i = 1:dimensions
    for j = 1:dimensions
        K(i, j) = sum(tf.Numerator{i, j})/sum(tf.Denominator{i, j});
    end
end
end

