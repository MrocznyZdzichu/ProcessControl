function [dynamicMatrix] = dynamicMatrix(horizPred, horizControl, A, B, C)
dynamicMatrix = [];
baseColumn = [];
factor = zeros(2, 2);

for i = 1 : horizPred
      factor = factor + A^(i-1); 
   baseColumn = [baseColumn; C*factor*B];
end

for col = 1 : horizControl
   dynamicMatrix = [dynamicMatrix, baseColumn];
   baseColumn = [zeros(2,2); baseColumn(1:end-2, :)];
   baseColumn(2*col - 1 : 2*col, :) = zeros(2, 2);
end
end

