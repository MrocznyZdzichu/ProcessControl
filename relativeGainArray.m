function L = relativeGainArray(staticGainArray)
L = staticGainArray.*inv(staticGainArray)';
end

