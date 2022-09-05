% measurment僅單一自由度
function match = evaluateTracking( measurment, coordinate_part_of_state, tolerance )
match = 0;
% calculate the square of tracking error
for i = 1 : length(measurment) 
    err =  abs(coordinate_part_of_state(i)- measurment(i)) ;
    if err < tolerance
        match = match + err;
    else
        match = inf;
        break;
    end
end

end
