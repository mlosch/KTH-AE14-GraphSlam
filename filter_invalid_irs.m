function ir_distances = filter_invalid_irs( ir_distances )

    ir_distances(ir_distances > 0.6 | ir_distances <= 0.0) = NaN;

end

