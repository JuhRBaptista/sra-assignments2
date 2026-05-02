function scan = scanFilter(scan)

    ranges = sqrt(scan(1,:).^2 + scan(2,:).^2);

    maxRange = 3.5;
    minRange = 0.1;

    valid = ~isnan(ranges) & ~isinf(ranges) & ...
            ranges < maxRange & ranges > minRange;

    scan = scan(:, valid);

end