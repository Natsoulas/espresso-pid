function [startIndex, endIndex, normalizedTime] = extractTemperature(time, temperature, temp_range)

    % Set the temperature range
    % temp_range = [84, 89];

    % Find indices where the temperature is within the specified range
    indicesInRange = find(temperature >= temp_range(1) & temperature <= temp_range(2));

    % Check if any data is within the specified range
    if isempty(indicesInRange)
        disp('No data in the specified temperature range.');
        startIndex = [];
        endIndex = [];
        startTime = [];
        return;
    end

    % Find consecutive indices
    consecutiveIndices = find(diff(indicesInRange) > 1, 1);

    % Extract the start and end indices of the first cycle
    if isempty(consecutiveIndices)
        startIndex = indicesInRange(1);
        endIndex = indicesInRange(end);
    else
        startIndex = indicesInRange(1);
        endIndex = indicesInRange(consecutiveIndices);
    end

    % Extract the start and end indices
    startIndex = indicesInRange(1);
    endIndex = indicesInRange(end);

    % Normalize time to start at 0 seconds
    normalizedTime = (time - time(startIndex))/1000;
    normalizedTime = (normalizedTime(startIndex:endIndex,:));

end
