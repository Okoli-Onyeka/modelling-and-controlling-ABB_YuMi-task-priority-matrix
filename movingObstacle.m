% New function defines a simple back and forth (oscillation) along a line moving obstacles for question 7

function p = movingObstacle(t, p0)
    % p0 is the initial obstacle position
    amplitude = 0.2;          % distance
    freq = 0.2;               %(Hz) speed of motion (updates)

    p = p0;
    p(2) = p0(2) + amplitude * sin(2*pi*freq*t);   % back-and-forth motion from sine wave formula
end