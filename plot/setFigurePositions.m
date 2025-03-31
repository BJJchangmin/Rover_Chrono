function setFigurePositions(cols)
    % Set Positions for All Figures
    % cols : 숫자, 열의 수
    % Get all figure handles
    fig_handles = findall(groot, 'Type', 'figure');
    fig_handles = flipud(fig_handles);  % Reverse the order of the handles
    % Number of figures
    num_figures = length(fig_handles);
    % Positioning parameters
    width = 600;
    height = 400;
    h_margin = 50;  % Increased horizontal margin
    v_margin = 50;   % Vertical margin remains the same
    % Compute the number of rows based on the number of figures and columns
    rows = ceil(num_figures / cols);
    % Compute positions for figures
    positions = zeros(num_figures, 4);
    for i = 1:num_figures
        row = floor((i-1) / cols);
        col = mod(i-1, cols);
        positions(i, :) = [col*(width + h_margin), (rows-row-1)*(height + v_margin), width, height];
    end
    % Apply positions to figures
    for i = 1:num_figures
        set(fig_handles(i), 'Position', positions(i, :));
    end
end