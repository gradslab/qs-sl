function addInset(ax, t, y, win, pos)
% win = [t1 t2] zoom window (s), pos = [x y w h] inset position, normalized to ax
    p = ax.Position;
    axIn = axes('Position', [p(1)+pos(1)*p(3), p(2)+pos(2)*p(4), pos(3)*p(3), pos(4)*p(4)]);
    plot(axIn, t, y, 'LineWidth', 1.2); grid(axIn, 'on'); box(axIn, 'on')
    k = t >= win(1) & t <= win(2);
    lo = min(y(k)); hi = max(y(k));
    xlim(axIn, win); ylim(axIn, [lo hi]);
    axIn.FontSize = ax.FontSize - 2;
    %rectangle('Parent', ax, 'Position', [win(1), lo, diff(win), hi-lo]);
end