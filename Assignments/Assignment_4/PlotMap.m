function PlotMap(BW)

% Get indices of points with value 1 (free) and 0 (not accessible)

[free_ii,free_jj]=find(BW==1);
[notfree_ii,notfree_jj]=find(BW==0);

% Plot points (!!! It's a matrix, so reference system's origin is top-left,
% with ii axis going top to down and jj axis going left to right)

plot(free_jj,free_ii,'oc', 'MarkerFaceColor','c')
hold on
plot(notfree_jj,notfree_ii,'ok','MarkerFaceColor','k')
box on
xlabel('jj')
ylabel('ii')
axis([1 size(BW,2) 1 size(BW,1)])
axis ij
set(gca,'XAxisLocation','top')
hold off

end
