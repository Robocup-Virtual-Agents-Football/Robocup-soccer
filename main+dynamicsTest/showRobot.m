
x = linspace(0, 10);
y1 = sin(2*pi*x);
figure(1)
% plot on large axes
plot(x, y1, 'LineWidth', 2)
grid on;
ax1 = gca; % Store handle to axes 1.
% Create smaller axes in top right, and plot on it
% Store handle to axes 2 in ax2.
ax2 = axes('Position',[0.1 0.1 0.2 0.2])


box off;
fileName = 'static2.png';

I = imread(fileName);
% J = imresize(imread(fileName),[150 150]);
% [sample, map, sample_alpha] = imread(fileName);
% [img, map, alphachannel] = imread(fileName);

% img2 = I;
% map = [];
% alphachannel2 = I(:,:,1);

rgb2 = imrotate(I, 90);
[img, map, alphachannel] = rgb2;

% image(Irotate)

image(img, 'AlphaData', alphachannel);

% imagesc(7, 10, img, 'AlphaData', alphachannel);
% imagesc([.1 .1], [.2 .2], B);
axis off;
% Now draw something back on axis 1
hold(ax1, 'on'); % Don't blow away existing curve.
y2 = cos(2*pi*x/0.7);
plot(ax1, x, y2, 'r-', 'LineWidth', 2);