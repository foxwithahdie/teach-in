function rectangle = bounds(x, y, width, height)
    rectangle = struct();

    rectangle.left = x - width / 2;
    rectangle.right = x + width / 2;
    rectangle.top = y + height / 2;
    rectangle.bottom = y - height / 2;
    rectangle.width = width;
    rectangle.height = height;
    rectangle.x = x;
    rectangle.y = y;
end